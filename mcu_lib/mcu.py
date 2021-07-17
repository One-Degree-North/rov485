# mcu.py
# a comprehensive library for communicating with the MATE OQBot.
# please read docs/packet_structure.md and docs/command_list.md for more information.
# there is a CLI for testing available at mcu_cli.py. read docs/mcu_cli.md for usage.

import serial
import struct
import time
import threading
from queue import Queue, Full

from mcu_lib.packets import *
from mcu_lib.command_constants import *

BYTESTRING_ZERO = chr(0x00).encode('latin')
FORWARD_PACKET_SIZE = 8
RETURN_PACKET_SIZE = 10
MAX_QUEUE_SIZE = 512


# bs ("byte-string")
# converts an int into a single character bytestring
def bs(v: int) -> bytes:
    return chr(v).encode('latin')


# converts a signed int8 to unsigned int8
def to_unsigned_int8(signed_int8: int) -> int:
    assert -128 <= signed_int8 <= 127
    neg = signed_int8 < 0
    if neg:
        return signed_int8 + 0xFF
    return signed_int8


class MCUInterface:
    """
    MCUInterface - Layer for interfacing between a Python program and the microcontroller over Serial.

    Attributes:
        latest_accel : List[float] - latest acceleration values, [x, y, z].
        latest_gyro : List[float] - latest angular velocity values, [x, y, z].
        latest_voltage : float - latest voltage on the 12V rail.
        latest_temp : float - latest temperature measured within the electronics box.
        test_queue : Queue[TestPacket] - Queue to access all received packets of TestPacket type.
        ok_queue : Queue[OKPacket] - Queue to access all received packets of OKPacket type.
        accel_queue : Queue[AccelPacket] - Queue to access all received packets of AccelPacket type.
        gyro_queue : Queue[GyroPacket] - Queue to access all received packets of GyroPacket type.
        volt_temp_queue : Queue[VoltageTemperaturePacket] - Queue to access all received packets of that type.

    Methods:
        get_port() -> str
            returns the interface's serial.Serial object's port.
        get_baud() -> str
            returns the interface's serial.Serial Object's baudrate.
        open_serial()
            opens the integrated serial interface.
        close_serial()
            closes the integrated serial interface.
        cmd_{PACKET_COMMAND}()
            refer to docs/command_list.md. most should be self-explanatory.
    """

    def __init__(self, port: str, baud: int = 230400, close_on_startup: bool = True,
                 refresh_rate: int = 1440, max_read: int = 16):
        """
        Default and only constructor for the MCUInterface class.

        :param port: Port to connect to.
        :param baud: Baudrate to connect at.
        :param close_on_startup: Whether to close the serial object at startup, so it can be opened later.
        :param refresh_rate: Number of times per seconds to refresh the serial cache for packets.
        :param max_read: Maximum number of bytes for the serial object to read at once.
                         Used to mitigate strange pySerial behavior.
        """
        self.__serial = serial.Serial(port, baud, timeout=0, write_timeout=0)
        self.__queue = Queue()
        self.__fetch_thread = threading.Thread(target=self.__read_serial)
        self.__parse_thread = threading.Thread(target=self.__parse_serial)
        self.__write_thread = threading.Thread(target=self.__write_packets)
        self.__refresh_time = 1 / refresh_rate
        self.__thread_enable = False
        self.__init_queues()
        self.__write_queue = Queue()
        self.__wait_half_byte_time = 4 / baud
        self.latest_accel = [0.0, 0.0, 0.0]
        self.latest_gyro = [0.0, 0.0, 0.0]
        self.latest_linear_accel = [0.0, 0.0, 0.0]
        self.latest_orientation = [0.0, 0.0, 0.0]
        self.latest_voltage = 0
        self.latest_temp = 0
        self.latest_motor_status: MotorStatusPacket = MotorStatusPacket((0, 0, 0, 0), 0, 0)
        self.current_calibration = IMUCalibrationPacket((0, 0, 0, 0), 0)
        self.read_size = max_read
        if close_on_startup:
            self.__serial.close()

    def __init_queues(self):
        self.test_queue = Queue(MAX_QUEUE_SIZE)
        self.ok_queue = Queue(MAX_QUEUE_SIZE)
        self.accel_queue = Queue(MAX_QUEUE_SIZE)
        self.gyro_queue = Queue(MAX_QUEUE_SIZE)
        self.linear_accel_queue = Queue(MAX_QUEUE_SIZE)
        self.orientation_queue = Queue(MAX_QUEUE_SIZE)
        self.volt_temp_queue = Queue(MAX_QUEUE_SIZE)
        self.motor_queue = Queue(MAX_QUEUE_SIZE)

    def get_port(self) -> str:
        """
        Get the integrated serial object's port.

        :return: the port.
        """
        return self.__serial.port

    def get_baud(self) -> int:
        """
        Get the integrated serial object's baudrate.

        :return: the baudrate.
        """
        return self.__serial.baudrate

    def open_serial(self):
        """
        Opens the integrated serial's connection to the microcontroller.
        """
        self.__serial.open()
        while not self.__serial.is_open:
            time.sleep(0.01)
        self.__thread_enable = True
        self.__fetch_thread.start()
        self.__parse_thread.start()
        self.__write_thread.start()

    def __read_serial(self):
        while self.__thread_enable:
            try:
                byte_string = self.__serial.read(size=self.read_size)
                for byte in byte_string:
                    self.__queue.put(bs(byte))
            except serial.SerialException:
                pass
            time.sleep(self.__refresh_time)

    def __parse_serial(self):
        while self.__thread_enable:
            if self.__queue.qsize() >= RETURN_PACKET_SIZE:
                packet = self.__read_packet()
                if packet:
                    self.__parse_packet(packet)
            time.sleep(self.__refresh_time)

    def close_serial(self):
        """
        Closes the integrated serial's connection to the microcontroller.
        """
        self.__thread_enable = False
        self.__fetch_thread.join()
        self.__parse_thread.join()
        self.__write_thread.join()
        self.__serial.close()

    def __read_packet(self):  # returns a generic ReturnPacket
        next_byte = self.__queue.get()
        while next_byte != bs(RETURN_HEADER) and self.__queue.qsize() >= RETURN_PACKET_SIZE:
            next_byte = self.__queue.get()
        if self.__queue.qsize() < RETURN_PACKET_SIZE - 1:
            return
        packet_data = []
        for i in range(RETURN_PACKET_SIZE - 1):  # 0x1 to 0x9
            packet_data.append(self.__queue.get())
        if packet_data[-1] != bs(RETURN_FOOTER):
            # invalid packet
            return
        packet = ReturnPacket(packet_data)
        return packet

    def __write_packets(self):
        while self.__thread_enable:
            try:
                pkt = self.__write_queue.get()
                self.__serial.write(pkt)
                time.sleep(self.__wait_half_byte_time)
            except serial.SerialTimeoutException:
                pass

    def __empty_queue(self, queue):
        while not queue.empty():
            queue.get()

    def __parse_packet(self, packet: ReturnPacket):
        data_bs = packet.data[0] + packet.data[1] + packet.data[2] + packet.data[3]
        if not packet:
            return
        # let's all pretend this was a Python 3.10+ match/case statement
        if packet.cmd == bs(RETURN_TEST):
            # test
            version = int.from_bytes(packet.data[0], 'big')
            contents = (packet.data[1] + packet.data[2] + packet.data[3]).decode('latin')
            valid = contents == "pog"
            if self.test_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.test_queue.put_nowait(TestPacket(valid, version, contents, packet.timestamp))
        elif packet.cmd == bs(RETURN_OK):
            # OK
            og_cmd = int.from_bytes(packet.og_cmd, 'big')
            og_param = int.from_bytes(packet.og_param, 'big')
            success = int.from_bytes(packet.param, 'big') > 0
            if self.ok_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.ok_queue.put_nowait(OKPacket(og_cmd, og_param, success, packet.timestamp))
        elif packet.cmd == bs(RETURN_ACCELEROMETER):
            # accel
            axis = int.from_bytes(packet.param, 'big') // AXIS_DIVISOR
            value = struct.unpack('f', data_bs)[0]
            if self.accel_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.accel_queue.put_nowait(AccelPacket(axis, value, packet.timestamp))
            self.latest_accel[axis] = value
        elif packet.cmd == bs(RETURN_GYROSCOPE):
            # gyro
            axis = int.from_bytes(packet.param, 'big') // AXIS_DIVISOR
            value = struct.unpack('f', data_bs)[0]
            if self.gyro_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.gyro_queue.put_nowait(GyroPacket(axis, value, packet.timestamp))
            self.latest_gyro[axis] = value
        elif packet.cmd == bs(RETURN_LINEAR_ACCEL):
            axis = int.from_bytes(packet.param, 'big') // AXIS_DIVISOR
            value = struct.unpack('f', data_bs)[0]
            if self.linear_accel_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.linear_accel_queue.put_nowait(LinearAccelPacket(axis, value, packet.timestamp))
            self.latest_linear_accel[axis] = value
        elif packet.cmd == bs(RETURN_ORIENTATION):
            axis = int.from_bytes(packet.param, 'big') // AXIS_DIVISOR
            value = struct.unpack('f', data_bs)[0]
            if self.orientation_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.orientation_queue.put_nowait(OrientationPacket(axis, value, packet.timestamp))
            self.latest_orientation[axis] = value
        elif packet.cmd == bs(RETURN_VOLT_TEMP):
            # temp/volt
            temp, volts = struct.unpack('HH', data_bs)
            temp /= 100
            volts /= 100
            self.latest_temp = temp
            self.latest_voltage = volts
            if self.volt_temp_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.volt_temp_queue.put_nowait(VoltageTemperaturePacket(volts, temp, packet.timestamp))
        elif packet.cmd == bs(RETURN_MOTOR):
            # motor status
            servo = struct.unpack('b', packet.param)[0]
            motors = struct.unpack('bbbb', data_bs)
            packet = MotorStatusPacket(motors, servo, packet.timestamp)
            if self.motor_queue.qsize() <= MAX_QUEUE_SIZE - 1:
                self.motor_queue.put_nowait(packet)
            self.latest_motor_status = packet
        elif packet.cmd == bs(RETURN_IMU_CALIBRATIONS):
            calibrations = struct.unpack('bbbb', data_bs)
            self.current_calibration = IMUCalibrationPacket(calibrations, packet.timestamp)
        else:
            print(f"Invalid packet received! Command: {packet.cmd}, Param: {packet.param}, Data: {packet.data}")

    def __send_packet(self, cmd: int, param: int, data: bytes):
        assert len(data) == 4, "data is not 4 bytes long!"
        packet = bs(FORWARD_HEADER) + bs(cmd) + bs(param) + data + bs(FORWARD_FOOTER)
        assert len(packet) == 8, "packet is not 8 bytes long!"
        self.__write_queue.put(packet)

    def cmd_test(self):
        self.__send_packet(COMMAND_TEST, PARAM_TEST_SYSTEM, BYTESTRING_ZERO * 4)

    def cmd_halt(self):
        self.__send_packet(COMMAND_HALT, PARAM_ZERO, BYTESTRING_ZERO * 4)

    def cmd_setMotorMicroseconds(self, motor: int, microseconds: int):
        assert 0 <= motor <= NUM_MOTORS - 1, f"There are only {NUM_MOTORS} motors"
        data = bs(microseconds // 0xFF) + bs(microseconds % 0xFF) + BYTESTRING_ZERO * 2
        self.__send_packet(COMMAND_SET_MOTOR_MICROSECONDS, motor, data)

    def cmd_setMotorCalibrated(self, motor: int, percent: int):
        assert 0 <= motor <= NUM_MOTORS - 1, f"There are only {NUM_MOTORS} motors"
        assert -100 <= percent <= 100, "Calibrated % Range is [-100, 100]"
        data = bs(to_unsigned_int8(percent)) + BYTESTRING_ZERO * 3
        self.__send_packet(COMMAND_SET_MOTOR_CALIBRATED, motor, data)

    def cmd_setMotorCalibration(self, motor: int, value: int):
        assert 0 <= motor <= NUM_MOTORS - 1, f"There are only {NUM_MOTORS} motors"
        assert 0 <= value <= 4000, "Calibration range is [0, 4000] where 1000 is normal"
        data = bs(value // 0xFF) + bs(value % 0xFF) + BYTESTRING_ZERO * 2
        self.__send_packet(COMMAND_SET_MOTOR_CALIBRATION, motor, data)

    def cmd_getIMU(self, device: int):
        assert device == PARAM_ACCEL or device == PARAM_GYRO \
               or device == PARAM_LINEAR_ACCEL or device == PARAM_ORIENTATION, "invalid device!"
        self.__send_packet(COMMAND_GET_IMU, device, BYTESTRING_ZERO * 4)

    def cmd_setAccelSettings(self, range: int, divisor: int, use_deprecated=False):
        if not use_deprecated:
            print("cmd_setAccelSettings is deprecated as of $VERSION=2!")
            return
        assert 0 <= range <= 3, "invalid range!"
        assert 1 <= divisor <= 0xFF, "invalid divisor!"
        data = bs(range) + bs(divisor) + BYTESTRING_ZERO * 2
        self.__send_packet(COMMAND_SET_ACCEL_SETTINGS, PARAM_ACCEL, data)

    def cmd_setGyroSettings(self, range: int, divisor: int, use_deprecated=False):
        if not use_deprecated:
            print("cmd_setGyroSettings is deprecated as of $VERSION=2!")
            return
        assert 0 <= range <= 3, "invalid range!"
        assert 1 <= divisor <= 0xFF, "invalid divisor!"
        data = bs(range) + bs(divisor) + BYTESTRING_ZERO * 2
        self.__send_packet(COMMAND_SET_GYRO_SETTINGS, PARAM_GYRO, data)

    def cmd_getVoltageAndTemperature(self):
        self.__send_packet(COMMAND_GET_VOLT_TEMP, PARAM_VOLT_TEMP, BYTESTRING_ZERO * 4)

    def cmd_setVoltageCalibration(self, calibration: float):
        self.__send_packet(COMMAND_SET_VOLTAGE_CALIBRATION, PARAM_VOLT_TEMP, struct.pack('f', calibration))

    def cmd_setAutoReport(self, device, enabled: bool, delay: int):
        assert device == PARAM_ACCEL or device == PARAM_GYRO or device == PARAM_VOLT_TEMP \
               or device == PARAM_ORIENTATION or device == PARAM_LINEAR_ACCEL, "invalid device!"
        assert 0 <= delay <= 0xFFFF, "invalid delay!"
        on = 0xFF if enabled else 0x00
        data = bs(on) + bs(delay // 0xFF) + bs(delay % 0xFF) + BYTESTRING_ZERO
        self.__send_packet(COMMAND_SET_AUTOREPORT, device, data)

    def cmd_setFeedback(self, enabled: bool):
        on = 0xFF if enabled else 0x00
        data = bs(on) + BYTESTRING_ZERO * 3
        self.__send_packet(COMMAND_SET_FEEDBACK, PARAM_SYSTEM, data)

    def cmd_getIMUSettings(self):
        self.__send_packet(COMMAND_GET_IMU_SETTINGS, PARAM_ZERO, BYTESTRING_ZERO * 4)


if __name__ == "__main__":
    # runs a simple test to verify that communication is working.
    mcu = MCUInterface(input("Port? \n"), int(input("Baudrate? \n")))
    mcu.open_serial()
    print("Sending cmd_test in 0.5 seconds:")
    time.sleep(0.5)
    mcu.cmd_test()
    print("Sent! Waiting 0.5 seconds for response...")
    time.sleep(0.5)
    print("If we received a packet, it would be here:")
    print(mcu.test_queue.get())
    mcu.close_serial()
