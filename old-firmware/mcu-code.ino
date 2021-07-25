/* microcontroller-side code for MATE R&D OQBot
** refer to docs/packet-structure and docs/command-list for more details
** status:
** all commands are implemented but untested.
 */


// --------------- SECTION: DEFINITIONS --------------- //

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Servo.h>

#define VERSION 2

// define debug
 #define Debug_Serial Serial
 #define DEBUG_BAUDRATE 230400

// enable debug
 #define DEBUG
 #define Debug(a) (Serial.print(a))
 #define Debugln(a) (Serial.println(a))
 #define VERBOSE

// disable debug
//#define Debug(a)
//#define Debugln(a)

// define communications
#define Comms Serial1
#define COMMS_BAUDRATE 230400

// define motor details
#define PWM_MIN 1000
#define PWM_MID 1500
#define PWM_MAX 2000
#define CLAW_MIN 1010
#define CLAW_MAX 1660
#define NUM_MOTORS 5
byte pins[] = {A4, A5, 9, 10, 11};
uint16_t calibration[] = {1000, 1000, 1000, 1000, 1000};
Servo motors[NUM_MOTORS];

// define voltage calibrations
#define VOLTAGE_PIN A2
#define R1 2000
#define R2 390
float voltage_calibration = 6.5;

// create sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t bnoLinAccel;
sensors_event_t bnoOrientation;
sensors_event_t bnoAccel;
sensors_event_t bnoGyro;
float temperature;

// values for settings
boolean enableFeedback = true;
boolean enableAutoReport[] = {false, false, false, false, false};
uint16_t autoReportDelay[] = {100, 100, 100, 100, 100};
unsigned long autoReportTimers[] = {0, 0, 0, 0, 0};

// values for status
int8_t motorPercents[] = {0, 0, 0, 0, 0};
uint16_t motorMicros[] = {PWM_MID, PWM_MID, PWM_MID, PWM_MID, PWM_MID};
uint16_t motorTargets[] = {PWM_MID, PWM_MID, PWM_MID, PWM_MID, PWM_MID};
unsigned long prevTime = millis();
#define INTERPOLATION_SPEED 500 // measured in delta-microseconds-motor per millisecond


// --------------- SECTION: INITIALIZATION --------------- //

void initSerial() {
  #ifdef DEBUG
  Debug_Serial.begin(DEBUG_BAUDRATE);
  #endif
  Comms.begin(COMMS_BAUDRATE);
  delay(200);
  Debugln("initSerial: Serial Init complete");
}

void initSensors() {
  if (!bno.begin()){
    Debugln("initSensors: BNO055 Init over I2C failed.");
    while(true){
      delay(10);
    }
  }
  Debugln("initSensors: BNO055 Init complete");
}

void initMotors() {
  for (int i = 0; i < NUM_MOTORS; i++){
    motors[i].attach(pins[i]); 
    Debug("initMotors: motor ");
    Debug(i);
    Debug(" attached to pin ");
    Debugln(pins[i]);
  }
  Debugln("initMotors: Motor Init complete");
}

void setup() {
  digitalWrite(8, HIGH);
  initSerial();
  initSensors();
  initMotors();
}

// --------------- SECTION: HELPER FUNCTIONS --------------- //

float getVoltage(int rawInput){
  return (rawInput / 1024.0) * (R1 + R2) / R2 * voltage_calibration;
}

int16_t percentToMicroseconds(int8_t percent){
  if (percent < -100 || percent > 100) { return PWM_MID; }
  if (percent >= 0){
    return (int16_t) PWM_MID + (int16_t) (percent * (PWM_MAX - PWM_MID) / 100);
  } else {
    return (int16_t) PWM_MID + (int16_t) (percent * (PWM_MID - PWM_MIN) / 100);
  }
}

uint16_t calibrate(byte motor, uint16_t microseconds){
  if (microseconds >= PWM_MID){
    return PWM_MID + min((microseconds - PWM_MID) * calibration[motor] / 1000, PWM_MAX - PWM_MID);
  } else {
    return PWM_MID - min((PWM_MID - microseconds) * calibration[motor] / 1000, PWM_MID - PWM_MIN);
  }
}

void setPWM(byte motor, uint16_t microseconds){
  Debug("setPWM: specified motor now at ");
  Debug(microseconds);
  // motors[motor].writeMicroseconds(microseconds);
  motorTargets[motor] = microseconds;
  if (motor == 4) { // handle servo case
    motorMicros[motor] = microseconds;
    motors[motor].writeMicroseconds(microseconds);
  }
}

void setPercent(byte motor, int8_t percent){
  setPWM(motor, calibrate(motor, percentToMicroseconds(percent)));
  motorPercents[motor] = percent;
}

void updateSensors(){
  Debugln("updateSensors: updating sensors");
  bno.getEvent(&bnoOrientation, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&bnoGyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&bnoLinAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&bnoAccel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
}

int8_t mapServo(){
  return (int8_t) map((long) motorMicros, CLAW_MIN, CLAW_MAX, -127, 128); 
}

uint16_t lerp(uint16_t a, uint16_t b, float x){
  return (uint16_t) (a + x * (float) (b - a));
}

// --------------- SECTION: SERIAL COMMUNICATION --------------- //

void readSerial(){
  if (Comms.available() >= 8){
    // read Comms until header is received or not enough bytes
    byte header = 0x00;
    while (Comms.available() > 7 && header != 0xCA){
      header = Comms.read();
    } 
    // verify header
    if (header != 0xCA) {
      digitalWrite(8, LOW);
      Debugln("readSerial: packet received, but header did not match.");
      #ifdef VERBOSE
      Debug("header: ");
      Debugln(header);
      #endif
      return; 
    } else {
      digitalWrite(8, HIGH);
    }
    // get the contents of the packet
    byte cmd = Comms.read(); // 0x1
    byte param = Comms.read(); // 0x2
    byte data[4];
    for (int i = 0; i < 4; i++){
      data[i] = Comms.read();
    }
    byte footer = Comms.read();
    // verify footer
    if (footer != 0x47){
      Debugln("readSerial: packet received, but footer did not match.");
      #ifdef VERBOSE
      Debug("packet: ");
      Debug(header);
      Debug(" ");
      Debug(cmd);
      Debug(" ");
      Debug(param);
      Debug(" ");
      Debug(data[0]);
      Debug(" ");
      Debug(data[1]);
      Debug(" ");
      Debug(data[2]);
      Debug(" ");
      Debug(data[3]);
      Debug(" ");
      Debugln(footer);
      #endif
      return;
    }
    // call parseSerial
    parseSerial(cmd, param, data);
  }
}

void parseSerial(byte cmd, byte param, byte *data){
  Debug("parseSerial: packet received, cmd: ");
  Debug(cmd);
  Debug(", param: ");
  Debugln(param);
  switch (cmd)
  {
    case 0x00:
      test(param);
      break;
    case 0x0F:
      halt(param);
      break;
    case 0x10:
      setMotorMicroseconds(param, data);
      break;
    case 0x12:
      setMotorCalibrated(param, data);
      break;
    case 0x13:
      setMotorCalibration(param, data);
      break;
    case 0x30:
      getIMU(param);
      break;
    case 0x33:
      setAccelSettings(param, data);
      break;
    case 0x34:
      setGyroSettings(param, data);
      break;
    case 0x35:
      getIMUSettings(param);
    case 0x40:
      getVoltageAndTemperature(param);
      break;
    case 0x43:
      setVoltageCalibration(param, data);
      break;
    case 0x50:
      setAutoReport(param, data);
      break;
    case 0x51:
      setFeedback(param, data);
      break;
  }
}

void sendPacket(byte ogcmd, byte ogparam, byte cmd, byte param, byte *data){
  Debug("sendPacket: sending packet with cmd ");
  Debug(cmd);
  Debug(" and param ");
  Debugln(param);
  Comms.write(0xAC);            // header
  Comms.write(ogcmd);           // original cmd
  Comms.write(ogparam);         // original param
  Comms.write(cmd);             // cmd
  Comms.write(param);           // param
  for (int i = 0; i < 4; i++){  
    Comms.write(data[i]);       // data field
  }
  Comms.write(0x74);            // footer
}

void ok(byte ogcmd, byte ogparam){
  byte emptyDataField[4];
  if (enableFeedback){
    sendPacket(ogcmd, ogparam, 0x0A, 0xFF, emptyDataField);
  }
}

void fail(byte ogcmd, byte ogparam){
  byte emptyDataField[4];
  if (enableFeedback){
    sendPacket(ogcmd, ogparam, 0x0A, 0x00, emptyDataField);
  }
}

// motor_status return: 0x1C
void sendMotorStatus(byte param){
  byte toSend[4];
  toSend[0] = motorPercents[0];
  toSend[1] = motorPercents[1];
  toSend[2] = motorPercents[2];
  toSend[3] = motorPercents[3];
  sendPacket(0x10, param, 0x1C, (uint8_t) mapServo(), toSend);
}

// test command: 0x00
void test(byte param){
  Debug("test: called with param ");
  Debugln(param);
  byte toSend[4];
  toSend[0] = VERSION;
  toSend[1] = 'p';
  toSend[2] = 'o';
  toSend[3] = 'g';
  // respond with 0x00 packet
  sendPacket(0x00, param, 0x00, 0xF0, toSend);
}

// halt command: 0x0F
void halt(byte param){
  Debugln("halt: HALTING!!!!!");
  setPWM(0, PWM_MID);
  setPWM(1, PWM_MID);
  setPWM(2, PWM_MID);
  setPWM(3, PWM_MID);
  setPWM(4, PWM_MID);
  ok(0x0F, param);
}

// setMotorMicroseconds command: 0x10
void setMotorMicroseconds(byte param, byte *data){
  Debug("setMotorMicroseconds: set motor ");
  Debug(param);
  Debug(" to ");
  uint16_t microseconds = data[0] * 0xFF + data[1];
  Debugln(microseconds);
  setPWM(param, microseconds);
  ok(0x10, param);
}

// setMotorCalibrated command: 0x12
void setMotorCalibrated(byte param, byte *data){
  Debug("setMotorCalibrated: set motor ");
  Debug(param);
  Debug(" to ");
  // data[0] should be int8_t... right?
  int8_t percent = (int8_t) data[0];
  Debugln(percent);
  setPercent(param, percent);
  ok(0x12, param);
  sendMotorStatus(param);
}

// setMotorCalibration command: 0x13
void setMotorCalibration(byte param, byte *data){
  Debug("setMotorCalibrated: set motor ");
  Debug(param);
  Debug(" to ");
  uint16_t cal = data[0] * 0xFF + data[1];
  Debugln(cal);
  calibration[param] = cal;
  ok(0x13, param);
}

// getIMU command: 0x30
void getIMU(byte param){
  updateSensors();
  Debug("getIMU: getting ");
  switch (param)
  {
    case 0x15:
    {
      // accel
      Debugln("accelerometer data");
      byte *x = (byte *) &bnoAccel.acceleration.x;
      byte *y = (byte *) &bnoAccel.acceleration.y;
      byte *z = (byte *) &bnoAccel.acceleration.z;
      sendPacket(0x30, 0x15, 0x3A, 0x00, x);
      sendPacket(0x30, 0x15, 0x3A, 0x30, y);
      sendPacket(0x30, 0x15, 0x3A, 0x60, z);
      break;
    }
    case 0x16:
    {
      // gyro
      Debugln("gyroscope data");
      byte *x = (byte *) &bnoGyro.gyro.x;
      byte *y = (byte *) &bnoGyro.gyro.y;
      byte *z = (byte *) &bnoGyro.gyro.z;
      sendPacket(0x30, 0x15, 0x3C, 0x00, x);
      sendPacket(0x30, 0x15, 0x3C, 0x30, y);
      sendPacket(0x30, 0x15, 0x3C, 0x60, z);
      break;
    }
    case 0x18:
    {
      // linear acceleration
      Debugln("linear acceleration data");
      byte *x = (byte *) &bnoLinAccel.acceleration.x;
      byte *y = (byte *) &bnoLinAccel.acceleration.y;
      byte *z = (byte *) &bnoLinAccel.acceleration.z;
      sendPacket(0x30, 0x15, 0x3B, 0x00, x);
      sendPacket(0x30, 0x15, 0x3B, 0x30, y);
      sendPacket(0x30, 0x15, 0x3B, 0x60, z);
      break;
    }
    case 0x19:
    {
      // orientation
      Debugln("orientation data");
      byte *x = (byte *) &bnoOrientation.orientation.x;
      byte *y = (byte *) &bnoOrientation.orientation.y;
      byte *z = (byte *) &bnoOrientation.orientation.z;
      sendPacket(0x30, 0x15, 0x3D, 0x00, x);
      sendPacket(0x30, 0x15, 0x3D, 0x30, y);
      sendPacket(0x30, 0x15, 0x3D, 0x60, z);
      break;
    }
    default:
    {
      Debugln("getIMU: invalid IMU device");
      break;
    }
  }
}

// setAccelSettings command: 0x33
void setAccelSettings (byte param, byte *data){
  if (param != 0x15) { return; }
  Debugln("setAccelSettings: Deprecated Command.");
  ok(0x33, param);
}

// setGyroSettings command: 0x34
void setGyroSettings (byte param, byte *data){
  if (param != 0x16) { return; }
  Debugln("setGyroSettings: Deprecated Command.");
  ok(0x34, param);
}

// getIMUSettings command: 0x35
void getIMUSettings (byte param){
  Debugln("getIMUSettings: Sending IMU calibration!");
  byte system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  byte toSend[4];
  toSend[0] = system;
  toSend[1] = gyro;
  toSend[2] = accel;
  toSend[3] = mag;
  sendPacket(0x35, param, 0x3E, 0x00, toSend);
}

// getVoltageAndTemperature command: 0x40
void getVoltageAndTemperature(byte param){
  if (param != 0x17) { return; }
  updateSensors();
  Debugln("getVoltageAndTemperature: getting voltage and temperature data");
  byte toSend[4];
  uint16_t temp = bno.getTemp() * 100;
  uint16_t voltage = (uint16_t) (getVoltage(analogRead(VOLTAGE_PIN)) * 100.0);
  toSend[0] = temp % 0xFF;
  toSend[1] = temp / 0xFF;
  toSend[2] = voltage % 0xFF;
  toSend[3] = voltage / 0xFF;
  sendPacket(0x40, param, 0x44, 0x17, toSend);
}

// setVoltageCalibration command: 0x43
void setVoltageCalibration(byte param, byte *data){
  if (param != 0x17) { return; }
  Debug("setVoltageCalibration: new calibration = ");
  voltage_calibration = *(float *) &data;
  Debug(voltage_calibration);
  ok(0x43, param);
}

// setAutoReport command: 0x50
void setAutoReport(byte param, byte *data){
  if (param < 0x15 || param > 0x19) { return; }
  byte device = param - 0x15;
  enableAutoReport[device] = data[0] > 0;
  autoReportDelay[device] = data[1] * 0xFF + data[2];
  Debug("setAutoReport: AutoReport for device ");
  Debug(device);
  Debug(" is now ");
  Debugln(enableAutoReport[device]);
}

// setFeedback command: 0x51
void setFeedback(byte param, byte *data){
  Debug("setFeedback: feedback is now ");
  if (param != 0x01){
    fail(0x51, param);
  }
  enableFeedback = data[0] > 0x0;
  Debugln(enableFeedback);
}

// --------------- SECTION: PROGRAM LOOP --------------- //

void autoReport(){
  // iterate through devices
  for (int dev = 0; dev < 5; dev ++){
    if (enableAutoReport[dev]){
      if (millis() > autoReportTimers[dev]){
        if (millis() - autoReportTimers[dev] > 2 * autoReportDelay[dev]){
          Debug("autoReport: OVERLOAD!!! Device ");
          Debug(dev);
          Debug(" is running ");
          Debug(millis() - autoReportTimers[dev]);
          Debugln("ms behind. note: ignore this if this is on startup");
        }
        autoReportTimers[dev] = millis() + autoReportDelay[dev];
        switch (dev)
        {
          case 0: // accelerometer
            getIMU(0x15);
            break;
          case 1: // gyroscope
            getIMU(0x16);
            break;
          case 2: // volt/temp
            getVoltageAndTemperature(0x17);
            break;
          case 3:
            getIMU(0x18);
            break;
          case 4:
            getIMU(0x19);
            break;
          default:
            Debugln("autoReport: What? This shouldn't be possible!");
            break;
        }
      } // end if timer active
    } // end if enabled
  } // end for
}

void lerpMotors(){
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - prevTime;
  prevTime = currentTime;
  for (int motor = 0; motor < 4; motor ++){
    if (motorMicros[motor] != motorTargets[motor]){
      float x = min(1, dt * INTERPOLATION_SPEED / abs(motorTargets[motor] - motorMicros[motor]));
      uint16_t new_value = lerp(motorMicros[motor], motorTargets[motor], x);
      motorMicros[motor] = new_value;
      motors[motor].writeMicroseconds(new_value);
    }
  }
}

void loop() {
  readSerial();
  autoReport();
  lerpMotors();
}
