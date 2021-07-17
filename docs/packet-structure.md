# Packet Structure
We have bidirectional data transfer over two wires, from TX (control) to RX (robot), and TX(robot) to RX(control). We refer to these as forward and return data transfer respectively. 

### Forward Data Transfer (commands)
Forward data transfer is used when it is necessary to tell the microcontroller to do something. For example, this could be setting the power or calibration of a motor, or configuring the sensitivity of the accelerometer.

A packet consists of 8 bytes, 0x0 to 0x7.
| byte | 0 | 1 | 2 | 3-6 | 7 |
|--|--|--|--|--|--|
| value | header | cmd | param | data field | footer | 

#### Header (byte 0)
The header byte's value will always be 0xCA, or 0b11001010. This is to ensure integrity of packets by specifying a binary signature that is hard to replicate through random noise. 

#### Command (byte 1)
The command is one byte that specifies what function to call on the microcontroller. Refer to the [command list](command-list.md).

#### Param (byte 2)
As all commands should have at least one parameter, and to ensure that the correct command has been sent when a parameter is not needed, the second byte is reserved for a primary parameter. This parameter is generally used to specify hardware device.

#### Data Field (bytes 3, 4, 5, 6)
Depending on the way the command is configured, the data field may or may not be used, and can represent between 1 and 4 values (1 32-bit integer/float or 4 bytes). Refer to the [command list](command-list.md).

#### Footer (byte 7)
The footer byte's value will always be 0x47, or 0b01000111. Similar to the header, this is to ensure packet integrity.

### Return Data Transfer (feedback)
Return data transfer is used by the microcontroller to return success/failure messages when a command is given, or send data when it is requested or when automatic reporting is enabled.

A packet consists of 10 bytes, 0x0 to 0x9.
| byte | 0 | 1 | 2 | 3 | 4 | 5-8 | 9 | 
|--|--|--|--|--|--|--|--|
| value | header | original cmd | original param | command | param | data field | footer |

#### Header (byte 0)
The header byte's value will always be 0xAC, or 0b10101100. This is to ensure integrity of packets by specifying a binary signature that is hard to replicate through random noise. 

#### Original Command and Param (byte 1, 2)
The command and parameter of the command sent previously is returned to the control system. If this was a packet that was automatically sent, the two bytes instead refer to the command that enabled automatic reporting.

#### Command (byte 3)
The command is one byte that refers to how to handle the data provided. Refer to the [command list](command-list.md).

#### Param (byte 4)
This byte is typically 0x00, as most return packets do not need to specify a parameter. However, when it is used, it refers to the hardware component returning this packet. 

#### Data Field (bytes 5,6,7,8)
This byte contains all the data returned by the packet, up to 10 bytes per packet. Refer to the [command list](command-list.md).

#### Footer (byte 9)
The footer byte's value will always be 0x74, or 0b01110100. Similar to the header, this is to ensure packet integrity.
