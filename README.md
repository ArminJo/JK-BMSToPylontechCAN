<div align = center>

# [Arduino JK-BMS To Pylontech CAN Protocol Converter](https://github.com/ArminJo/JK-BMSToPylontechCAN)

Converts the JK-BMS RS485 data to Pylontech CAN data for inverters which are not compatible with JK-BMS protocol but with Pylontech protocol, like Deye inverters.<br/>
Display of many BMS information and alarms on a locally attached 2004 LCD.<br/>

[![Badge License: GPLv3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp;
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/JK-BMSToPylontechCAN?include_prereleases&color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ArminJo/JK-BMSToPylontechCAN/releases/latest)
 &nbsp; &nbsp;
[![Badge Commits since latest](https://img.shields.io/github/commits-since/ArminJo/JK-BMSToPylontechCAN/latest?color=yellow)](https://github.com/ArminJo/JK-BMSToPylontechCAN/commits/main)
 &nbsp; &nbsp;
[![Badge Build Status](https://github.com/ArminJo/JK-BMSToPylontechCAN/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/JK-BMSToPylontechCAN/actions)
 &nbsp; &nbsp;
![Badge Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_JK-BMSToPylontechCAN)
<br/>

Based on https://github.com/syssi/esphome-jk-bms and https://github.com/maxx-ukoo/jk-bms2pylontech.<br/>
The JK-BMS RS485 data (e.g. at connector GPS) are provided as RS232 TTL with 105200 Bit/s.

</div>

#### If you find this program useful, please give it a star.

<br/>

# Features
- Protocol converter.
- Display of BMS information and alarms on a locally attached serial 2004 LCD.
- Switch off LCD backlight after timeout.
- Beep on alarm and timeouts.

**!!! On a MCP2515 / TJA1050 kit for Arduino you must [replace the assembled 8 MHz crystal with a 16 MHz one](https://www.mittns.de/thread/1340-mcp2515-8mhz-auf-16mhz-upgrade/) !!!**

**!!! The MCP2515 / TJA1050 kit for Arduino must be supplied by an extra 5 V regulator, because the Arduino-Nano internal regulator cannot provide more than 100 mA and get defect on my site after a few days !!!**

<br/>

![Overview](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BreadboardAndOverviewPage.jpg)

<br/>

# Example on Wokwi
- [Wokwi JK-BMSToPylontechCAN example](https://wokwi.com/projects/371657348012321793).

 <br/>

 # Pictures and screenshots

 The screenshots are taken from the Wokwi example with `STANDALONE_TEST` enabled and therefore contain random data.

| Breadboard detail | Automatic brightness |
| :-: | :-: |
| ![Breadbaoard detail](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BreadbaoardDetail.jpg) | ![Automatic brightness](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/AutomaticBrightness.jpg) |
| Big Info Page | Cell Info Page with maximum and minimum indicators |
| ![Big Info Page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BigInfoPage.png) | ![Cell Info Page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CellInfoPage.png) |
| Overview Page | CAN Info Page |
| ![Overview Page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/OverviewPage.png) | ![CAN Info Page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CANInfoPage.png) |
| Error Page with start of error message in first line |  |
| ![Overview Page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/ErrorPage.png) |  |

## No breadboard version

| Overview | Overview |
| :-: | :-: |
| ![Overview](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NoBreadboardOverview1.jpg) | ![Overview](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NoBreadboardOverview2.jpg) |
| Nano top view | Nano bottom view |
| ![Nano top view](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NanoTop.jpg) | ![Nano bottom view](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NanoBottom.jpg) |

<br/>

# Connection schematic
The standard RX of the Arduino is used for the JK_BMS connection.<br/>
A schottky diode is inserted into the RX line to allow programming the AVR with the JK-BMS still connected.<br/>
If you use the cable from the separate RS485 adapter of the JK-BMS and follow the labeling on the board, you have to swap the lines for RX and TX on the nano.<br/>
On the Deye, connect cable before setting `Battery Mode` to `Lithium`, to avoid alarm. `Lithium Mode` for CAN is `00`.

```
                                           ___ 78L05
  Extern 6.6 V from Battery 2 >--o--------|___|-------o
                                 |          |         | 5V
  __________ Schottky diode  ____|____     ---    ____|____             _________
 |        TX|----|<|-- RX ->|RX Vin   |<-- SPI ->|   5V    |           |         |
 |        RX|<-------- TX --|4  Uno/  |          | MCP2515 |           |         |
 |  JK-BMS  |               |   Nano  |          |   CAN   |<-- CAN -->|  DEYE   |
 |          |<------- GND ->|         |<-- GND-->|         |           |         |
 |__________|               |_________|          |_________|           |_________|

 # JK-BMS UART-TTL socket (4 Pin, JST 1.25mm pitch)
  ___ ________ ___
 |                |
 | O   O   O   O  |
 |GND  RX  TX VBAT|
 |________________|
   |   |   |
   |   |   --|>|-- RX of Uno / Nano
   |   ----------- D4 (or other pin, if specified)
   --------------- GND

```
### Board connections:
- [Nano](https://store.arduino.cc/products/arduino-nano#docs)
- [Uno](https://store.arduino.cc/products/arduino-uno-rev3#docs)
 <br/>

# Principle of operation
1. A request to deliver all informations is sent to the BMS (1.85 ms).
2. Wait and receive the The BMS reply frame (0.18 to 1 ms + 25.5 ms).
3. The BMS reply frame is stored in a buffer and parity and other plausi checks are made.
4. The cell data are converted and enhanced to fill the JKConvertedCellInfoStruct.
   Other frame data are mapped to a C structure.
   But all words and longs in this structure are filled with big endian and thus cannot be read directly but must be swapped on reading.
5. Other frame data are converted and enhanced to fill the JKComputedDataStruct.
6. The content of the result frame is printed. After reset, all info is printed once, then only dynamic info is printed.
7. The required CAN data is filled in the according PylontechCANFrameInfoStruct.
8. Dynamic data and errors are displayed on the optional 2004 LCD if attached.
9. CAN data is sent.

<br/>

# Compile with the Arduino IDE
Download and extract the repository. In the Arduino IDE open the sketch with File -> Open... and select the JK-BMSToPylontechCAN folder.<br/>
All libraries, especially the modified ones, are included in this project.

# Compile options / macros for this software
To customize the software to different requirements, there are some compile options / macros available.<br/>
Modify them by enabling / disabling them, or change the values if applicable.

| Name | Default value | Description |
|-|-|-|
| `MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS` | 2000 | % |
| `DISPLAY_ALWAYS_ON` | disabled | If activated, the display backlight is always on. This disables the value of `DISPLAY_ON_TIME_SECONDS`. |
| `MILLISECONDS_BETWEEN_CAN_FRAME_SEND` | 2000 | % |
| `DISPLAY_ON_TIME_SECONDS` | 300 | 300 s / 5 min after the last button press, the backlight of the LCD display is switched off. |
| `DISPLAY_ON_TIME_SECONDS_IF_TIMEOUT` | 180 | 180 s / 3 min after the first timeout / BMS shutdown, the backlight of the LCD display is switched off. |
| `BEEP_ON_TIME_SECONDS_IF_TIMEOUT` | 60 | If timeout was detected, beep for 60 s. |
| `NO_MULTIPLE_BEEPS_ON_TIMEOUT` | disabled | If activated, only beep once if timeout was detected. |
| `SUPPRESS_LIFEPO4_PLAUSI_WARNING` | disabled | Disables warning on Serial out about using LiFePO4 beyond 3.0 v to 3.45 V. |
| `MAXIMUM_NUMBER_OF_CELLS` | 24 | Maximum number of cell info which can be converted. Saves RAM. |

There may be some some more options like `BUTTON_DEBOUNCING_MILLIS`, which are only for very special requirements.

<br/>

# Libraries used
This program uses the following libraries, which are already included in this repository:

- [SoftwareSerialTX](https://reference.arduino.cc/reference/en/libraries/liquidcrystal-i2c/) for sending Serial to JK-BMS.
- Modified [LiquidCrystal_I2C](https://reference.arduino.cc/reference/en/libraries/liquidcrystal-i2c/) for LCD connected by I2C.
- [SoftI2CMaster](https://github.com/felias-fogg/SoftI2CMaster) for LCD minimal I2C functions.
- [LCDBigNumbers](https://github.com/ArminJo/LCDBigNumbers) for LCD big number generation.
- [EasyButtonAtInt01](https://github.com/ArminJo/EasyButtonAtInt01) for LCD page switching button.
- Modified mcp_can_dfs.h file from Seed-Studio [Seeed_Arduino_CAN](https://github.com/Seeed-Studio/Seeed_Arduino_CAN).

 <br/>

# BOM
### Required
- Breadboard.
- Jumper wire.
- Pin header to connect cables to breadboard.
- Schottky diode e.g. BAT 43.
- Arduino Nano.
- 16 (or 20) MHz crystal.
- MCP2515 / TJA1050 kit for Arduino. !!! You must replace the assembled 8 MHz crystal with a 16 MHz (20 MHz) one !!!

### Optional
- 2004 LCD with serial I2C interface adapter.
- 2 pin female header for automatic LCD brightness control.
- LDR for automatic LCD brightness control.
- BC 549 or any NPN type for automatic LCD brightness control. The effect varies, depending on the LDR and the hFE of the transistor.

<br/>

### Links:
- https://www.kvaser.com/support/calculators/bit-timing-calculator/
- https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication


# Sample Serial output
See also [here](https://github.com/ArminJo/tree/main/extras).

```
START ../src/JK-BMSToPylontechCAN.cpp
Version 1.2 from Aug  7 2023
Serial to JK-BMS started with 115200 bit/s!
CAN started with 500 kbit/s!
If you connect debug pin 8 to ground, additional debug data is printed
2000 ms between 2 BMS requests
2000 ms between 2 CAN transmissions

*** BMS INFO ***
Protocol Version Number=1
Software Version Number=11.XW_S11.26___
Modify Parameter Password=123456
# External Temperature Sensors=2

*** BATTERY INFO ***
Manufacturer Date=2307
Manufacturer Id=Armins__BMS1_
Device ID String=Armins__
Device Address=1
Total Battery Capacity[Ah]=110, Low Capacity Alarm Percent=20
Charging Cycles=0
Total Charging Cycle Capacity=0
# Battery Cells=16, Cell Count=16

*** VOLTAGE PROTECTION INFO ***
Battery Overvoltage Protection[mV]=55200, Undervoltage=48000
Cell Overvoltage Protection[mV]=3450, Recovery=3400, Delay[s]=5
Cell Undervoltage Protection[mV]=3000, Recovery=3050, Delay[s]=5
Cell Voltage Difference Protection[mV]=300
Discharging Overcurrent Protection[A]=80, Delay[s]=30
Charging Overcurrent Protection[A]=50, Delay[s]=30

*** TEMPERATURE PROTECTION INFO ***
Power MosFet Temperature Protection=80, Recovery=70
Sensor1 Temperature Protection=100, Recovery=100
Sensor1 to Sensor2 Temperature Difference Protection=20
Charge Overtemperature Protection=60, Discharge=70
Charge Undertemperature Protection=5, Recovery=10
Discharge Undertemperature Protection=-20, Recovery=-10

*** MISC INFO ***
Balance Starting Cell Voltage=[mV]3200
Balance Triggering Voltage Difference[mV]=10

Current Calibration[mA]=1048
Sleep Wait Time[s]=10

Dedicated Charge Switch Active=0
Start Current Calibration State=0
Battery Actual Capacity[Ah]=110

Total Runtime Minutes=3716 ->    2D13H56M
*** CELL INFO ***
 1=3271 mV,  2=3263 mV,  3=3271 mV,  4=3270 mV,  5=3271 mV,  6=3270 mV,  7=3269 mV,  8=3271 mV,
 9=3266 mV, 10=3266 mV, 11=3266 mV, 12=3266 mV, 13=3265 mV, 14=3265 mV, 15=3265 mV, 16=3265 mV,
Minimum=3263 mV at cell #2, Maximum=3271 mV at cell #1 of 16
Delta=8 mV, Average=3267 mV

Temperature: Power MosFet=22, Sensor 1=21, Sensor 2=22
SOC[%]=60 -> Remaining Capacity[Ah]=66
Battery Voltage[V]=52.28, Current[A]=0.00, Power[W]=0
Charging MosFet enabled, active | Discharging MosFet enabled, active | Balancing enabled, not active
Set LCD display page to: 0
Set LCD display page to: 1
Set LCD display page to: 2
Debug mode just activated
Send CAN
CANId=0x351, FrameLength=8, Data=0x28, 0x2, 0xF4, 0x1, 0x20, 0x3, 0xE0, 0x1
CANId=0x355, FrameLength=4, Data=0x3C, 0x0, 0x64, 0x0
CANId=0x356, FrameLength=6, Data=0x6C, 0x14, 0x0, 0x0, 0xDC, 0x0
CANId=0x35E, FrameLength=8, Data=0x50, 0x59, 0x4C, 0x4F, 0x4E, 0x20, 0x20, 0x20
CANId=0x35C, FrameLength=2, Data=0xC0, 0x0
CANId=0x305, FrameLength=8, Data=0x21, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
CANId=0x359, FrameLength=7, Data=0x0, 0x0, 0x0, 0x0, 0x1, 0x50, 0x4E

Send requestFrame with TxToJKBMS
 0x4E 0x57 0x0 0x13 0x0 0x0 0x0 0x0 0x6 0x3 0x0 0x0 0x0 0x0 0x0 0x0 0x68 0x0 0x0 0x1 0x29
291 bytes received
0x00  0x4E 0x57 0x01 0x21 0x00 0x00 0x00 0x00 0x06 0x00 0x01
0x0B  0x79 0x30 0x01 0x0C 0xC6
0x10  0x02 0x0C 0xBE 0x03 0x0C 0xC7 0x04 0x0C 0xC7 0x05 0x0C 0xC7 0x06 0x0C 0xC5 0x07
0x20  0x0C 0xC6 0x08 0x0C 0xC7 0x09 0x0C 0xC2 0x0A 0x0C 0xC2 0x0B 0x0C 0xC2 0x0C 0x0C
0x30  0xC2 0x0D 0x0C 0xC1 0x0E 0x0C 0xC1 0x0F 0x0C 0xC1 0x10 0x0C 0xC1
0x3D  0x80 0x00 0x16
0x40  0x81 0x00 0x15 0x82 0x00 0x16 0x83 0x14 0x6C 0x84 0x00 0x00 0x85 0x3C 0x86 0x02
0x50  0x87 0x00 0x00 0x89 0x00 0x00 0x00 0x00 0x8A 0x00 0x10 0x8B 0x00 0x00 0x8C 0x00
0x60  0x03 0x8E 0x15 0x90 0x8F 0x12 0xC0 0x90 0x0D 0x7A 0x91 0x0D 0x48 0x92 0x00 0x05
0x70  0x93 0x0B 0xB8 0x94 0x0B 0xEA 0x95 0x00 0x05 0x96 0x01 0x2C 0x97 0x00 0x50 0x98
0x80  0x00 0x1E 0x99 0x00 0x32 0x9A 0x00 0x1E 0x9B 0x0C 0x80 0x9C 0x00 0x0A 0x9D 0x01
0x90  0x9E 0x00 0x50 0x9F 0x00 0x46 0xA0 0x00 0x64 0xA1 0x00 0x64 0xA2 0x00 0x14 0xA3
0xA0  0x00 0x3C 0xA4 0x00 0x46 0xA5 0x00 0x05 0xA6 0x00 0x0A 0xA7 0xFF 0xEC 0xA8 0xFF
0xB0  0xF6 0xA9 0x10 0xAA 0x00 0x00 0x00 0x6E 0xAB 0x01 0xAC 0x01 0xAD 0x04 0x18 0xAE
0xC0  0x01 0xAF 0x00 0xB0 0x00 0x0A 0xB1 0x14 0xB2 0x31 0x32 0x33 0x34 0x35 0x36 0x00
0xD0  0x00 0x00 0x00 0xB3 0x00 0xB4 0x41 0x72 0x6D 0x69 0x6E 0x73 0x5F 0x5F 0xB5 0x32
0xE0  0x33 0x30 0x37 0xB6 0x00 0x00 0x0E 0x85 0xB7 0x31 0x31 0x2E 0x58 0x57 0x5F 0x53
0xF0  0x31 0x31 0x2E 0x32 0x36 0x5F 0x5F 0x5F 0xB8 0x00 0xB9 0x00 0x00 0x00 0x6E 0xBA
0x100  0x41 0x72 0x6D 0x69 0x6E 0x73 0x5F 0x5F 0x42 0x4D 0x53 0x00 0x4A 0x4B 0x5F 0x42
0x110  0x32 0x41 0x32 0x30 0x53 0x32 0x30 0x50 0xC0 0x01
0x11A  0x00 0x00 0x00 0x00 0x68 0x00
0x120  0x00 0x51 0xCA

```
