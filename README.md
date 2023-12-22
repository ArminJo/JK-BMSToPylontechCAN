<div align = center>

# [Arduino JK-BMS To Pylontech CAN Protocol Converter](https://github.com/ArminJo/JK-BMSToPylontechCAN)

Converts the JK-BMS RS485 data to Pylontech CAN data for inverters which are not compatible with JK-BMS protocol but with Pylontech protocol, like Deye inverters.<br/>
Display of many BMS information and alarms on a locally attached serial 2004 LCD.<br/>

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
The JK-BMS RS485 data (e.g. at connector GPS) are provided as RS232 TTL with 115200 Bit/s.

</div>

#### If you find this program useful, please give it a star.

&#x1F30E; [Google Translate](https://translate.google.com/translate?sl=en&u=https://github.com/ArminJo/JK-BMSToPylontechCAN)

<br/>

# Features
- Protocol converter from the JK-BMS status frame to Pylontech CAN frames.
- Supports sending of total capayity for SMA and Luxpower inverters.
- Optional linear **reducing maximum current above 80% SOC** (values can be adapted to your needs).
- Display of BMS information, Cell voltages, statistics and alarms on a locally attached **serial 2004 LCD**.
- Page button for switching **5 LCD display pages**.
- Debug output and extra **CAN info** and **Capacity info page** on long press of button.
- Statistics of minimum and maximum cells during balancing to **identify conspicuous cells**.
- Switch off LCD backlight after timeout (can be disabled).
- Beep on alarm and connection timeouts with selectable timeout.
- Serial.print() function is still available for monitoring and debugging.

**On a MCP2515 / TJA1050 kit for Arduino you must [replace the assembled 8 MHz crystal with a 16 MHz one](https://www.mittns.de/thread/1340-mcp2515-8mhz-auf-16mhz-upgrade/).**

**The MCP2515 / TJA1050 kit for Arduino must be supplied by an extra 5 V regulator, because the Arduino-Nano internal regulator cannot provide more than 100 mA and got defect on my site after a few days.**

**By default, the program sends a request to force charge the battery if SOC is below 5 %. This can be adapted by changing the line `#define SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I 5`.**

At around 100% SOC, the JK-BMS seems to send strange current information of more than +/- 1 ampere.<br/>
If CAN communications breaks, the inverter may use different values for controlling the battery (e.g. "Use Batt V"), which may lead to additional discharging / charging.

<br/>

![Overview](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BreadboardAndOverviewPage.jpg)

<br/>

# Screenshots
 The screenshots are taken from the [Wokwi example](https://wokwi.com/projects/371657348012321793) with `STANDALONE_TEST` enabled and therefore may contain random data.

| Big Info page with:<br/>- SOC and Power<br/>- Maximum of 3 Temperatures and Ampere in/out<br/>- Difference between maximum and current battery voltage - Volt to full<br/>- Display of  "C"harging "D"ischarging and "B"alancing active flags | Overview page |
| :-: | :-: |
| ![Big info page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BigInfoPage.png) | ![Overview page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/OverviewPage.png) |
| Cell info page with maximum and minimum indicators |  Overview / Error page with start of error message in first line |
| ![Cell info page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CellInfoPage.png) | ![Error page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/ErrorPage.png) |
| Maximum Cell Statistics page with total Time of Balancing | Maximum Cell Statistics page |
| ![Max page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/StatisticsMaxPage.png) | ![Min page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/StatisticsMinPage.png)  |
| CAN info page |  |
| ![CAN info page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CANInfoPage.png) |  |
| Capacity Statistics page percentages | Capacity Statistics page voltages |
| ![Percentage page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CapacityStatisticsPercentagePage.png) | ![Voltage page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CapacityStatisticsVoltagePage.png)  |


<br/>

# Pictures

| My installation | Automatic brightness |
| :-: | :-: |
| ![My installation](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CompleteInstallation.jpg) | ![Automatic brightness](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/AutomaticBrightness.jpg) |
| Breadboard detail |  |
| ![Breadboard detail](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BreadboardDetail.jpg) |  |
| Breadboard without BMS connected | Standalone test |
| ![Breadboard detail](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BreadboardSimple.jpg) | ![Standalone test](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/StandaloneTest.jpg) |
| No-breadboard version overview | No-breadboard version overview |
| ![Overview no breadboard](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NoBreadboardOverview1.jpg) | ![Overview no breadboard](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NoBreadboardOverview2.jpg) |
| Nano top view | Nano bottom view |
| ![Nano top view](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NanoTop.jpg) | ![Nano bottom view](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/NanoBottom.jpg) |

<br/>


# Example on Wokwi
Also usable as connection schematic.

- [Wokwi JK-BMSToPylontechCAN example](https://wokwi.com/projects/371657348012321793).

 <br/>

# Connection schematic
The standard **RX** of the Arduino is used for the JK_BMS connection.<br/>
A **schottky diode** is inserted into the RX line to allow programming the AVR with the JK-BMS still connected and switched on.
Keep in mind that programming will fail if JK-BMS is connected and switched off.<br/>
The standard **TX** of the Arduino is used for Serial.print() for monitoring and debugging. The short **request to JK-BMS is sent by `SoftwareSerialTX` using pin 4**.<br/>
If you use the cable from the separate RS485 adapter of the JK-BMS and follow the labeling on the board, you have to **swap the lines for RX and TX (pin 4)** on the Uno / Nano.

**Power** is taken from an **USB power supply** connected to the Nano. Current is 40 mA for a dark 2004 display and 55 mA for a bright one.<br/>
Optionally, power can be taken from the second battery, but then you may require an external 5V regulator. My built in regulator broke 2 times (with LCD connected).<br/>
Or use the complete battery voltage for Nano supply and a buck converters for around 48 to 60 volt. But my test converter had an idle current of 6 mA (0.3 W), so I decided to stay with the USB power supply.


On the Deye, connect cable before setting `Battery Mode` to `Lithium`, to avoid alarm. `Lithium Mode` for Pylontech CAN is `0` or `PYLON`.

```
 ALTERNATIVE EXTERNAL POWER SUPPLY:
                                          78L05    Optional Schottky diode - From Uno/Nano 5 V
                                           ___                         to enable powering CAN
 Optional 6.6 V from Battery #2 >-o-------|___|-------o-|<|-< Uno 5V   module by Nano USB, if
                                  |         |         |                battery is not attached
                                  |        ---        |
 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
                                  |                   |
  __________ Schottky diode   ____|____           ____|____             _________
 |        TX|>---|<|-- RX -->|RX Vin   |<- SPI ->|   5V    |           |         |
 |        RX|<-------- TX --<|4  Uno/  |         | MCP2515 |           |         |
 |  JK-BMS  |                |   Nano  |         |   CAN   |<-- CAN -->|  DEYE   |
 |          |<------- GND -->|         |<- GND ->|         |           |         |
 |__________|                |_________|         |_________|           |_________|



 # Connection diagram for JK-BMS GPS / UART-TTL socket (4 Pin, JST 1.25mm pitch)
  ___ ________ ___
 |                |
 | O   O   O   O  |
 |GND  RX  TX VBAT|
 |________________|
   |   |   |
   |   |   --|<|-- RX of Uno / Nano
   |   ----------- D4 (or other pin, if specified)
   --------------- GND


  # Automatic brightness control for 2004 LCD
   5V O------o------o
             |      |
            .-.     |
            | |     |
        LDR | |     |
            '-'     |
             |    |/
             o----|
                  |>
                    |
                    O To anode of LCD backlight

```


[EasyEda shematics by Ngoc Dang Dinh](https://easyeda.com/editor#id=0d1a2556b7634c8bbd22e9c0474cd401|8c434b5057fc4d769d664a0dc9c7d66c)
![EasyEda shematics by Ngoc Dang Dinh](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/EasyEda_shematics_by_Ngoc_Dang_Dinh.png)


### Board pinout diagrams
- [Nano](https://store.arduino.cc/products/arduino-nano#docs)
- [Uno](https://store.arduino.cc/products/arduino-uno-rev3#docs)
 <br/>

# Principle of operation
1. A request to deliver all informations is sent to the BMS (1.85 ms).
2. Wait and receive the BMS status frame (wait for 0.18 to 1 ms + receive 25.5 ms).
3. The BMS status frame is stored in a buffer and parity and other plausi checks are made.
4. The cell data are converted and enhanced to fill the JKConvertedCellInfoStruct.
   Other frame data are mapped to a C structure.
   But all words and longs in this structure are filled with big endian and thus cannot be read directly but must be swapped on reading.
5. Other frame data are converted and enhanced to fill the JKComputedDataStruct.
6. The content of the status frame is printed. After reset, all info is printed once, then only dynamic info is printed.
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
| `MILLISECONDS_BETWEEN_CAN_FRAME_SEND` | 2000 | % |
| `NO_BEEP_ON_ERROR` | disabled | . |
| `ONE_BEEP_ON_ERROR` | disabled | If activated, only beep once if error was detected. |
| `BEEP_TIMEOUT_SECONDS` | 60 | 1 minute, every 2 seconds. |
| `MULTIPLE_BEEPS_WITH_TIMEOUT` | enabled | If error was detected, beep for 60 s. |
| `SUPPRESS_LIFEPO4_PLAUSI_WARNING` | disabled | Disables warning on Serial out about using LiFePO4 beyond 3.0 v to 3.45 V. |
| `MAXIMUM_NUMBER_OF_CELLS` | 24 | Maximum number of cell info which can be converted. Saves RAM. |
| `USE_NO_LCD` | disabled | If activated, the code for the LCD display is deactivated. Saves 25% program space on a Nano. |
| `DISPLAY_ALWAYS_ON` | disabled | If activated, the display backlight is always on. This disables the value of `DISPLAY_ON_TIME_SECONDS`. |
| `DISPLAY_ON_TIME_SECONDS` | 300 | 300 s / 5 min after the last button press, the backlight of the LCD display is switched off. |
| `DISPLAY_ON_TIME_SECONDS_IF_TIMEOUT` | 180 | 180 s / 3 min after the first timeout / BMS shutdown, the backlight of the LCD display is switched off. |
| `USE_NO_COMMUNICATION_STATUS_LEDS` | disabled | If activated, the code for the BMS and CAN communication status LED is deactivated and the pins are not switched to output. |
| `STANDALONE_TEST` | disabled | If activated, fixed BMS data is sent to CAN bus and displayed on LCD. |
| `NO_SMA_EXTENSIONS` | disabled | If activated, supress sending of SMA extension frame over CAN. |
| `NO_LUXPOWER_EXTENSIONS` | disabled | If activated, supress sending of Luxpower extension frame over CAN. |
| `SHOW_SHORT_CELL_VOLTAGES` | disabled | If activated, print 3 digits cell voltage (value - 3.0 V) on Cell Info page. Enables display of up to 20 voltages or display of additional information on this page. |
| `DISABLE_MONITORING` | disabled | If activated, no cell and current values CSV data are written to serial output 
. |
| | | |
| `CAN_DATA_MODIFICATION` | disabled | If activated, it currently enables the function to reduce max current at high SOC level. |
| `MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT` | 80 | Start SOC for linear reducing maximum current. |
| `MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE` | 50 | Value of current at 100 % SOC. Units are 100 mA! |

There may be some some more options like `BUTTON_DEBOUNCING_MILLIS`, which are only for very special requirements.

The CSV data have the caption:
`Cell_1;Cell_2;Cell_3;Cell_4;Cell_5;Cell_6;Cell_7;Cell_8;Cell_9;Cell_10;Cell_11;Cell_12;Cell_13;Cell_14;Cell_15;Cell_16;Voltage,Current;SOC;Balancing`
Example data: `CSV: 270;262;271;271;271;269;270;271;266;266;266;266;265;262;265;265;5228;-22.56;71;1`
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
- Schottky diode e.g. BAT 42, BAT 43, BAT 85.
- Arduino Nano.
- 16 MHz crystal.
- MCP2515 / TJA1050 kit for Arduino. !!! You must replace the assembled 8 MHz crystal with a 16 MHz one !!!

### Optional
- 2004 LCD with serial I2C interface adapter.
- 2 pin female header for automatic LCD brightness control.
- LDR for automatic LCD brightness control.
- BC 549 or any NPN type for automatic LCD brightness control. The effect varies, depending on the LDR and the hFE of the transistor.

<br/>

### Links:
- https://www.kvaser.com/support/calculators/bit-timing-calculator/
- https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication

<br/>

# Tested Inverter
- SUN-5K-SG05LP1-EU
- Growatt SPH6000

# Revision History
### Version 2.4.0
- Added optional user function to reduce max current at high SOC level.
- Optional 3 digits cell voltage on LCD.
- Support for communication status LED.
- Internal capacity computing.

### Version 2.3.0
- Added frame 0x35F for total capacity as SMA extension, which is no problem for Deye inverters.
- Added frame 0x379 for total capacity as Luxpower extension, which is no problem for Deye inverters.

### Version 2.2.0
- Fixed LCD backlight timeout bug introduced in 1.3.0.
- Long press of page button instead of debug button.
- Do not use sleep any more by default.

### Version 2.1.1
- Fixed SOH == 0 bug.

### Version 2.1
- New compile option / macro `USE_NO_LCD`.
- GUI and print improvements.

### Version 2.0
- New page for minimum and maximum cell statistics and balancing time.
- Show difference between actual and 100% voltage.

### Version 1.3
- First release version.

### Version 0.9
- Initial version.

# Sample Serial output
See also [here](https://github.com/ArminJo/JK-BMSToPylontechCAN/tree/main/extras).

```
START ../src/JK-BMSToPylontechCAN.cpp
Version 2.0 from Sep  8 2023
Serial to JK-BMS started with 115200 bit/s!
CAN started with 500 kbit/s!
If you connect debug pin 3 to ground, additional debug data is printed
2000 ms between 2 BMS requests
2000 ms between 2 CAN transmissions

*** BMS INFO ***
Protocol Version Number=1
Software Version Number=11.XW_S11.26___
Modify Parameter Password=1234
# External Temperature Sensors=2

*** BATTERY INFO ***
Manufacturer Date=2307
Manufacturer Id=Armins__BMS1JK_B2A20S20P
Device ID String=Armins__
Device Address=1
Total Battery Capacity[Ah]=115, Low Capacity Alarm Percent=20
Charging Cycles=1
Total Charging Cycle Capacity=218
# Battery Cells=16, Cell Count=16

*** VOLTAGE PROTECTION INFO ***
Battery Overvoltage Protection[mV]=55200, Undervoltage=48000
Cell Overvoltage Protection[mV]=3450, Recovery=3400, Delay[s]=5
Cell Undervoltage Protection[mV]=3000, Recovery=3050, Delay[s]=5
Cell Voltage Difference Protection[mV]=300
Discharging Overcurrent Protection[A]=80, Delay[s]=30
Charging Overcurrent Protection[A]=43, Delay[s]=30

*** TEMPERATURE PROTECTION INFO ***
Power MosFet Temperature Protection=80, Recovery=70
Sensor1 Temperature Protection=100, Recovery=100
Sensor1 to Sensor2 Temperature Difference Protection=20
Charge Overtemperature Protection=45, Discharge=45
Charge Undertemperature Protection=3, Recovery=5
Discharge Undertemperature Protection=-20, Recovery=-10

*** MISC INFO ***
Balance Starting Cell Voltage=[mV]3350
Balance Triggering Voltage Difference[mV]=10

Current Calibration[mA]=1048
Sleep Wait Time[s]=10

Dedicated Charge Switch Active=0
Start Current Calibration State=0
Battery Actual Capacity[Ah]=115

Total Runtime Minutes=49260 ->   34D05H00M
*** CELL INFO ***
16 Cells, Minimum=3397 mV, Maximum=3408mV, Delta=11 mV, Average=3403 mV
 1=3397 mV,  2=3399 mV,  3=3400 mV,  4=3403 mV,  5=3406 mV,  6=3407 mV,  7=3407 mV,  8=3408 mV,
 9=3400 mV,  10=3406 mV, 11=3407 mV, 12=3404 mV, 13=3404 mV, 14=3403 mV, 15=3401 mV, 16=3402 mV,

Temperature: Power MosFet=28, Sensor 1=25, Sensor 2=25
SOC[%]=99 -> Remaining Capacity[Ah]=113
Battery Voltage[V]=54.41, Current[A]=-1.25, Power[W]=-68
Battery Voltage difference to full[V]=0.8
Charging MosFet enabled, active | Discharging MosFet enabled, active | Balancing enabled, not active
Battery Voltage[V]=54.41, Current[A]=1.25, Power[W]=68
Battery Voltage difference to full[V]=0.8
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
