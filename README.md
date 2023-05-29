<div align = center>

# [JK-BMS To Pylontech CAN Protocol Converter](https://github.com/ArminJo/JK-BMSToPylontechCAN)

Converts the JK-BMS RS485 data to Pylontech CAN data for inverters which are not compatible with JK-BMS protocol but with Pylontech protocol, like Deye inverters.<br/>
Display of many BMS information and alarms on a locally attached 2004 LCD.<br/>

[![Badge License: GPLv3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp; 
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/OpenledRace?include_prereleases&color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ArminJo/JK-BMSToPylontechCAN/releases/latest)
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
<br/>

# Features
- Protocol converter.
- Display of BMS information and alarms on a locally attached serial 2004 LCD.
- Beep on Alarm.

**On a MCP2515 / TJA1050 kit for Arduino you must [replace the assembled 8 MHz crystal with a 16 MHz one](https://www.mittns.de/thread/1340-mcp2515-8mhz-auf-16mhz-upgrade/)**
 
 <br/>

![Overview](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BreadboardAndOverviewPage.jpg)

 <br/>
 
| Breadboard detail | Automatic brightness |
| :-: | :-: |
| ![Breadbaoard detail](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BreadbaoardDetail.jpg) | ![Automatic brightness](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/AutomaticBrightness.jpg) |
| Big Info Page | Cell Info Page |
| ![Big Info Page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/BigInfoPage.jpg) | ![Cell Info Page](https://github.com/ArminJo/JK-BMSToPylontechCAN/blob/main/pictures/CellInfoPage.jpg) |

 <br/>

# Principle of operation
1. A request is sent to the BMS.
2. The BMS reply frame is stored in a buffer and parity and other plausi checks are made.
3. The cell data are converted and enhanced to fill the JKConvertedCellInfoStruct.
4. Other frame data are converted and enhanced to fill the JKComputedDataStruct.
5. The content of the result frame is printed. After reset, all info is printed once, then only dynamic info is printed.
6. The required CAN data is filled in the according PylontechCANFrameInfoStruct.
7. Dynamic data and errors are displayed on the optional 2004 LCD if attached.
8. CAN data is sent.

<br/>

# Compile with the Arduino IDE
Download and extract the repository. In the Arduino IDE open the sketch with File -> Open... and select the JK-BMSToPylontechCAN folder.<br/>
All libraries, especially the modified ones, are included in this project.

# Libraries used
This program uses the following libraries, which are included in this repository:

- [SoftwareSerialTX](https://reference.arduino.cc/reference/en/libraries/liquidcrystal-i2c/) for sending Serial to JK-BMS.
- Modified [LiquidCrystal_I2C]() for I2C connected LCD.
- [LCDBigNumbers](https://github.com/ArminJo/LCDBigNumbers) for LCD big number generation.
- [EasyButtonAtInt01](https://github.com/ArminJo/EasyButtonAtInt01) for LCD page switching button.
- [SoftI2CMaster](https://github.com/felias-fogg/SoftI2CMaster) for minimal I2C functions.
- Modified mcp_can_dfs.h file from Seed-Studio [Seeed_Arduino_CAN](https://github.com/Seeed-Studio/Seeed_Arduino_CAN).

 <br/>
 
# Disclaimer
Currently (1.6.2023) the program is tested only with a JK-BMS JK-B2A20S20P and a 10 cell LiIon battery.<br/>
It was not connected to a Deye inverter so far, since the target 16 cell LiFePo battery is stil on its way.

 <br/>

# BOM
### Required
- Breadboard.
- Jumper wire.
- Pin header to connect cables to breadboard.
- Shottky diode e.g. BAT 43.
- Arduino Nano.
- 16 (or 20) MHz crystal.
- MCP2515 / TJA1050 kit for Arduino. !!! You must replace the assembled 8 MHz crystal with a 16 MHz one !!!

### Optional
- 2004 LCD with serial I2C interface adapter.
- LDR for automatic LCD brightness control.
- BC 549C or any type with hFe > 250 for automatic LCD brightness control.

<br/>

### Links:
- https://www.kvaser.com/support/calculators/bit-timing-calculator/
- https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication

#### If you find this program useful, please give it a star.

# Sample Serial output
See also [here](https://github.com/ArminJo/tree/main/extras).

```
START ../src/JK-BMSToPylontechCAN.cpp
Version 1.0 from May 29 2023
Serial to JK-BMS started with 115200 bit/s!
CAN started with 500 kbit/s!
If you connect debug pin DEBUG_BUTTON_PIN to ground, additional debug data is printed

*** BMS INFO ***
Protocol Version Number=1
Software Version Number=11.XW_S11.26___
Modify Parameter Password=123456
# External Temperature Sensors=2

*** BATTERY INFO ***
Manufacturer Id=123456789012JK_B2A20S20P
Manufacturer Date=2305
Device ID String=12345678
Device Address=1
Total Battery Capacity[Ah]=30, Low Capacity Alarm Percent=20
Charging Cycles=0
Total Charging Cycle Capacity=0
# Battery Cells=10, Cell Count=10

*** VOLTAGE PROTECTION INFO ***
Battery Overvoltage Protection[mV]=42000, Undervoltage=28200
Cell Overvoltage Protection[mV]=4200, Recovery=4180, Delay[s]=5
Cell Undervoltage Protection[mV]=2820, Recovery=2850, Delay[s]=5
Cell Voltage Difference Protection[mV]=300
Discharging Overcurrent Protection[A]=1, Delay[s]=20
Charging Overcurrent Protection[A]=1, Delay[s]=30

*** TEMPERATURE PROTECTION INFO ***
Power MosFet Temperature Protection=55, Recovery=50
Sensor1 Temperature Protection=100, Recovery=100
Sensor1 to Sensor2 Temperature Difference Protection=20
Charge Overtemperature Protection=80, Discharge=70
Charge Undertemperature Protection=-5, Recovery=0
Discharge Undertemperature Protection=-20, Recovery=-10

*** MISC INFO ***
Balance Starting Cell Voltage=[mV]3000
Balance Opening Voltage Difference[mV]=10

Current Calibration[mA]=1039
Sleep Wait Time[s]=10

Dedicated Charge Switch Active=0
Start Current Calibration State=0
Battery Actual Capacity[Ah]=30

*** CELL INFO ***
 1=3967 mV,  2=3965 mV,  3=3965 mV,  4=3965 mV,  5=3965 mV,  6=3967 mV,  7=3968 mV,  8=3968 mV, 
 9=3969 mV, 10=3969 mV, 
Minimum=3965 mV at cell #2, Maximum=3969 mV at cell #9
Delta=4 mV, Average=3966 mV

*** DYNAMIC INFO ***
Total Runtime Minutes=1640 ->    1D 3H20M
Temperature: Power MosFet=26, Sensor 1=44, Sensor 2=57

SOC[%]=83 -> Remaining Capacity[Ah]=24
Battery Voltage[V]=39.66, Current[A]=0.00, Power[W]=0

Charging MosFet enabled, active | Discharging MosFet enabled, active | Balancing enabled, not active


*** CELL INFO ***
 1=3967 mV,  2=3965 mV,  3=3965 mV,  4=3965 mV,  5=3965 mV,  6=3967 mV,  7=3968 mV,  8=3967 mV, 
 9=3969 mV, 10=3968 mV, 
Minimum=3965 mV at cell #2, Maximum=3969 mV at cell #9
Delta=4 mV, Average=3966 mV

*** DYNAMIC INFO ***
Total Runtime Minutes=1641 ->    1D 3H21M
Temperature: Power MosFet=26, Sensor 1=44, Sensor 2=57

SOC[%]=83 -> Remaining Capacity[Ah]=24
Battery Voltage[V]=39.66, Current[A]=0.00, Power[W]=0

Charging MosFet enabled, active | Discharging MosFet enabled, active | Balancing enabled, not active

```
