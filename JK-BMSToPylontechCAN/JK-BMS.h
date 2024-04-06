/*
 * JK-BMS.h
 *
 * Definitions of the data structures used by JK-BMS and the converter
 *
 *  Copyright (C) 2023-2024  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ArduinoUtils https://github.com/ArminJo/JK-BMSToPylontechCAN.
 *
 *  JK-BMSToPylontechCAN is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 * The protocol at the display connector RS485 is 2400 baud 294 bytes in 1.2 seconds with a pause of 0.5 s.
 * It starts with A5 5A 5D 82 10 00 0F 7C-7E
 * at 0x16 there are 10 words 0F 7C-7F
 * At 0x32 there are 46 bytes 0x00 for error flags, (tested with sensor over temperature resulting in a 01 at 0x57)
 * At 0x60 there are 24 Values A5 5A 05 82 2 (0 0 to 1 7) 3 for 24 cell values
 * of 16 bit each (for the status display page) followed by the END token A5 5A 03 80 01 00
 *
 *
 */
#ifndef _JK_BMS_H
#define _JK_BMS_H

#include "SoftwareSerialTX.h"

#define JK_FRAME_START_BYTE_0   0x4E
#define JK_FRAME_START_BYTE_1   0x57
#define JK_FRAME_END_BYTE       0x68

void requestJK_BMSStatusFrame(SoftwareSerialTX *aSerial, bool aDebugModeActive = false);

void initJKReplyFrameBuffer();
void printJKReplyFrameBuffer();

#define JK_BMS_RECEIVE_OK           0
#define JK_BMS_RECEIVE_FINISHED     1
#define JK_BMS_RECEIVE_ERROR        2
uint8_t readJK_BMSStatusFrameByte();
void fillJKComputedData();

extern uint16_t sReplyFrameBufferIndex;            // Index of next byte to write to array, thus starting with 0.
extern uint8_t JKReplyFrameBuffer[350];            // The raw big endian data as received from JK BMS
extern struct JKReplyStruct *sJKFAllReplyPointer;
extern bool sJKBMSFrameHasTimeout; // For sending CAN data
extern bool sSwitchPageAndShowAlarmInfoOnce;
extern bool sAlarmJustGetsActive;

extern char sUpTimeString[12]; // "1000D23H12M" is 11 bytes long
extern bool sUpTimeStringMinuteHasChanged;

#if !defined(USE_NO_LCD)
extern const char *sAlarmStringForLCD;
#endif

int16_t getJKTemperature(uint16_t aJKRAWTemperature);
int16_t getCurrent(uint16_t aJKRAWCurrent);

uint8_t swap(uint8_t aByte);
uint16_t swap(uint16_t aWordToSwapBytes);
uint32_t swap(uint32_t aLongToSwapBytes);

void myPrintln(const __FlashStringHelper *aPGMString, uint8_t a8BitValue);
void myPrint(const __FlashStringHelper *aPGMString, uint8_t a8BitValue);
void myPrintln(const __FlashStringHelper *aPGMString, uint16_t a16BitValue);
void myPrint(const __FlashStringHelper *aPGMString, uint16_t a16BitValue);
void myPrintln(const __FlashStringHelper *aPGMString, int16_t a16BitValue);
void myPrint(const __FlashStringHelper *aPGMString, int16_t a16BitValue);
void myPrintlnSwap(const __FlashStringHelper *aPGMString, uint16_t a16BitValue);
void myPrintlnSwap(const __FlashStringHelper *aPGMString, int16_t a16BitValue);
void myPrintSwap(const __FlashStringHelper *aPGMString, int16_t a16BitValue);
void myPrintIntAsFloatSwap(const __FlashStringHelper *aPGMString, int16_t a16BitValue);
void myPrintlnSwap(const __FlashStringHelper *aPGMString, uint32_t a32BitValue);

void computeUpTimeString();
void printJKStaticInfo();
void printJKDynamicInfo();
void handleAndPrintAlarmInfo();
#if !defined(DISABLE_MONITORING)
void printMonitoringInfo();
#endif

struct JKCellInfoStruct {
    uint16_t CellMillivolt;
#if !defined(USE_NO_LCD)
    uint8_t VoltageIsMinMaxOrBetween; // One of VOLTAGE_IS_MINIMUM, VOLTAGE_IS_MAXIMUM or VOLTAGE_IS_BETWEEN_MINIMUM_AND_MAXIMUM
#endif
};

struct JKConvertedCellInfoStruct {
    uint8_t ActualNumberOfCellInfoEntries;
    JKCellInfoStruct CellInfoStructArray[MAXIMUM_NUMBER_OF_CELLS];
    uint16_t MinimumCellMillivolt;
    uint8_t IndexOfMinimumCellMillivolt;
    uint16_t MaximumCellMillivolt;
    uint8_t IndexOfMaximumCellMillivolt;
    uint16_t DeltaCellMillivolt;    // Difference between MinimumVoltagCell and MaximumVoltagCell
    uint16_t RoundedAverageCellMillivolt;
};
extern struct JKConvertedCellInfoStruct JKConvertedCellInfo;  // The converted little endian cell voltage data
void fillJKConvertedCellInfo();

#define VOLTAGE_IS_BETWEEN_MINIMUM_AND_MAXIMUM  0
#define VOLTAGE_IS_MINIMUM                      1
#define VOLTAGE_IS_MAXIMUM                      2

#if !defined(NO_CELL_STATISTICS)
/*
 * Arrays of counters, which count the times, a cell has minimal or maximal voltage
 * To identify runaway cells
 */
extern uint16_t CellMinimumArray[MAXIMUM_NUMBER_OF_CELLS];
extern uint16_t CellMaximumArray[MAXIMUM_NUMBER_OF_CELLS];
extern uint8_t CellMinimumPercentageArray[MAXIMUM_NUMBER_OF_CELLS];
extern uint8_t CellMaximumPercentageArray[MAXIMUM_NUMBER_OF_CELLS];
#define MINIMUM_BALANCING_COUNT_FOR_DISPLAY         60 //  120 seconds / 2 minutes of balancing
extern uint32_t sBalancingCount;            // Count of active balancing in SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS (2 seconds) units
#endif // NO_CELL_STATISTICS

#define JK_BMS_FRAME_HEADER_LENGTH              11
#define JK_BMS_FRAME_TRAILER_LENGTH             9
#define JK_BMS_FRAME_CELL_INFO_LENGTH           2 // 1 byte +1 for token 0x79
#define JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH  (JK_BMS_FRAME_HEADER_LENGTH + 1) // +1 for token 0x79
#define MINIMAL_JK_BMS_FRAME_LENGTH             19

/*
 * All 16 and 32 bit values are stored byte swapped, i.e. MSB is stored in lower address.
 * Must be read with swap()
 */
struct JKFrameHeaderStruct {
    uint16_t StartFrameToken;   // 0x4E57
    uint16_t LengthOfFrame;     // Excluding StartFrameToken
    uint32_t BMS_ID;            // Highest byte is default 00
    uint8_t Function;           // 0x01 (activation), 0x02 (write), 0x03 (read), 0x05 (password), 0x06 (read all)
    uint8_t FrameSource;        // 0=BMS, 1=Bluetooth, 2=GPRS, 3=PC
    uint8_t TransportType;      // 0=Request, 1=Response, 2=BMSActiveUpload
};

struct JKFrameTailStruct {
    uint32_t RecordNumber;      // High byte is random code, low 3 bytes is record number
    uint8_t EndToken;           // 0x68
    uint16_t UnusedChecksum;    // 0x0000
    uint16_t Checksum;          // Including StartFrameToken
};

/*
 * This structure contains all converted and computed data useful for display
 */
struct JKComputedDataStruct {
    int16_t TemperaturePowerMosFet;     // Degree Celsius
    int16_t TemperatureSensor1;
    int16_t TemperatureSensor2;
    int16_t TemperatureMaximum;         // Computed value

    uint16_t TotalCapacityAmpereHour;
    uint16_t RemainingCapacityAmpereHour; // Computed value
    uint16_t BatteryFullVoltage10Millivolt; // Computed by BMS! Is ActualNumberOfCellInfoEntries *  CellOvervoltageProtectionMillivolt
    uint16_t BatteryEmptyVoltage10Millivolt;
    uint16_t BatteryVoltage10Millivolt;
//    uint16_t BatteryVoltageDifferenceToFull10Millivolt; // saves 60 bytes program memory :-)
    uint16_t BatteryVoltageDifferenceToEmpty10Millivolt;
    float BatteryVoltageFloat;          // Volt
    int16_t Battery10MilliAmpere;       // Charging is positive discharging is negative
    float BatteryLoadCurrentFloat;      // Ampere
    int16_t BatteryLoadPower;           // Watt Computed value, Charging is positive discharging is negative
    bool BMSIsStarting;                 // True if SOC and Cycles are both 0, for around 16 seconds during JK-BMS startup.
};
extern struct JKComputedDataStruct JKComputedData;        // All derived converted and computed data useful for display

struct JKLastPrintedDataStruct {
    int16_t TemperaturePowerMosFet;     // Degree Celsius
    int16_t TemperatureSensor1;
    int16_t TemperatureSensor2;
    uint16_t RemainingCapacityAmpereHour; // Computed value
    float BatteryVoltageFloat;          // Volt
    int16_t BatteryLoadPower;           // Watt Computed value, Charging is positive discharging is negative
};
extern struct JKLastPrintedDataStruct JKLastPrintedData;

/*
 * Only for documentation
 */
union BMSStatusUnion {
    uint16_t StatusAsWord;
    struct {
        uint8_t ReservedStatusHighByte;         // This is the low byte of StatusAsWord, but it was sent as high byte of status
        bool ChargeMosFetActive :1;             // 0x01 // Is disabled e.g. on over current or temperature
        bool DischargeMosFetActive :1;          // 0x02 // Is disabled e.g. on over current or temperature
        bool BalancerActive :1;                 // 0x04
        bool BatteryDown :1;                    // 0x08
        uint8_t ReservedStatus :4;
    } StatusBits;
};

/*
 * Structure representing the semantic of the JK reply, except cell voltage info.
 * 221 bytes.
 *
 * All 16 and 32 bit values in this structure are filled with big endian by the JK protocol i.e. the higher byte is located at the lower memory address.
 * AVR and others are using little endian i.e. the lower byte is located at the lower memory address.
 * !!! => all 16 and 32 bit values in this structure must be "swapped" before interpreting them!!!
 *
 * All temperatures are in degree celsius.
 * Power MosFet temperature sensor is originally named PowerTube
 * Sensor1 temperature sensor is originally named Battery Box
 * Sensor2 temperature sensor is originally named Battery
 * Battery values are often originally named Total
 *
 * List of values seen in Bluetooth, but not in reply:
 * ChargeOvercurrentRecoverySeconds (Charge OCPR Time(S))
 * DischargeOvercurrentRecoverySeconds (Discharge OCPR Time(S))
 * ShortCircuitProtectionDelay  (SCP Delay(us))
 * ShortCircuitProtectionRecoverySeconds (SCPR Time(S))
 * CellPowerOffVoltage (Power Off Vol.(V))
 *
 * List of values in reply, but not in Bluetooth:
 * CellOvervoltageDelaySeconds (0x92)
 * CellUndervoltageDelaySeconds (0x95)
 */

#define NUMBER_OF_DEFINED_ALARM_BITS    14
struct JKReplyStruct {
    uint8_t TokenTemperaturePowerMosFet;    // 0x80
    uint16_t TemperaturePowerMosFet;        // 99 = 99 degree Celsius, 100 = 100, 101 = -1, 140 = -40
    uint8_t TokenTemperatureSensor1;        // 0x81 TemperatureSensor1
    uint16_t TemperatureSensor1;            // originally named Battery Box, the outmost sensor beneath the led
    uint8_t TokenTemperatureSensor2;        // 0x82
    uint16_t TemperatureSensor2;            // originally named Battery, the inner sensor beneath the battery+
    uint8_t TokenVoltage;                   // 0x83
    uint16_t Battery10Millivolt;
    uint8_t TokenCurrent;                   // 0x84
    // Current values start with 200mA at 240 mA real current -> 410 -> 620 -> 830mA@920mA real -> 1030@1080mA real -> 1.33 => Resolution is 0.21A
    uint16_t Battery10MilliAmpere; // Highest bit: 0=Discharge, 1=Charge -> depends on ProtocolVersionNumber. Maximal current is 327 A
    uint8_t TokenSOCPercent;                // 0x85
    uint8_t SOCPercent;                     // 0-100%
    uint8_t TokenNumberOfTemperatureSensors; // 0x86
    uint8_t NumberOfTemperatureSensors;     // 2
    uint8_t TokenCycles;                    // 0x87
    uint16_t Cycles;
    uint8_t TokenTotalBatteryCycleCapacity; // 0x89
    uint32_t TotalBatteryCycleCapacity;     // Ah

    uint8_t TokenNumberOfBatteryCells;      // 0x8A
    uint16_t NumberOfBatteryCells;
    uint8_t TokenBatteryAlarm;              // 0x8B

    union {
        uint16_t AlarmsAsWord;
        struct {
            // High byte of alarms
            bool Sensor2OvertemperatureAlarm :1;    // 0x0100
            bool Sensor1Or2UndertemperatureAlarm :1; // 0x0200 Disables charging, but Has no effect on discharging
            bool CellOvervoltageAlarm :1;           // 0x0400
            bool CellUndervoltageAlarm :1; // 0x0800 Discharging undervoltage forces SOC to 0 and a few seconds later switches off discharge mosfet
            bool _309_A_ProtectionAlarm :1;         // 0x1000
            bool _309_B_ProtectionAlarm :1;
            bool Reserved1Alarm :1;                 // Two highest bits are reserved
            bool Reserved2Alarm :1;

            // Low byte of alarms
            bool LowCapacityAlarm :1;               // 0x0001
            bool PowerMosFetOvertemperatureAlarm :1;
            bool ChargeOvervoltageAlarm :1;         // 0x0004 This happens quite often, if battery charging is approaching 100 %
            bool DischargeUndervoltageAlarm :1;
            bool Sensor1Or2OvertemperatureAlarm :1; // 0x0010 - Affects the charging/discharging MosFet state, not the enable flags
            /*
             * Set with delay of (Dis)ChargeOvercurrentDelaySeconds / "OCP Delay(S)" seconds initially or on retry.
             * Retry is done after "OCPR Time(S)"
             */
            bool ChargeOvercurrentAlarm :1; // 0x0020 - Set with delay of ChargeOvercurrentDelaySeconds seconds initially or on retry
            bool DischargeOvercurrentAlarm :1; // 0x0040 - Set with delay of DischargeOvercurrentDelaySeconds seconds initially or on retry
            bool CellVoltageDifferenceAlarm :1;     // 0x0080
        } AlarmBits;
    } AlarmUnion;

    uint8_t TokenBatteryStatus;                     // 0x8C
    union {
        uint16_t StatusAsWord;
        struct {
            uint8_t ReservedStatusHighByte;         // This is the low byte of StatusAsWord, but it was sent as high byte of status
            bool ChargeMosFetActive :1;             // 0x01 // Is disabled e.g. on over current or temperature
            bool DischargeMosFetActive :1;          // 0x02 // Is disabled e.g. on over current or temperature
            bool BalancerActive :1;                 // 0x04
            bool BatteryDown :1;                    // 0x08
            uint8_t ReservedStatus :4;
        } StatusBits;
    } BMSStatus;

    uint8_t TokenBatteryOvervoltageProtection10Millivolt; // 0x8E
    uint16_t BatteryOvervoltageProtection10Millivolt;   // 1000 to 15000 BMS computed: # of cells * CellOvervoltageProtectionMillivolt
    uint8_t TokenBatteryUndervoltageProtection10Millivolt; // 0x8F
    uint16_t BatteryUndervoltageProtection10Millivolt;  // 1000 to 15000 BMS computed: # of cells * CellUndervoltageProtectionMillivolt
    uint8_t TokenCellOvervoltageProtectionMillivolt;    // 0x90
    uint16_t CellOvervoltageProtectionMillivolt;        // 1000 to 4500
    uint8_t TokenCellOvervoltageRecoveryMillivolt;      // 0x91
    uint16_t CellOvervoltageRecoveryMillivolt;          // 1000 to 4500
    uint8_t TokenCellOvervoltageDelaySeconds;           // 0x92
    uint16_t CellOvervoltageDelaySeconds;               // Cannot be set by App! I received 5
    uint8_t TokenCellUndervoltageProtectionMillivolt;   // 0x93
    uint16_t CellUndervoltageProtectionMillivolt;
    uint8_t TokenCellUndervoltageRecoveryMillivolt;     // 0x94
    uint16_t CellUndervoltageRecoveryMillivolt;
    uint8_t TokenCellUndervoltageDelaySeconds;          // 0x95
    uint16_t CellUndervoltageDelaySeconds; // Cannot be set by App! I received 5 but the BMS took 100 seconds to signal an undervoltage, see CellUndervoltage.log

    uint8_t TokenVoltageDifferenceProtectionMillivolt;  // 0x96
    uint16_t VoltageDifferenceProtectionMillivolt;      // 0 to 100

    uint8_t TokenDischargeOvercurrentProtectionAmpere;  // 0x97
    uint16_t DischargeOvercurrentProtectionAmpere;      // 1 to 1000
    uint8_t TokenDischargeOvercurrentDelaySeconds;      // 0x98
    uint16_t DischargeOvercurrentDelaySeconds; // DischargeOvercurrentRecoverySeconds can be set by App, but is not contained in reply
    uint8_t TokenChargeOvercurrentProtectionAmpere;     // 0x99
    uint16_t ChargeOvercurrentProtectionAmpere;         // 1 to 1000
    uint8_t TokenChargeOvercurrentDelaySeconds;         // 0x9A
    uint16_t ChargeOvercurrentDelaySeconds;     // ChargeOvercurrentRecoverySeconds can be set by App, but is not contained in reply

    uint8_t TokenBalancingStartVoltage;                 // 0x9B
    uint16_t BalancingStartMillivolt;                   // 2000 to 4500
    uint8_t TokenBalancingStartDifferentialVoltage;     // 0x9C
    uint16_t BalancingStartDifferentialMillivolt;       // 10 to 1000
    uint8_t TokenBalancingState;                        // 0x9D
    uint8_t BalancingIsEnabled;                         // 0=off 1=on

    uint8_t TokenPowerMosFetTemperatureProtection;      // 0x9E
    uint16_t PowerMosFetTemperatureProtection;          // 0 to 100
    uint8_t TokenPowerMosFetRecoveryTemperature;        // 0x9F
    uint16_t PowerMosFetRecoveryTemperature;            // 0 to 100
    uint8_t TokenSensor1TemperatureProtection;          // 0xA0
    uint16_t Sensor1TemperatureProtection;              // 40 to 100
    uint8_t TokenSensor1RecoveryTemperature;            // 0xA1
    uint16_t Sensor1RecoveryTemperature;                // 40 to 100

    uint8_t TokenBatteryDifferenceTemperatureProtection; // 0xA2
    uint16_t BatteryDifferenceTemperatureProtection;    // 2 to 20

    uint8_t TokenChargeOvertemperatureProtection;       // 0xA3
    uint16_t ChargeOvertemperatureProtection;           // 0 to 100
    uint8_t TokenDischargeOvertemperatureProtection;    // 0xA4
    uint16_t DischargeOvertemperatureProtection;        // 0 to 100

    uint8_t TokenChargeUndertemperatureProtection;      // 0xA5
    int16_t ChargeUndertemperatureProtection;           // -45 to 25
    uint8_t TokenChargeRecoveryUndertemperature;        // 0xA6
    int16_t ChargeRecoveryUndertemperature;             // -45 to 25
    uint8_t TokenDischargeUndertemperatureProtection;   // 0xA7
    int16_t DischargeUndertemperatureProtection;        // -45 to 25
    uint8_t TokenDischargeRecoveryUndertemperature;     // 0xA8
    int16_t DischargeRecoveryUndertemperature;          // -45 to 25

    uint8_t TokenBatteryCellCount;              // 0xA9
    uint8_t BatteryCellCount;                   // 3 to 32

    uint8_t TokenTotalCapacity;                 // 0xAA
    uint32_t TotalCapacityAmpereHour;           // Ah

    uint8_t TokenChargeMosFetState;             // 0xAB
    uint8_t ChargeIsEnabled;                    // 0=off 1=on
    uint8_t TokenDischargeMosFetState;          // 0xAC
    uint8_t DischargeIsEnabled;                 // 0=off 1=on

    uint8_t TokenCurrentCalibration;            // 0xAD
    uint16_t CurrentCalibrationMilliampere;     // 100 to 20000 mA - 1039 for my BMS (factory calibration?)

    uint8_t TokenBoardAddress;                  // 0xAE
    uint8_t BoardAddress;                       // 1 -used for cascading

    uint8_t TokenBatteryType;                   // 0xAF
    uint8_t BatteryType;                       // 0 (lithium iron phosphate), 1 (ternary), 2 (lithium titanate), value is constant 1

    uint8_t TokenSleepWaitingTime;              // 0xB0
    uint16_t SleepWaitingTimeSeconds;           //

    uint8_t TokenLowCapacityAlarm;              // 0xB1
    uint8_t LowCapacityAlarmPercent;            // 0 to 80

    uint8_t TokenModifyParameterPassword;       // 0xB2
    char ModifyParameterPassword[10];           // "123456" - can be HEX

    uint8_t TokenDedicatedChargerSwitchState;   // 0xB3
    uint8_t DedicatedChargerSwitchIsActive;     // 0=off 1=on

    uint8_t TokenDeviceIdString;                // 0xB4
    char DeviceIdString[8];                // First 8 characters of the manufacturer id entered in the app field "User Private Data"

    uint8_t TokenManufacturerDate;              // 0xB5
    char ManufacturerDate[4];                   // "YYMM" - Date of first connection with app

    uint8_t TokenSystemWorkingMinutes;          // 0xB6
    uint32_t SystemWorkingMinutes;              // Minutes

    uint8_t TokenSoftwareVersionNumber;         // 0xB7
    char SoftwareVersionNumber[15];             // "11.XW_S11.26___" or from documentation: "NW_1_0_0_200428"

    uint8_t TokenStartCurrentCalibration;       // 0xB8
    uint8_t StartCurrentCalibration;            // 0=stop 1=start

    uint8_t TokenActualBatteryCapacity;         // 0xB9
    uint32_t ActualBatteryCapacityAmpereHour;   // Ah

    uint8_t TokenManufacturerId;                // 0xBA
    char ManufacturerId[24]; // First 12 characters of the 13 characters manufacturer id entered in the app field "User Private Data"
                             // followed by "JK_B2A20S20P" for my balancer

//    uint8_t TokenRestartSystem;                 // 0xBB
//    uint8_t RestartSystem;                      // 0=stop 1=restart
//    uint8_t TokenFactoryDataReset;              // 0xBC
//    uint8_t FactoryDataReset;                   // 0=stop 1=reset
//    uint8_t TokenRemoteUpgradeIdentification;   // 0xBD
//    uint8_t RemoteUpgradeIdentification;        // 0=stop 1=start
//
//    uint8_t TokenGPSTurnOffVoltage;             // 0xBE
//    uint16_t GPSTurnOffVoltageMillivolt;
//    uint8_t TokenGPSRecoveryVoltage;            // 0xBF
//    uint16_t GPSRecoveryVoltageMillivolt;

    uint8_t TokenProtocolVersionNumber;         // 0xC0
    uint8_t ProtocolVersionNumber;              // 00, 01 -> Redefine the 0x84 current data as 10 mA,
                                                // with the highest bit being 0 for discharge and 1 for charge
};

/*
 * Contains only 4 unconverted values (not in JKComputedDataStruct) which are compared witch current ones to detect changes
 */
struct JKLastReplyStruct {
    uint8_t SOCPercent;                         // 0-100% 0x85
    union {                                     // 0x8B
        uint16_t AlarmsAsWord;
        struct {
            // High byte of alarms
            bool Sensor2OvertemperatureAlarm :1;    // 0x0100
            bool Sensor1Or2UndertemperatureAlarm :1; // 0x0200 Disables charging, but Has no effect on discharging
            bool CellOvervoltageAlarm :1;           // 0x0400
            bool CellUndervoltageAlarm :1;
            bool _309_A_ProtectionAlarm :1;         // 0x1000
            bool _309_B_ProtectionAlarm :1;
            bool Reserved1Alarm :1;                 // Two highest bits are reserved
            bool Reserved2Alarm :1;

            // Low byte of alarms
            bool LowCapacityAlarm :1;               // 0x0001
            bool PowerMosFetOvertemperatureAlarm :1;
            bool ChargeOvervoltageAlarm :1;         // 0x0004 This happens quite often, if battery charging is approaching 100 %
            bool DischargeUndervoltageAlarm :1;
            bool Sensor1Or2OvertemperatureAlarm :1; // 0x0010 - Affects the charging/discharging MosFet state, not the enable flags
            /*
             * Set with delay of (Dis)ChargeOvercurrentDelaySeconds / "OCP Delay(S)" seconds initially or on retry.
             * Retry is done after "OCPR Time(S)"
             */
            bool ChargeOvercurrentAlarm :1; // 0x0020 - Set with delay of ChargeOvercurrentDelaySeconds seconds initially or on retry
            bool DischargeOvercurrentAlarm :1; // 0x0040 - Set with delay of DischargeOvercurrentDelaySeconds seconds initially or on retry
            bool CellVoltageDifferenceAlarm :1;     // 0x0080
        } AlarmBits;
    } AlarmUnion;

    union {                                         // 0x8C
        uint16_t StatusAsWord;
        struct {
            uint8_t ReservedStatusHighByte;         // This is the low byte of StatusAsWord, but it was sent as high byte of status
            bool ChargeMosFetActive :1;             // 0x01 // Is disabled e.g. on over current or temperature
            bool DischargeMosFetActive :1;          // 0x02 // Is disabled e.g. on over current or temperature
            bool BalancerActive :1;                 // 0x04
            bool BatteryDown :1;                    // 0x08
            uint8_t ReservedStatus :4;
        } StatusBits;
    } BMSStatus;

    uint32_t SystemWorkingMinutes;              // Minutes 0xB6
};

#endif // _JK_BMS_H
