/*
 * Pylontech_CAN.h
 *
 * Definitions for the CAN frames to send as Pylon protocol.
 * TODO The generated output does not correspond to the logs below and
 * published at https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication
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

// Based on information in:
// https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication

#ifndef _PYLONTECH_CAN_H
#define _PYLONTECH_CAN_H

#include <Arduino.h>
#include "JK-BMS.h"
#include "LongUnion.h"

/*
 * Frame ID's and sample output without CRC
 */
#define PYLON_CAN_NETWORK_ALIVE_MSG_FRAME_ID        0x305 // All zeros
// CANId=0x305, FrameLength=8, Data=0x21, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0

#define PYLON_CAN_BATTERY_LIMITS_FRAME_ID           0x351 // Battery voltage + current limits
// CANId=0x351, FrameLength=8, Data=0x28, 0x2, 0xF4, 0x1, 0x20, 0x3, 0xE0, 0x1 | 0x228->55.2V, 0x1F4->50.0A, 0x320->80.0A 0x1E0-> 48.0V

#define PYLON_CAN_BATTERY_SOC_SOH_FRAME_ID          0x355 // State of Charge (SOC) / State  of Health (SOH)
// CANId=0x355, FrameLength=4, Data=0x3C, 0x0, 0x64, 0x0 | 0x3C->60% SOC, 0x64->100% SOH

#define PYLON_CAN_BATTERY_CURRENT_VALUES_U_I_T_FRAME_ID     0x356 // Voltage / Current / Temperature
// CANId=0x356, FrameLength=6, Data=0x6C, 0x14, 0x4, 0x0, 0xDC, 0x0 | 0x146C->52.28V, 0.4A, 0xDC->22.0C

#define PYLON_CAN_BATTERY_ERROR_WARNINGS_FRAME_ID   0x359 // Protection & Alarm flags
// CANId=0x359, FrameLength=7, Data=0x0, 0x0, 0x0, 0x0, 0x1, 0x50, 0x4E | No alarms

#define PYLON_CAN_BATTERY_CHARGE_REQUEST_FRAME_ID   0x35C // Battery charge request flags
// CANId=0x35C, FrameLength=2, Data=0xC0, 0x0 | 0xC0->Charge and discharge enable

#define PYLON_CAN_BATTERY_MANUFACTURER_FRAME_ID     0x35E // Manufacturer name
// CANId=0x35E, FrameLength=8, Data=0x50, 0x59, 0x4C, 0x4F, 0x4E, 0x20, 0x20, 0x20 | "PYLON"

#define PYLON_CAN_BATTERY_SMA_CAPACITY_FRAME_ID     0x35F // Capacity for SMA Sunny Island inverters
#define BYD_CAN_BATTERY_CELL_LIMITS_FRAME_ID        0x373 // Cell limits info frame from BYD battery
#define PYLON_CAN_BATTERY_LUXPOWER_CAPACITY_FRAME_ID    0x379 // Capacity for Luxpower SNA inverters

extern struct PylontechCANBatteryLimitsFrame351Struct PylontechCANBatteryLimitsFrame351;
extern struct PylontechCANSohSocFrame355Struct PylontechCANSohSocFrame355;
extern struct PylontechCANCurrentValuesFrame356Struct PylontechCANCurrentValuesFrame356;
extern struct PylontechCANManufacturerFrameStruct35E PylontechCANManufacturerFrame35E;
extern struct PylontechCANBatteryRequesFrame35CStruct PylontechCANBatteryRequestFrame35C;
extern struct PylontechCANAliveFrame305Struct PylontechCANAliveFrameStruct;
extern struct PylontechCANErrorsWarningsFrame359Struct PylontechCANErrorsWarningsFrame359;
extern struct PylontechCANSMACapacityFrame35FStruct PylontechCANSMACapacityFrame35F;
extern struct BYDCANCellLimitsFrame373Struct BYDCANCellLimitsFrame373;
extern struct PylontechCANLuxpowerCapacityFrame379Struct PylontechCANLuxpowerCapacityFrame379;

void fillAllCANData(struct JKReplyStruct *aJKFAllReply);
void sendAllCANFrames(bool aDebugModeActive);
void modifyAllCanDataToInactive();

struct CANFrameInfoStruct {
    // Both values will be statically initialized in each instance
    const uint16_t CANId;
    const uint8_t FrameLength;
};

struct CANFrameStruct {
    struct CANFrameInfoStruct CANFrameInfo;
    LongLongUnion FrameData; // 8 byte
};

struct PylontechCANAliveFrame305Struct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_NETWORK_ALIVE_MSG_FRAME_ID, 8 }; // 0x305
    struct {
        uint8_t AlivePacketArray[8] = { 33 };
    } FrameData;
};

/*
 * -Static data-
 * Limits
 */
struct PylontechCANBatteryLimitsFrame351Struct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_LIMITS_FRAME_ID, 8 }; // 0x351
    struct {
        int16_t BatteryChargeOvervoltage100Millivolt;       // 0 to 750     | Maximum of all, so you can disable one by setting its value low
        int16_t BatteryChargeCurrentLimit100Milliampere;    // 0 to 5000    | SUM of all Charge enabled modules
        int16_t BatteryDischargeCurrentLimit100Milliampere; // -5000 to 0   | SUM of all Discharge enabled modules
        int16_t BatteryDischarge100Millivolt;               // 0 to 65535   | Minimum of all, so you can disable one by setting its value high
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        FrameData.BatteryChargeOvervoltage100Millivolt = JKComputedData.BatteryFullVoltage10Millivolt / 10;
        FrameData.BatteryChargeCurrentLimit100Milliampere = swap(aJKFAllReply->ChargeOvercurrentProtectionAmpere) * 10;
        FrameData.BatteryDischargeCurrentLimit100Milliampere = swap(aJKFAllReply->DischargeOvercurrentProtectionAmpere) * 10;
        FrameData.BatteryDischarge100Millivolt = JKComputedData.BatteryEmptyVoltage10Millivolt / 10;
    }
};

/*
 * -Dynamic data-
 * SOC value, SOH is fixed to 100%
 */
struct PylontechCANSohSocFrame355Struct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_SOC_SOH_FRAME_ID, 4 }; // 0x355
    struct {
        uint16_t SOCPercent;
        uint16_t SOHPercent = 100; // fixed 100
#if defined(CAPACITY_35F_EXTENSIONS)
        uint16_t SOCHighDefinition100PPM = 10000; // for SMA Sunny Island inverters
#endif
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        FrameData.SOCPercent = aJKFAllReply->SOCPercent;
    }
};

/*
 * -Dynamic data-
 */
struct PylontechCANCurrentValuesFrame356Struct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_CURRENT_VALUES_U_I_T_FRAME_ID, 6 }; // 0x356
    struct {
        int16_t Voltage10Millivolt;         // 0 to 32767       | Average of all
        int16_t Current100Milliampere;      // -2500 to 2500    | Sum of all
        int16_t Temperature100Millicelsius; // -500 to 750      | Maximum of all
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        (void) aJKFAllReply; // To avoid [-Wunused-parameter] warning
        FrameData.Voltage10Millivolt = JKComputedData.BatteryVoltage10Millivolt;
        FrameData.Current100Milliampere = JKComputedData.Battery10MilliAmpere / 10;
        FrameData.Temperature100Millicelsius = JKComputedData.TemperatureMaximum * 10;
    }
};

/*
 * -Dynamic data-
 * Errors and warnings
 */
struct PylontechCANErrorsWarningsFrame359Struct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_ERROR_WARNINGS_FRAME_ID, 7 }; // 0x359
    struct FrameDataStruct {
        // 0=off 1=on
        // Byte 0
        bool :1; // unused
        bool CellOvervoltageError :1;           // 0x02
        bool CellUndervoltageError :1;          // 0x04
        bool CellOvertemperatureError :1;       // 0x08
        bool CellUndertemperatureError :1;      // 0x10
        bool :2; // unused
        bool DischargeOvercurrentError :1;      // 0x80

        // Byte 1
//    bool :0;
        bool ChargeOvercurrentError :1;         // 0x01
        bool :6; // unused
        bool SystemError :1;                    // 0x80

        // Byte 2
//    bool :0;
        bool :1; // unused
        bool CellHighVoltageWarning :1;         // 0x02
        bool CellLowVoltageWarning :1;          // 0x04
        bool CellHighTemperatureWarning :1;     // 0x08
        bool CellLowTemperatureWarning :1;      // 0x10
        bool :2; // unused
        bool DischargeHighCurrentWarning :1;    // 0x80

        // Byte 3
//    bool :0;
        bool ChargeHighCurrentWarning :1;       // 0x01
        // found in documentation
//        bool :2; // unused
//        bool InternalCommunicationFail :1;
//        bool :4; // unused

        // Found in software
        bool :6; // unused
        bool SystemWarning :1;                  // 0x80

        // Byte 4 to 6
//    bool :0;
        uint8_t ModuleNumber = 1;           // 0 to 255. 1 is displayed on Deye as "LiBms01"
        uint8_t Token1 = 0x50;              // 'P'
        uint8_t Token2 = 0x4E;              // 'N'
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        /*
         * Pylon has no battery over voltage alarm but cell over voltage warning and error
         * We (mis)use the battery alarms as cell warnings
         */
        // Byte 0
        FrameData.CellOvervoltageError = aJKFAllReply->AlarmUnion.AlarmBits.CellOvervoltageAlarm;
        FrameData.CellUndervoltageError = aJKFAllReply->AlarmUnion.AlarmBits.CellUndervoltageAlarm;
        FrameData.CellOvertemperatureError = aJKFAllReply->AlarmUnion.AlarmBits.PowerMosFetOvertemperatureAlarm
                || aJKFAllReply->AlarmUnion.AlarmBits.Sensor1Or2OvertemperatureAlarm;
        FrameData.CellUndertemperatureError = aJKFAllReply->AlarmUnion.AlarmBits.Sensor1Or2UndertemperatureAlarm;
        FrameData.DischargeOvercurrentError = aJKFAllReply->AlarmUnion.AlarmBits.DischargeOvercurrentAlarm;

        // Byte 1
        FrameData.ChargeOvercurrentError = aJKFAllReply->AlarmUnion.AlarmBits.ChargeOvercurrentAlarm;
        FrameData.SystemError = aJKFAllReply->BMSStatus.StatusBits.BatteryDown;
//        if (aJKFAllReply->SOCPercent < 5) {
//            FrameData.SystemError = 1;
//        }

        // Byte 2
        // (mis)use the battery alarms as cell warnings for Pylon
        FrameData.CellHighVoltageWarning = aJKFAllReply->AlarmUnion.AlarmBits.ChargeOvervoltageAlarm;
        FrameData.CellLowVoltageWarning = aJKFAllReply->AlarmUnion.AlarmBits.DischargeUndervoltageAlarm;
        // Use the same values as for error here
        FrameData.CellHighTemperatureWarning = aJKFAllReply->AlarmUnion.AlarmBits.PowerMosFetOvertemperatureAlarm
                || aJKFAllReply->AlarmUnion.AlarmBits.Sensor1Or2OvertemperatureAlarm;
        FrameData.CellLowTemperatureWarning = aJKFAllReply->AlarmUnion.AlarmBits.Sensor1Or2UndertemperatureAlarm;
        FrameData.DischargeHighCurrentWarning = aJKFAllReply->AlarmUnion.AlarmBits.DischargeOvercurrentAlarm;

        // Byte 3
        // Use the same values as for error here
        FrameData.ChargeHighCurrentWarning = aJKFAllReply->AlarmUnion.AlarmBits.ChargeOvercurrentAlarm;
        FrameData.SystemError = aJKFAllReply->BMSStatus.StatusBits.BatteryDown;

//        if (aJKFAllReply->SOCPercent < 10) {
//            FrameData.SystemWarning = 1;
//        }

    }
};

/*
 * -Dynamic data-
 * ForceChargeRequestI / bit 5 is designed for inverter allows battery to shut down, and able to wake battery up to charge it.
 * ForceChargeRequestII / bit 4 is designed for inverter doesn`t want battery to shut down, able to charge battery before shut down to avoid low energy.
 * 2 bytes
 */
struct PylontechCANBatteryRequesFrame35CStruct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_CHARGE_REQUEST_FRAME_ID, 2 }; // 0x35C
    struct {
        bool :3; // unused
        // 0=off 1=Request
        /*
         * From Pylontech manual: Depending on the soc level, there will be a regularly (3 month) fully charge requesting during continuous operation as well.
         * It will be handled automatically by the communication between BESS and external device.
         * This is to stop SOC calculations drifting too far from reality when the battery has not had a full charge for n days.
         */
        bool FullChargeRequest :1; // sent as always 0

        // Force to charge battery even from the grid.
        // From Battery-Communications-Integration-Guide-V2.5-1.pdf: Command sent by the BMS telling the inverter to charge the battery from any available power source regardless of inverter settings.
        bool ForceChargeRequestII :1;
        bool ForceChargeRequestI :1;
        bool DischargeEnable :1;
        bool ChargeEnable :1;
        uint8_t Filler = 0;
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        FrameData.FullChargeRequest = 0;

#if SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I > 0
        if (aJKFAllReply->SOCPercent < SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I) {
            // ForceChargeRequestI forces the inverter to charge the battery from any available power source regardless of inverter settings
            FrameData.ForceChargeRequestI = 1;
        } else {
            FrameData.ForceChargeRequestI = 0;
        }
#else
        FrameData.ForceChargeRequestI = 0;
#endif
        // If battery drops below lower voltage. See https://powerforum.co.za/topic/13587-battery-anomaly-on-synsynk-hybrid-inverter/
        if (swap(aJKFAllReply->Battery10Millivolt) < JKComputedData.BatteryEmptyVoltage10Millivolt) {
            FrameData.ForceChargeRequestII = 1;
        } else {
            FrameData.ForceChargeRequestII = 0;
        }
        FrameData.DischargeEnable = aJKFAllReply->BMSStatus.StatusBits.ChargeMosFetActive;
        FrameData.ChargeEnable = aJKFAllReply->BMSStatus.StatusBits.DischargeMosFetActive;
    }
};

/*
 * -Static data, no fill-
 * Character array DIYPYLON is not recognized by Deye, array PYLONDIY is recognized as PYLON
 */
struct PylontechCANManufacturerFrameStruct35E {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_MANUFACTURER_FRAME_ID, 8 }; // 0x35E
    struct {
        char ManufacturerName[8] = { 'P', 'Y', 'L', 'O', 'N', ' ', ' ', ' ' };
    } FrameData;
};

/**************** Extensions to the standard Pylontech protocol *************/
/*
 * -Dynamic data-
 * Frame for total capacity for SMA - Sunny Island inverters
 * Description was found in: https://github.com/Uksa007/esphome-jk-bms-can/blob/main/docs/SMA%20CAN%20Protocol%20Mapping.pdf
 * and in UserManual9R_SMA.pdf of www.rec-bms.com
 * All values except CapacityAmpereHour are best guesses
 */
struct PylontechCANSMACapacityFrame35FStruct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_SMA_CAPACITY_FRAME_ID, 8 }; // 0x35F
    struct {
        uint16_t CellChemistry = 0;
        uint8_t HardwareVersionLowByte = '0';
        uint8_t HardwareVersionHighByte = '1';
        uint16_t CapacityAmpereHour;
        uint8_t SoftwareVersionLowByte;
        uint8_t SoftwareVersionHighByte;
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        FrameData.SoftwareVersionLowByte = aJKFAllReply->SoftwareVersionNumber[1];
        FrameData.SoftwareVersionHighByte = aJKFAllReply->SoftwareVersionNumber[0];
        FrameData.CapacityAmpereHour = JKComputedData.TotalCapacityAmpereHour;
    }
};

/*
 * -Static data-
 * BYD Cell Limits
 * These data need to be updated every 5 minutes.
 */
struct BYDCANCellLimitsFrame373Struct {
    struct CANFrameInfoStruct CANFrameInfo = { BYD_CAN_BATTERY_CELL_LIMITS_FRAME_ID, 8 }; // 0x373
    struct {
        uint16_t CellVoltageMinimumMilliVolt;       //
        uint16_t CellVoltageMaximumMilliVolt;       //
        uint16_t CellTemperatureMinimumKelvin;      //
        uint16_t CellTemperatureMaximumKelvin;      //
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        FrameData.CellVoltageMinimumMilliVolt = swap(aJKFAllReply->CellUndervoltageProtectionMillivolt);
        FrameData.CellVoltageMaximumMilliVolt = swap(aJKFAllReply->CellOvervoltageProtectionMillivolt);
        FrameData.CellTemperatureMinimumKelvin =
                min(swap(aJKFAllReply->ChargeUndertemperatureProtection), swap(aJKFAllReply->DischargeUndertemperatureProtection))
                        + 273;
        FrameData.CellTemperatureMaximumKelvin =
                max(swap(aJKFAllReply->ChargeOvertemperatureProtection), swap(aJKFAllReply->DischargeOvertemperatureProtection))
                        + 273;
    }
};

/*
 * -Dynamic data-
 * Frame for total capacity for Luxpower - SNA inverters
 * Description was found in: https://github.com/dfch/BydCanProtocol/tree/main
 */
struct PylontechCANLuxpowerCapacityFrame379Struct {
    struct CANFrameInfoStruct CANFrameInfo = { PYLON_CAN_BATTERY_LUXPOWER_CAPACITY_FRAME_ID, 8 }; // 0x379
    struct {
        uint16_t CapacityAmpereHour;
        uint16_t Unknown1;
        uint32_t Unknown2;
    } FrameData;
    void fillFrame(struct JKReplyStruct *aJKFAllReply) {
        (void) aJKFAllReply; // To avoid [-Wunused-parameter] warning
        FrameData.CapacityAmpereHour = JKComputedData.TotalCapacityAmpereHour;
    }
};
#endif // _PYLONTECH_CAN_H
