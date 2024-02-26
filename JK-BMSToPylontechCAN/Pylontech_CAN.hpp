/*
 * Pylontech_CAN.hpp
 *
 * Functions to fill and send CAN data defined in Pylontech_CAN.h
 *
 * Useful links:
 * https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication
 * https://www.skpang.co.uk/products/teensy-4-1-triple-can-board-with-240x240-ips-lcd
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

#ifndef _PYLONTECH_CAN_HPP
#define _PYLONTECH_CAN_HPP

#include <Arduino.h>

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

#include "MCP2515_TX.h" // my reduced driver
#include "Pylontech_CAN.h"

struct PylontechCANBatteryLimitsFrame351Struct PylontechCANBatteryLimitsFrame351;
struct PylontechCANSohSocFrame355Struct PylontechCANSohSocFrame355;
struct PylontechCANCurrentValuesFrame356Struct PylontechCANCurrentValuesFrame356;
struct PylontechCANBatteryRequesFrame35CStruct PylontechCANBatteryRequestFrame35C;
struct PylontechCANErrorsWarningsFrame359Struct PylontechCANErrorsWarningsFrame359;
// Extensions to the standard Pylontech protocol
struct PylontechCANSMACapacityFrame35FStruct PylontechCANSMACapacityFrame35F;
struct PylontechCANLuxpowerCapacityFrame379Struct PylontechCANLuxpowerCapacityFrame379;
struct BYDCANCellLimitsFrame373Struct BYDCANCellLimitsFrame373;

// Frames with fixed data
struct PylontechCANManufacturerFrameStruct35E PylontechCANManufacturerFrame35E;
struct PylontechCANAliveFrame305Struct PylontechCANAliveFrame305;

void modifyCANData(); // user function, which currently enables the function to reduce max current at high SOC level

void fillAllCANData(struct JKReplyStruct *aJKFAllReply) {
    PylontechCANBatteryLimitsFrame351.fillFrame(aJKFAllReply);
    PylontechCANSohSocFrame355.fillFrame(aJKFAllReply);
    PylontechCANBatteryRequestFrame35C.fillFrame(aJKFAllReply);
    PylontechCANErrorsWarningsFrame359.fillFrame(aJKFAllReply);
    PylontechCANCurrentValuesFrame356.fillFrame(aJKFAllReply);
#if defined(CAPACITY_35F_EXTENSIONS)
    PylontechCANSMACapacityFrame35F.fillFrame(aJKFAllReply);
#endif
#if defined(CAPACITY_379_EXTENSIONS)
    PylontechCANLuxpowerCapacityFrame379.fillFrame(aJKFAllReply);
#endif
#if defined(BYD_LIMITS_373_EXTENSIONS)
    BYDCANCellLimitsFrame373.fillFrame(aJKFAllReply);
#endif
#if defined(CAN_DATA_MODIFICATION)
    modifyCANData();
#endif
}

void sendCANFrame(struct CANFrameStruct *aPylontechCANFrame) {
    sendCANMessage(aPylontechCANFrame->CANFrameInfo.CANId, aPylontechCANFrame->CANFrameInfo.FrameLength,
            aPylontechCANFrame->FrameData.UBytes);
}

/*
 * Called in case of BMS communication timeout
 */
void modifyAllCanDataToInactive() {
    PylontechCANCurrentValuesFrame356.FrameData.Current100Milliampere = 0;
    // Clear all requests in case of timeout / BMS switched off, before sending
    reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryRequestFrame35C)->FrameData.UWords[0] = 0;
    reinterpret_cast<struct CANFrameStruct*>(&PylontechCANErrorsWarningsFrame359)->FrameData.ULong.LowLong = 0;
}

void printCANFrame(struct CANFrameStruct *aPylontechCANFrame) {
    Serial.print(F("CANId=0x"));
    Serial.print(aPylontechCANFrame->CANFrameInfo.CANId, HEX);
    Serial.print(F(", FrameLength="));
    Serial.print(aPylontechCANFrame->CANFrameInfo.FrameLength);
    Serial.print(F(", Data=0x"));
    for (uint_fast8_t i = 0; i < aPylontechCANFrame->CANFrameInfo.FrameLength; ++i) {
        if (i != 0) {
            Serial.print(F(", 0x"));
        }
        Serial.print(aPylontechCANFrame->FrameData.UBytes[i], HEX);
    }
    Serial.println();
}

/*
 * Inverter reply every second: 0x305: 00-00-00-00-00-00-00-00
 * If no CAN receiver is attached, every frame is retransmitted once, because of the NACK error.
 * Or use CAN.writeRegister(REG_CANCTRL, 0x08); // One Shot Mode
 */
void sendAllCANFrames(bool aDebugModeActive) {
    if (aDebugModeActive) {
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryLimitsFrame351));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSohSocFrame355));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANCurrentValuesFrame356));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANManufacturerFrame35E));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryRequestFrame35C));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANAliveFrame305));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANErrorsWarningsFrame359));
#if defined(CAPACITY_35F_EXTENSIONS)
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSMACapacityFrame35F));
#endif
#if defined(CAPACITY_379_EXTENSIONS)
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANLuxpowerCapacityFrame379));
#endif
#if defined(BYD_LIMITS_373_EXTENSIONS)
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&BYDCANCellLimitsFrame373));
#endif

    }
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryLimitsFrame351));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSohSocFrame355));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANCurrentValuesFrame356));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANManufacturerFrame35E));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryRequestFrame35C));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANAliveFrame305));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANErrorsWarningsFrame359));
#if defined(CAPACITY_35F_EXTENSIONS)
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSMACapacityFrame35F));
#endif
#if defined(CAPACITY_379_EXTENSIONS)
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANLuxpowerCapacityFrame379));
#endif
#if defined(BYD_LIMITS_373_EXTENSIONS)
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&BYDCANCellLimitsFrame373));
#endif
}

/*
 * User defined function to modify CAN data sent to inverter.
 * Currently implemented is a function to reduce max current at high SOC level
 */
#if !defined(MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT)
#define MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT        80  // Start SOC for linear reducing maximum current. Default 80
#endif
#if !defined(MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE)
#define MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE       50  // Value of current at 100 % SOC. Units are 100 mA! Default 50
#endif

#if !defined(USE_OWN_MODIFY_FUNCTION) && !defined(USE_CCCV_MODIFY_FUNCTION)
void modifyCANData() {
    if (sJKFAllReplyPointer->SOCPercent >= MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT) {
        /*
         * Reduce max current linear from 100% at MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD (80%) SOC
         * to MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE (1A) at 100% SOC
         */
        PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere = map(sJKFAllReplyPointer->SOCPercent,
        MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT, 100,
                PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere,
                MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE);
    }
}
#elif defined(USE_CCCV_MODIFY_FUNCTION)
/*
 * Example from https://github.com/paulsteigel/JK-BMSToPylontechCAN/blob/b96af8e2e39850211ca847f347a632ed57e5ce4f/JK-BMSToPylontechCAN/Pylontech_CAN.hpp#L152
 */
uint8_t ReachChargeLimit();
void resetCharge();
// For controlling charge scheme
const uint8_t CHARGE_PHASE_1 = 45;            // 45 minutes warming up, charging current will go up in linear mode
const uint8_t CHARGE_PHASE_3 = 45;            // 45 minutes warming up, charging current will go down gradually in linear mode
//#define SOC_END_CONSTANT_CURRENT  80        // milestone for changing charging scheme
#define CHARGING_CURRENT_PER_CAPACITY 3       // to be three tenth of capacity
#define MAX_SOC_BULK_CHARGE_THRESHOLD_PERCENT 95 // SOC Level to move to absorption mode

const uint8_t CHARGE_RATIO = 60L;
const uint32_t MOMENTARY_CHARGE_DURATION = (2L * CHARGE_RATIO * 1000L);       // if charge is continuing in this duration, the main charging sequense started
const uint32_t CHARGE_STATUS_REFRESH_INTERVAL = (1L * CHARGE_RATIO * 1000L);  // interval for refreshing charge status

//bool IsCharging = false;            // Keep state of charging one getcurrent return positive
//uint16_t Max_Charge_Current_100_milliAmp;   // keep peak charging current during warm up for stabilising

uint16_t Computed_Current_limits_100mA;      //
uint16_t Charge_Current_100_milliAmp;
uint32_t StartChargeTime = 0;                // Store starting time for charge
uint32_t LastCheckTime = 0;                  // Record time count for each charging check
uint8_t ChargePhase = 0;                     // Charging phase (1CC,2CC,3CV)
uint8_t MinuteCount = 0; // counting minutes for charge
uint8_t ChargeTryEffort = 0;                 // Count of Charge try when battery is full for stopping the charge action from Inverter

void modifyCANData() {
    uint8_t ChargeStatusRef = 0;
    uint16_t Local_Charge_Current_100_milliAmp;

    if (JKComputedData.BatteryLoadCurrentFloat > 0) {
        ChargeStatusRef = ReachChargeLimit();
        if (StartChargeTime == 0) {
            if (ChargeStatusRef >= 1 && ChargeTryEffort > 2) {
                // Too much for now, request charging off
                ChargeTryEffort++;
                PylontechCANBatteryRequestFrame35C.FrameData.ChargeEnable = 0;
                Serial.println(F("Setting charge to OFF:"));
                return;
            } else {
                PylontechCANBatteryRequestFrame35C.FrameData.ChargeEnable =
                        sJKFAllReplyPointer->BMSStatus.StatusBits.DischargeMosFetActive;
            }
            StartChargeTime = millis(); // Store starting time for charge
            // Get the proper charging current: either BMS limit or using 0.3C
            Computed_Current_limits_100mA = min(swap(sJKFAllReplyPointer->ChargeOvercurrentProtectionAmpere) * 10,
                    JKComputedData.TotalCapacityAmpereHour * CHARGING_CURRENT_PER_CAPACITY);
            Serial.print(F("Charging check: >Selected Current:"));
            Serial.println(Computed_Current_limits_100mA);
        }
    } else {
        resetCharge();
        return;
    }

    // Get into current charging phase
    if (StartChargeTime == 0) return; // No charging detected
    // Allow this to be triggered once
    if (ChargePhase == 0) {
        if ((millis() - StartChargeTime) < MOMENTARY_CHARGE_DURATION) return;
        // Charge started in more than MOMENTARY_CHARGE_DURATION, charging started, get into phase 1
        //MinuteCount = MOMENTARY_CHARGE_DURATION / (CHARGE_RATIO * 1000L); // Marking the counter for warming up charge
        MinuteCount = map(JKComputedData.BatteryLoadCurrentFloat * 10, 1, Computed_Current_limits_100mA, 0, CHARGE_PHASE_1) + 1;
        ChargePhase = 1;
        Serial.println(F("Enter phase 1:"));
        Serial.print(MinuteCount);
    }

    // Check if end of phase 1, move to phase 2 or next
    if (ChargePhase == 1) {
        if ((millis() - StartChargeTime) >= ((CHARGE_PHASE_1 + 1) * CHARGE_RATIO * 1000L)) {
            ChargePhase = 2;
            MinuteCount = 0; //reset the minute counter to enter phase 2
            Serial.print(F("Enter phase 2:"));
            Serial.println(MinuteCount);
        }
    } else if (ChargePhase == 2) {
        //ChargeStatusRef = ReachChargeLimit();
        if (ChargeStatusRef == 1) {
            ChargePhase = 3;
            MinuteCount = 0; //reset the minute counter during phase 2
        } else if (ChargeStatusRef == 2) {
            // reducing current by 10%
            Charge_Current_100_milliAmp = Charge_Current_100_milliAmp * 0.98;
        }
    }

    if ((millis() - LastCheckTime) < CHARGE_STATUS_REFRESH_INTERVAL) return;
    LastCheckTime = millis(); //reset the counter for resuming the check
    switch (ChargePhase) {
    case 1:
        // Linear mapping of charging current until reaching 0.3C
        Local_Charge_Current_100_milliAmp = map(MinuteCount, 0, CHARGE_PHASE_1, 1, Computed_Current_limits_100mA);
        Charge_Current_100_milliAmp =
                (Local_Charge_Current_100_milliAmp > Computed_Current_limits_100mA) ?
                        Computed_Current_limits_100mA : Local_Charge_Current_100_milliAmp;
        PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere = Charge_Current_100_milliAmp;
        MinuteCount++;
        Serial.print(F("Charging phase 1: minute count::"));
        Serial.println(MinuteCount);
        Serial.print(F("Applied charged current::"));
        Serial.print(PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere);
        Serial.print(F("/"));
        Serial.println(Computed_Current_limits_100mA);

        break;
    case 2:
        // in this phase, do nothing to current at all
        Serial.print(F("Charging phase 2: Minute count::"));
        Serial.println(MinuteCount);
        Serial.print(F("Applied charged current::"));
        Serial.print(Charge_Current_100_milliAmp);
        Serial.print(F("/"));
        Serial.println(Computed_Current_limits_100mA);
        MinuteCount++;
        break;
    case 3:
        //Keep charging till full or when the Inverter stop charging, need a new routine for this charging
        Serial.print(F("Charging phase 3: Minute count::"));
        Serial.println(MinuteCount);
        MinuteCount++;
        if (ChargeStatusRef != 2)
            Charge_Current_100_milliAmp = map(MinuteCount, 1, CHARGE_PHASE_3, Computed_Current_limits_100mA, 0);
        PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere = Charge_Current_100_milliAmp;
        Serial.print(F("Applied charged current::"));
        Serial.print(PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere);
        Serial.print(F("/"));
        Serial.println(Computed_Current_limits_100mA);

        if (Charge_Current_100_milliAmp == 0) {
            // tell the inverter to stop charging or keep it to maintain the charge
            Serial.print(F("End of charge"));
            resetCharge();
        }
        break;
    default:
        // statements
        break;
    }
}

void resetCharge() {
    Serial.println(F("Reset charging parameters"));
    StartChargeTime = 0; // reset charge starting time
    LastCheckTime = 0;
    MinuteCount = 0;
    ChargePhase = 0;
    ChargeTryEffort = 0;
    // recover the charging limits
    if (PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere
            != swap(sJKFAllReplyPointer->ChargeOvercurrentProtectionAmpere) * 10) {
        PylontechCANBatteryLimitsFrame351.FrameData.BatteryChargeCurrentLimit100Milliampere = swap(
                sJKFAllReplyPointer->ChargeOvercurrentProtectionAmpere) * 10;
    }
}

uint8_t ReachChargeLimit() {
    /* Status return:
     *  0: nothing;
     *  1: stop phase;
     *  2: reducing current 10%;
     */
    //will do this check every 5 minutes
    uint16_t Charge_MilliVolt_limit = 0;
    if ((millis() - LastCheckTime) < CHARGE_STATUS_REFRESH_INTERVAL) return 0;
    // first check over voltage
    if (sJKFAllReplyPointer->BatteryType == 0) { //LFP battery
        Charge_MilliVolt_limit = 3450;
    } else if (sJKFAllReplyPointer->BatteryType == 1) { //Lithium ion
        Charge_MilliVolt_limit = 4200;
    }
    // check SOC First
    return (sJKFAllReplyPointer->SOCPercent >= MAX_SOC_BULK_CHARGE_THRESHOLD_PERCENT) ? 1 : 0;
    Serial.print(F("Battery type:"));
    Serial.println(sJKFAllReplyPointer->BatteryType);
    if ((JKConvertedCellInfo.MaximumCellMillivolt * 1.02) > Charge_MilliVolt_limit) {
        Serial.print(F("Status check::"));
        Serial.println((JKConvertedCellInfo.MaximumCellMillivolt * 1.02) - Charge_MilliVolt_limit);
        return 2;
    }
}
#else
/*
 * Put your own modification code here :-)
 */
void modifyCANData() {

}
#endif

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _PYLONTECH_CAN_HPP
