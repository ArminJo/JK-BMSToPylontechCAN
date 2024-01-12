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

struct PylontechCANBatteryLimitsFrameStruct PylontechCANBatteryLimitsFrame;
struct PylontechCANSohSocFrameStruct PylontechCANSohSocFrame;
struct PylontechCANCurrentValuesFrameStruct PylontechCANCurrentValuesFrame;
struct PylontechCANBatteryRequesFrameStruct PylontechCANBatteryRequestFrame;
struct PylontechCANErrorsWarningsFrameStruct PylontechCANErrorsWarningsFrame;
// Extensions to the standard Pylontech protocol
struct PylontechCANSMACapacityFrameStruct PylontechCANSMACapacityFrame;
struct PylontechCANLuxpowerCapacityFrameStruct PylontechCANLuxpowerCapacityFrame;
struct BYDCANCellLimitsFrameStruct BYDCANCellLimitsFrame;

// Frames with fixed data
struct PylontechCANManufacturerFrameStruct PylontechCANManufacturerFrame;
struct PylontechCANAliveFrameStruct PylontechCANAliveFrame;

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
void modifyCANData() {
    if (sJKFAllReplyPointer->SOCPercent >= MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT) {
        /*
         * Reduce max current linear from 100% at MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD (80%) SOC
         * to MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE (1A) at 100% SOC
         */
        PylontechCANBatteryLimitsFrame.FrameData.BatteryChargeCurrentLimit100Milliampere = map(sJKFAllReplyPointer->SOCPercent,
                MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT, 100,
                PylontechCANBatteryLimitsFrame.FrameData.BatteryChargeCurrentLimit100Milliampere,
                MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE);
    }
}

void fillAllCANData(struct JKReplyStruct *aJKFAllReply) {
    PylontechCANBatteryLimitsFrame.fillFrame(aJKFAllReply);
    PylontechCANSohSocFrame.fillFrame(aJKFAllReply);
    PylontechCANBatteryRequestFrame.fillFrame(aJKFAllReply);
    PylontechCANErrorsWarningsFrame.fillFrame(aJKFAllReply);
    PylontechCANCurrentValuesFrame.fillFrame(aJKFAllReply);
#if defined(SMA_EXTENSIONS)
    PylontechCANSMACapacityFrame.fillFrame(aJKFAllReply);
#endif
#if defined(LUXPOWER_EXTENSIONS)
    PylontechCANLuxpowerCapacityFrame.fillFrame(aJKFAllReply);
#endif
#if defined(BYD_EXTENSIONS)
    BYDCANCellLimitsFrame.fillFrame(aJKFAllReply);
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
    PylontechCANCurrentValuesFrame.FrameData.Current100Milliampere = 0;
    // Clear all requests in case of timeout / BMS switched off, before sending
    reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryRequestFrame)->FrameData.UWords[0] = 0;
    reinterpret_cast<struct CANFrameStruct*>(&PylontechCANErrorsWarningsFrame)->FrameData.ULong.LowLong = 0;
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
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryLimitsFrame));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSohSocFrame));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANCurrentValuesFrame));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANManufacturerFrame));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryRequestFrame));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANAliveFrame));
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANErrorsWarningsFrame));
#if defined(SMA_EXTENSIONS)
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSMACapacityFrame));
#endif
#if defined(LUXPOWER_EXTENSIONS)
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANLuxpowerCapacityFrame));
#endif
#if defined(BYD_EXTENSIONS)
        printCANFrame(reinterpret_cast<struct CANFrameStruct*>(&BYDCANCellLimitsFrame));
#endif

    }
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryLimitsFrame));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSohSocFrame));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANCurrentValuesFrame));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANManufacturerFrame));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANBatteryRequestFrame));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANAliveFrame));
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANErrorsWarningsFrame));
#if defined(SMA_EXTENSIONS)
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANSMACapacityFrame));
#endif
#if defined(LUXPOWER_EXTENSIONS)
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&PylontechCANLuxpowerCapacityFrame));
#endif
#if defined(BYD_EXTENSIONS)
    sendCANFrame(reinterpret_cast<struct CANFrameStruct*>(&BYDCANCellLimitsFrame));
#endif
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _PYLONTECH_CAN_HPP
