/*
 * Pylontech_CAN.cpp
 *
 * Functions to fill and send CAN data defined in Pylontech_CAN.h
 *
 * Useful links:
 * https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication
 * https://www.skpang.co.uk/products/teensy-4-1-triple-can-board-with-240x240-ips-lcd
 *
 *  Copyright (C) 2023  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ArduinoUtils https://github.com/ArminJo/PVUtils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
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

#ifndef _PYLONTECH_CAN_CPP
#define _PYLONTECH_CAN_CPP

#include <Arduino.h>

#if !defined(LOCAL_DEBUG)
//#define LOCAL_DEBUG
#endif

#include "MCP2515_TX.h" // my reduced driver
#include "Pylontech_CAN.h"

struct PylontechCANBatteryLimitsFrameStruct PylontechCANBatteryLimitsFrame;
struct PylontechCANSohSocFrameStruct PylontechCANSohSocFrame;
struct PylontechCANCurrentValuesFrameStruct PylontechCANCurrentValuesFrame;
struct PylontechCANManufacturerFrameStruct PylontechCANManufacturerFrame;
struct PylontechCANBatteryRequesFrameStruct PylontechCANBatteryRequesFrame;
struct PylontechCANAliveFrameStruct PylontechCANAliveFrameStruct;
struct PylontechCANErrorsWarningsFrameStruct PylontechCANErrorsWarningsFrame;

void fillAllCANData(struct JKReplyStruct *aJKFAllReply) {
    PylontechCANBatteryLimitsFrame.fillFrame(aJKFAllReply);
    PylontechCANSohSocFrame.fillFrame(aJKFAllReply);
    PylontechCANBatteryRequesFrame.fillFrame(aJKFAllReply);
    PylontechCANErrorsWarningsFrame.fillFrame(aJKFAllReply);
    PylontechCANCurrentValuesFrame.fillFrame(aJKFAllReply);
}

void sendPylontechCANFrame(struct PylontechCANFrameStruct *aPylontechCANFrame) {
    sendCANMessage(aPylontechCANFrame->PylontechCANFrameInfo.CANId, aPylontechCANFrame->PylontechCANFrameInfo.FrameLength,
            aPylontechCANFrame->FrameData.UBytes);
}

void printPylontechCANFrame(struct PylontechCANFrameStruct *aPylontechCANFrame) {
    Serial.print(F("CANId=0x"));
    Serial.print(aPylontechCANFrame->PylontechCANFrameInfo.CANId, HEX);
    Serial.print(F(", FrameLength="));
    Serial.print(aPylontechCANFrame->PylontechCANFrameInfo.FrameLength);
    Serial.print(F(", Data=0x"));
    for (uint_fast8_t i = 0; i < aPylontechCANFrame->PylontechCANFrameInfo.FrameLength; ++i) {
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
void sendPylontechAllCANFrames(bool aDebugModeActive) {
    if (aDebugModeActive) {
        printPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANBatteryLimitsFrame));
        printPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANSohSocFrame));
        printPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANCurrentValuesFrame));
        printPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANManufacturerFrame));
        printPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANBatteryRequesFrame));
        printPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANAliveFrameStruct));
        printPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANErrorsWarningsFrame));
    }
    sendPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANBatteryLimitsFrame));
    sendPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANSohSocFrame));
    sendPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANCurrentValuesFrame));
    sendPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANManufacturerFrame));
    sendPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANBatteryRequesFrame));
    sendPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANAliveFrameStruct));
    sendPylontechCANFrame(reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANErrorsWarningsFrame));
}

#endif // _PYLONTECH_CAN_H
