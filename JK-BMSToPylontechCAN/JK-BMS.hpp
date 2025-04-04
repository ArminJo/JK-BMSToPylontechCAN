/*
 * JK-BMS.hpp
 *
 * Functions to read, convert and print JK-BMS data
 *
 *  We use 6 Structures:
 *  1. JKReplyStruct - the main reply structure, containing raw BMS reply data Big Endian, which must be swapped.
 *  2. JKLastReplyStruct - copy of SOC, Uptime, Alarm and Status flags of last reply to detect changes.
 *  3. JKConvertedCellInfoStruct - including statistics (min, max, average etc.) for print and LCD usage.
 *  4. JKComputedDataStruct - swapped and computed data based on JKReplyStruct content.
 *  5. JKLastPrintedDataStruct - part of last JKComputedDataStruct to detect changes.
 *  6. CellStatisticsStruct - for minimum and maximum cell voltages statistics.
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

#ifndef _JK_BMS_HPP
#define _JK_BMS_HPP

#include <Arduino.h>

#if defined(HANDLE_MULTIPLE_BMS)
#  if !defined(NUMBER_OF_SUPPORTED_BMS)
#define NUMBER_OF_SUPPORTED_BMS 2
#  endif
#endif

#include "JK-BMS.h"
#include "HexDump.hpp" // include sources for printBufferHex()

#define USE_SOFTWARE_SERIAL
#if defined(USE_SOFTWARE_SERIAL)
/*
 * Software serial for JK-BMS request frame sending
 */
#include "SoftwareSerialTX.h"
#endif

#include "LocalDebugLevelCheck.h"
// This definition must be located after the includes of other *.hpp files
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#include "LocalDebugLevelStart.h"

// see JK Communication protocol.pdf http://www.jk-bms.com/Upload/2022-05-19/1621104621.pdf
uint8_t JKRequestStatusFrame[21] = { 0x4E, 0x57 /*4E 57 = StartOfFrame*/, 0x00, 0x13 /*0x13 | 19 = LengthOfFrame*/, 0x00, 0x00,
        0x00, 0x00/*BMS ID, highest byte is default 00*/, 0x06/*Function 1=Activate, 3=ReadIdentifier, 6=ReadAllData*/,
        0x03/*Frame source 0=BMS, 1=Bluetooth, 2=GPRS, 3=PC*/, 0x00 /*TransportType 0=Request, 1=Response, 2=BMSActiveUpload*/,
        0x00/*0=ReadAllData or commandToken*/, 0x00, 0x00, 0x00,
        0x00/*RecordNumber High byte is random code, low 3 bytes is record number*/, JK_FRAME_END_BYTE/*0x68 = EndIdentifier*/,
        0x00, 0x00, 0x01, 0x29 /*Checksum, high 2 bytes for checksum not yet enabled -> 0, low 2 Byte for checksum*/};
//uint8_t JKrequestStatusFrameOld[] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };

/*
 * Size of reply is 291 bytes for 16 cells. sizeof(JKReplyStruct) is 221.
 */
uint8_t JKReplyFrameBuffer[350];            // The raw big endian data as received from JK BMS.

#define LENGTH_OF_UPTIME_STRING 11
char sUpTimeString[LENGTH_OF_UPTIME_STRING + 1]; // "9999D23H12M" is 11 bytes long
char sBalancingTimeString[11] = { ' ', ' ', '0', 'D', '0', '0', 'H', '0', '0', 'M', '\0' };    // "999D23H12M" is 10 bytes long
bool sUpTimeStringMinuteHasChanged;
bool sUpTimeStringTenthOfMinuteHasChanged;
char sLastUpTimeTenthOfMinuteCharacter;     // For detecting changes in string and setting sUpTimeStringTenthOfMinuteHasChanged
bool sUpTimeStringHourHasChanged;
char sLastUpTimeHourCharacter;              // For setting sUpTimeStringHourHasChanged

/*
 * ALARM stuff
 */
const char lowCapacity[] PROGMEM = "Low capacity";                          // Byte 0.0,
const char MosFetOvertemperature[] PROGMEM = "Power MosFet overtemperature"; // Byte 0.1;
const char chargingOvervoltage[] PROGMEM = "Battery is full";               // Byte 0.2,  // Charging overvoltage
const char dischargingUndervoltage[] PROGMEM = "Discharging undervoltage";  // Byte 0.3,
const char Sensor2Overtemperature[] PROGMEM = "Sensor1_2 overtemperature";  // Byte 0.4,
const char chargingOvercurrent[] PROGMEM = "Charging overcurrent";          // Byte 0.5,
const char dischargingOvercurrent[] PROGMEM = "Discharging overcurrent";    // Byte 0.6,
const char CellVoltageDifference[] PROGMEM = "Cell voltage difference";     // Byte 0.7,
const char Sensor1Overtemperature[] PROGMEM = "Sensor2 overtemperature";    // Byte 1.0,
const char Sensor2LowLemperature[] PROGMEM = "Sensor1_2 low temperature";   // Byte 1.1,
const char CellOvervoltage[] PROGMEM = "Cell overvoltage";                  // Byte 1.2,
const char CellUndervoltage[] PROGMEM = "Cell undervoltage";                // Byte 1.3,
const char _309AProtection[] PROGMEM = "309_A protection";                  // Byte 1.4,
const char _309BProtection[] PROGMEM = "309_B protection";                  // Byte 1.5,

const char *const JK_BMSAlarmStringsArray[NUMBER_OF_DEFINED_ALARM_BITS] PROGMEM = { lowCapacity, MosFetOvertemperature,
        chargingOvervoltage, dischargingUndervoltage, Sensor2Overtemperature, chargingOvercurrent, dischargingOvercurrent,
        CellVoltageDifference, Sensor1Overtemperature, Sensor2LowLemperature, CellOvervoltage, CellUndervoltage, _309AProtection,
        _309BProtection };

/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#if defined(HANDLE_MULTIPLE_BMS)
JK_BMS *JK_BMS::BMSArray[NUMBER_OF_SUPPORTED_BMS];
uint8_t JK_BMS::sBMS_ArrayNextIndex = 0;
JKMultiBMSDataStruct JKMultiBMSData;
#endif

JK_BMS::JK_BMS() { // @suppress("Class members should be properly initialized")
#if defined(HANDLE_MULTIPLE_BMS)
    BMSArray[sBMS_ArrayNextIndex] = this;
    sBMS_ArrayNextIndex++;
    NumberOfThisBMS = sBMS_ArrayNextIndex;
#endif
}

/*
 * 115200 baud soft serial to JK-BMS. For serial from BMS we use the hardware Serial RX.
 */
void JK_BMS::init(uint8_t aTxPinNumber) {
#if defined(USE_SOFTWARE_SERIAL)
    TxToJKBMS.setTX(aTxPinNumber);
    TxToJKBMS.begin(115200);
#else
#endif
}

/*
 * 1.85 ms
 */
void JK_BMS::requestJK_BMSStatusFrame(bool aDebugModeActive) {
    if (aDebugModeActive) {
        Serial.println();
#if defined(HANDLE_MULTIPLE_BMS)
        Serial.print(F("Send requestFrame to BMS "));
        Serial.println(NumberOfThisBMS);
#else
        Serial.println(F("Send requestFrame to BMS"));
#endif
        printBufferHex(JKRequestStatusFrame, sizeof(JKRequestStatusFrame));
    }
    Serial.flush();

    for (uint8_t i = 0; i < sizeof(JKRequestStatusFrame); ++i) {
#if defined(USE_SOFTWARE_SERIAL)
        TxToJKBMS.write(JKRequestStatusFrame[i]);
#else
#endif
    }
}

/*
 * Request status frame
 */
void JK_BMS::RequestStatusFrame(bool aDebugModeActive) {
    /*
     * Flush input buffer (e.g. after JK_BMS_RECEIVE_ERROR) and send request to JK-BMS
     */
    while (Serial.available()) {
        Serial.read();
    }
#if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, HIGH);
#endif
    /*
     * Copy last complete reply and computed values for change determination
     */
    lastJKReply.SOCPercent = JKAllReplyPointer->SOCPercent;
    lastJKReply.BatteryAlarmFlags.AlarmsAsWord = JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord;
    lastJKReply.BMSStatus.StatusAsWord = JKAllReplyPointer->BMSStatus.StatusAsWord;
    lastJKReply.SystemWorkingMinutes = JKAllReplyPointer->SystemWorkingMinutes;

    requestJK_BMSStatusFrame(aDebugModeActive); // 1.85 ms
#if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, LOW);
#endif
    ReplyFrameBufferIndex = 0;  // init reply frame buffer
    MillisOfLastReceivedByte = millis(); // initialize reply timeout
}

/*
 * Is assumed to be called if Serial.available() is true
 * @return JK_BMS_RECEIVE_ONGOING, if still receiving; JK_BMS_RECEIVE_FINISHED, if complete frame was successfully read
 *          JK_BMS_RECEIVE_ERROR, if frame plausibility does not hold.
 * Reply starts 0.18 ms to 0.45 ms after request was received
 */
uint8_t JK_BMS::readJK_BMSStatusFrameByte() {
    uint8_t tReceivedByte = Serial.read();
    JKReplyFrameBuffer[ReplyFrameBufferIndex] = tReceivedByte;

    /*
     * Plausi check and get length of frame
     */
    if (ReplyFrameBufferIndex == 0) {
        // start byte 1
        if (tReceivedByte != JK_FRAME_START_BYTE_0) {
            Serial.println(F(" start frame token != 0x4E"));
            return JK_BMS_RECEIVE_ERROR;
        }
    } else if (ReplyFrameBufferIndex == 1) {
        if (tReceivedByte != JK_FRAME_START_BYTE_1) {
            //
            return JK_BMS_RECEIVE_ERROR;
        }

    } else if (ReplyFrameBufferIndex == 3) {
        // length of frame
        ReplyFrameLength = (JKReplyFrameBuffer[2] << 8) + tReceivedByte;

    } else if (ReplyFrameLength > MINIMAL_JK_BMS_FRAME_LENGTH && ReplyFrameBufferIndex == ReplyFrameLength - 3) {
        // Check end token 0x68
        if (tReceivedByte != JK_FRAME_END_BYTE) {
            Serial.print(F(" end frame token 0x"));
            Serial.print(tReceivedByte, HEX);
            Serial.print(F(" at index"));
            Serial.print(ReplyFrameBufferIndex);
            Serial.print(F(" is != 0x68. ReplyFrameLength= "));
            Serial.print(ReplyFrameLength);
            Serial.print(F(" | 0x"));
            Serial.println(ReplyFrameLength, HEX);
            return JK_BMS_RECEIVE_ERROR;
        }

    } else if (ReplyFrameLength > MINIMAL_JK_BMS_FRAME_LENGTH && ReplyFrameBufferIndex == ReplyFrameLength + 1) {
        /*
         * Frame received completely, perform checksum check
         */
        uint16_t tComputedChecksum = 0;
        for (uint16_t i = 0; i < ReplyFrameLength - 2; i++) {
            tComputedChecksum = tComputedChecksum + JKReplyFrameBuffer[i];
        }
        uint16_t tReceivedChecksum = (JKReplyFrameBuffer[ReplyFrameLength] << 8) + tReceivedByte;
        if (tComputedChecksum != tReceivedChecksum) {
            Serial.print(F("Checksum , computed checksum=0x"));
            Serial.print(tComputedChecksum, HEX);
            Serial.print(F(", received checksum=0x"));
            Serial.println(tReceivedChecksum, HEX);

            return JK_BMS_RECEIVE_ERROR;
        } else {
            /*
             * Checksum OK, transfer finished :-)
             */
            return JK_BMS_RECEIVE_FINISHED;
        }
    }
    ReplyFrameBufferIndex++;
    return JK_BMS_RECEIVE_ONGOING;
}

/*
 * Check for byte at serial input if response is expected.
 * @return  JK_BMS_RECEIVE_ONGOING, if still receiving; JK_BMS_RECEIVE_FINISHED, if complete frame was successfully read and processed
 *          JK_BMS_RECEIVE_TIMEOUT if timeout happened; JK_BMS_RECEIVE_ERROR, if frame plausibility does not hold.
 *          If first timeout, set TimeoutJustDetected
 */
uint8_t JK_BMS::checkForReplyFromBMSOrTimeout() {
    if (Serial.available()) {
        MillisOfLastReceivedByte = millis();
        uint8_t tReceiveResultCode = readJK_BMSStatusFrameByte();
        if (tReceiveResultCode == JK_BMS_RECEIVE_FINISHED) {
            /*
             * All JK-BMS status frame data received
             */
            if (JKBMSFrameHasTimeout) {
#if defined(HANDLE_MULTIPLE_BMS)
                JK_INFO_PRINT(F("Successfully receiving first BMS "));
                JK_INFO_PRINT(NumberOfThisBMS);
                JK_INFO_PRINTLN(F(" frame after communication timeout"));
#else
                JK_INFO_PRINTLN(F("Successfully receiving first BMS frame after communication timeout"));
#endif
                // First successful response frame after timeout :-)
                JKBMSFrameHasTimeout = false;
                TimeoutJustDetected = false;
            }

        } else if (tReceiveResultCode != JK_BMS_RECEIVE_ONGOING) {
            /*
             * Error here
             */
            Serial.print(F("Receive error="));
            Serial.print(tReceiveResultCode);
            Serial.print(F(" at index"));
            Serial.println(ReplyFrameBufferIndex);

            printJKReplyFrameBuffer();
        }
        return tReceiveResultCode;
    } else if (millis() - MillisOfLastReceivedByte >= TIMEOUT_MILLIS_FOR_FRAME_REPLY) {
        /*
         * Here we have timeout at receiving. We requested response frame, but serial was not available fore a longer time.
         */
        if (!JKBMSFrameHasTimeout) {
            //Do this only once per timeout
            JKBMSFrameHasTimeout = true;
#if defined(HANDLE_MULTIPLE_BMS)
            JKMultiBMSData.anyTimeout  = true;
#endif
            TimeoutJustDetected = true; // This forces the beep
        }
        return JK_BMS_RECEIVE_TIMEOUT;
    }
    return JK_BMS_RECEIVE_ONGOING; // No requested or ongoing receive
}

/*
 * Fills up all self computed data
 */
void JK_BMS::processReceivedData() {
    /*
     * Set the static pointer to the start of the reply data which depends on the number of cell voltage entries
     * The JKFrameAllDataStruct starts behind the header + cell data header 0x79 + CellInfoSize + the variable length cell data 3 bytes per cell, (CellInfoSize is contained in JKReplyFrameBuffer[12])
     */
    JKAllReplyPointer = reinterpret_cast<JKReplyStruct*>(&JKReplyFrameBuffer[JK_BMS_FRAME_HEADER_LENGTH + 2
            + JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH]]);

    fillJKConvertedCellInfo();
    /*
     * Print newline, if SOC changed
     */
    if (JKAllReplyPointer->SOCPercent != lastJKReply.SOCPercent) {
        Serial.println();
    }
    fillJKComputedData();

    computeUpTimeString();
    detectAndPrintAlarmInfo(); // UpTimeString is used here
}

/*
 * Prints formatted reply buffer raw content
 * 11 bytes header
 * 2 bytes cell info length
 * Cell info
 * Blank line
 * Other content
 * 9 bytes trailer
 */
void JK_BMS::printJKReplyFrameBuffer() {
    uint8_t *tBufferAddress = JKReplyFrameBuffer;
    Serial.print(ReplyFrameBufferIndex + 1);
#if defined(HANDLE_MULTIPLE_BMS)
    Serial.print(F(" bytes received from BMS "));
    Serial.println(NumberOfThisBMS);
#else
    Serial.println(F(" bytes received"));
#endif
    printBufferHex(tBufferAddress, JK_BMS_FRAME_HEADER_LENGTH);

    tBufferAddress += JK_BMS_FRAME_HEADER_LENGTH;
    printBufferHex(tBufferAddress, JK_BMS_FRAME_CELL_INFO_LENGTH);

    tBufferAddress += JK_BMS_FRAME_CELL_INFO_LENGTH;
    uint8_t tCellInfoLength = JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH];
    printBufferHex(tBufferAddress, tCellInfoLength); // Cell info
    Serial.println();

    tBufferAddress += tCellInfoLength;
    int16_t tRemainingDataLength = ((int16_t) ReplyFrameBufferIndex + 1)
            - (JK_BMS_FRAME_HEADER_LENGTH + JK_BMS_FRAME_CELL_INFO_LENGTH + JK_BMS_FRAME_TRAILER_LENGTH + tCellInfoLength);
    if (tRemainingDataLength <= 0) {
        return;
    }
    printBufferHex(tBufferAddress, tRemainingDataLength);

    tBufferAddress += tRemainingDataLength;
    printBufferHex(tBufferAddress, JK_BMS_FRAME_TRAILER_LENGTH); // Trailer
}

/*
 * Highest bit is set means charge
 * @return Charge is positive, discharge is negative
 */
int16_t JK_BMS::getCurrent(uint16_t aJKRAWCurrent) {
    uint16_t tCurrent = swap(aJKRAWCurrent);
    if (tCurrent == 0 || (tCurrent & 0x8000) == 0x8000) {
        // Charge - NO two's complement!
        return (tCurrent & 0x7FFF);
    }
// discharge
    return tCurrent * -1;

}

int16_t JK_BMS::getJKTemperature(uint16_t aJKRAWTemperature) {
    uint16_t tTemperature = swap(aJKRAWTemperature);
    if (tTemperature <= 100) {
        return tTemperature;
    }
    return 100 - tTemperature;
}

int32_t JK_BMS::getOnePercentCapacityAsAccumulator10Milliampere() {
    return (AMPERE_HOUR_AS_ACCUMULATOR_10_MILLIAMPERE / 100) * JKComputedData.TotalCapacityAmpereHour;
}

#if defined(HANDLE_MULTIPLE_BMS)

// is shorter than computing on the fly
float JK_BMS::getSumOfBatteryLoadCurrentFloat() {
    float tBatteryLoadCurrentFloatSum = 0;
    for (uint_fast8_t i = 0; i < JK_BMS::sBMS_ArrayNextIndex; ++i) {
        tBatteryLoadCurrentFloatSum += JK_BMS::BMSArray[i]->JKComputedData.BatteryLoadCurrentFloat;
    }
    return tBatteryLoadCurrentFloatSum;
}

uint8_t JK_BMS::getAverageOfBatterySOC() {
    uint16_t tBatterySOCSum = 0;
    for (uint_fast8_t i = 0; i < JK_BMS::sBMS_ArrayNextIndex; ++i) {
        tBatterySOCSum += JK_BMS::BMSArray[i]->lastJKReply.SOCPercent;
    }
    return tBatterySOCSum / JK_BMS::sBMS_ArrayNextIndex;
}

void JK_BMS::resetJKMultiBMSData() {
    memset(&JKMultiBMSData, 0, sizeof(JKMultiBMSData));
//    JKMultiBMSData.TemperatureMaximum = -127;
}
#endif

// Identity function to avoid swapping if accidentally called
uint8_t swap(uint8_t aByte) {
    return (aByte);
}

uint16_t swap(uint16_t aWordToSwapBytes) {
    return ((aWordToSwapBytes << 8) | (aWordToSwapBytes >> 8));
}

int16_t swap(int16_t aWordToSwapBytes) {
    return ((aWordToSwapBytes << 8) | (aWordToSwapBytes >> 8));
}

uint32_t swap(uint32_t aLongToSwapBytes) {
    return ((aLongToSwapBytes << 24) | ((aLongToSwapBytes & 0xFF00) << 8) | ((aLongToSwapBytes >> 8) & 0xFF00)
            | (aLongToSwapBytes >> 24));
}

void myPrintln(const __FlashStringHelper *aPGMString, uint8_t a8BitValue) {
    Serial.print(aPGMString);
    Serial.println(a8BitValue);
}

// Not used yet, since we print uint8_t + 1 which is int16_t
void myPrint(const __FlashStringHelper *aPGMString, uint8_t a8BitValue) {
    Serial.print(aPGMString);
    Serial.print(a8BitValue);
}

void myPrintln(const __FlashStringHelper *aPGMString, uint16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.println(a16BitValue);
}

void myPrint(const __FlashStringHelper *aPGMString, uint16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.print(a16BitValue);
}

void myPrint100MillivoltFloat(const __FlashStringHelper *aPGMString, uint16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.print(a16BitValue / 100.0, 2);
}

void myPrintln(const __FlashStringHelper *aPGMString, int16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.println(a16BitValue);
}

void myPrint(const __FlashStringHelper *aPGMString, int16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.print(a16BitValue);
}

void myPrintlnSwap(const __FlashStringHelper *aPGMString, uint16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.println(swap(a16BitValue));
}

void myPrintlnSwap(const __FlashStringHelper *aPGMString, int16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.println((int16_t) swap((uint16_t) a16BitValue));
}

void myPrintSwap(const __FlashStringHelper *aPGMString, int16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.print((int16_t) swap((uint16_t) a16BitValue));
}

void myPrintIntAsFloatSwap(const __FlashStringHelper *aPGMString, int16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.print((float) swap((uint16_t) a16BitValue), 2);
}

void myPrintlnSwap(const __FlashStringHelper *aPGMString, uint32_t a32BitValue) {
    Serial.print(aPGMString);
    Serial.println(swap(a32BitValue));
}

/*
 * Convert the big endian cell voltage data from JKReplyFrameBuffer to little endian data in JKConvertedCellInfo
 * and compute minimum, maximum, delta, and average
 */
void JK_BMS::fillJKConvertedCellInfo() {
//    uint8_t *tJKCellInfoReplyPointer = &TestJKReplyStatusFrame[11];
    uint8_t *tJKCellInfoReplyPointer = &JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH];

    uint8_t tNumberOfCellInfo = (*tJKCellInfoReplyPointer++) / 3;
    JKConvertedCellInfo.ActualNumberOfCellInfoEntries = tNumberOfCellInfo;
    if (tNumberOfCellInfo > MAXIMUM_NUMBER_OF_CELLS) {
        Serial.print(F(": Program compiled with \"MAXIMUM_NUMBER_OF_CELLS=" STR(MAXIMUM_NUMBER_OF_CELLS) "\", but "));
        Serial.print(tNumberOfCellInfo);
        Serial.println(F(" cell info were sent"));
        return;
    }

    uint16_t tVoltage;
    uint32_t tMillivoltSum = 0;
    uint8_t tNumberOfNonNullCellInfo = 0;
    uint16_t tMinimumMillivolt = UINT16_MAX;
    uint16_t tMaximumMillivolt = 0;

    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        tJKCellInfoReplyPointer++;                                  // Skip Cell number
        uint8_t tHighByte = *tJKCellInfoReplyPointer++;                                  // Copy CellMillivolt
        tVoltage = tHighByte << 8 | *tJKCellInfoReplyPointer++;
        JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt = tVoltage;
        if (tVoltage > 0) {
            tNumberOfNonNullCellInfo++;
            tMillivoltSum += tVoltage;
            if (tMinimumMillivolt > tVoltage) {
                tMinimumMillivolt = tVoltage;
                JKConvertedCellInfo.IndexOfMinimumCellMillivolt = i;
            }
            if (tMaximumMillivolt < tVoltage) {
                tMaximumMillivolt = tVoltage;
                JKConvertedCellInfo.IndexOfMaximumCellMillivolt = i;
            }
        }
    }
    JKConvertedCellInfo.MinimumCellMillivolt = tMinimumMillivolt;
    JKConvertedCellInfo.MaximumCellMillivolt = tMaximumMillivolt;
    JKConvertedCellInfo.DeltaCellMillivolt = tMaximumMillivolt - tMinimumMillivolt;
    JKConvertedCellInfo.RoundedAverageCellMillivolt = (tMillivoltSum + (tNumberOfNonNullCellInfo / 2)) / tNumberOfNonNullCellInfo;

#if !defined(NO_CELL_STATISTICS) && defined(USE_SERIAL_2004_LCD)
        /*
         * Mark and count minimum and maximum cell voltages
         */
    bool tBalancerIsActive = JKAllReplyPointer->BMSStatus.StatusAsByteArray[1] & STATUS_BYTE_BALANCER_MASK;
        for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
            if (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt == tMinimumMillivolt) {
                JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_MINIMUM;
                if (tBalancerIsActive) {
                    CellStatistics.CellMinimumArray[i]++; // count for statistics
                }
            } else if (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt == tMaximumMillivolt) {
                JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_MAXIMUM;
                if (tBalancerIsActive) {
                    CellStatistics.CellMaximumArray[i]++;
                }
            } else {
                JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_BETWEEN_MINIMUM_AND_MAXIMUM;
            }
        }
#endif

#if !defined(NO_CELL_STATISTICS)
    /*
     * Process minimum statistics
     */
    uint32_t tCellStatisticsSum = 0;
    bool tDoDaylyScaling = false;
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        /*
         * After 43200 counts (a whole day being the minimum / maximum) we do scaling
         */
        uint16_t tCellStatisticsCount = CellStatistics.CellMinimumArray[i];
        tCellStatisticsSum += tCellStatisticsCount;
        if (tCellStatisticsCount > (60UL * 60UL * 24UL * 1000UL / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS)) {
            /*
             * After 43200 counts (a whole day being the minimum / maximum) we do scaling
             */
            tDoDaylyScaling = true;
        }
    }

// Here, we demand 2 minutes of balancing as minimum
    if (tCellStatisticsSum > 60) {
        for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
            CellStatistics.CellMinimumPercentageArray[i] = ((uint32_t) (CellStatistics.CellMinimumArray[i] * 100UL))
                    / tCellStatisticsSum;
        }
    }

    if (tDoDaylyScaling) {
        /*
         * Do scaling by dividing all values by 2 resulting in an Exponential Moving Average filter for values
         */
        Serial.println(F("Do scaling of minimum counts"));
        for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
            CellStatistics.CellMinimumArray[i] = CellStatistics.CellMinimumArray[i] / 2;
        }
    }

    /*
     * Process maximum statistics
     */
    tCellStatisticsSum = 0;
    tDoDaylyScaling = false;
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        /*
         * After 43200 counts (a whole day being the minimum / maximum) we do scaling
         */
        uint16_t tCellStatisticsCount = CellStatistics.CellMaximumArray[i];
        tCellStatisticsSum += tCellStatisticsCount;
        if (tCellStatisticsCount > (60UL * 60UL * 24UL * 1000UL / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS)) {
            /*
             * After 43200 counts (a whole day being the minimum / maximum) we do scaling
             */
            tDoDaylyScaling = true;
        }
    }

// Here, we demand 2 minutes of balancing as minimum
    if (tCellStatisticsSum > 60) {
        for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
            CellStatistics.CellMaximumPercentageArray[i] = ((uint32_t) (CellStatistics.CellMaximumArray[i] * 100UL))
                    / tCellStatisticsSum;
        }
    }
    if (tDoDaylyScaling) {
        /*
         * Do scaling by dividing all values by 2 resulting in an Exponential Moving Average filter for values
         */
        Serial.println(F("Do scaling of maximum counts"));
        for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
            CellStatistics.CellMaximumArray[i] = CellStatistics.CellMaximumArray[i] / 2;
        }
    }
#endif // NO_CELL_STATISTICS

#if defined(LOCAL_DEBUG)
        Serial.print(tNumberOfCellInfo);
        Serial.println(F(" cell voltages processed"));
#endif
// During JK-BMS startup all cell voltages are sent as zero for around 6 seconds
    if (tNumberOfNonNullCellInfo < tNumberOfCellInfo && !JKComputedData.BMSIsStarting) {
        Serial.print(F("Problem: "));
        Serial.print(tNumberOfCellInfo);
        Serial.print(F(" cells configured in BMS, but only "));
        Serial.print(tNumberOfNonNullCellInfo);
        Serial.println(F(" cells seems to be connected"));
    }
}

#if !defined(NO_CELL_STATISTICS)
/*
 * Print formatted cell info on Serial
 */
void JK_BMS::printJKCellStatisticsInfo() {
    uint8_t tNumberOfCellInfo = JKConvertedCellInfo.ActualNumberOfCellInfoEntries;

    /*
     * Cell statistics
     */
    char tStringBuffer[18]; // "12=12 % |  4042, "

    Serial.println(F("Cell Minimum percentages"));
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (i != 0 && (i % 8) == 0) {
            Serial.println();
        }
        snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%2u=%2u %% |%5u, "), i + 1,
                CellStatistics.CellMinimumPercentageArray[i], CellStatistics.CellMinimumArray[i]);
        Serial.print(tStringBuffer);
    }
    Serial.println();

    Serial.println(F("Cell Maximum percentages"));
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (i != 0 && (i % 8) == 0) {
            Serial.println();
        }
        snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%2u=%2u %% |%5u, "), i + 1,
                CellStatistics.CellMaximumPercentageArray[i], CellStatistics.CellMaximumArray[i]);
        Serial.print(tStringBuffer);
    }
    Serial.println();

    Serial.println();
}

#endif // NO_CELL_STATISTICS

void JK_BMS::fillJKComputedData() {
    JKComputedData.TemperaturePowerMosFet = getJKTemperature(JKAllReplyPointer->TemperaturePowerMosFet);
    int16_t tMaxTemperature = JKComputedData.TemperaturePowerMosFet;

    JKComputedData.TemperatureSensor1 = getJKTemperature(JKAllReplyPointer->TemperatureSensor1);
    if (tMaxTemperature < JKComputedData.TemperatureSensor1) {
        tMaxTemperature = JKComputedData.TemperatureSensor1;
    }

    JKComputedData.TemperatureSensor2 = getJKTemperature(JKAllReplyPointer->TemperatureSensor2);
    if (tMaxTemperature < JKComputedData.TemperatureSensor2) {
        tMaxTemperature = JKComputedData.TemperatureSensor2;
    }
    JKComputedData.TemperatureMaximum = tMaxTemperature;
#if defined(HANDLE_MULTIPLE_BMS)
    if (JKMultiBMSData.TemperatureMaximum < tMaxTemperature) {
        JKMultiBMSData.TemperatureMaximum = tMaxTemperature;
    }
#endif
    JKComputedData.TotalCapacityAmpereHour = swap(JKAllReplyPointer->TotalCapacityAmpereHour);
#if defined(HANDLE_MULTIPLE_BMS)
    JKMultiBMSData.SumOfTotalCapacityAmpereHour +=JKComputedData.TotalCapacityAmpereHour;
#endif
// 16 bit multiplication gives overflow at 640 Ah
    JKComputedData.RemainingCapacityAmpereHour = ((uint32_t) JKComputedData.TotalCapacityAmpereHour * JKAllReplyPointer->SOCPercent)
            / 100;

// Two values which are zero during JK-BMS startup for around 16 seconds
    JKComputedData.BMSIsStarting = (JKAllReplyPointer->SOCPercent == 0 && JKAllReplyPointer->Cycles == 0);

// Initialize capacity accumulator with sensible value
    JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere = (AMPERE_HOUR_AS_ACCUMULATOR_10_MILLIAMPERE / 100)
            * JKAllReplyPointer->SOCPercent * JKComputedData.TotalCapacityAmpereHour;

    JKComputedData.BatteryFullVoltage10Millivolt = swap(JKAllReplyPointer->BatteryOvervoltageProtection10Millivolt);
    JKComputedData.BatteryVoltage10Millivolt = swap(JKAllReplyPointer->Battery10Millivolt);
//    JKComputedData.BatteryVoltageDifferenceToFull10Millivolt = JKComputedData.BatteryFullVoltage10Millivolt
//            - JKComputedData.BatteryVoltage10Millivolt;

    JKComputedData.BatteryEmptyVoltage10Millivolt = swap(JKAllReplyPointer->BatteryUndervoltageProtection10Millivolt);
    JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt = JKComputedData.BatteryVoltage10Millivolt
            - JKComputedData.BatteryEmptyVoltage10Millivolt;

    JKComputedData.BatteryVoltageFloat = JKComputedData.BatteryVoltage10Millivolt / 100.0;

    JKComputedData.Battery10MilliAmpere = getCurrent(JKAllReplyPointer->Battery10MilliAmpere);
#if defined(HANDLE_MULTIPLE_BMS)
    JKMultiBMSData.SumOfBattery10MilliAmpere += JKComputedData.Battery10MilliAmpere;
#endif
    JKComputedData.BatteryLoadCurrentFloat = JKComputedData.Battery10MilliAmpere / 100.0;
    JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere += JKComputedData.Battery10MilliAmpere;
    if (lastJKReply.SOCPercent == 0 && JKAllReplyPointer->SOCPercent == 1) {
        JK_INFO_PRINTLN(F("Reset capacity to 1%"));
        // Reset capacity at transition from 0 to 1
        JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere = getOnePercentCapacityAsAccumulator10Milliampere();
        JKLastPrintedData.BatteryCapacityAccumulator10MilliAmpere = JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere;
    }

//    Serial.print("Battery10MilliAmpere=0x");
//    Serial.print(JKAllReplyPointer->Battery10MilliAmpere, HEX);
//    Serial.print(" Battery10MilliAmpere swapped=0x");
//    Serial.println(swap(JKAllReplyPointer->Battery10MilliAmpere), HEX);
//    Serial.print(" Battery10MilliAmpere=");
//    Serial.print(JKComputedData.Battery10MilliAmpere);
//    Serial.print(" BatteryLoadCurrent=");
//    Serial.println(JKComputedData.BatteryLoadCurrentFloat);

    JKComputedData.BatteryLoadPower = JKComputedData.BatteryVoltageFloat * JKComputedData.BatteryLoadCurrentFloat;
#if defined(HANDLE_MULTIPLE_BMS)
    JKMultiBMSData.SumOfBatteryLoadPower += JKComputedData.BatteryLoadPower;
#endif

#if !defined(NO_CELL_STATISTICS)
    /*
     * Increment BalancingCount and fill sBalancingTimeString
     */
    if (JKAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
        CellStatistics.BalancingCount++;
        snprintf_P(sBalancingTimeString, sizeof(sBalancingTimeString), PSTR("%3uD%02uH%02uM"),
                (uint16_t) (CellStatistics.BalancingCount / (60 * 24 * 30UL)),
                (uint16_t) ((CellStatistics.BalancingCount / (60 * 30)) % 24),
                (uint16_t) (CellStatistics.BalancingCount / 30) % 60);
    }
#endif // NO_CELL_STATISTICS
#if defined(HANDLE_MULTIPLE_BMS)
    JKMultiBMSData.SumOfChargeOvercurrentProtectionAmpere += swap(JKAllReplyPointer->ChargeOvercurrentProtectionAmpere);
    JKMultiBMSData.SumOfDischargeOvercurrentProtectionAmpere += swap(JKAllReplyPointer->DischargeOvercurrentProtectionAmpere);
    JKMultiBMSData.oredAlarms.AlarmsAsWord |= JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord;
    JKMultiBMSData.oredStatusAsByte |= JKAllReplyPointer->BMSStatus.StatusAsWord >> 8; // flags are in upper byte (sent first)
#endif
}

void JK_BMS::printJKCellInfoOverview() {
    myPrint(F(" Minimum at "), JKConvertedCellInfo.IndexOfMinimumCellMillivolt + 1);
    myPrint(F("="), JKConvertedCellInfo.MinimumCellMillivolt);
    myPrint(F(" mV, Maximum at "), JKConvertedCellInfo.IndexOfMaximumCellMillivolt + 1);
    myPrint(F("="), JKConvertedCellInfo.MaximumCellMillivolt);
    myPrint(F("mV, Delta="), JKConvertedCellInfo.DeltaCellMillivolt);
    myPrint(F(" mV, Average="), JKConvertedCellInfo.RoundedAverageCellMillivolt);
    Serial.println(F(" mV"));
}
/*
 * Print formatted cell info on Serial
 */
void JK_BMS::printJKCellInfo() {
    uint8_t tNumberOfCellInfo = JKConvertedCellInfo.ActualNumberOfCellInfoEntries;
    Serial.print(JKConvertedCellInfo.ActualNumberOfCellInfoEntries);
    Serial.print(F(" Cells,"));

// Print summary
    printJKCellInfoOverview();

    /*
     * Individual cell voltages
     */
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (i != 0 && (i % 8) == 0) {
            Serial.println();
        }

        Serial.print(i + 1);
        Serial.print('=');
        Serial.print(JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt);
#if defined(LOCAL_TRACE)
            Serial.print(F("|0x"));
            Serial.print(JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt, HEX);
#endif
        Serial.print(F(" mV, "));
        if (i < 8) {
            Serial.print(' ');
        }
    }
    Serial.println();
}

void JK_BMS::printVoltageProtectionInfo() {
    JKReplyStruct *tJKAllReplyPointer = JKAllReplyPointer;
    /*
     * Voltage protection
     */
    Serial.print(F("Battery Overvoltage Protection[V]="));
    Serial.print(JKComputedData.BatteryFullVoltage10Millivolt / 100.0, 2);
    Serial.print(F(", Undervoltage="));
    Serial.println(JKComputedData.BatteryEmptyVoltage10Millivolt / 100.0, 2);

    myPrintSwap(F("Cell Overvoltage Protection[mV]="), tJKAllReplyPointer->CellOvervoltageProtectionMillivolt);
    myPrintSwap(F(", Recovery="), tJKAllReplyPointer->CellOvervoltageRecoveryMillivolt);
    myPrintlnSwap(F(", Delay[s]="), tJKAllReplyPointer->CellOvervoltageDelaySeconds);

    myPrintSwap(F("Cell Undervoltage Protection[mV]="), tJKAllReplyPointer->CellUndervoltageProtectionMillivolt);
    myPrintSwap(F(", Recovery="), tJKAllReplyPointer->CellUndervoltageRecoveryMillivolt);
    myPrintlnSwap(F(", Delay[s]="), tJKAllReplyPointer->CellUndervoltageDelaySeconds);

    myPrintlnSwap(F("Cell Voltage Difference Protection[mV]="), tJKAllReplyPointer->VoltageDifferenceProtectionMillivolt);

    myPrintSwap(F("Discharging Overcurrent Protection[A]="), tJKAllReplyPointer->DischargeOvercurrentProtectionAmpere);
    myPrintlnSwap(F(", Delay[s]="), tJKAllReplyPointer->DischargeOvercurrentDelaySeconds);

    myPrintSwap(F("Charging Overcurrent Protection[A]="), tJKAllReplyPointer->ChargeOvercurrentProtectionAmpere);
    myPrintlnSwap(F(", Delay[s]="), tJKAllReplyPointer->ChargeOvercurrentDelaySeconds);
    Serial.println();
}

void JK_BMS::printTemperatureProtectionInfo() {
    JKReplyStruct *tJKAllReplyPointer = JKAllReplyPointer;
    /*
     * Temperature protection
     */
    myPrintSwap(F("Power MosFet Temperature Protection="), tJKAllReplyPointer->PowerMosFetTemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKAllReplyPointer->PowerMosFetRecoveryTemperature);

    myPrintSwap(F("Sensor1 Temperature Protection="), tJKAllReplyPointer->Sensor1TemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKAllReplyPointer->Sensor1RecoveryTemperature);

    myPrintlnSwap(F("Sensor1 to Sensor2 Temperature Difference Protection="),
            tJKAllReplyPointer->BatteryDifferenceTemperatureProtection);

    myPrintSwap(F("Charge Overtemperature Protection="), tJKAllReplyPointer->ChargeOvertemperatureProtection);
    myPrintlnSwap(F(", Discharge="), tJKAllReplyPointer->DischargeOvertemperatureProtection);

    myPrintSwap(F("Charge Undertemperature Protection="), tJKAllReplyPointer->ChargeUndertemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKAllReplyPointer->ChargeRecoveryUndertemperature);

    myPrintSwap(F("Discharge Undertemperature Protection="), tJKAllReplyPointer->DischargeUndertemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKAllReplyPointer->DischargeRecoveryUndertemperature);
    Serial.println();
}

void JK_BMS::printBatteryInfo() {
    JKReplyStruct *tJKAllReplyPointer = JKAllReplyPointer;

    Serial.print(F("Manufacturer Date="));
    tJKAllReplyPointer->TokenSystemWorkingMinutes = '\0'; // Set end of string token
    Serial.println(tJKAllReplyPointer->ManufacturerDate);

    Serial.print(F("Manufacturer Id=")); // First 8 characters of the manufacturer id entered in the app field "User Private Data"
    tJKAllReplyPointer->TokenProtocolVersionNumber = '\0'; // Set end of string token
    Serial.println(tJKAllReplyPointer->ManufacturerId);

    Serial.print(F("Device ID String=")); // First 8 characters of ManufacturerId
    tJKAllReplyPointer->TokenManufacturerDate = '\0'; // Set end of string token
    Serial.println(tJKAllReplyPointer->DeviceIdString);

    myPrintln(F("Device Address="), tJKAllReplyPointer->BoardAddress);

    myPrint(F("Total Battery Capacity[Ah]="), JKComputedData.TotalCapacityAmpereHour); // 0xAA
    myPrintln(F(", Low Capacity Alarm Percent="), tJKAllReplyPointer->LowCapacityAlarmPercent); // 0xB1

    myPrintlnSwap(F("Charging Cycles="), tJKAllReplyPointer->Cycles);
    myPrintlnSwap(F("Total Charging Cycle Capacity="), tJKAllReplyPointer->TotalBatteryCycleCapacity);
    myPrintSwap(F("# Battery Cells="), tJKAllReplyPointer->NumberOfBatteryCells); // 0x8A Total number of battery strings
    myPrintln(F(", Cell Count="), tJKAllReplyPointer->BatteryCellCount); // 0xA9 Battery string count settings
    Serial.println();
}

void JK_BMS::printBMSInfo() {
    JKReplyStruct *tJKAllReplyPointer = JKAllReplyPointer;

    myPrintln(F("Protocol Version Number="), tJKAllReplyPointer->ProtocolVersionNumber);

    Serial.print(F("Software Version Number="));
    tJKAllReplyPointer->TokenStartCurrentCalibration = '\0'; // set end of string token
    Serial.println(tJKAllReplyPointer->SoftwareVersionNumber);

    Serial.print(F("Modify Parameter Password="));
    tJKAllReplyPointer->TokenDedicatedChargerSwitchState = '\0'; // set end of string token
    Serial.println(tJKAllReplyPointer->ModifyParameterPassword);

    myPrintln(F("# External Temperature Sensors="), tJKAllReplyPointer->NumberOfTemperatureSensors); // 0x86

    Serial.println();
}

void JK_BMS::printMiscellaneousInfo() {
    JKReplyStruct *tJKAllReplyPointer = JKAllReplyPointer;

    myPrintlnSwap(F("Balance Starting Cell Voltage[mV]="), tJKAllReplyPointer->BalancingStartMillivolt);
    myPrintlnSwap(F("Balance Triggering Voltage Difference[mV]="), tJKAllReplyPointer->BalancingStartDifferentialMillivolt);

    myPrintlnSwap(F("Current Calibration[mA]="), tJKAllReplyPointer->CurrentCalibrationMilliampere);
    myPrintlnSwap(F("Sleep Wait Time[s]="), tJKAllReplyPointer->SleepWaitingTimeSeconds);

    myPrintln(F("Dedicated Charge Switch Active="), tJKAllReplyPointer->DedicatedChargerSwitchIsActive);
    myPrintln(F("Start Current Calibration State="), tJKAllReplyPointer->StartCurrentCalibration);
    myPrintlnSwap(F("Battery Actual Capacity[Ah]="), tJKAllReplyPointer->ActualBatteryCapacityAmpereHour);

    Serial.println();
}

/*
 * Token 0x8B. Prints alarm info only once for each change
 * Sets sAlarmIndexToShowOnLCD and string for LCD in sAlarmStringForLCD
 * Sets sAlarmJustGetsActive
 * ChargeOvervoltageAlarm is displayed as 'O' in printShortStateOnLCD()
 * ChargeUndervoltageAlarm is displayed as 'U' in printShortStateOnLCD()
 */
void JK_BMS::detectAndPrintAlarmInfo() {
    JKReplyStruct *tJKAllReplyPointer = JKAllReplyPointer;

    /*
     * Do it only once per change
     */
    if (tJKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord != lastJKReply.BatteryAlarmFlags.AlarmsAsWord) {
        if (tJKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord == NO_ALARM_WORD_CONTENT) {
            Serial.println(F("All alarms are cleared now"));
            AlarmActive = false;
        } else {
            AlarmJustGetsActive = true; // This forces the beep, which is NOT suppressed for over and undervoltage
            AlarmActive = true;
#if defined(HANDLE_MULTIPLE_BMS)
            JKMultiBMSData.anyAlarm = true;
#endif
            /*
             * Print alarm info
             */
            uint16_t tAlarms = swap(tJKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord);
            Serial.println(F("*** ALARM FLAGS ***"));
            Serial.print(sUpTimeString); // print uptime to have a timestamp for the alarm
            Serial.print(F(": Alarm bits=0x"));
            Serial.println(tAlarms, HEX);

            /*
             * Determine alarm index and string
             */
            uint16_t tAlarmMask = 1;
            for (uint_fast8_t i = 0; i < NUMBER_OF_DEFINED_ALARM_BITS; ++i) {
                if (tAlarms & tAlarmMask) {
                    Serial.print(F("Alarm bit=0b"));
                    Serial.print(tAlarmMask, BIN);
                    Serial.print(F(" -> "));
#if defined(USE_SERIAL_2004_LCD)
                        AlarmIndexToShowOnLCD = i;
#endif
                    Serial.println(
                            reinterpret_cast<const __FlashStringHelper*>((char*) (pgm_read_word(&JK_BMSAlarmStringsArray[i]))));
                }
                tAlarmMask <<= 1;
            }
            Serial.println();
            // print cell info in case of alarm
            printJKCellInfo();
        }
    }
}

void JK_BMS::printEnabledState(bool aIsEnabled) {
    if (aIsEnabled) {
        Serial.print(F(" enabled"));
    } else {
        Serial.print(F(" disabled"));
    }
}

void JK_BMS::printActiveState(bool aIsActive) {
    if (!aIsActive) {
        Serial.print(F(" not"));
    }
    Serial.print(F(" active"));
}

/*
 * Called exclusively once by processJK_BMSStatusFrame()
 */
void JK_BMS::printJKStaticInfo() {

#if defined(HANDLE_MULTIPLE_BMS)
    Serial.print(F("*** BMS "));
    Serial.print(NumberOfThisBMS);
    Serial.println(F(" INFO ***"));
#else
    Serial.println(F("*** BMS INFO ***"));
#endif
    printBMSInfo();

    Serial.println(F("*** BATTERY INFO ***"));
    printBatteryInfo();

    Serial.println(F("*** VOLTAGE PROTECTION INFO ***"));
    printVoltageProtectionInfo();

    Serial.println(F("*** TEMPERATURE PROTECTION INFO ***"));
    printTemperatureProtectionInfo();

    Serial.println(F("*** MISC INFO ***"));
    printMiscellaneousInfo();
}

void JK_BMS::computeUpTimeString() {
    if (JKAllReplyPointer->SystemWorkingMinutes != lastJKReply.SystemWorkingMinutes) {
        sUpTimeStringMinuteHasChanged = true;

        uint32_t tSystemWorkingMinutes = swap(JKAllReplyPointer->SystemWorkingMinutes);
// 1 kByte for sprintf  creates string "1234D23H12M"
        snprintf_P(sUpTimeString, sizeof(sUpTimeString), PSTR("%4uD%02uH%02uM"), (uint16_t) (tSystemWorkingMinutes / (60 * 24)),
                (uint16_t) ((tSystemWorkingMinutes / 60) % 24), (uint16_t) (tSystemWorkingMinutes % 60));
        if (sLastUpTimeTenthOfMinuteCharacter != sUpTimeString[8]) {
            sLastUpTimeTenthOfMinuteCharacter = sUpTimeString[8];
            sUpTimeStringTenthOfMinuteHasChanged = true;
        }
        if (sLastUpTimeHourCharacter != sUpTimeString[6]) {
            sLastUpTimeHourCharacter = sUpTimeString[6];
            sUpTimeStringHourHasChanged = true;
        }
    }
}

// Declaration here, because definition below is better for documentation
extern const char sCSVCaption[] PROGMEM;
/*
 * Print received data
 * Use converted cell voltage info from JKConvertedCellInfo
 * All other data are used unconverted and are therefore printed by swap() functions.
 */
void JK_BMS::printJKDynamicInfo() {
    JKReplyStruct *tJKAllReplyPointer = JKAllReplyPointer;

#if defined(ENABLE_MONITORING)
#  if defined(MONOTORING_PERIOD_FAST)
    // Print every dataset, every 2 seconds, and caption every minute
    printCSVLine();
    if (sUpTimeStringMinuteHasChanged) {
        sUpTimeStringMinuteHasChanged = false;
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(sCSVCaption));
    }
#  elif defined(MONOTORING_PERIOD_SLOW)
    // print CSV line every 10 minutes and CSV caption every hour
    if (sUpTimeStringTenthOfMinuteHasChanged) {
        sUpTimeStringTenthOfMinuteHasChanged = false;
        printCSVLine();
    }
#  else
// print CSV line every minute and CSV caption every 10 minutes
    if (sUpTimeStringMinuteHasChanged) {
        sUpTimeStringMinuteHasChanged = false;
        printCSVLine();
    }
#  endif
// Print +CSV line every percent of nominal battery capacity (TotalCapacityAmpereHour) for capacity to voltage graph
    if (abs(
                    JKLastPrintedData.BatteryCapacityAccumulator10MilliAmpere
                    - JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere) > getOnePercentCapacityAsAccumulator10Milliampere()) {
        JKLastPrintedData.BatteryCapacityAccumulator10MilliAmpere = JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere;
        printCSVLine('+');
    }

#endif

#  if defined(MONOTORING_PERIOD_SLOW)
    if (sUpTimeStringHourHasChanged) {
        sUpTimeStringHourHasChanged = false;
#  else
    /*
     * Print CSV caption, runtime, and Cell Info every ten minutes
     */
    if (sUpTimeStringTenthOfMinuteHasChanged) {
        sUpTimeStringTenthOfMinuteHasChanged = false;
#  endif
        Serial.println();
        Serial.print(F("Total Runtime: "));
        Serial.print(swap(JKAllReplyPointer->SystemWorkingMinutes));
        Serial.print(F(" minutes -> "));
        Serial.println(sUpTimeString);

        Serial.println(F("*** CELL INFO ***"));
        printJKCellInfo();
#if !defined(NO_CELL_STATISTICS)
        /*
         * Print cell statistics only if balancing count changed and is big enough for reasonable info
         */
        if (CellStatistics.LastPrintedBalancingCount
                != CellStatistics.BalancingCount&& CellStatistics.BalancingCount > MINIMUM_BALANCING_COUNT_FOR_DISPLAY) {
            CellStatistics.LastPrintedBalancingCount = CellStatistics.BalancingCount;
            Serial.println(F("*** CELL STATISTICS ***"));
            Serial.print(F("Total balancing time="));

            Serial.print(CellStatistics.BalancingCount * (MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS / 1000));
            Serial.print(F(" s -> "));
            Serial.print(sBalancingTimeString);
            // Append seconds
            char tString[4]; // "03S" is 3 bytes long
            snprintf_P(tString, sizeof(tString), PSTR("%02uS"), (uint16_t) (CellStatistics.BalancingCount % 30) * 2);
            Serial.println(tString);
            printJKCellStatisticsInfo();
        }
#endif

#if !defined(SUPPRESS_LIFEPO4_PLAUSI_WARNING)
        if (swap(tJKAllReplyPointer->CellOvervoltageProtectionMillivolt) > 3450) {
            // https://www.evworks.com.au/page/technical-information/lifepo4-care-guide-looking-after-your-lithium-batt/
            Serial.print(F("Warning: CellOvervoltageProtectionMillivolt value "));
            Serial.print(swap(tJKAllReplyPointer->CellOvervoltageProtectionMillivolt));
            Serial.println(F(" mV > 3450 mV is not recommended for LiFePO4 chemistry."));
            Serial.println(F("There is less than 1% extra capacity above 3.5V."));
        }
        if (swap(tJKAllReplyPointer->CellUndervoltageProtectionMillivolt) < 3000) {
            // https://batteryfinds.com/lifepo4-voltage-chart-3-2v-12v-24v-48v/
            Serial.print(F("Warning: CellUndervoltageProtectionMillivolt value "));
            Serial.print(swap(tJKAllReplyPointer->CellUndervoltageProtectionMillivolt));
            Serial.println(F(" mV < 3000 mV is not recommended for LiFePO4 chemistry."));
            Serial.println(F("There is less than 10% capacity below 3.0V and 20% capacity below 3.2V."));
        }
#endif
#if defined(ENABLE_MONITORING) && !defined(MONOTORING_PERIOD_FAST)
            /*
             * Print CSV caption every 10 minute
             */
            Serial.println(reinterpret_cast<const __FlashStringHelper*>(sCSVCaption));
#endif
    } // Print it every ten minutes

    /*
     * Temperatures
     * Print only if temperature changed more than 1 degree
     */
#if defined(LOCAL_DEBUG)
        Serial.print(F("TokenTemperaturePowerMosFet=0x"));
        Serial.println(JKAllReplyPointer->TokenTemperaturePowerMosFet, HEX);
#endif
    if (abs(JKComputedData.TemperaturePowerMosFet - JKLastPrintedData.TemperaturePowerMosFet) > 2
            || abs(JKComputedData.TemperatureSensor1 - JKLastPrintedData.TemperatureSensor1) > 2
            || abs(JKComputedData.TemperatureSensor2 - JKLastPrintedData.TemperatureSensor2) > 2) {
        myPrint(F("Temperature: Power MosFet="), JKComputedData.TemperaturePowerMosFet);
        myPrint(F(", Sensor 1="), JKComputedData.TemperatureSensor1);
        myPrintln(F(", Sensor 2="), JKComputedData.TemperatureSensor2);
        JKLastPrintedData.TemperaturePowerMosFet = JKComputedData.TemperaturePowerMosFet;
        JKLastPrintedData.TemperatureSensor1 = JKComputedData.TemperatureSensor1;
        JKLastPrintedData.TemperatureSensor2 = JKComputedData.TemperatureSensor2;
    }

    /*
     * SOC
     */
    if (tJKAllReplyPointer->SOCPercent != lastJKReply.SOCPercent) {
        /*
         * SOC changed
         */
        myPrint(F("SOC[%]="), tJKAllReplyPointer->SOCPercent);
        myPrintln(F(" -> Remaining Capacity[Ah]="), JKComputedData.RemainingCapacityAmpereHour);
    }

    /*
     * Charge and Discharge values
     * Print it only if NO LCD connected
     * CSV values are printed anyway every minute
     */
#if !defined(USE_SERIAL_2004_LCD)
    if (abs(JKComputedData.BatteryVoltageFloat - JKLastPrintedData.BatteryVoltageFloat) > 0.03
            || abs(JKComputedData.BatteryLoadPower - JKLastPrintedData.BatteryLoadPower) >= 50) {
        Serial.print(F("Battery Voltage[V]="));
        Serial.print(JKComputedData.BatteryVoltageFloat, 2);
        Serial.print(F(", Current[A]="));
        Serial.print(JKComputedData.BatteryLoadCurrentFloat, 2);
        myPrint(F(", Power[W]="), JKComputedData.BatteryLoadPower);
        Serial.print(F(", Difference to empty[V]="));
        Serial.println(JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 100.0, 2);
        JKLastPrintedData.BatteryVoltageFloat = JKComputedData.BatteryVoltageFloat;
        JKLastPrintedData.BatteryLoadPower = JKComputedData.BatteryLoadPower;
    }
#endif

    /*
     * Charge, Discharge and Balancer flags
     */
    auto tStatus = tJKAllReplyPointer->BMSStatus.StatusAsByteArray[1];
    auto tLastStatus = lastJKReply.BMSStatus.StatusAsByteArray[1];

    if ((tStatus ^ tLastStatus) & (STATUS_BYTE_DISCHARGE_ACTIVE_MASK | STATUS_BYTE_CHARGE_ACTIVE_MASK)) {
        /*
         * This happens quite seldom!
         */
        Serial.print(F("Charging MosFet"));
        printEnabledState(tJKAllReplyPointer->ChargeIsEnabled);
        Serial.print(',');
        printActiveState(tStatus & STATUS_BYTE_CHARGE_ACTIVE_MASK);
        Serial.print(F(" | Discharging MosFet"));
        printEnabledState(tJKAllReplyPointer->DischargeIsEnabled);
        Serial.print(',');
        printActiveState(tStatus & STATUS_BYTE_DISCHARGE_ACTIVE_MASK);
        Serial.print(F(" | Balancing")); // including balancer state to be complete :-)
        printEnabledState(tJKAllReplyPointer->BalancingIsEnabled);
        Serial.print(',');
        printActiveState(tStatus & STATUS_BYTE_BALANCER_MASK);
        Serial.println(); // printActiveState does no println()
    } else if ((tStatus ^ tLastStatus) & STATUS_BYTE_BALANCER_MASK) {
        /*
         * Only Balancer, since it happens very often
         */
        Serial.print(F("Balancing"));
        printActiveState(tStatus & STATUS_BYTE_BALANCER_MASK);
        if (tStatus & STATUS_BYTE_BALANCER_MASK) {
            printJKCellInfoOverview();
        } else {
            Serial.println(); // printActiveState does no println()
        }
    }
}

#if defined(ENABLE_MONITORING)
const char sCSVCaption[] PROGMEM
        = "Uptime[min];Cell_1;Cell_2;Cell_3;Cell_4;Cell_5;Cell_6;Cell_7;Cell_8;Cell_9;Cell_10;Cell_11;Cell_12;Cell_13;Cell_14;Cell_15;Cell_16;Voltage[mV];Current[A];Capacity[100mAh];SOC[%]";

/*
 * Fills sStringBuffer with CSV data
 * Can be used for SD Card also, otherwise direct printing would be more efficient.
 * E.g. 13753;185;185;186;185;185;206;185;214;183;185;201;186;186;186;185;186;5096;-5.65;36;54;1
 *
 */
void JK_BMS::setCSVString() {

    /*
     * Uptime minutes
     */
    uint_fast8_t tBufferIndex = snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%lu;"), (swap(JKAllReplyPointer->SystemWorkingMinutes)));

    /*
     * Individual cell voltages
     */
//    for (uint8_t i = 0; i < JKConvertedCellInfo.ActualNumberOfCellInfoEntries; ++i) {
    if (sizeof(sStringBuffer) > (5 * 16)) {
        for (uint8_t i = 0; i < 16; ++i) { // only 16 fits into sStringBuffer[90]
            // check for valid data, otherwise we will get a string buffer overflow
            if (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt > 2500) {
                tBufferIndex += sprintf_P(&sStringBuffer[tBufferIndex], PSTR("%d;"),
                        (int16_t) (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt - 3000)); // difference may become negative
            }
        }
    }

    if ((uint_fast8_t) (tBufferIndex + 17) >= sizeof(sStringBuffer)) {
        Serial.print(F("String buffer overflow, tBufferIndex="));
        Serial.print(tBufferIndex);
        Serial.print(F(" sizeof(sStringBuffer)="));
        Serial.println(sizeof(sStringBuffer));
        sStringBuffer[0] = '\0';
    } else {
        // maximal string 50960;-20.00;3200;100;1 -> 18 characters
        char tCurrentAsFloatString[7];
        dtostrf(JKComputedData.BatteryLoadCurrentFloat, 4, 2, &tCurrentAsFloatString[0]);
        sprintf_P(&sStringBuffer[tBufferIndex], PSTR("%u;%s;%ld;%d"), JKComputedData.BatteryVoltage10Millivolt * 10,
                tCurrentAsFloatString,
                JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere / (AMPERE_HOUR_AS_ACCUMULATOR_10_MILLIAMPERE / 10), /* 100mAh units*/
                JKAllReplyPointer->SOCPercent);
    }
}

void JK_BMS::printCSVLine(char aLeadingChar) {
    if (aLeadingChar != '\0') {
        Serial.print(aLeadingChar);
    }
    Serial.print(F("CSV: "));
    setCSVString();
    Serial.println(sStringBuffer);
}
#endif // defined(ENABLE_MONITORING)

#include "LocalDebugLevelEnd.h"
#endif // _JK_BMS_HPP
