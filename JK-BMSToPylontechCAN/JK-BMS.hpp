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

#include "JK-BMS.h"
#if !defined(USE_NO_LCD)
#include "JK-BMS_LCD.h" // for sLCDDisplayPageNumber and JK_BMS_PAGE_OVERVIEW
#endif

JKLastReplyStruct lastJKReply;

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
uint16_t sReplyFrameBufferIndex = 0;        // Index of next byte to write to array, except for last byte received. Starting with 0.
uint16_t sReplyFrameLength;                 // Received length of frame
uint8_t JKReplyFrameBuffer[350];            // The raw big endian data as received from JK BMS.

JKComputedDataStruct JKComputedData;            // All derived converted and computed data useful for display
JKLastPrintedDataStruct JKLastPrintedData;      // For detecting changes for printing

#define LENGTH_OF_UPTIME_STRING 11
char sUpTimeString[LENGTH_OF_UPTIME_STRING + 1]; // "9999D23H12M" is 11 bytes long
char sBalancingTimeString[11] = { ' ', ' ', '0', 'D', '0', '0', 'H', '0', '0', 'M', '\0' };    // "999D23H12M" is 10 bytes long
bool sUpTimeStringMinuteHasChanged;
bool sUpTimeStringTenthOfMinuteHasChanged;
char sLastUpTimeTenthOfMinuteCharacter;     // For detecting changes in string and setting sUpTimeStringTenthOfMinuteHasChanged
bool sUpTimeStringHourHasChanged;
char sLastUpTimeHourCharacter;              // For setting sUpTimeStringHourHasChanged

JKConvertedCellInfoStruct JKConvertedCellInfo;  // The converted little endian cell voltage data
#if !defined(NO_CELL_STATISTICS)
struct CellStatisticsStruct CellStatistics;
#endif //NO_CELL_STATISTICS

/*
 * The JKFrameAllDataStruct starts behind the header + cell data header 0x79 + CellInfoSize
 * + the variable length cell data 3 bytes per cell, (CellInfoSize is contained in JKReplyFrameBuffer[12])
 */
JKReplyStruct *sJKFAllReplyPointer = reinterpret_cast<JKReplyStruct*>(&JKReplyFrameBuffer[JK_BMS_FRAME_HEADER_LENGTH + 2 + 48]); // assume 16 cells

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

/*
 * Since over and undervoltage is indicated by O or U in state info, it is not necessary to switch to Overview page
 */
//#define ENABLE_OVER_AND_UNDER_VOLTAGE_WARNING_ON_LCD // Enables switching to Overview page and showing over- and undervoltage data.
#define MASK_OF_CHARGING_AND_DISCHARGING_OVERVOLTAGE_ALARM_UNSWAPPED    0x0C00
// Required for displaying specific info for this alarms
#define INDEX_OF_CHARGING_OVERVOLTAGE_ALARM                 2
#define MASK_OF_CHARGING_OVERVOLTAGE_ALARM_UNSWAPPED        0x0800
#define INDEX_OF_DISCHARGING_UNDERVOLTAGE_ALARM             3
#define MASK_OF_DISCHARGING_UNDERVOLTAGE_ALARM_UNSWAPPED    0x0400
#define INDEX_NO_ALARM                                      0xFF

const char *const JK_BMSAlarmStringsArray[NUMBER_OF_DEFINED_ALARM_BITS] PROGMEM = { lowCapacity, MosFetOvertemperature,
        chargingOvervoltage, dischargingUndervoltage, Sensor2Overtemperature, chargingOvercurrent, dischargingOvercurrent,
        CellVoltageDifference, Sensor1Overtemperature, Sensor2LowLemperature, CellOvervoltage, CellUndervoltage, _309AProtection,
        _309BProtection };

/*
 * Flags for alarm handling
 */
/*
 * sAlarmJustGetsActive is set and reset by detectAndPrintAlarmInfo() and reset by checkButtonPressForLCD() and beep handling.
 * Can also be set to true if 2 alarms are active and one of them gets inactive. If true beep (with optional timeout) is generated.
 */
bool sAlarmJustGetsActive = false; // True if alarm bits changed and any alarm is still active. False if alarm bits changed and no alarm is active.
#if defined(USE_SERIAL_2004_LCD)
uint8_t sAlarmIndexToShowOnLCD = INDEX_NO_ALARM; // Index of current alarm to show with Alarm / Overview page. Set by detectAndPrintAlarmInfo() and reset on page switch.
bool sPrintAlarmInfoOnlyOnce = false; // True -> fill main part of LCD alarm page only once. Is set by detectAndPrintAlarmInfo().
bool sShowAlarmInsteadOfOverview = false; // is reset on button press
#endif
#if defined(SUPPRESS_CONSECUTIVE_SAME_ALARMS)
uint16_t sLastActiveAlarmsAsWord;   // Used to disable new alarm beep on recurring of same alarm
#define SUPPRESS_CONSECUTIVE_SAME_ALARMS_TIMEOUT_SECONDS    3600 // Allow consecutive same alarms after 1 hour of no alarm
uint16_t sNoAlarmCounter; // Counts no alarms in order to reset suppression of consecutive alarms.
#endif

/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/*
 * 1.85 ms
 */
void requestJK_BMSStatusFrame(SoftwareSerialTX *aSerial, bool aDebugModeActive) {
    if (aDebugModeActive) {
        Serial.println();
        Serial.println(F("Send requestFrame with TxToJKBMS"));
        printBufferHex(JKRequestStatusFrame, sizeof(JKRequestStatusFrame));
    }
    Serial.flush();

    for (uint8_t i = 0; i < sizeof(JKRequestStatusFrame); ++i) {
        aSerial->write(JKRequestStatusFrame[i]);
    }
}

void initJKReplyFrameBuffer() {
    sReplyFrameBufferIndex = 0;
}

/*
 * Prints formatted reply buffer raw content
 */
void printJKReplyFrameBuffer() {
    uint8_t *tBufferAddress = JKReplyFrameBuffer;
    printBufferHex(tBufferAddress, JK_BMS_FRAME_HEADER_LENGTH);

    tBufferAddress += JK_BMS_FRAME_HEADER_LENGTH;
    printBufferHex(tBufferAddress, JK_BMS_FRAME_CELL_INFO_LENGTH);

    tBufferAddress += JK_BMS_FRAME_CELL_INFO_LENGTH;
    uint8_t tCellInfoLength = JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH];
    printBufferHex(tBufferAddress, tCellInfoLength); // Cell info
    Serial.println();

    tBufferAddress += tCellInfoLength;
    int16_t tRemainingDataLength = ((int16_t) sReplyFrameBufferIndex + 1)
            - (JK_BMS_FRAME_HEADER_LENGTH + JK_BMS_FRAME_CELL_INFO_LENGTH + JK_BMS_FRAME_TRAILER_LENGTH + tCellInfoLength);
    if (tRemainingDataLength <= 0) {
        return;
    }
    printBufferHex(tBufferAddress, tRemainingDataLength);

    tBufferAddress += tRemainingDataLength;
    printBufferHex(tBufferAddress, JK_BMS_FRAME_TRAILER_LENGTH); // Trailer
}

#define JK_BMS_RECEIVE_OK           0
#define JK_BMS_RECEIVE_FINISHED     1
#define JK_BMS_RECEIVE_        2
/*
 * Is assumed to be called if Serial.available() is true
 * @return JK_BMS_RECEIVE_OK, if still receiving; JK_BMS_RECEIVE_FINISHED, if complete frame was successfully read
 *          JK_BMS_RECEIVE_, if frame has s.
 * Reply starts 0.18 ms to 0.45 ms after request was received
 */
uint8_t readJK_BMSStatusFrameByte() {
    uint8_t tReceivedByte = Serial.read();
    JKReplyFrameBuffer[sReplyFrameBufferIndex] = tReceivedByte;

    /*
     * Plausi check and get length of frame
     */
    if (sReplyFrameBufferIndex == 0) {
        // start byte 1
        if (tReceivedByte != JK_FRAME_START_BYTE_0) {
            Serial.println(F(" start frame token != 0x4E"));
            return JK_BMS_RECEIVE_;
        }
    } else if (sReplyFrameBufferIndex == 1) {
        if (tReceivedByte != JK_FRAME_START_BYTE_1) {
            //
            return JK_BMS_RECEIVE_;
        }

    } else if (sReplyFrameBufferIndex == 3) {
        // length of frame
        sReplyFrameLength = (JKReplyFrameBuffer[2] << 8) + tReceivedByte;

    } else if (sReplyFrameLength > MINIMAL_JK_BMS_FRAME_LENGTH && sReplyFrameBufferIndex == sReplyFrameLength - 3) {
        // Check end token 0x68
        if (tReceivedByte != JK_FRAME_END_BYTE) {
            Serial.print(F(" end frame token 0x"));
            Serial.print(tReceivedByte, HEX);
            Serial.print(F(" at index"));
            Serial.print(sReplyFrameBufferIndex);
            Serial.print(F(" is != 0x68. sReplyFrameLength= "));
            Serial.print(sReplyFrameLength);
            Serial.print(F(" | 0x"));
            Serial.println(sReplyFrameLength, HEX);
            return JK_BMS_RECEIVE_;
        }

    } else if (sReplyFrameLength > MINIMAL_JK_BMS_FRAME_LENGTH && sReplyFrameBufferIndex == sReplyFrameLength + 1) {
        /*
         * Frame received completely, perform checksum check
         */
        uint16_t tComputedChecksum = 0;
        for (uint16_t i = 0; i < sReplyFrameLength - 2; i++) {
            tComputedChecksum = tComputedChecksum + JKReplyFrameBuffer[i];
        }
        uint16_t tReceivedChecksum = (JKReplyFrameBuffer[sReplyFrameLength] << 8) + tReceivedByte;
        if (tComputedChecksum != tReceivedChecksum) {
            Serial.print(F("Checksum , computed checksum=0x"));
            Serial.print(tComputedChecksum, HEX);
            Serial.print(F(", received checksum=0x"));
            Serial.println(tReceivedChecksum, HEX);

            return JK_BMS_RECEIVE_;
        } else {
            return JK_BMS_RECEIVE_FINISHED;
        }
    }
    sReplyFrameBufferIndex++;
    return JK_BMS_RECEIVE_OK;
}

/*
 * Highest bit is set means charge
 * @return Charge is positive, discharge is negative
 */
int16_t getCurrent(uint16_t aJKRAWCurrent) {
    uint16_t tCurrent = swap(aJKRAWCurrent);
    if (tCurrent == 0 || (tCurrent & 0x8000) == 0x8000) {
        // Charge - NO two's complement!
        return (tCurrent & 0x7FFF);
    }
    // discharge
    return tCurrent * -1;

}

int16_t getJKTemperature(uint16_t aJKRAWTemperature) {
    uint16_t tTemperature = swap(aJKRAWTemperature);
    if (tTemperature <= 100) {
        return tTemperature;
    }
    return 100 - tTemperature;
}

int32_t getOnePercentCapacityAsAccumulator10Milliampere() {
    return (AMPERE_HOUR_AS_ACCUMULATOR_10_MILLIAMPERE / 100) * JKComputedData.TotalCapacityAmpereHour;
}

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
void fillJKConvertedCellInfo() {
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
    uint16_t tMinimumMillivolt = 0xFFFF;
    uint16_t tMaximumMillivolt = 0;

    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        tJKCellInfoReplyPointer++;                                  // Skip Cell number
        uint8_t tHighByte = *tJKCellInfoReplyPointer++;             // Copy CellMillivolt
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

#if !defined(NO_CELL_STATISTICS) && !defined(USE_NO_LCD)
    /*
     * Mark and count minimum and maximum cell voltages
     */
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt == tMinimumMillivolt) {
            JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_MINIMUM;
            if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
                CellStatistics.CellMinimumArray[i]++; // count for statistics
            }
        } else if (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt == tMaximumMillivolt) {
            JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_MAXIMUM;
            if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
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
void printJKCellStatisticsInfo() {
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
        sprintf_P(tStringBuffer, PSTR("%2u=%2u %% |%5u, "), i + 1, CellStatistics.CellMinimumPercentageArray[i],
                CellStatistics.CellMinimumArray[i]);
        Serial.print(tStringBuffer);
    }
    Serial.println();

    Serial.println(F("Cell Maximum percentages"));
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (i != 0 && (i % 8) == 0) {
            Serial.println();
        }
        sprintf_P(tStringBuffer, PSTR("%2u=%2u %% |%5u, "), i + 1, CellStatistics.CellMaximumPercentageArray[i],
                CellStatistics.CellMaximumArray[i]);
        Serial.print(tStringBuffer);
    }
    Serial.println();

    Serial.println();
}

#endif // NO_CELL_STATISTICS

void initializeComputedData() {
    // Initialize capacity accumulator with sensible value
    JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere = (AMPERE_HOUR_AS_ACCUMULATOR_10_MILLIAMPERE / 100)
            * sJKFAllReplyPointer->SOCPercent * JKComputedData.TotalCapacityAmpereHour;
}

void fillJKComputedData() {
    JKComputedData.TemperaturePowerMosFet = getJKTemperature(sJKFAllReplyPointer->TemperaturePowerMosFet);
    int16_t tMaxTemperature = JKComputedData.TemperaturePowerMosFet;

    JKComputedData.TemperatureSensor1 = getJKTemperature(sJKFAllReplyPointer->TemperatureSensor1);
    if (tMaxTemperature < JKComputedData.TemperatureSensor1) {
        tMaxTemperature = JKComputedData.TemperatureSensor1;
    }

    JKComputedData.TemperatureSensor2 = getJKTemperature(sJKFAllReplyPointer->TemperatureSensor2);
    if (tMaxTemperature < JKComputedData.TemperatureSensor2) {
        tMaxTemperature = JKComputedData.TemperatureSensor2;
    }
    JKComputedData.TemperatureMaximum = tMaxTemperature;

    JKComputedData.TotalCapacityAmpereHour = swap(sJKFAllReplyPointer->TotalCapacityAmpereHour);
// 16 bit multiplication gives overflow at 640 Ah
    JKComputedData.RemainingCapacityAmpereHour = ((uint32_t) JKComputedData.TotalCapacityAmpereHour
            * sJKFAllReplyPointer->SOCPercent) / 100;

// Two values which are zero during JK-BMS startup for around 16 seconds
    JKComputedData.BMSIsStarting = (sJKFAllReplyPointer->SOCPercent == 0 && sJKFAllReplyPointer->Cycles == 0);

    JKComputedData.BatteryFullVoltage10Millivolt = swap(sJKFAllReplyPointer->BatteryOvervoltageProtection10Millivolt);
    JKComputedData.BatteryVoltage10Millivolt = swap(sJKFAllReplyPointer->Battery10Millivolt);
//    JKComputedData.BatteryVoltageDifferenceToFull10Millivolt = JKComputedData.BatteryFullVoltage10Millivolt
//            - JKComputedData.BatteryVoltage10Millivolt;

    JKComputedData.BatteryEmptyVoltage10Millivolt = swap(sJKFAllReplyPointer->BatteryUndervoltageProtection10Millivolt);
    JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt = JKComputedData.BatteryVoltage10Millivolt
            - JKComputedData.BatteryEmptyVoltage10Millivolt;

    JKComputedData.BatteryVoltageFloat = JKComputedData.BatteryVoltage10Millivolt / 100.0;

    JKComputedData.Battery10MilliAmpere = getCurrent(sJKFAllReplyPointer->Battery10MilliAmpere);
    JKComputedData.BatteryLoadCurrentFloat = JKComputedData.Battery10MilliAmpere / 100.0;
    JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere += JKComputedData.Battery10MilliAmpere;
    if (lastJKReply.SOCPercent == 0 && sJKFAllReplyPointer->SOCPercent == 1) {
        JK_INFO_PRINTLN(F("Reset capacity to 1%"));
        // Reset capacity at transition from 0 to 1
        JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere = getOnePercentCapacityAsAccumulator10Milliampere();
        JKLastPrintedData.BatteryCapacityAccumulator10MilliAmpere = JKComputedData.BatteryCapacityAsAccumulator10MilliAmpere;
    }

//    Serial.print("Battery10MilliAmpere=0x");
//    Serial.print(sJKFAllReplyPointer->Battery10MilliAmpere, HEX);
//    Serial.print(" Battery10MilliAmpere swapped=0x");
//    Serial.println(swap(sJKFAllReplyPointer->Battery10MilliAmpere), HEX);
//    Serial.print(" Battery10MilliAmpere=");
//    Serial.print(JKComputedData.Battery10MilliAmpere);
//    Serial.print(" BatteryLoadCurrent=");
//    Serial.println(JKComputedData.BatteryLoadCurrentFloat);

    JKComputedData.BatteryLoadPower = JKComputedData.BatteryVoltageFloat * JKComputedData.BatteryLoadCurrentFloat;

#if !defined(NO_CELL_STATISTICS)
    /*
     * Increment BalancingCount and fill sBalancingTimeString
     */
    if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
        CellStatistics.BalancingCount++;
        sprintf_P(sBalancingTimeString, PSTR("%3uD%02uH%02uM"), (uint16_t) (CellStatistics.BalancingCount / (60 * 24 * 30UL)),
                (uint16_t) ((CellStatistics.BalancingCount / (60 * 30)) % 24),
                (uint16_t) (CellStatistics.BalancingCount / 30) % 60);
    }
#endif // NO_CELL_STATISTICS
}

void printJKCellInfoOverview() {
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
void printJKCellInfo() {
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

void printVoltageProtectionInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;
    /*
     * Voltage protection
     */
    Serial.print(F("Battery Overvoltage Protection[V]="));
    Serial.print(JKComputedData.BatteryFullVoltage10Millivolt / 100.0, 2);
    Serial.print(F(", Undervoltage="));
    Serial.println(JKComputedData.BatteryEmptyVoltage10Millivolt / 100.0, 2);

    myPrintSwap(F("Cell Overvoltage Protection[mV]="), tJKFAllReplyPointer->CellOvervoltageProtectionMillivolt);
    myPrintSwap(F(", Recovery="), tJKFAllReplyPointer->CellOvervoltageRecoveryMillivolt);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReplyPointer->CellOvervoltageDelaySeconds);

    myPrintSwap(F("Cell Undervoltage Protection[mV]="), tJKFAllReplyPointer->CellUndervoltageProtectionMillivolt);
    myPrintSwap(F(", Recovery="), tJKFAllReplyPointer->CellUndervoltageRecoveryMillivolt);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReplyPointer->CellUndervoltageDelaySeconds);

    myPrintlnSwap(F("Cell Voltage Difference Protection[mV]="), tJKFAllReplyPointer->VoltageDifferenceProtectionMillivolt);

    myPrintSwap(F("Discharging Overcurrent Protection[A]="), tJKFAllReplyPointer->DischargeOvercurrentProtectionAmpere);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReplyPointer->DischargeOvercurrentDelaySeconds);

    myPrintSwap(F("Charging Overcurrent Protection[A]="), tJKFAllReplyPointer->ChargeOvercurrentProtectionAmpere);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReplyPointer->ChargeOvercurrentDelaySeconds);
    Serial.println();
}

void printTemperatureProtectionInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;
    /*
     * Temperature protection
     */
    myPrintSwap(F("Power MosFet Temperature Protection="), tJKFAllReplyPointer->PowerMosFetTemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReplyPointer->PowerMosFetRecoveryTemperature);

    myPrintSwap(F("Sensor1 Temperature Protection="), tJKFAllReplyPointer->Sensor1TemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReplyPointer->Sensor1RecoveryTemperature);

    myPrintlnSwap(F("Sensor1 to Sensor2 Temperature Difference Protection="),
            tJKFAllReplyPointer->BatteryDifferenceTemperatureProtection);

    myPrintSwap(F("Charge Overtemperature Protection="), tJKFAllReplyPointer->ChargeOvertemperatureProtection);
    myPrintlnSwap(F(", Discharge="), tJKFAllReplyPointer->DischargeOvertemperatureProtection);

    myPrintSwap(F("Charge Undertemperature Protection="), tJKFAllReplyPointer->ChargeUndertemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReplyPointer->ChargeRecoveryUndertemperature);

    myPrintSwap(F("Discharge Undertemperature Protection="), tJKFAllReplyPointer->DischargeUndertemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReplyPointer->DischargeRecoveryUndertemperature);
    Serial.println();
}

void printBatteryInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;

    Serial.print(F("Manufacturer Date="));
    tJKFAllReplyPointer->TokenSystemWorkingMinutes = '\0'; // Set end of string token
    Serial.println(tJKFAllReplyPointer->ManufacturerDate);

    Serial.print(F("Manufacturer Id=")); // First 8 characters of the manufacturer id entered in the app field "User Private Data"
    tJKFAllReplyPointer->TokenProtocolVersionNumber = '\0'; // Set end of string token
    Serial.println(tJKFAllReplyPointer->ManufacturerId);

    Serial.print(F("Device ID String=")); // First 8 characters of ManufacturerId
    tJKFAllReplyPointer->TokenManufacturerDate = '\0'; // Set end of string token
    Serial.println(tJKFAllReplyPointer->DeviceIdString);

    myPrintln(F("Device Address="), tJKFAllReplyPointer->BoardAddress);

    myPrint(F("Total Battery Capacity[Ah]="), JKComputedData.TotalCapacityAmpereHour); // 0xAA
    myPrintln(F(", Low Capacity Alarm Percent="), tJKFAllReplyPointer->LowCapacityAlarmPercent); // 0xB1

    myPrintlnSwap(F("Charging Cycles="), tJKFAllReplyPointer->Cycles);
    myPrintlnSwap(F("Total Charging Cycle Capacity="), tJKFAllReplyPointer->TotalBatteryCycleCapacity);
    myPrintSwap(F("# Battery Cells="), tJKFAllReplyPointer->NumberOfBatteryCells); // 0x8A Total number of battery strings
    myPrintln(F(", Cell Count="), tJKFAllReplyPointer->BatteryCellCount); // 0xA9 Battery string count settings
    Serial.println();
}

void printBMSInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;

    myPrintln(F("Protocol Version Number="), tJKFAllReplyPointer->ProtocolVersionNumber);

    Serial.print(F("Software Version Number="));
    tJKFAllReplyPointer->TokenStartCurrentCalibration = '\0'; // set end of string token
    Serial.println(tJKFAllReplyPointer->SoftwareVersionNumber);

    Serial.print(F("Modify Parameter Password="));
    tJKFAllReplyPointer->TokenDedicatedChargerSwitchState = '\0'; // set end of string token
    Serial.println(tJKFAllReplyPointer->ModifyParameterPassword);

    myPrintln(F("# External Temperature Sensors="), tJKFAllReplyPointer->NumberOfTemperatureSensors); // 0x86

    Serial.println();
}

void printMiscellaneousInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;

    myPrintlnSwap(F("Balance Starting Cell Voltage[mV]="), tJKFAllReplyPointer->BalancingStartMillivolt);
    myPrintlnSwap(F("Balance Triggering Voltage Difference[mV]="), tJKFAllReplyPointer->BalancingStartDifferentialMillivolt);
    Serial.println();
    myPrintlnSwap(F("Current Calibration[mA]="), tJKFAllReplyPointer->CurrentCalibrationMilliampere);
    myPrintlnSwap(F("Sleep Wait Time[s]="), tJKFAllReplyPointer->SleepWaitingTimeSeconds);
    Serial.println();
    myPrintln(F("Dedicated Charge Switch Active="), tJKFAllReplyPointer->DedicatedChargerSwitchIsActive);
    myPrintln(F("Start Current Calibration State="), tJKFAllReplyPointer->StartCurrentCalibration);
    myPrintlnSwap(F("Battery Actual Capacity[Ah]="), tJKFAllReplyPointer->ActualBatteryCapacityAmpereHour);
    Serial.println();
}

/*
 * Token 0x8B. Prints alarm info only once for each change
 * Sets sAlarmIndexToShowOnLCD and string for LCD in sAlarmStringForLCD
 * Sets sAlarmJustGetsActive
 * ChargeOvervoltageAlarm is displayed as 'O' in printShortStateOnLCD()
 * ChargeUndervoltageAlarm is displayed as 'U' in printShortStateOnLCD()
 */
void detectAndPrintAlarmInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;

#if defined(SUPPRESS_CONSECUTIVE_SAME_ALARMS)
    if (tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord == NO_ALARM_WORD_CONTENT) {
        sNoAlarmCounter++; // integer overflow does not really matter here
        // sNoAlarmCounter == 1800 - Allow consecutive same alarms after 1 hour of no alarm
        if (sNoAlarmCounter
                == (SUPPRESS_CONSECUTIVE_SAME_ALARMS_TIMEOUT_SECONDS * MILLIS_IN_ONE_SECOND) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
            sLastActiveAlarmsAsWord = NO_ALARM_WORD_CONTENT; // Reset LastActiveAlarmsAsWord
        }
    }
#endif

    /*
     * Do it only once per change
     */
    if (tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord != lastJKReply.AlarmUnion.AlarmsAsWord) {
        if (tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord == NO_ALARM_WORD_CONTENT) {
            Serial.println(F("All alarms are cleared now"));
            sAlarmJustGetsActive = false;
        } else {
#if defined(SUPPRESS_CONSECUTIVE_SAME_ALARMS)
            sNoAlarmCounter = 0; // reset counter
            // only start actions if new alarm is different
            if(sLastActiveAlarmsAsWord == tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord){
                sDoAlarmOrTimeoutBeep = true; // do only one beep for recurring alarms
            } else {
                /*
                 * At least one alarm is active and it is not the last alarm already processed
                 */
                sLastActiveAlarmsAsWord = tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord;
#endif
#if defined(USE_SERIAL_2004_LCD)
#  if defined(ENABLE_OVER_AND_UNDER_VOLTAGE_WARNING_ON_LCD)
                sPrintAlarmInfoOnlyOnce = true; // This forces a switch to Alarm page
#  else
                if((tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord & ~MASK_OF_CHARGING_AND_DISCHARGING_OVERVOLTAGE_ALARM_UNSWAPPED)
                        || sLCDDisplayPageNumber == JK_BMS_PAGE_OVERVIEW) {
                    // Other than over  / undervoltage alarm bit is active
                    sPrintAlarmInfoOnlyOnce = true; // This forces display on Alarm page
                }
#  endif
#endif
            sAlarmJustGetsActive = true; // This forces the beep
#if defined(SUPPRESS_CONSECUTIVE_SAME_ALARMS)
            }
#endif
            /*
             * Print alarm info
             */
            uint16_t tAlarms = swap(tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord);
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
                    sAlarmIndexToShowOnLCD = i;
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

void printEnabledState(bool aIsEnabled) {
    if (aIsEnabled) {
        Serial.print(F(" enabled"));
    } else {
        Serial.print(F(" disabled"));
    }
}

void printActiveState(bool aIsActive) {
    if (!aIsActive) {
        Serial.print(F(" not"));
    }
    Serial.print(F(" active"));
}

/*
 * Called exclusively once by processJK_BMSStatusFrame()
 */
void printJKStaticInfo() {

    Serial.println(F("*** BMS INFO ***"));
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

void computeUpTimeString() {
    if (sJKFAllReplyPointer->SystemWorkingMinutes != lastJKReply.SystemWorkingMinutes) {
        sUpTimeStringMinuteHasChanged = true;

        uint32_t tSystemWorkingMinutes = swap(sJKFAllReplyPointer->SystemWorkingMinutes);
// 1 kByte for sprintf  creates string "1234D23H12M"
        sprintf_P(sUpTimeString, PSTR("%4uD%02uH%02uM"), (uint16_t) (tSystemWorkingMinutes / (60 * 24)),
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
void printJKDynamicInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;

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
        Serial.print(swap(sJKFAllReplyPointer->SystemWorkingMinutes));
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
            sprintf_P(tString, PSTR("%02uS"), (uint16_t) (CellStatistics.BalancingCount % 30) * 2);
            Serial.println(tString);
            printJKCellStatisticsInfo();
        }
#endif

#if !defined(SUPPRESS_LIFEPO4_PLAUSI_WARNING)
        if (swap(tJKFAllReplyPointer->CellOvervoltageProtectionMillivolt) > 3450) {
            // https://www.evworks.com.au/page/technical-information/lifepo4-care-guide-looking-after-your-lithium-batt/
            Serial.print(F("Warning: CellOvervoltageProtectionMillivolt value "));
            Serial.print(swap(tJKFAllReplyPointer->CellOvervoltageProtectionMillivolt));
            Serial.println(F(" mV > 3450 mV is not recommended for LiFePO4 chemistry."));
            Serial.println(F("There is less than 1% extra capacity above 3.5V."));
        }
        if (swap(tJKFAllReplyPointer->CellUndervoltageProtectionMillivolt) < 3000) {
            // https://batteryfinds.com/lifepo4-voltage-chart-3-2v-12v-24v-48v/
            Serial.print(F("Warning: CellUndervoltageProtectionMillivolt value "));
            Serial.print(swap(tJKFAllReplyPointer->CellUndervoltageProtectionMillivolt));
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
    Serial.println(sJKFAllReplyPointer->TokenTemperaturePowerMosFet, HEX);
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
    if (tJKFAllReplyPointer->SOCPercent != lastJKReply.SOCPercent) {
        /*
         * SOC changed
         */
        myPrint(F("SOC[%]="), tJKFAllReplyPointer->SOCPercent);
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
    if (tJKFAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive != lastJKReply.BMSStatus.StatusBits.ChargeMosFetActive
            || tJKFAllReplyPointer->BMSStatus.StatusBits.DischargeMosFetActive
                    != lastJKReply.BMSStatus.StatusBits.DischargeMosFetActive) {
        /*
         * This happens quite seldom!
         */
        Serial.print(F("Charging MosFet"));
        printEnabledState(tJKFAllReplyPointer->ChargeIsEnabled);
        Serial.print(',');
        printActiveState(tJKFAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive);
        Serial.print(F(" | Discharging MosFet"));
        printEnabledState(tJKFAllReplyPointer->DischargeIsEnabled);
        Serial.print(',');
        printActiveState(tJKFAllReplyPointer->BMSStatus.StatusBits.DischargeMosFetActive);
        Serial.print(F(" | Balancing")); // including balancer state to be complete :-)
        printEnabledState(tJKFAllReplyPointer->BalancingIsEnabled);
        Serial.print(',');
        printActiveState(tJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive);
        Serial.println(); // printActiveState does no println()
    } else if (tJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive != lastJKReply.BMSStatus.StatusBits.BalancerActive) {
        /*
         * Only Balancer, since it happens very often
         */
        Serial.print(F("Balancing"));
        printActiveState(tJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive);
        if (tJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
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
void setCSVString() {

    /*
     * Uptime minutes
     */
    uint_fast8_t tBufferIndex = sprintf_P(sStringBuffer, PSTR("%lu;"), (swap(sJKFAllReplyPointer->SystemWorkingMinutes)));

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
                sJKFAllReplyPointer->SOCPercent);
    }
}

void printCSVLine(char aLeadingChar) {
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
