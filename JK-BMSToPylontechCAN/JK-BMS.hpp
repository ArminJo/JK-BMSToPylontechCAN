/*
 * JK-BMS.hpp
 *
 * Functions to read, convert and print JK-BMS data
 *
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

JKLastReplyStruct lastJKReply;

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

// see JK Communication protocol.pdf http://www.jk-bms.com/Upload/2022-05-19/1621104621.pdf
uint8_t JKRequestStatusFrame[21] = { 0x4E, 0x57 /*4E 57 = StartOfFrame*/, 0x00, 0x13 /*0x13 | 19 = LengthOfFrame*/, 0x00, 0x00,
        0x00, 0x00/*BMS ID, highest byte is default 00*/, 0x06/*Function 1=Activate, 3=ReadIdentifier, 6=ReadAllData*/,
        0x03/*Frame source 0=BMS, 1=Bluetooth, 2=GPRS, 3=PC*/, 0x00 /*TransportType 0=Request, 1=Response, 2=BMSActiveUpload*/,
        0x00/*0=ReadAllData or commandToken*/, 0x00, 0x00, 0x00,
        0x00/*RecordNumber High byte is random code, low 3 bytes is record number*/, JK_FRAME_END_BYTE/*0x68 = EndIdentifier*/,
        0x00, 0x00, 0x01, 0x29 /*Checksum, high 2 bytes for checksum not yet enabled -> 0, low 2 Byte for checksum*/};
//uint8_t JKrequestStatusFrameOld[] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };

uint16_t sReplyFrameBufferIndex = 0;        // Index of next byte to write to array, except for last byte received. Starting with 0.
uint16_t sReplyFrameLength;                 // Received length of frame
uint8_t JKReplyFrameBuffer[350];            // The raw big endian data as received from JK BMS
bool sJKBMSFrameHasTimeout;                 // If true, timeout message or CAN Info page is displayed.

JKComputedDataStruct JKComputedData;            // All derived converted and computed data useful for display
JKComputedDataStruct lastJKComputedData;        // For detecting changes
char sUpTimeString[12];                         // "9999D23H12M" is 11 bytes long
char sBalancingTimeString[11] = { ' ', ' ', '0', 'D', '0', '0', 'H', '0', '0', 'M', '\0' };    // "999D23H12M" is 10 bytes long
bool sUpTimeStringMinuteHasChanged;
bool sUpTimeStringTenthOfMinuteHasChanged;
char sLastUpTimeTenthOfMinuteCharacter;     // For detecting changes in string and setting sUpTimeStringTenthOfMinuteHasChanged

JKConvertedCellInfoStruct JKConvertedCellInfo;  // The converted little endian cell voltage data
#if !defined(NO_INTERNAL_STATISTICS)
/*
 * Arrays of counters, which count the times, a cell has minimal or maximal voltage
 * To identify runaway cells
 */
uint16_t CellMinimumArray[MAXIMUM_NUMBER_OF_CELLS];
uint16_t CellMaximumArray[MAXIMUM_NUMBER_OF_CELLS];
uint8_t CellMinimumPercentageArray[MAXIMUM_NUMBER_OF_CELLS];
uint8_t CellMaximumPercentageArray[MAXIMUM_NUMBER_OF_CELLS];
uint32_t sBalancingCount;            // Count of active balancing in SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS (2 seconds) units

/*
 * The first entry [0] holds the current computed value if more than 1 Ah are accumulated
 */
#if defined(STANDALONE_TEST)
struct JKComputedCapacityStruct JKComputedCapacity[SIZE_OF_COMPUTED_CAPACITY_ARRAY] = { { 0, 0, 20, 15, 80, 100 }, { 10, 55, 99, 10,
        40, 42 }, { 100, 5, 20, 45, 120, 150 }, { 20, 45, 100, 5, 120, 150 } };
#else
struct JKComputedCapacityStruct JKComputedCapacity[SIZE_OF_COMPUTED_CAPACITY_ARRAY];
#endif

CapacityComputationInfoStruct sCapacityComputationInfo;

EEMEM SOCDataPointDeltaStruct SOCDataPointsEEPROMArray[NUMBER_OF_SOC_DATA_POINTS]; // 256 for 1 kB EEPROM
SOCDataPointsInfoStruct SOCDataPointsInfo;
#endif //NO_INTERNAL_STATISTICS

/*
 * The JKFrameAllDataStruct starts behind the header + cell data header 0x79 + CellInfoSize + the variable length cell data (CellInfoSize is contained in JKReplyFrameBuffer[12])
 */
JKReplyStruct *sJKFAllReplyPointer;

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

const char *const JK_BMSErrorStringsArray[NUMBER_OF_DEFINED_ALARM_BITS] PROGMEM = { lowCapacity, MosFetOvertemperature,
        chargingOvervoltage, dischargingUndervoltage, Sensor2Overtemperature, chargingOvercurrent, dischargingOvercurrent,
        CellVoltageDifference, Sensor1Overtemperature, Sensor2LowLemperature, CellOvervoltage, CellUndervoltage, _309AProtection,
        _309BProtection };
const char *sErrorStringForLCD; // store of the error string of the highest error bit, NULL otherwise
bool sErrorStatusJustChanged = false; // True -> display overview page. Is set by handleAndPrintAlarmInfo(), if error flags changed, and reset on switching to overview page.
bool sErrorStatusIsError = false; // True if status is error and beep should be started. False e.g. for "Battery full".

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
        for (uint8_t i = 0; i < sizeof(JKRequestStatusFrame); ++i) {
            Serial.print(F(" 0x"));
            Serial.print(JKRequestStatusFrame[i], HEX);
        }
        Serial.println();
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
    for (uint16_t i = 0; i < (sReplyFrameBufferIndex + 1); ++i) {
        /*
         * Insert newline and address after header (11 byte), before and after cell data before trailer (9 byte) and after each 16 byte
         */
        if (i == JK_BMS_FRAME_HEADER_LENGTH || i == JK_BMS_FRAME_HEADER_LENGTH + 2
                || i
                        == (uint16_t) (JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH + 1
                                + JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH])
                || i == ((sReplyFrameBufferIndex + 1) - JK_BMS_FRAME_TRAILER_LENGTH)
                || (i < ((sReplyFrameBufferIndex + 1) - JK_BMS_FRAME_TRAILER_LENGTH) && i % 16 == 0) /* no 16 byte newline in trailer*/
                ) {
            if (i != 0) {
                Serial.println();
            }
            Serial.print(F("0x"));
            if (i < 0x10) {
                Serial.print('0'); // padding with zero
            }
            Serial.print(i, HEX);
            Serial.print(F("  "));
        }

        Serial.print(F("0x"));
        if (JKReplyFrameBuffer[i] < 0x10) {
            Serial.print('0'); // padding with zero
        }
        Serial.print(JKReplyFrameBuffer[i], HEX);
        Serial.print(' ');

    }
    Serial.println();
}

#define JK_BMS_RECEIVE_OK           0
#define JK_BMS_RECEIVE_FINISHED     1
#define JK_BMS_RECEIVE_ERROR        2
/*
 * Is assumed to be called if Serial.available() is true
 * @return JK_BMS_RECEIVE_OK, if still receiving; JK_BMS_RECEIVE_FINISHED, if complete frame was successfully read
 *          JK_BMS_RECEIVE_ERROR, if frame has errors.
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
            Serial.println(F("Error start frame token != 0x4E"));
            return JK_BMS_RECEIVE_ERROR;
        }
    } else if (sReplyFrameBufferIndex == 1) {
        if (tReceivedByte != JK_FRAME_START_BYTE_1) {
            // Error
            return JK_BMS_RECEIVE_ERROR;
        }

    } else if (sReplyFrameBufferIndex == 3) {
        // length of frame
        sReplyFrameLength = (JKReplyFrameBuffer[2] << 8) + tReceivedByte;

    } else if (sReplyFrameLength > MINIMAL_JK_BMS_FRAME_LENGTH && sReplyFrameBufferIndex == sReplyFrameLength - 3) {
        // Check end token 0x68
        if (tReceivedByte != JK_FRAME_END_BYTE) {
            Serial.print(F("Error end frame token 0x"));
            Serial.print(tReceivedByte, HEX);
            Serial.print(F(" at index"));
            Serial.print(sReplyFrameBufferIndex);
            Serial.print(F(" is != 0x68. sReplyFrameLength= "));
            Serial.print(sReplyFrameLength);
            Serial.print(F(" | 0x"));
            Serial.println(sReplyFrameLength, HEX);
            return JK_BMS_RECEIVE_ERROR;
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
            Serial.print(F("Checksum error, computed checksum=0x"));
            Serial.print(tComputedChecksum, HEX);
            Serial.print(F(", received checksum=0x"));
            Serial.println(tReceivedChecksum, HEX);

            return JK_BMS_RECEIVE_ERROR;
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
        Serial.print(F("Error: Program compiled with \"MAXIMUM_NUMBER_OF_CELLS=" STR(MAXIMUM_NUMBER_OF_CELLS) "\", but "));
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
            }
            if (tMaximumMillivolt < tVoltage) {
                tMaximumMillivolt = tVoltage;
            }
        }
    }
    JKConvertedCellInfo.MinimumCellMillivolt = tMinimumMillivolt;
    JKConvertedCellInfo.MaximumCellMillivolt = tMaximumMillivolt;
    JKConvertedCellInfo.DeltaCellMillivolt = tMaximumMillivolt - tMinimumMillivolt;
    JKConvertedCellInfo.AverageCellMillivolt = tMillivoltSum / tNumberOfNonNullCellInfo;

#if !defined(USE_NO_LCD)
    /*
     * Mark and count minimum and maximum cell voltages
     */
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt == tMinimumMillivolt) {
            JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_MINIMUM;
            if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
                CellMinimumArray[i]++; // count for statistics
            }
        } else if (JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt == tMaximumMillivolt) {
            JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_MAXIMUM;
            if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
                CellMaximumArray[i]++;
            }
        } else {
            JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween = VOLTAGE_IS_BETWEEN_MINIMUM_AND_MAXIMUM;
        }
    }
#endif

#if !defined(NO_INTERNAL_STATISTICS)
    /*
     * Process minimum statistics
     */
    uint32_t tCellStatisticsSum = 0;
    bool tDoDaylyScaling = false;
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        /*
         * After 43200 counts (a whole day being the minimum / maximum) we do scaling
         */
        uint16_t tCellStatisticsCount = CellMinimumArray[i];
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
            CellMinimumPercentageArray[i] = ((uint32_t) (CellMinimumArray[i] * 100UL)) / tCellStatisticsSum;
        }
    }

    if (tDoDaylyScaling) {
        /*
         * Do scaling by dividing all values by 2 resulting in an Exponential Moving Average filter for values
         */
        Serial.println(F("Do scaling of minimum counts"));
        for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
            CellMinimumArray[i] = CellMinimumArray[i] / 2;
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
        uint16_t tCellStatisticsCount = CellMaximumArray[i];
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
            CellMaximumPercentageArray[i] = ((uint32_t) (CellMaximumArray[i] * 100UL)) / tCellStatisticsSum;
        }
    }
    if (tDoDaylyScaling) {
        /*
         * Do scaling by dividing all values by 2 resulting in an Exponential Moving Average filter for values
         */
        Serial.println(F("Do scaling of maximum counts"));
        for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
            CellMaximumArray[i] = CellMaximumArray[i] / 2;
        }
    }
#endif // NO_INTERNAL_STATISTICS

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

#if !defined(NO_INTERNAL_STATISTICS)
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
        sprintf_P(tStringBuffer, PSTR("%2u=%2u %% |%5u, "), i + 1, CellMinimumPercentageArray[i], CellMinimumArray[i]);
        Serial.print(tStringBuffer);
    }
    Serial.println();

    Serial.println(F("Cell Maximum percentages"));
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (i != 0 && (i % 8) == 0) {
            Serial.println();
        }
        sprintf_P(tStringBuffer, PSTR("%2u=%2u %% |%5u, "), i + 1, CellMaximumPercentageArray[i], CellMaximumArray[i]);
        Serial.print(tStringBuffer);
    }
    Serial.println();

    Serial.println();
}

/*
 * Just clear the complete EEPROM
 * Not used yet
 */
void updateEEPROMTo_FF() {
//    if (!sOnlyPlotterOutput) {
//        Serial.println(F("Clear EEPROM"));
//    }
    for (int i = 0; i < E2END; ++i) {
        eeprom_update_byte((uint8_t*) i, 0xFF);
    }
}

/*
 * Looks for first 0xFF entry or for a SOC jump >
 * Sets SOCDataPointsInfo.ArrayStartIndex and SOCDataPointsInfo.ArrayLength
 */
void findFirstSOCDataPointIndex() {

// Default values
    uint8_t tSOCDataPointsArrayStartIndex = 0;
    uint16_t tSOCDataPointsArrayLength = NUMBER_OF_SOC_DATA_POINTS;

    uint8_t tOldSOCPercent;
    for (uint_fast8_t i = 0; i < NUMBER_OF_SOC_DATA_POINTS - 1; ++i) {
        uint8_t tSOCPercent = eeprom_read_byte(&SOCDataPointsEEPROMArray[i].SOCPercent);
        if (tSOCPercent == 0xFF) {
            // We found an empty entry, so EEPROM was not fully written => SOCDataPointsInfo.ArrayStartIndex is 0
            tSOCDataPointsArrayLength = i;
            break;
        }

        // TODO handle a second jump in data set, i.e. second set written from index 200 to 20
        if (i > 0 && abs(tOldSOCPercent - tSOCPercent) > 1) {
            // If SOC values make a jump > 1 we have the first entry not overwritten by new data
            tSOCDataPointsArrayStartIndex = i;
            break;
        }
        tOldSOCPercent = tSOCPercent;
    }
    SOCDataPointsInfo.ArrayStartIndex = tSOCDataPointsArrayStartIndex;
    SOCDataPointsInfo.ArrayLength = tSOCDataPointsArrayLength;
}

/*
 * Read and print SOC EEPROM data for Arduino Plotter
 */
void readAndPrintSOCData() {
    if (SOCDataPointsInfo.ArrayLength == 0) {
        return;
    }

    SOCDataPointDeltaStruct tCurrentSOCDataPoint;

    SOCDataPointStruct tMinimumSOCData;
    SOCDataPointStruct tMaximumSOCData;

    // Read first block
    auto tSOCDataPointsArrayIndex = SOCDataPointsInfo.ArrayStartIndex;
    eeprom_read_block(&tCurrentSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayIndex], sizeof(tCurrentSOCDataPoint));
//    tSOCDataMinimum = tCurrentSOCDataPoint;
    int16_t tCurrentCapacity100MilliampereHour = (tCurrentSOCDataPoint.SOCPercent * JKComputedData.TotalCapacityAmpereHour) / 10;

    tMinimumSOCData.SOCPercent = tCurrentSOCDataPoint.SOCPercent;
    tMaximumSOCData.SOCPercent = tCurrentSOCDataPoint.SOCPercent;
    tMinimumSOCData.VoltageDifferenceToEmpty40Millivolt = tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt;
    tMaximumSOCData.VoltageDifferenceToEmpty40Millivolt = tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt;
    tMinimumSOCData.Capacity100MilliampereHour = tCurrentCapacity100MilliampereHour;
    tMaximumSOCData.Capacity100MilliampereHour = tCurrentCapacity100MilliampereHour;

    /*
     * Print more than 500 data points in order to shift unwanted entries out to left of plotter window
     */
    Serial.println(F("Print SOC data with each entry printed twice"));

    uint_fast8_t i;
    for (i = 0; i < SOCDataPointsInfo.ArrayLength - 1; ++i) {
        /*
         * Compute minimum and maximum values for caption below
         */
        if (tMinimumSOCData.SOCPercent > tCurrentSOCDataPoint.SOCPercent) {
            tMinimumSOCData.SOCPercent = tCurrentSOCDataPoint.SOCPercent;
        } else if (tMaximumSOCData.SOCPercent < tCurrentSOCDataPoint.SOCPercent) {
            tMaximumSOCData.SOCPercent = tCurrentSOCDataPoint.SOCPercent;
        }
        tCurrentCapacity100MilliampereHour += tCurrentSOCDataPoint.Delta100MilliampereHour;
        if (tMinimumSOCData.Capacity100MilliampereHour > tCurrentCapacity100MilliampereHour) {
            tMinimumSOCData.Capacity100MilliampereHour = tCurrentCapacity100MilliampereHour;
        } else if (tMaximumSOCData.Capacity100MilliampereHour < tCurrentCapacity100MilliampereHour) {
            tMaximumSOCData.Capacity100MilliampereHour = tCurrentCapacity100MilliampereHour;
        }
        if (tMinimumSOCData.VoltageDifferenceToEmpty40Millivolt > tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt) {
            tMinimumSOCData.VoltageDifferenceToEmpty40Millivolt = tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt;
        } else if (tMaximumSOCData.VoltageDifferenceToEmpty40Millivolt < tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt) {
            tMaximumSOCData.VoltageDifferenceToEmpty40Millivolt = tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt;
        }

        for (uint_fast8_t j = 0; j < 2; ++j) {
            Serial.print(tCurrentSOCDataPoint.SOCPercent);
            Serial.print(' ');
            Serial.print((tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt * 4) / 10);
            Serial.print(' ');
            Serial.print(constrain(tCurrentSOCDataPoint.Delta100MilliampereHour, -30, 90)); // clip display to -30 and 90
            Serial.print(' ');
            Serial.print(tCurrentCapacity100MilliampereHour / 10); // print Ah
            Serial.print(' ');
            Serial.print(tCurrentSOCDataPoint.DeltaTimeTenSeconds / 6);
            Serial.println(F(" 0 0 0 0 0")); // to clear unwanted entries from former prints
        }

        // Read next block
        tSOCDataPointsArrayIndex = (tSOCDataPointsArrayIndex + 1) % NUMBER_OF_SOC_DATA_POINTS;
        eeprom_read_block(&tCurrentSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayIndex], sizeof(tCurrentSOCDataPoint));
    }

// print last entry with caption
    for (uint_fast8_t j = 0; j < 2; ++j) {
        Serial.print(F("SOC="));
        Serial.print(tMinimumSOCData.SOCPercent);
        Serial.print(F("%->"));
        Serial.print(tMaximumSOCData.SOCPercent);
        Serial.print(F("%:"));
        Serial.print(tCurrentSOCDataPoint.SOCPercent);
        Serial.print(F(" VoltToEmpty_"));
        Serial.print((tMinimumSOCData.VoltageDifferenceToEmpty40Millivolt * 4) / 100.0, 2);
        Serial.print(F("V->"));
        Serial.print((tMaximumSOCData.VoltageDifferenceToEmpty40Millivolt * 4) / 100.0, 2);
        Serial.print(F("V_[0.1V]:"));
        Serial.print((tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt * 4) / 10);
        Serial.print(F(" Delta_capacity[0.1Ah]:"));
        Serial.print(constrain(tCurrentSOCDataPoint.Delta100MilliampereHour, -30, 90)); // clip display to -30 and 90
        Serial.print(F(" Capacity_"));
        Serial.print((tMaximumSOCData.Capacity100MilliampereHour - tMinimumSOCData.Capacity100MilliampereHour) / 10); // Delta Ah
        Serial.print(F("Ah_"));
        Serial.print(tMinimumSOCData.Capacity100MilliampereHour / 10); // Minimum Ah
        Serial.print(F("->"));
        Serial.print(tMaximumSOCData.Capacity100MilliampereHour / 10); // Maximum Ah
        Serial.print(':');
        Serial.print(tCurrentCapacity100MilliampereHour / 10); // print Ah
        Serial.print(F(" Delta_time[min]:"));
        Serial.print(tCurrentSOCDataPoint.DeltaTimeTenSeconds / 6);
        Serial.print(F(" Empty_voltage_"));
        Serial.print(JKComputedData.BatteryEmptyVoltage10Millivolt / 100.0, 1);
        Serial.print('_');
        Serial.print(swap(sJKFAllReplyPointer->CellUndervoltageProtectionMillivolt) / 1000.0, 2);
        Serial.println(F("V:0 _:0 _:0 _:0 _:0 _:0 _:0 _:0 _:0"));
    }
    // If not enough data points, padding to 500 data points to guarantee, that old data is shifted out
    for (; i < 249; ++i) {
        Serial.println(F(" 0 0 0 0 0 0 0 0 0"));
        Serial.println(F(" 0 0 0 0 0 0 0 0 0"));
    }
}

/*
 * Compute delta values and write them to EEPROM
 */
void writeSOCData() {
    auto tCurrentSOCPercent = sJKFAllReplyPointer->SOCPercent;
    SOCDataPointDeltaStruct tSOCDataPoint;

    /*
     * For SOC graph
     * Values are taken at the lower edge of SOC
     */
    SOCDataPointsInfo.DeltaAccumulator10Milliampere += JKComputedData.Battery10MilliAmpere;
    ;
#if defined(DEBUG)
    Serial.print(F("sDelta10MilliampereAccumulator="));
    Serial.println(SOCDataPointsInfo.DeltaAccumulator10Milliampere);
#endif
    uint8_t tSOCDataPointsArrayLastWriteIndex = (SOCDataPointsInfo.ArrayStartIndex + SOCDataPointsInfo.ArrayLength - 1)
            % NUMBER_OF_SOC_DATA_POINTS;
    auto tLastWrittenSOCPercent = eeprom_read_byte(&SOCDataPointsEEPROMArray[tSOCDataPointsArrayLastWriteIndex].SOCPercent);
    if (tCurrentSOCPercent > tLastWrittenSOCPercent || tCurrentSOCPercent < (tLastWrittenSOCPercent - 1)) {
        /*
         * Insert new entry
         */
        if (SOCDataPointsInfo.ArrayLength == NUMBER_OF_SOC_DATA_POINTS) {
            // Array is full, overwrite old start entry
            SOCDataPointsInfo.ArrayStartIndex++;
        } else {
            // Array is not full, increase length
            SOCDataPointsInfo.ArrayLength++;
        }

        // Compute new SOC to be written
        if (tCurrentSOCPercent > tLastWrittenSOCPercent) {
            // Here we SOC was just incremented
            tSOCDataPoint.SOCPercent = tCurrentSOCPercent;
        } else {
            /*
             * Here tCurrentSOCPercent < (SOCDataPointsEEPROMArray[tSOCDataPointsArrayIndex].SOCPercent - 1
             * SOC is decreasing, so we take values at the point just below the lower edge of the value below for the SOC just above this edge
             */
            tSOCDataPoint.SOCPercent = tCurrentSOCPercent + 1;
        }
        tSOCDataPoint.VoltageDifferenceToEmpty40Millivolt = JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 4;
        tSOCDataPoint.DeltaTimeTenSeconds = (millis() - SOCDataPointsInfo.MillisOfLastValidEntry) / 10000;
        SOCDataPointsInfo.MillisOfLastValidEntry += ((long) tSOCDataPoint.DeltaTimeTenSeconds) * 10000;
        // Compute current value and adjust accumulator
        tSOCDataPoint.Delta100MilliampereHour = SOCDataPointsInfo.DeltaAccumulator10Milliampere
                / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10);
        SOCDataPointsInfo.DeltaAccumulator10Milliampere -= tSOCDataPoint.Delta100MilliampereHour
                * (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10);
        /*
         * Write to eeprom
         */
        uint8_t tSOCDataPointsArrayNextWriteIndex = (tSOCDataPointsArrayLastWriteIndex + 1) % NUMBER_OF_SOC_DATA_POINTS;
        eeprom_write_block(&tSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayNextWriteIndex], sizeof(tSOCDataPoint));
#if defined(DEBUG)
        Serial.print(F("SOC data write at index "));
        Serial.print(tSOCDataPointsArrayNextWriteIndex);
        Serial.print(F(", current length="));
        Serial.print(SOCDataPointsInfo.ArrayLength);
        Serial.print(F(", Delta100MilliampereHour="));
        Serial.println(tSOCDataPoint.Delta100MilliampereHour);
#endif
    }
}

/*
 * Compute total capacity based on current and SOC
 *
 * If state is idle and current > 1 A: If current direction is from battery, start discharge computation else start charge computation.
 * If current direction is 5 times wrong, stop computation.
 * If SOC delta between start and end > 40% store value to array.
 */
void FillCapacityStatistics() {
    auto tCurrentSOCPercent = sJKFAllReplyPointer->SOCPercent;
    auto tBattery10MilliAmpere = JKComputedData.Battery10MilliAmpere;

    /*
     * Total capacity computation
     */
    if (sCapacityComputationInfo.Mode == CAPACITY_COMPUTATION_MODE_IDLE) {
        /*
         * Check for start condition
         */
        if (tBattery10MilliAmpere < -100) {
            sCapacityComputationInfo.Mode = CAPACITY_COMPUTATION_MODE_DISCHARGE;
        } else if (tBattery10MilliAmpere > 100) {
            sCapacityComputationInfo.Mode = CAPACITY_COMPUTATION_MODE_CHARGE;
        }

        JKComputedCapacity[0].Start100MilliVoltToEmpty = JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 10;
        JKComputedCapacity[0].StartSOCPercent = tCurrentSOCPercent;
    } else {
        /*
         * Mode CAPACITY_COMPUTATION_MODE_CHARGE and CAPACITY_COMPUTATION_MODE_DISCHARGE here
         */
        if (sCapacityComputationInfo.Mode == CAPACITY_COMPUTATION_MODE_DISCHARGE) {
            // Convert discharge current
            tBattery10MilliAmpere = -tBattery10MilliAmpere;
        }
        // Add capacity, even the first value with wrong direction
        sCapacityComputationInfo.Accumulator10Milliampere += tBattery10MilliAmpere;
        /*
         * Write current data to array. If Capacity < 1, then values are not displayed
         */
        uint8_t tDeltaSOC = abs(JKComputedCapacity[0].StartSOCPercent - tCurrentSOCPercent);
        if (tDeltaSOC > 1) {
            JKComputedCapacity[0].EndSOCPercent = tCurrentSOCPercent;
            JKComputedCapacity[0].End100MilliVoltToEmpty = JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 10;
            uint16_t tCapacityComputationAccumulator10MilliAmpereHour = sCapacityComputationInfo.Accumulator10Milliampere
                    / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 100); // -> sCapacityComputationInfo.Accumulator10Milliampere / 1800
            JKComputedCapacity[0].Capacity = tCapacityComputationAccumulator10MilliAmpereHour / 100;
            JKComputedCapacity[0].TotalCapacity = tCapacityComputationAccumulator10MilliAmpereHour / tDeltaSOC;
            // Direct 32 bit computation. It is 12 bytes longer
            //            JKComputedCapacity[0].Capacity = sCapacityComputationInfo.Accumulator10Milliampere / CAPACITY_ACCUMULATOR_1_AMPERE_HOUR;
            //            JKComputedCapacity[0].TotalCapacity = sCapacityComputationInfo.Accumulator10Milliampere
            //                    / ((CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 100) * tDeltaSOC);
            if (sCapacityComputationInfo.LastDeltaSOC != tDeltaSOC) {
                // Delta SOC changed by 1 -> print values
                sCapacityComputationInfo.LastDeltaSOC = tDeltaSOC;
                printComputedCapacity(0);
            }
        }
        /*
         * Check for wrong current direction. 0 mA is no direction :-)
         */
        if (tBattery10MilliAmpere > 0) {
            sCapacityComputationInfo.WrongDirectionCount = 0;
        } else if (tBattery10MilliAmpere < 0) {
            sCapacityComputationInfo.WrongDirectionCount++;
            if (sCapacityComputationInfo.WrongDirectionCount > CAPACITY_COMPUTATION_MAX_WRONG_CHARGE_DIRECTION) {
                if (JKComputedCapacity[0].Capacity != 0) {
                    // process only if we have valid data
                    Serial.println(
                            F(
                                    "More than " STR(CAPACITY_COMPUTATION_MAX_WRONG_CHARGE_DIRECTION) " wrong current directions -> end capacity computation"));
                    printComputedCapacity(0);
                    checkAndStoreCapacityComputationValues();
                }
                /*
                 * Reset capacity computation mode
                 */
                memset(&JKComputedCapacity, 0, sizeof(JKComputedCapacity[0]));
                //                JKComputedCapacity[0].StartSOCPercent = 0;
                //                JKComputedCapacity[0].EndSOCPercent = 0;
                //                JKComputedCapacity[0].Capacity = 0;
                //                JKComputedCapacity[0].TotalCapacity = 0;
                sCapacityComputationInfo.LastDeltaSOC = 0;
                sCapacityComputationInfo.Mode = CAPACITY_COMPUTATION_MODE_IDLE;
                sCapacityComputationInfo.Accumulator10Milliampere = 0;
                sCapacityComputationInfo.WrongDirectionCount = 0;
            }
        }
    }
}
#endif // NO_INTERNAL_STATISTICS

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

    JKComputedData.BatteryVoltageFloat = JKComputedData.BatteryVoltage10Millivolt;
    JKComputedData.BatteryVoltageFloat /= 100;

    JKComputedData.Battery10MilliAmpere = getCurrent(sJKFAllReplyPointer->Battery10MilliAmpere);
    JKComputedData.BatteryLoadCurrentFloat = JKComputedData.Battery10MilliAmpere;
    JKComputedData.BatteryLoadCurrentFloat /= 100;

//    Serial.print("Battery10MilliAmpere=0x");
//    Serial.print(sJKFAllReplyPointer->Battery10MilliAmpere, HEX);
//    Serial.print(" Battery10MilliAmpere swapped=0x");
//    Serial.println(swap(sJKFAllReplyPointer->Battery10MilliAmpere), HEX);
//    Serial.print(" Battery10MilliAmpere=");
//    Serial.print(JKComputedData.Battery10MilliAmpere);
//    Serial.print(" BatteryLoadCurrent=");
//    Serial.println(JKComputedData.BatteryLoadCurrentFloat);

    JKComputedData.BatteryLoadPower = JKComputedData.BatteryVoltageFloat * JKComputedData.BatteryLoadCurrentFloat;

#if !defined(NO_INTERNAL_STATISTICS)
    /*
     * Increment sBalancingCount and fill sBalancingTimeString
     */
    if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
        sBalancingCount++;
        sprintf_P(sBalancingTimeString, PSTR("%3uD%02uH%02uM"), (uint16_t) (sBalancingCount / (60 * 24 * 30UL)),
                (uint16_t) ((sBalancingCount / (60 * 30)) % 24), (uint16_t) (sBalancingCount / 30) % 60);
    }

    FillCapacityStatistics();
    writeSOCData();
#endif // NO_INTERNAL_STATISTICS
}

#if !defined(NO_INTERNAL_STATISTICS)
/*
 * Print only valid data, i.e. Capacity != 0
 */
void printComputedCapacity(uint8_t aCapacityArrayIndex) {
    if (JKComputedCapacity[aCapacityArrayIndex].Capacity != 0) {
        snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%u%% -> %u%% = %uAh => 100%% = %uAh"),
                JKComputedCapacity[aCapacityArrayIndex].StartSOCPercent, JKComputedCapacity[aCapacityArrayIndex].EndSOCPercent,
                JKComputedCapacity[aCapacityArrayIndex].Capacity, JKComputedCapacity[aCapacityArrayIndex].TotalCapacity);
        Serial.println(sStringBuffer);
    }
}

void checkAndStoreCapacityComputationValues() {
    int8_t tDeltaSOC = JKComputedCapacity[0].StartSOCPercent - JKComputedCapacity[0].EndSOCPercent;
    if (tDeltaSOC <= -40 || 40 <= tDeltaSOC) {
        // free first array index by moving values direction end
        memmove(&JKComputedCapacity[1], &JKComputedCapacity[0],
                sizeof(JKComputedCapacity[0]) * (SIZE_OF_COMPUTED_CAPACITY_ARRAY - 1));

        Serial.println(F("Store computed capacity"));
        for (uint8_t i = 2; i < SIZE_OF_COMPUTED_CAPACITY_ARRAY; ++i) {
            printComputedCapacity(i);
        }
    }
}
#endif // NO_INTERNAL_STATISTICS

/*
 * Print formatted cell info on Serial
 */
void printJKCellInfo() {
    uint8_t tNumberOfCellInfo = JKConvertedCellInfo.ActualNumberOfCellInfoEntries;

    /*
     * Summary
     */
    Serial.print(tNumberOfCellInfo);
    myPrint(F(" Cells, Minimum="), JKConvertedCellInfo.MinimumCellMillivolt);
    myPrint(F(" mV, Maximum="), JKConvertedCellInfo.MaximumCellMillivolt);
    myPrint(F("mV, Delta="), JKConvertedCellInfo.DeltaCellMillivolt);
    myPrint(F(" mV, Average="), JKConvertedCellInfo.AverageCellMillivolt);
    Serial.println(F(" mV"));

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
 * Token 0x8B. Prints info only if errors existent and changed from last value
 * Stores error string for LCD in sErrorStringForLCD
 */
void handleAndPrintAlarmInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;

    /*
     * Do it only once per change
     */
    if (tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord != lastJKReply.AlarmUnion.AlarmsAsWord) {
        // ChargeOvervoltageAlarm is displayed separately
        if (!tJKFAllReplyPointer->AlarmUnion.AlarmBits.ChargeOvervoltageAlarm) {
            sErrorStatusJustChanged = true; // This forces a switch to Overview page
            sErrorStatusIsError = true; //  This forces the beep
        }

        if (tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord == 0) {
            sErrorStringForLCD = NULL; // reset error string
            sErrorStatusIsError = false;
            Serial.println(F("All alarms are cleared now"));
        } else {
            uint16_t tAlarms = swap(tJKFAllReplyPointer->AlarmUnion.AlarmsAsWord);
            Serial.println(F("*** ALARM FLAGS ***"));
            Serial.print(sUpTimeString); // print uptime to have a timestamp for the alarm
            Serial.print(F(": Alarm bits=0x"));
            Serial.println(tAlarms, HEX);

            uint16_t tAlarmMask = 1;
            for (uint_fast8_t i = 0; i < NUMBER_OF_DEFINED_ALARM_BITS; ++i) {
                if (tAlarms & tAlarmMask) {
                    Serial.print(F("Alarm bit=0b"));
                    Serial.print(tAlarmMask, BIN);
                    Serial.print(F(" -> "));
                    const char *tErrorStringPtr = (char*) (pgm_read_word(&JK_BMSErrorStringsArray[i]));
                    sErrorStringForLCD = tErrorStringPtr;
                    Serial.println(reinterpret_cast<const __FlashStringHelper*>(tErrorStringPtr));
                }
                tAlarmMask <<= 1;
            }
            Serial.println();
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
    }
}

const char sCSVCaption[] PROGMEM
        = "Cell_1;Cell_2;Cell_3;Cell_4;Cell_5;Cell_6;Cell_7;Cell_8;Cell_9;Cell_10;Cell_11;Cell_12;Cell_13;Cell_14;Cell_15;Cell_16;Voltage,Current;SOC;Balancing";

/*
 * Print received data
 * Use converted cell voltage info from JKConvertedCellInfo
 * All other data are used unconverted and are therefore printed by swap() functions.
 */
void printJKDynamicInfo() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;

#if defined(ENABLE_MONITORING)
    printMonitoringInfo();
        if (sUpTimeStringMinuteHasChanged) {
            sUpTimeStringMinuteHasChanged = false;
            Serial.print(sCSVCaption);
        }
#endif

    /*
     * Print it every ten minutes
     */
//    // Print it every minute
//    if (sUpTimeStringMinuteHasChanged) {
//        sUpTimeStringMinuteHasChanged = false;
    if (sUpTimeStringTenthOfMinuteHasChanged) {
        sUpTimeStringTenthOfMinuteHasChanged = false;

        Serial.print(F("Total Runtime_"));
        Serial.print(swap(sJKFAllReplyPointer->SystemWorkingMinutes));
        Serial.print(F(" minutes -> "));
        Serial.println(sUpTimeString);

        Serial.println(F("*** CELL INFO ***"));
        printJKCellInfo();
#if !defined(NO_INTERNAL_STATISTICS)
        if (sBalancingCount > MINIMUM_BALANCING_COUNT_FOR_DISPLAY) {
            Serial.println(F("*** CELL STATISTICS ***"));
            Serial.print(F("Total balancing time="));

            Serial.print(sBalancingCount * (MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS / 1000));
            Serial.print(F(" s -> "));
            Serial.print(sBalancingTimeString);
            // Append seconds
            char tString[4]; // "03S" is 3 bytes long
            sprintf_P(tString, PSTR("%02uS"), (uint16_t) (sBalancingCount % 30) * 2);
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
    }

    /*
     * Temperatures
     * Print only if temperature changed more than 1 degree
     */
#if defined(LOCAL_DEBUG)
    Serial.print(F("TokenTemperaturePowerMosFet=0x"));
    Serial.println(sJKFAllReplyPointer->TokenTemperaturePowerMosFet, HEX);
#endif
    if (abs(JKComputedData.TemperaturePowerMosFet - lastJKComputedData.TemperaturePowerMosFet) > 2
            || abs(JKComputedData.TemperatureSensor1 - lastJKComputedData.TemperatureSensor1) > 2
            || abs(JKComputedData.TemperatureSensor2 - lastJKComputedData.TemperatureSensor2) > 2) {
        myPrint(F("Temperature: Power MosFet="), JKComputedData.TemperaturePowerMosFet);
        myPrint(F(", Sensor 1="), JKComputedData.TemperatureSensor1);
        myPrintln(F(", Sensor 2="), JKComputedData.TemperatureSensor2);
    }

    /*
     * SOC
     */
    if (tJKFAllReplyPointer->SOCPercent != lastJKReply.SOCPercent
            || JKComputedData.RemainingCapacityAmpereHour != lastJKComputedData.RemainingCapacityAmpereHour) {
        myPrint(F("SOC[%]="), tJKFAllReplyPointer->SOCPercent);
        myPrintln(F(" -> Remaining Capacity[Ah]="), JKComputedData.RemainingCapacityAmpereHour);
    }

    /*
     * Charge and Discharge values
     */
    if (abs(JKComputedData.BatteryVoltageFloat - lastJKComputedData.BatteryVoltageFloat) > 0.02 // Meant is 0.02 but use 15 to avoid strange floating point effects
    || abs(JKComputedData.BatteryLoadPower - lastJKComputedData.BatteryLoadPower) >= 20) {
        Serial.print(F("Battery Voltage[V]="));
        Serial.print(JKComputedData.BatteryVoltageFloat, 2);
        Serial.print(F(", Current[A]="));
        Serial.print(JKComputedData.BatteryLoadCurrentFloat, 2);
        myPrint(F(", Power[W]="), JKComputedData.BatteryLoadPower);
        Serial.print(F(", Difference to empty[V]="));
        Serial.println(JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 100.0, 1);
    }

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
        Serial.println(); // printActiveState does no println()
    }
}

#if defined(ENABLE_MONITORING)
/*
 * Fills sStringBuffer with CSV data
 * Can be used for SD Card also, otherwise direct printing would be more efficient.
 * Print all (cell voltages - 3000 mV), voltage, current, SOC in CSV format
 * E.g. 185;185;186;185;185;206;185;214;183;185;201;186;186;186;185;186;5096;-565;54;1
 *
 */
void setCSVString() {
    JKReplyStruct *tJKFAllReplyPointer = sJKFAllReplyPointer;
    /*
     * Individual cell voltages
     */
    uint_fast8_t tBufferIndex = 0;

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
        // maximal string 5096;-20.00;100;1 -> 17 characters
        char tCurrentAsFloatString[7];
        dtostrf(JKComputedData.BatteryLoadCurrentFloat, 4, 2, &tCurrentAsFloatString[0]);
        tBufferIndex += sprintf_P(&sStringBuffer[tBufferIndex], PSTR("%u;%s;%d;%d"), JKComputedData.BatteryVoltage10Millivolt,
                tCurrentAsFloatString, tJKFAllReplyPointer->SOCPercent, tJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive);
    }
}

void printMonitoringInfo() {
    Serial.print(F("CSV: "));
    setCSVString();
    Serial.println(sStringBuffer);
}
#endif // ENABLE_MONITORING

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _JK_BMS_HPP
