/*
 * JK-BMS.cpp
 *
 * Functions to read, convert and print JK-BMS data
 *
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

#ifndef _JK_BMS_RS458_C
#define _JK_BMS_RS458_C

#include <Arduino.h>

#include "JK-BMS.h"

#if !defined(LOCAL_DEBUG)
//#define LOCAL_DEBUG
#endif

// see JK Communication protocol.pdf http://www.jk-bms.com/Upload/2022-05-19/1621104621.pdf
uint8_t JKRequestStatusFrame[21] = { 0x4E, 0x57 /*4E 57 = StartOfFrame*/, 0x00, 0x13 /*0x13 | 19 = LengthOfFrame*/, 0x00, 0x00,
        0x00, 0x00/*BMS ID, highest byte is default 00*/, 0x06/*Function 1=Activate, 3=ReadIdentifier, 6=ReadAllData*/,
        0x03/*Frame source 0=BMS, 1=Bluetooth, 2=GPRS, 3=PC*/, 0x00 /*TransportType 0=Request, 1=Response, 2=BMSActiveUpload*/,
        0x00/*0=ReadAllData or commandToken*/, 0x00, 0x00, 0x00,
        0x00/*RecordNumber High byte is random code, low 3 bytes is record number*/, JK_FRAME_END_BYTE/*0x68 = EndIdentifier*/,
        0x00, 0x00, 0x01, 0x29 /*Checksum, high 2 bytes for checksum not yet enabled -> 0, low 2 Byte for checksum*/};
uint8_t JKrequestStatusFrameOld[] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };

uint16_t sReplyFrameBufferIndex = 0;        // Index of next byte to write to array, except for last byte received. Starting with 0.
uint16_t sReplyFrameLength;                     // Received length of frame
uint8_t JKReplyFrameBuffer[350];                // The raw big endian data as received from JK BMS
JKConvertedCellInfoStruct JKConvertedCellInfo;  // The converted little endian cell voltage data
JKComputedDataStruct JKComputedData;            // All derived converted and computed data useful for display
char sUpTimeString[16]; // " -> 1000D23H12M" is 15 bytes long
char sLastUpTimeCharacter;  // for detecting changes in string
bool sForcePrintUpTime = true; // for LCD printing

/*
 * The JKFrameAllDataStruct starts behind the header + cell data header 0x79 + CellInfoSize + the variable length cell data (CellInfoSize is contained in JKReplyFrameBuffer[12])
 */
JKReplyStruct *sJKFAllReplyPointer;

const char lowCapacity[] PROGMEM = "Low capacity";                          // Byte 0.0, warning
const char MosFetOvertemperature[] PROGMEM = "Power MosFet overtemperature"; // Byte 0.1; alarm
const char chargingOvervoltage[] PROGMEM = "Charging overvoltage";          // Byte 0.2, alarm
const char dischargingUndervoltage[] PROGMEM = "Discharging undervoltage";  // Byte 0.3, alarm
const char Sensor2Overtemperature[] PROGMEM = "Sensor1_2 overtemperature";  // Byte 0.4, alarm
const char chargingOvercurrent[] PROGMEM = "Charging overcurrent";          // Byte 0.5, alarm
const char dischargingOvercurrent[] PROGMEM = "Discharging overcurrent";    // Byte 0.6, alarm
const char CellVoltageDifference[] PROGMEM = "Cell voltage difference";     // Byte 0.7, alarm
const char Sensor1Overtemperature[] PROGMEM = "Sensor2 overtemperature";    // Byte 1.0, alarm
const char Sensor2LowLemperature[] PROGMEM = "Sensor1_2 low temperature";   // Byte 1.1, alarm
const char CellOvervoltage[] PROGMEM = "Cell overvoltage";                  // Byte 1.2, alarm
const char CellUndervoltage[] PROGMEM = "Cell undervoltage";                // Byte 1.3, alarm
const char _309AProtection[] PROGMEM = "309_A protection";                  // Byte 1.4, alarm
const char _309BProtection[] PROGMEM = "309_B protection";                  // Byte 1.5, alarm

const char *const JK_BMSErrorStringsArray[NUMBER_OF_DEFINED_ALARM_BITS] PROGMEM = { lowCapacity, MosFetOvertemperature,
        chargingOvervoltage, dischargingUndervoltage, Sensor2Overtemperature, chargingOvercurrent, dischargingOvercurrent,
        CellVoltageDifference, Sensor1Overtemperature, Sensor2LowLemperature, CellOvervoltage, CellUndervoltage, _309AProtection,
        _309BProtection };
const char *ErrorStringForLCD; // store of the error string of the highest error bit, NULL otherwise

void requestJK_BMSStatusFrame(SoftwareSerialTX *aSerial, bool aDebugModeActive) {
    for (uint8_t i = 0; i < sizeof(JKRequestStatusFrame); ++i) {
        aSerial->write(JKRequestStatusFrame[i]);
    }
    if (aDebugModeActive) {
        Serial.println(F("Send requestFrame with TxToJKBMS"));
        for (uint8_t i = 0; i < sizeof(JKRequestStatusFrame); ++i) {
            Serial.print(F(" 0x"));
            Serial.print(JKRequestStatusFrame[i], HEX);
        }
        Serial.println();
    }
}

void initJKReplyFrameBuffer() {
    sReplyFrameBufferIndex = 0;
}

void printJKReplyFrameBuffer() {
    if (sReplyFrameBufferIndex == 0) {
        Serial.println(F("sReplyFrameBufferIndex is 0"));
    } else {
        for (uint16_t i = 0; i < (sReplyFrameBufferIndex + 1); ++i) {
            if (i == JK_BMS_FRAME_HEADER_LENGTH || i == ((sReplyFrameBufferIndex + 1) - JK_BMS_FRAME_TRAILER_LENGTH) || i % 16 == 0
                    || i
                            == (uint16_t) (JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH + 1
                                    + JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH])) {
                // Insert newline and address after header, after cell data and after each 16 bit
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
}

#define JK_BMS_RECEIVE_OK           0
#define JK_BMS_RECEIVE_FINISHED     1
#define JK_BMS_RECEIVE_ERROR        2
/*
 * Is assumed to be called if Serial.available() is true
 * @return JK_BMS_RECEIVE_OK, if still receiving; JK_BMS_RECEIVE_FINISHED, if complete frame was successfully read
 *          JK_BMS_RECEIVE_ERROR, if frame has errors.
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
 * Charge is positive, discharge is negative
 */
int16_t getCurrent(uint16_t aJKRAWCurrent) {
    uint16_t tCurrent = swap(aJKRAWCurrent);
    if (tCurrent == 0 || (tCurrent & 0x8000) == 0x8000) {
        // Charge
        return (tCurrent & 0x7FFF);
    }
    // discharge
    return tCurrent * -1;

}

int16_t getTemperature(uint16_t aJKRAWTemperature) {
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

uint32_t swap(uint32_t aLongToSwapBytes) {
    return ((aLongToSwapBytes << 24) | ((aLongToSwapBytes & 0xFF00) << 8) | ((aLongToSwapBytes >> 8) & 0xFF00)
            | (aLongToSwapBytes >> 24));
}

void myPrintln(const __FlashStringHelper *aPGMString, uint16_t a16BitValue) {
    Serial.print(aPGMString);
    Serial.println(a16BitValue);
}

void myPrint(const __FlashStringHelper *aPGMString, uint16_t a16BitValue) {
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
    JKConvertedCellInfo.NumberOfCellInfoEnties = tNumberOfCellInfo;

    uint16_t tVoltage;
    uint32_t tMillivoltSum = 0;
    uint8_t tNumberOfNonNullCellInfo = 0;
    uint16_t tMinimumMillivolt = 0xFFFF;
    uint16_t tMaximumMillivolt = 0;

    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        JKConvertedCellInfo.CellInfoStructArray[i].CellNumber = *tJKCellInfoReplyPointer++; // Copy CellNumber

        uint8_t tHighByte = *tJKCellInfoReplyPointer++;              // Copy CellMillivolt
        tVoltage = tHighByte << 8 | *tJKCellInfoReplyPointer++;
        JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt = tVoltage;
        if (tVoltage > 0) {
            tNumberOfNonNullCellInfo++;
            tMillivoltSum += tVoltage;
            if (tMinimumMillivolt > tVoltage) {
                tMinimumMillivolt = tVoltage;
                JKConvertedCellInfo.MinimumVoltagCellIndex = i;
            }
            if (tMaximumMillivolt < tVoltage) {
                tMaximumMillivolt = tVoltage;
                JKConvertedCellInfo.MaximumVoltagCellIndex = i;
            }
        }
    }
    JKConvertedCellInfo.DeltaCellMillivolt = tMaximumMillivolt - tMinimumMillivolt;
    JKConvertedCellInfo.AverageCellMillivolt = tMillivoltSum / tNumberOfNonNullCellInfo;

#if defined(LOCAL_DEBUG)
    Serial.print(tNumberOfCellInfo);
    Serial.println(F(" cell voltages processed"));
#endif
    if (tNumberOfNonNullCellInfo < tNumberOfCellInfo) {
        Serial.print(F("Problem: "));
        Serial.print(tNumberOfCellInfo);
        Serial.print(F(" cells configured, but only "));
        Serial.print(tNumberOfNonNullCellInfo);
        Serial.println(F(" cells are connected"));
    }
}

void fillJKComputedData() {
    JKComputedData.TemperaturePowerMosFet = getTemperature(sJKFAllReplyPointer->TemperaturePowerMosFet);
    int16_t tMaxTemperature = JKComputedData.TemperaturePowerMosFet;

    JKComputedData.TemperatureSensor1 = getTemperature(sJKFAllReplyPointer->TemperatureSensor1);
    if (tMaxTemperature < JKComputedData.TemperatureSensor1) {
        tMaxTemperature = JKComputedData.TemperatureSensor1;
    }

    JKComputedData.TemperatureSensor2 = getTemperature(sJKFAllReplyPointer->TemperatureSensor2);
    if (tMaxTemperature < JKComputedData.TemperatureSensor2) {
        tMaxTemperature = JKComputedData.TemperatureSensor2;
    }
    JKComputedData.TemperatureMaximum = tMaxTemperature;

    JKComputedData.TotalCapacityAmpereHour = swap(sJKFAllReplyPointer->TotalCapacityAmpereHour);
    // 16 bit multiplication gives overflow at 640 Ah
    JKComputedData.RemainingCapacityAmpereHour = ((uint32_t) JKComputedData.TotalCapacityAmpereHour
            * sJKFAllReplyPointer->SOCPercent) / 100;

    JKComputedData.BatteryVoltage10Millivolt = swap(sJKFAllReplyPointer->Battery10Millivolt);
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
}

void printJKCellInfo() {
    uint8_t tNumberOfCellInfo = JKConvertedCellInfo.NumberOfCellInfoEnties;

//    Serial.print(tNumberOfCellInfo);
//    Serial.println(F(" cell voltages:"));
    for (uint8_t i = 0; i < tNumberOfCellInfo; ++i) {
        if (i != 0 && (i % 8) == 0) {
            Serial.println();
        }
        if (JKConvertedCellInfo.CellInfoStructArray[i].CellNumber < 10) {
            Serial.print(' ');
        }
        Serial.print(JKConvertedCellInfo.CellInfoStructArray[i].CellNumber);
        Serial.print(F("="));
        Serial.print(JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt);
#if defined(LOCAL_TRACE)
     Serial.print(F("|0x"));
     Serial.print(JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt, HEX);
#endif
        Serial.print(F(" mV, "));
    }
    Serial.println();

    /*
     * Cell statistics
     */
    myPrint(F("Minimum="), JKConvertedCellInfo.CellInfoStructArray[JKConvertedCellInfo.MinimumVoltagCellIndex].CellMillivolt);
    myPrint(F(" mV at cell #"), JKConvertedCellInfo.MinimumVoltagCellIndex + 1);
    myPrint(F(", Maximum="), JKConvertedCellInfo.CellInfoStructArray[JKConvertedCellInfo.MaximumVoltagCellIndex].CellMillivolt);
    myPrintln(F(" mV at cell #"), JKConvertedCellInfo.MaximumVoltagCellIndex + 1);
    myPrint(F("Delta="), JKConvertedCellInfo.DeltaCellMillivolt);
    myPrint(F(" mV, Average="), JKConvertedCellInfo.AverageCellMillivolt);
    Serial.println(F(" mV"));
    Serial.println();
}

void printVoltageProtectionInfo() {
    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;
    /*
     * Voltage protection
     */
    myPrint(F("Battery Overvoltage Protection[mV]="), swap(tJKFAllReply->BatteryOvervoltageProtection10Millivolt) * 10);
    myPrintln(F(", Undervoltage="), swap(tJKFAllReply->BatteryUndervoltageProtection10Millivolt) * 10);
    myPrintSwap(F("Cell Overvoltage Protection[mV]="), tJKFAllReply->CellOvervoltageProtectionMillivolt);
    myPrintSwap(F(", Recovery="), tJKFAllReply->CellOvervoltageRecoveryMillivolt);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReply->CellOvervoltageDelaySeconds);
    myPrintSwap(F("Cell Undervoltage Protection[mV]="), tJKFAllReply->CellUndervoltageProtectionMillivolt);
    myPrintSwap(F(", Recovery="), tJKFAllReply->CellUndervoltageRecoveryMillivolt);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReply->CellUndervoltageDelaySeconds);
    myPrintlnSwap(F("Cell Voltage Difference Protection[mV]="), tJKFAllReply->VoltageDifferenceProtectionMillivolt);
    myPrintSwap(F("Discharging Overcurrent Protection[A]="), tJKFAllReply->DischargeOvercurrentProtectionAmpere);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReply->DischargeOvercurrentDelaySeconds);
    myPrintSwap(F("Charging Overcurrent Protection[A]="), tJKFAllReply->ChargeOvercurrentProtectionAmpere);
    myPrintlnSwap(F(", Delay[s]="), tJKFAllReply->ChargeOvercurrentDelaySeconds);
    Serial.println();
}

void printTemperatureProtectionInfo() {
    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;
    /*
     * Temperature protection
     */
    myPrintSwap(F("Power MosFet Temperature Protection="), tJKFAllReply->PowerMosFetTemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReply->PowerMosFetRecoveryTemperature);
    myPrintSwap(F("Sensor1 Temperature Protection="), tJKFAllReply->Sensor1TemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReply->Sensor1RecoveryTemperature);
    myPrintlnSwap(F("Sensor1 to Sensor2 Temperature Difference Protection="), tJKFAllReply->BatteryDifferenceTemperatureProtection);
    myPrintSwap(F("Charge Overtemperature Protection="), tJKFAllReply->ChargeOvertemperatureProtection);
    myPrintlnSwap(F(", Discharge="), tJKFAllReply->DischargeOvertemperatureProtection);
    myPrintSwap(F("Charge Undertemperature Protection="), tJKFAllReply->ChargeUndertemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReply->ChargeRecoveryUndertemperature);
    myPrintSwap(F("Discharge Undertemperature Protection="), tJKFAllReply->DischargeUndertemperatureProtection);
    myPrintlnSwap(F(", Recovery="), tJKFAllReply->DischargeRecoveryUndertemperature);
    Serial.println();
}

void printBatteryInfo() {
    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;

    Serial.print(F("Manufacturer Id="));
    tJKFAllReply->TokenProtocolVersionNumber = '\0'; // set end of string token
    Serial.println(tJKFAllReply->ManufacturerId);
    Serial.print(F("Manufacturer Date="));
    tJKFAllReply->TokenSystemWorkingMinutes = '\0'; // set end of string token
    Serial.println(tJKFAllReply->ManufacturerDate);
    Serial.print(F("Device ID String="));
    tJKFAllReply->TokenManufacturerDate = '\0'; // set end of string token
    Serial.println(tJKFAllReply->DeviceIdString);
    myPrintln(F("Device Address="), tJKFAllReply->BoardAddress);
    myPrint(F("Total Battery Capacity[Ah]="), JKComputedData.TotalCapacityAmpereHour); // 0xAA
    myPrintln(F(", Low Capacity Alarm Percent="), tJKFAllReply->LowCapacityAlarmPercent); // 0xB1
    myPrintlnSwap(F("Charging Cycles="), tJKFAllReply->Cycles);
    myPrintlnSwap(F("Total Charging Cycle Capacity="), tJKFAllReply->TotalBatteryCycleCapacity);
    myPrintSwap(F("# Battery Cells="), tJKFAllReply->NumberOfBatteryCells); // 0x8A Total number of battery strings
    myPrintln(F(", Cell Count="), tJKFAllReply->BatteryCellCount); // 0xA9 Battery string count settings
    Serial.println();
}

void printBMSInfo() {
    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;

    myPrintln(F("Protocol Version Number="), tJKFAllReply->ProtocolVersionNumber);
    Serial.print(F("Software Version Number="));
    tJKFAllReply->TokenStartCurrentCalibration = '\0'; // set end of string token
    Serial.println(tJKFAllReply->SoftwareVersionNumber);
    Serial.print(F("Modify Parameter Password="));
    tJKFAllReply->TokenDedicatedChargerSwitchState = '\0'; // set end of string token
    Serial.println(tJKFAllReply->ModifyParameterPassword);

    myPrintln(F("# External Temperature Sensors="), tJKFAllReply->NumberOfTemperatureSensors); // 0x86

    Serial.println();
}

void printMiscelaneousInfo() {
    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;

    myPrintlnSwap(F("Balance Starting Cell Voltage=[mV]"), tJKFAllReply->BalancingStartMillivolt);
    myPrintlnSwap(F("Balance Opening Voltage Difference[mV]="), tJKFAllReply->BalancingStartDifferentialMillivolt); // ??? semantics
    Serial.println();
    myPrintlnSwap(F("Current Calibration[mA]="), tJKFAllReply->CurrentCalibrationMilliampere);
    myPrintlnSwap(F("Sleep Wait Time[s]="), tJKFAllReply->SleepWaitingTimeSeconds);
    Serial.println();
    myPrintln(F("Dedicated Charge Switch Active="), tJKFAllReply->DedicatedChargerSwitchIsActive);
    myPrintln(F("Start Current Calibration State="), tJKFAllReply->StartCurrentCalibration);
    myPrintlnSwap(F("Battery Actual Capacity[Ah]="), tJKFAllReply->ActualBatteryCapacityAmpereHour);
    Serial.println();
}

/*
 * Token 0x8B. Prints info only if errors existent and changed from last value
 * Stores error string for LCD in ErrorStringForLCD
 */
void printAlarmInfo() {
    static uint16_t sLastAlarms = 0;
    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;

    uint16_t tAlarms = swap(tJKFAllReply->AlarmUnion.AlarmsAsWord);

    if (sLastAlarms != tAlarms) {
        sLastAlarms = tAlarms;
        if (tAlarms == 0) {
            ErrorStringForLCD = NULL;
            sForcePrintUpTime = true; // to force overwriting of alarm
        }
    }

    if (tAlarms != 0) {
//        sLastAlarms = tAlarms;
        Serial.println(F("*** ALARM FLAGS ***"));

        Serial.print(F("Alarm bits=0b"));
        Serial.println(tAlarms, BIN);

        uint16_t tAlarmMask = 1;
        for (uint_fast8_t i = 0; i < NUMBER_OF_DEFINED_ALARM_BITS; ++i) {
            if (tAlarms & tAlarmMask) {
                Serial.print(F("Alarm bit=0b"));
                Serial.print(tAlarmMask, BIN);
                Serial.print(F(" -> "));
                const char *tErrorStringPtr = (char*) (pgm_read_word(&JK_BMSErrorStringsArray[i]));
                ErrorStringForLCD = tErrorStringPtr;
                Serial.println(reinterpret_cast<const __FlashStringHelper*>(tErrorStringPtr));
            }
            tAlarmMask <<= 1;
        }
    }
    Serial.println();

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
 * Token 0x8C. Print if changed
 * NOT YET USED
 */
void printStatusFlags() {
    static uint16_t sLastStatus = 0;

    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;
//    Serial.print(F("StatusAsWord="));
//    Serial.println(tJKFAllReply->StatusUnion.StatusAsWord);

    uint16_t tStatus = swap(tJKFAllReply->StatusUnion.StatusAsWord);
    if (sLastStatus != tStatus) {
        sLastStatus = tStatus;
        Serial.println(F("*** STATUS FLAGS ***"));
        Serial.print(F("Charging MosFet"));
        printActiveState(tJKFAllReply->StatusUnion.StatusBits.ChargeMosFetActive);
        Serial.print(F(", Discharging MosFet"));
        printActiveState(tJKFAllReply->StatusUnion.StatusBits.DischargeMosFetActive);
        Serial.print(F(", Balancer"));
        printActiveState(tJKFAllReply->StatusUnion.StatusBits.BalancerActive);
        Serial.print(F(", Shutdown"));
        printActiveState(tJKFAllReply->StatusUnion.StatusBits.BatteryDown);
        Serial.println();
        Serial.println();
    }
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
    printMiscelaneousInfo();
}

/*
 * Print received data
 * Use converted cell voltage info from JKConvertedCellInfo
 * All other data are used unconverted and are therefore printed by swap() functions.
 */
void printJKDynamicInfo() {

    JKReplyStruct *tJKFAllReply = sJKFAllReplyPointer;

    Serial.println(F("*** CELL INFO ***"));
    printJKCellInfo();

    Serial.println(F("*** DYNAMIC INFO ***"));
    uint32_t tSystemWorkingMinutes = swap(tJKFAllReply->SystemWorkingMinutes);
    // 1 kByte for sprintf
    sprintf_P(sUpTimeString, PSTR(" -> %4uD%2uH%2uM"), (uint16_t) (tSystemWorkingMinutes / (60 * 24)),
            (uint16_t) ((tSystemWorkingMinutes / 60) % 24), (uint16_t) tSystemWorkingMinutes % 60);
    if (sLastUpTimeCharacter != sUpTimeString[13]) {
        sLastUpTimeCharacter = sUpTimeString[13];
        sForcePrintUpTime = true;
        myPrint(F("Total Runtime Minutes="), tSystemWorkingMinutes);
        Serial.println(sUpTimeString);
    }

    /*
     * Temperatures
     */
#if defined(LOCAL_DEBUG)
    Serial.print(F("TokenTemperaturePowerMosFet=0x"));
    Serial.println(tJKFAllReply->TokenTemperaturePowerMosFet, HEX);
#endif
    myPrint(F("Temperature: Power MosFet="), JKComputedData.TemperaturePowerMosFet);
    myPrint(F(", Sensor 1="), JKComputedData.TemperatureSensor1);
    myPrintln(F(", Sensor 2="), JKComputedData.TemperatureSensor2);
    Serial.println();

    /*
     * Capacity
     */
    myPrint(F("SOC[%]="), tJKFAllReply->SOCPercent);
    myPrintln(F(" -> Remaining Capacity[Ah]="), JKComputedData.RemainingCapacityAmpereHour);

    Serial.print(F("Battery Voltage[V]="));
    Serial.print(JKComputedData.BatteryVoltageFloat, 2);
    Serial.print(F(", Current[A]="));
    Serial.print(JKComputedData.BatteryLoadCurrentFloat, 2);
    myPrintln(F(", Power[W]="), JKComputedData.BatteryLoadPower);
    Serial.println();

    if (sJKFAllReplyPointer->StatusUnion.StatusBits.ChargeMosFetActive) {

    }
    Serial.print(F("Charging MosFet"));
    printEnabledState(tJKFAllReply->ChargeIsEnabled);
    Serial.print(',');
    printActiveState(tJKFAllReply->StatusUnion.StatusBits.ChargeMosFetActive);
    Serial.print(F(" | Discharging MosFet"));
    printEnabledState(tJKFAllReply->DischargeIsEnabled);
    Serial.print(',');
    printActiveState(tJKFAllReply->StatusUnion.StatusBits.DischargeMosFetActive);
    Serial.print(F(" | Balancing"));
    printEnabledState(tJKFAllReply->BalancingIsEnabled);
    Serial.print(',');
    printActiveState(tJKFAllReply->StatusUnion.StatusBits.BalancerActive);

    Serial.println();

    printAlarmInfo();

}
#endif // _JK_BMS_H
