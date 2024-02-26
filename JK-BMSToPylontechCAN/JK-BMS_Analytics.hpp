/*
 * JK-BMS_Analytics.hpp
 *
 * Functions for computing the capacity and storing and displaying the SOC graph
 *
 *
 *  Copyright (C) 2024  Armin Joachimsmeyer
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

#ifndef _JK_BMS_ANALYTICS_HPP
#define _JK_BMS_ANALYTICS_HPP

#include <Arduino.h>

#include "JK-BMS_Analytics.h"

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#endif

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

/*
 * Just clear the complete EEPROM
 */
void updateEEPROMTo_FF() {
//    if (!sOnlyPlotterOutput) {
//        Serial.println(F("Clear EEPROM"));
//    }
    for (int i = 0; i <= E2END; ++i) {
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
    tMinimumSOCData.AverageAmpere = tCurrentSOCDataPoint.AverageAmpere;
    tMaximumSOCData.AverageAmpere = tCurrentSOCDataPoint.AverageAmpere;
//    float tDeltaTimeMinutes = 0;

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
        if (tMinimumSOCData.AverageAmpere > tCurrentSOCDataPoint.AverageAmpere) {
            tMinimumSOCData.AverageAmpere = tCurrentSOCDataPoint.AverageAmpere;
        } else if (tMaximumSOCData.AverageAmpere < tCurrentSOCDataPoint.AverageAmpere) {
            tMaximumSOCData.AverageAmpere = tCurrentSOCDataPoint.AverageAmpere;
        }
        if (tMinimumSOCData.VoltageDifferenceToEmpty40Millivolt > tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt) {
            tMinimumSOCData.VoltageDifferenceToEmpty40Millivolt = tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt;
        } else if (tMaximumSOCData.VoltageDifferenceToEmpty40Millivolt < tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt) {
            tMaximumSOCData.VoltageDifferenceToEmpty40Millivolt = tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt;
        }

//        if (tCurrentSOCDataPoint.AverageAmpere != 0) {
//            tDeltaTimeMinutes = (tCurrentSOCDataPoint.Delta100MilliampereHour * 6.0) / tCurrentSOCDataPoint.AverageAmpere;
//        }

        for (uint_fast8_t j = 0; j < 2; ++j) {
            Serial.print(tCurrentSOCDataPoint.SOCPercent);
            Serial.print(' ');
            Serial.print((tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt * 4) / 10);
            Serial.print(' ');
            Serial.print(constrain(tCurrentSOCDataPoint.Delta100MilliampereHour, -30, 90)); // clip display to -30 and 90
            Serial.print(' ');
            Serial.print(tCurrentSOCDataPoint.AverageAmpere); // print ampere
            Serial.print(' ');
            Serial.print(tCurrentCapacity100MilliampereHour / 10); // print Ah
//            Serial.print(' ');
//            Serial.print(tDeltaTimeMinutes, 1);
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
        Serial.print(F(" Current_"));
        Serial.print(tMinimumSOCData.AverageAmpere);
        Serial.print(F("A->"));
        Serial.print(tMaximumSOCData.AverageAmpere);
        Serial.print(F("A:"));
        Serial.print(tCurrentSOCDataPoint.AverageAmpere);
        Serial.print(F(" Capacity_"));
        uint16_t tTotalCapacity = ((tMaximumSOCData.Capacity100MilliampereHour - tMinimumSOCData.Capacity100MilliampereHour) * 10)
                / (tMaximumSOCData.SOCPercent - tMinimumSOCData.SOCPercent);
        Serial.print(tTotalCapacity);
        Serial.print(F("Ah_"));
        Serial.print(tMinimumSOCData.Capacity100MilliampereHour / 10); // Minimum Ah
        Serial.print(F("->"));
        Serial.print(tMaximumSOCData.Capacity100MilliampereHour / 10); // Maximum Ah
        Serial.print(F("_"));
        Serial.print((tMaximumSOCData.Capacity100MilliampereHour - tMinimumSOCData.Capacity100MilliampereHour) / 10); // Delta Ah
        Serial.print(F("Ah:"));
        Serial.print(tCurrentCapacity100MilliampereHour / 10); // print Ah
//        Serial.print(F(" Delta_time[min]:"));
//        Serial.print(tDeltaTimeMinutes, 1);
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
 * Compute delta values and write them to EEPROM if SOC changed
 * Is called for every new dataset
 * Values are taken at the lower edge of SOC, e.g. at SOC = 10.001
 */
void writeSOCData() {
    auto tCurrentSOCPercent = sJKFAllReplyPointer->SOCPercent;
    SOCDataPointDeltaStruct tSOCDataPoint;

    /*
     * For SOC graph
     * Values are taken at the lower edge of SOC, e.g. at SOC = 10.001
     */
    SOCDataPointsInfo.DeltaAccumulator10Milliampere += JKComputedData.Battery10MilliAmpere;
    SOCDataPointsInfo.Accumulator10Milliampere += JKComputedData.Battery10MilliAmpere;
    SOCDataPointsInfo.NumberOfSamples++;
#if defined(LOCAL_DEBUG)
    Serial.print(F("NumberOfSamples="));
    Serial.print(SOCDataPointsInfo.DeltaAccumulator10Milliampere);
    Serial.print(F(", DeltaAccumulator10Milliampere="));
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

        // Compute current value and adjust accumulator
        tSOCDataPoint.Delta100MilliampereHour = SOCDataPointsInfo.DeltaAccumulator10Milliampere
                / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10);
        SOCDataPointsInfo.DeltaAccumulator10Milliampere -= tSOCDataPoint.Delta100MilliampereHour
                * (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10); // We can have a residual of up to 18000 after write
        // compute rounded average ampere
        // TODO handle overflow
        tSOCDataPoint.AverageAmpere = (SOCDataPointsInfo.Accumulator10Milliampere + (SOCDataPointsInfo.NumberOfSamples * 50))
                / (SOCDataPointsInfo.NumberOfSamples * 100);

        /*
         * Write to eeprom
         */
        uint8_t tSOCDataPointsArrayNextWriteIndex = (tSOCDataPointsArrayLastWriteIndex + 1) % NUMBER_OF_SOC_DATA_POINTS;
        eeprom_write_block(&tSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayNextWriteIndex], sizeof(tSOCDataPoint));

        JK_INFO_PRINT(F("SOC data write at index "));
        JK_INFO_PRINT(tSOCDataPointsArrayNextWriteIndex);
        JK_INFO_PRINT(F(", current length="));
        JK_INFO_PRINT(SOCDataPointsInfo.ArrayLength);
        JK_INFO_PRINT(F(", NumberOfSamples="));
        JK_INFO_PRINT(SOCDataPointsInfo.NumberOfSamples);
        JK_INFO_PRINT(F(", SOCPercent="));
        JK_INFO_PRINT(tSOCDataPoint.SOCPercent);
        JK_INFO_PRINT(F(", VoltageDifferenceToEmpty40Millivolt="));
        JK_INFO_PRINT(tSOCDataPoint.VoltageDifferenceToEmpty40Millivolt);
        JK_INFO_PRINT(F(", AverageAmpere="));
        JK_INFO_PRINT(tSOCDataPoint.AverageAmpere);
        JK_INFO_PRINT(F(", Delta100MilliampereHour="));
        JK_INFO_PRINTLN(tSOCDataPoint.Delta100MilliampereHour);

        SOCDataPointsInfo.Accumulator10Milliampere = 0;
        SOCDataPointsInfo.NumberOfSamples = 0;
        SOCDataPointsInfo.MillisOfLastValidEntry = millis();
    }
}

/******************************************************************
 * Compute total capacity based on current and SOC
 *
 * If state is idle and current > 1 A: If current direction is from battery, start discharge computation else start charge computation.
 * If current direction is 5 times wrong, stop computation.
 * If SOC delta between start and end > 40% store value to array.
 *****************************************************************/
void computeCapacity() {
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

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _JK_BMS_ANALYTICS_HPP
