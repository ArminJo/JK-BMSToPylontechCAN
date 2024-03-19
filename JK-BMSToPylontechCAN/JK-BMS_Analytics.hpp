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
#include <limits.h>

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

/*
 * Cyclic buffer start is SOCDataPointsInfo.ArrayStartIndex and length (required if not fully written) SOCDataPointsInfo.ArrayLength
 */
EEMEM SOCDataPointDeltaStruct SOCDataPointsEEPROMArray[NUMBER_OF_SOC_DATA_POINTS]; // 256 for 1 kB EEPROM
SOCDataPointsInfoStruct SOCDataPointsInfo;

/*
 * Just clear the complete EEPROM
 */
void updateEEPROMTo_FF() {
    for (int i = 0; i <= E2END; ++i) {
        eeprom_update_byte((uint8_t*) i, 0xFF);
    }
    SOCDataPointsInfo.ArrayStartIndex = 0;
    SOCDataPointsInfo.ArrayLength = 0;

    Serial.println(F("Cleared EEPROM"));
}

/*
 * Looks for first 0xFF entry or for a change in the SOC_EVEN_EEPROM_PAGE_INDICATION_BIT stored in SOC value.
 * Sets SOCDataPointsInfo.ArrayStartIndex and SOCDataPointsInfo.ArrayLength
 */
void findFirstSOCDataPointIndex() {

    // Default values
    uint8_t tSOCDataPointsArrayStartIndex = 0; // value if buffer was not fully written
    uint16_t tSOCDataPointsArrayLength = NUMBER_OF_SOC_DATA_POINTS; // value if SOC jump was found

    bool tStartPageIsEvenFlag;
    for (uint_fast8_t i = 0; i < NUMBER_OF_SOC_DATA_POINTS - 1; ++i) {
        uint8_t tSOCPercent = eeprom_read_byte(&SOCDataPointsEEPROMArray[i].SOCPercent);
        if (tSOCPercent == 0xFF) {
            // We found an empty entry, so EEPROM was not fully written => SOCDataPointsInfo.ArrayStartIndex is 0
            tSOCDataPointsArrayLength = i;
            break;
        }

        bool tPageIsEvenFlag = tSOCPercent & SOC_EVEN_EEPROM_PAGE_INDICATION_BIT;
        if (i == 0) {
            tStartPageIsEvenFlag = tPageIsEvenFlag; // Flag at begin of check
            SOCDataPointsInfo.currentlyWritingOnAnEvenPage = tPageIsEvenFlag;
        } else if (tStartPageIsEvenFlag ^ tPageIsEvenFlag) {
            // Here data page changes from even to odd or vice versa
#if defined(LOCAL_DEBUG)
            Serial.print(F("Found even/odd toggling at index="));
            Serial.print(i);
#endif
            tSOCDataPointsArrayStartIndex = i;
            break;
        }
    }

#if defined(LOCAL_DEBUG)
    Serial.print(F(" even="));
    Serial.println(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
#endif
    SOCDataPointsInfo.ArrayStartIndex = tSOCDataPointsArrayStartIndex;
    SOCDataPointsInfo.ArrayLength = tSOCDataPointsArrayLength;
}

/*
 * Read and print SOC EEPROM data for Arduino Plotter
 */
//#define SHOW_DELTA_CAPACITY
void readAndPrintSOCData() {
    if (SOCDataPointsInfo.ArrayLength == 0) {
        return;
    }

    SOCDataPointMinMaxStruct tMinimumSOCData;
    tMinimumSOCData.SOCPercent = UINT8_MAX;
    tMinimumSOCData.VoltageDifferenceToEmpty10Millivolt = INT16_MAX;
    tMinimumSOCData.CapacityAmpereHour = INT16_MAX;
    tMinimumSOCData.AverageAmpere = INT8_MAX;

    SOCDataPointMinMaxStruct tMaximumSOCData;
    tMaximumSOCData.SOCPercent = 0;
    tMaximumSOCData.VoltageDifferenceToEmpty10Millivolt = INT16_MIN;
    tMaximumSOCData.CapacityAmpereHour = INT16_MIN;
    tMaximumSOCData.AverageAmpere = INT8_MIN;

// Read first block
    SOCDataPointDeltaStruct tCurrentSOCDataPoint;
    auto tSOCDataPointsArrayIndex = SOCDataPointsInfo.ArrayStartIndex;
    int16_t tCurrentCapacityAmpereHour;
    int16_t tCurrentCapacity100MilliampereHour;
    int16_t tVoltageDifferenceToEmpty100Millivolt;

    uint8_t tLastSOCPercent = tCurrentSOCDataPoint.SOCPercent;

//    float tDeltaTimeMinutes = 0;

    /*
     * Print more than 500 data points in order to shift unwanted entries out to left of plotter window
     */
    Serial.println(F("Print SOC data with each entry printed twice"));

    uint16_t i;
    for (i = 0; i < SOCDataPointsInfo.ArrayLength; ++i) {
        eeprom_read_block(&tCurrentSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayIndex], sizeof(tCurrentSOCDataPoint));
        tCurrentSOCDataPoint.SOCPercent &= ~SOC_EVEN_EEPROM_PAGE_INDICATION_BIT; // remove indication bit

        if (i == 0) {
            // initialize capacity with a reasonable value
            tCurrentCapacityAmpereHour = (tCurrentSOCDataPoint.SOCPercent * JKComputedData.TotalCapacityAmpereHour) / 100;
            tCurrentCapacity100MilliampereHour = (tCurrentSOCDataPoint.SOCPercent * JKComputedData.TotalCapacityAmpereHour) / 10;
        }

        /*
         * Check for transition from 1 to 2 in order to reset capacity to value expected at 2 %.
         * Use 2 times the delta value from 1 % to 2 %.
         */
        if (tLastSOCPercent == 1 && tCurrentSOCDataPoint.SOCPercent == 2) {
            tCurrentCapacity100MilliampereHour = tCurrentSOCDataPoint.Delta100MilliampereHour;
            tMinimumSOCData.CapacityAmpereHour = 0; // Set minimum to 0
        }
        tLastSOCPercent = tCurrentSOCDataPoint.SOCPercent;

        /*
         * Compute minimum and maximum values for caption
         */
        if (tMinimumSOCData.SOCPercent > tCurrentSOCDataPoint.SOCPercent) {
            tMinimumSOCData.SOCPercent = tCurrentSOCDataPoint.SOCPercent;
        } else if (tMaximumSOCData.SOCPercent < tCurrentSOCDataPoint.SOCPercent) {
            tMaximumSOCData.SOCPercent = tCurrentSOCDataPoint.SOCPercent;
        }

        tCurrentCapacity100MilliampereHour += tCurrentSOCDataPoint.Delta100MilliampereHour;
        tCurrentCapacityAmpereHour = tCurrentCapacity100MilliampereHour / 10;
        if (tMinimumSOCData.CapacityAmpereHour > tCurrentCapacityAmpereHour) {
            tMinimumSOCData.CapacityAmpereHour = tCurrentCapacityAmpereHour;
        } else if (tMaximumSOCData.CapacityAmpereHour < tCurrentCapacityAmpereHour) {
            tMaximumSOCData.CapacityAmpereHour = tCurrentCapacityAmpereHour;
        }

        if (tMinimumSOCData.AverageAmpere > tCurrentSOCDataPoint.AverageAmpere) {
            tMinimumSOCData.AverageAmpere = tCurrentSOCDataPoint.AverageAmpere;
        } else if (tMaximumSOCData.AverageAmpere < tCurrentSOCDataPoint.AverageAmpere) {
            tMaximumSOCData.AverageAmpere = tCurrentSOCDataPoint.AverageAmpere;
        }

#if defined(BATTERY_ESR_MILLIOHM)
        if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) == LOW) {
            tVoltageDifferenceToEmpty100Millivolt = (tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt * 4) / 10;
        } else {
            tVoltageDifferenceToEmpty100Millivolt = ((tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt * 4)
                - ((tCurrentSOCDataPoint.AverageAmpere * BATTERY_ESR_MILLIOHM) / 10)) / 10;
        }

#else
        tVoltageDifferenceToEmpty100Millivolt = (tCurrentSOCDataPoint.VoltageDifferenceToEmpty40Millivolt * 4) / 10;
#endif
        if (tMinimumSOCData.VoltageDifferenceToEmpty10Millivolt > tVoltageDifferenceToEmpty100Millivolt) {
            tMinimumSOCData.VoltageDifferenceToEmpty10Millivolt = tVoltageDifferenceToEmpty100Millivolt;
        } else if (tMaximumSOCData.VoltageDifferenceToEmpty10Millivolt < tVoltageDifferenceToEmpty100Millivolt) {
            tMaximumSOCData.VoltageDifferenceToEmpty10Millivolt = tVoltageDifferenceToEmpty100Millivolt;
        }

        if (i < SOCDataPointsInfo.ArrayLength - 1) {
            for (uint_fast8_t j = 0; j < 2; ++j) {
                Serial.print(tCurrentSOCDataPoint.SOCPercent);
                Serial.print(' ');
                Serial.print(tCurrentCapacityAmpereHour); // print capacity in Ah
                Serial.print(' ');
                Serial.print(tVoltageDifferenceToEmpty100Millivolt);
                Serial.print(' ');
#if defined(SHOW_DELTA_CAPACITY)
            Serial.print(constrain(tCurrentSOCDataPoint.Delta100MilliampereHour, -30, 90)); // clip display to -30 and 90
            Serial.print(' ');
#endif
                Serial.print(tCurrentSOCDataPoint.AverageAmpere); // print ampere
                Serial.println(F(" 0 0 0 0 0 0")); // to clear unwanted entries from former prints
            }
        } else {
            // print last entry with caption
            for (uint_fast8_t j = 0; j < 2; ++j) {
                Serial.print(F("SOC="));
                Serial.print(tMinimumSOCData.SOCPercent);
                Serial.print(F("%->"));
                Serial.print(tMaximumSOCData.SOCPercent);
                Serial.print(F("%:"));
                Serial.print(tCurrentSOCDataPoint.SOCPercent);
                Serial.print(F(" Capacity_"));
                uint16_t tTotalCapacity = ((tMaximumSOCData.CapacityAmpereHour - tMinimumSOCData.CapacityAmpereHour) * 100)
                        / (tMaximumSOCData.SOCPercent - tMinimumSOCData.SOCPercent);
                Serial.print(tTotalCapacity);
                Serial.print(F("Ah_"));
                Serial.print(tMinimumSOCData.CapacityAmpereHour); // Minimum Ah
                Serial.print(F("->"));
                Serial.print(tMaximumSOCData.CapacityAmpereHour); // Maximum Ah
                Serial.print(F("_"));
                Serial.print(tMaximumSOCData.CapacityAmpereHour - tMinimumSOCData.CapacityAmpereHour); // Delta Ah
                Serial.print(F("Ah:"));
                Serial.print(tCurrentCapacityAmpereHour); // print Ah
                Serial.print(F(" VoltToEmpty_"));
                Serial.print(tMinimumSOCData.VoltageDifferenceToEmpty10Millivolt / 10.0, 2);
                Serial.print(F("V->"));
                Serial.print(tMaximumSOCData.VoltageDifferenceToEmpty10Millivolt / 10.0, 2);
                Serial.print(F("V_[0.1V]:"));
                Serial.print(tVoltageDifferenceToEmpty100Millivolt);
#if defined(SHOW_DELTA_CAPACITY)
                    Serial.print(F(" Delta_capacity[0.1Ah]:"));
                    Serial.print(constrain(tCurrentSOCDataPoint.Delta100MilliampereHour, -30, 90)); // clip display to -30 and 90
            #endif
                Serial.print(F(" Current_"));
                Serial.print(tMinimumSOCData.AverageAmpere);
                Serial.print(F("A->"));
                Serial.print(tMaximumSOCData.AverageAmpere);
                Serial.print(F("A:"));
                Serial.print(tCurrentSOCDataPoint.AverageAmpere);
#if defined(BATTERY_ESR_MILLIOHM)
                if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) != LOW) {
                    Serial.print(F(" ESR_" STR(BATTERY_ESR_MILLIOHM) "mOhm:0"));
                } else {
                    Serial.println(F(" 0"));
                }
#endif
                Serial.print(F(" Empty_voltage_"));
                Serial.print(JKComputedData.BatteryEmptyVoltage10Millivolt / 100.0, 1);
                Serial.print('_');
                Serial.print(swap(sJKFAllReplyPointer->CellUndervoltageProtectionMillivolt) / 1000.0, 2);
                Serial.println(F("V:0 _:0 _:0 _:0 _:0 _:0 _:0 _:0 _:0"));
            }
            // If not enough data points, padding to 500 data points to guarantee, that old data is shifted out
            for (; i < 249; ++i) {
                Serial.println(F(" 0 0 0 0 0 0 0 0 0 0"));
                Serial.println(F(" 0 0 0 0 0 0 0 0 0 0"));
            }
        }

        // Prepare for next block of cyclic buffer
        tSOCDataPointsArrayIndex = (tSOCDataPointsArrayIndex + 1) % NUMBER_OF_SOC_DATA_POINTS;
    }
}

/*
 * Compute delta values and write them to EEPROM if SOC changed
 * Is called for every new dataset
 * Values are taken at the lower edge of SOC.
 * E.g. values for SOC 10 are written for rising SOC at SOC = 10.001 for falling SOC at 9.990
 * => we can not write values for SOC 0!
 */
void writeSOCData() {
    SOCDataPointDeltaStruct tSOCDataPoint;

    /*
     * Accumulate values at each call
     * If we charge beyond 100% SOC, because the battery is not fully charged at the level the BMS thinks it is 100%,
     * we get an positive value for the transition from 100 % to 99 % because the new 99 % learned by the BMS is bigger than the old 100 %.
     * This can happen at the start of the system, when there was no full 0% to 100% capacity cycle
     * and the BMS has not yet learned the correct capacity for 100%.
     */
    SOCDataPointsInfo.DeltaAccumulator10Milliampere += JKComputedData.Battery10MilliAmpere; // Can hold values of +/-11930 Ah
    SOCDataPointsInfo.AverageAccumulator10Milliampere += JKComputedData.Battery10MilliAmpere;
    SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty += JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt;
    SOCDataPointsInfo.NumberOfSamples++; // For one sample each 2 seconds, we can store up to 36.4 hours here.
#if defined(LOCAL_DEBUG)
    Serial.print(F("NumberOfSamples="));
    Serial.print(SOCDataPointsInfo.DeltaAccumulator10Milliampere);
    Serial.print(F(", DeltaAccumulator10Milliampere="));
    Serial.println(SOCDataPointsInfo.DeltaAccumulator10Milliampere);
#endif
    /*
     * Check for transition from 0 to 1, where we do not write values, but reset all accumulators
     */
    auto tCurrentSOCPercent = sJKFAllReplyPointer->SOCPercent;
    if (lastJKReply.SOCPercent == 0 && tCurrentSOCPercent == 1) {
        Serial.println(F("SOC 0 -> 1 -> reset SOC data values"));
        SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty = 0;
        SOCDataPointsInfo.DeltaAccumulator10Milliampere = 0;
        SOCDataPointsInfo.AverageAccumulator10Milliampere = 0;
        SOCDataPointsInfo.NumberOfSamples = 0;
        SOCDataPointsInfo.MillisOfLastValidEntry = millis();
        return;
    }

    uint8_t tSOCDataPointsArrayLastWriteIndex = (SOCDataPointsInfo.ArrayStartIndex + SOCDataPointsInfo.ArrayLength - 1)
            % NUMBER_OF_SOC_DATA_POINTS;
    auto tLastWrittenSOCPercent = eeprom_read_byte(&SOCDataPointsEEPROMArray[tSOCDataPointsArrayLastWriteIndex].SOCPercent);
    /*
     * check for buffer wrap around and toggle currentlyWritingOnAnEvenPage flag
     */
    if (tSOCDataPointsArrayLastWriteIndex == (NUMBER_OF_SOC_DATA_POINTS - 1)) {
        // Here we write next entry at index 0, i.e. we start at next page, so we must switch even / odd indicator
        SOCDataPointsInfo.currentlyWritingOnAnEvenPage = !(tLastWrittenSOCPercent & SOC_EVEN_EEPROM_PAGE_INDICATION_BIT);
        JK_INFO_PRINT(F("Buffer wrap around detected, even="));
        JK_INFO_PRINTLN(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
    }

    tLastWrittenSOCPercent &= ~SOC_EVEN_EEPROM_PAGE_INDICATION_BIT;
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
            // Here SOC was just incremented
            tSOCDataPoint.SOCPercent = tCurrentSOCPercent;
        } else {
            /*
             * Here tCurrentSOCPercent < (SOCDataPointsEEPROMArray[tSOCDataPointsArrayIndex].SOCPercent - 1
             * SOC is decreasing, so we take values at the point just below the lower edge of the SOC, i.e. at upper edge of SOC - 1.
             * We take this as values for the SOC just above this edge :-).
             */
            tSOCDataPoint.SOCPercent = tCurrentSOCPercent + 1;
        }
        if (SOCDataPointsInfo.currentlyWritingOnAnEvenPage) {
            tSOCDataPoint.SOCPercent |= SOC_EVEN_EEPROM_PAGE_INDICATION_BIT;
        }

        // Compute current value and adjust accumulator
        int tDelta100MilliampereHour = SOCDataPointsInfo.DeltaAccumulator10Milliampere / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10); // / 18000
        tSOCDataPoint.Delta100MilliampereHour = constrain(tDelta100MilliampereHour, CHAR_MIN, CHAR_MAX);

        // We can have a residual of up to 18000 after write and bigger if we have an overflow, which only happens at the learning phase of the BMS
        SOCDataPointsInfo.DeltaAccumulator10Milliampere -= tSOCDataPoint.Delta100MilliampereHour
                * (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10);

        // compute rounded average ampere
        tSOCDataPoint.AverageAmpere = (SOCDataPointsInfo.AverageAccumulator10Milliampere + (SOCDataPointsInfo.NumberOfSamples * 50))
                / (SOCDataPointsInfo.NumberOfSamples * 100);
        // compute rounded average volt to empty
        tSOCDataPoint.VoltageDifferenceToEmpty40Millivolt = (SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty
                + (SOCDataPointsInfo.NumberOfSamples * 2)) / (SOCDataPointsInfo.NumberOfSamples * 4);

        /*
         * Write to eeprom
         */
        uint8_t tSOCDataPointsArrayNextWriteIndex = (tSOCDataPointsArrayLastWriteIndex + 1) % NUMBER_OF_SOC_DATA_POINTS;
        eeprom_write_block(&tSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayNextWriteIndex], sizeof(tSOCDataPoint));

        JK_INFO_PRINT(F("SOC data write at index "));
        JK_INFO_PRINT(tSOCDataPointsArrayNextWriteIndex);
        if (SOCDataPointsInfo.ArrayLength != NUMBER_OF_SOC_DATA_POINTS) {
            JK_INFO_PRINT(F(", current length="));
            JK_INFO_PRINT(SOCDataPointsInfo.ArrayLength);
        }
        JK_INFO_PRINT(F(", NumberOfSamples="));
        JK_INFO_PRINT(SOCDataPointsInfo.NumberOfSamples);
        JK_INFO_PRINT(F(", even="));
        JK_INFO_PRINT(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
        JK_INFO_PRINT(F(", SOCPercent="));
        JK_INFO_PRINT(tSOCDataPoint.SOCPercent & (~SOC_EVEN_EEPROM_PAGE_INDICATION_BIT));
        JK_INFO_PRINT(F(", VoltageDifferenceToEmpty40Millivolt="));
        JK_INFO_PRINT(tSOCDataPoint.VoltageDifferenceToEmpty40Millivolt);
        JK_INFO_PRINT(F(", AverageAmpere="));
        JK_INFO_PRINT(tSOCDataPoint.AverageAmpere);
        JK_INFO_PRINT(F(", Delta100MilliampereHour="));
        JK_INFO_PRINT(tSOCDataPoint.Delta100MilliampereHour);
        JK_INFO_PRINT(F(", Left100MilliampereHour="));
        JK_INFO_PRINTLN(SOCDataPointsInfo.DeltaAccumulator10Milliampere / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10.0), 2);

        /*
         * Write to eeprom
         */
        eeprom_write_block(&tSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayNextWriteIndex], sizeof(tSOCDataPoint));

        SOCDataPointsInfo.AverageAccumulator10Milliampere = 0;
        SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty = 0;
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
                    / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 100); // -> sCapacityComputationInfo.AverageAccumulator10Milliampere / 1800
            JKComputedCapacity[0].Capacity = tCapacityComputationAccumulator10MilliAmpereHour / 100;
            JKComputedCapacity[0].TotalCapacity = tCapacityComputationAccumulator10MilliAmpereHour / tDeltaSOC;
            // Direct 32 bit computation. It is 12 bytes longer
            //            JKComputedCapacity[0].Capacity = sCapacityComputationInfo.AverageAccumulator10Milliampere / CAPACITY_ACCUMULATOR_1_AMPERE_HOUR;
            //            JKComputedCapacity[0].TotalCapacity = sCapacityComputationInfo.AverageAccumulator10Milliampere
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
