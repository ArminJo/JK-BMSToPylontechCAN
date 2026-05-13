/*
 * JK-BMS_CapacityInfo.hpp
 *
 * Functions for computing the capacity deltas and their corresponding SOC and voltage deltas
 *
 *  Copyright (C) 2024-2026  Armin Joachimsmeyer
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

#ifndef _JK_BMS_CAPACITY_INFO_HPP
#define _JK_BMS_CAPACITY_INFO_HPP

#include <Arduino.h>
#include <limits.h>

#include "JK-BMS_CapacityInfo.h"

#include "LocalDebugLevelCheck.h"
// This block must be located after the includes of other *.hpp files
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
//#define LOCAL_TRACE // This enables trace output only for this file - only for development
#include "LocalDebugLevelStart.h"

#if !defined(NO_CAPACITY_INFO)
/*
 * The first entry [0] holds the current computed value if more than 1 Ah are accumulated
 */
#  if defined(STANDALONE_TEST)
struct JKComputedCapacityStruct JKComputedCapacity[SIZE_OF_COMPUTED_CAPACITY_ARRAY] = { {0, 0, 20, 15, 80, 100}, {10, 55, 99, 10,
        40, 42}, {100, 5, 20, 45, 120, 150}, {20, 45, 100, 5, 120, 150}};
#else
struct JKComputedCapacityStruct JKComputedCapacity[SIZE_OF_COMPUTED_CAPACITY_ARRAY];
#  endif
CapacityComputationInfoStruct sCapacityComputationInfo;
#endif

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

/*
 * Compute total capacity based on current and SOC
 *
 * If state is idle and current > 1 A: If current direction is from battery, start discharge computation else start charge computation.
 * If current direction is 5 times wrong, stop computation.
 * If SOC delta between start and end > 40% store value to array.
 */
void fillCapacityInfo() {
    auto tCurrentSOCPercent = JK_BMS_1.JKAllReplyPointer->SOCPercent;
    auto tBattery10MilliAmpere = JK_BMS_1.JKComputedData.Battery10MilliAmpere;

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

        JKComputedCapacity[0].Start100MilliVoltToEmpty = JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 10;
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
            JKComputedCapacity[0].End100MilliVoltToEmpty = JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 10;
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

#include "LocalDebugLevelEnd.h"
#endif // _JK_BMS_CAPACITY_INFO_HPP
