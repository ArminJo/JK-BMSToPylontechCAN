/*
 * JK-BMS_Analytics.hpp
 *
 * Functions for computing the capacity and storing and displaying the SOC graph
 *
 * Automatic computation of ESR (Equivalent Series Resistor) = delta voltage / delta current by function readAndPrintSOCData().
 * Voltage output in graph is compensated by ESR.
 * Printout of the uncompensated voltage is forced by connecting pin DISABLE_ESR_IN_GRAPH_OUTPUT_PIN (8) to low.
 *
 * The right ESR if found by computing the sum of deltas for current ESR and ESR + 1 and ESR - 1.
 * If ESR + 1 or ESR - 1 deltas are at least 3 smaller than the current value, then the computation is run again with the ESR of the smaller value.
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

//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#include "LocalDebugLevelStart.h"

EEMEM uint8_t sBatteryESRMilliohm_EEPROM;
uint8_t sBatteryESRMilliohm;
// To consume the rest of the first 4 bytes (of a former SOCDataPointDeltaStruct)
volatile EEMEM uint8_t sFiller1_EEPROM;
volatile EEMEM uint8_t sFiller2_EEPROM;
volatile EEMEM uint8_t sFiller3_EEPROM;

/*
 * Cyclic buffer start is SOCDataPointsInfo.ArrayStartIndex and length (required if not fully written) SOCDataPointsInfo.ArrayLength
 */
EEMEM SOCDataPointDeltaStruct SOCDataPointsEEPROMArray[NUMBER_OF_SOC_DATA_POINTS]; // 255 for 1 kB EEPROM
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

void readBatteryESRfromEEPROM() {
    sBatteryESRMilliohm = eeprom_read_byte(&sBatteryESRMilliohm_EEPROM);
    if (sBatteryESRMilliohm > 0x30) {
        sBatteryESRMilliohm = 10; // initial value, if EEPROM was not initialized
    }
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
            DEBUG_PRINT(F("Found even/odd toggling at index="));
            DEBUG_PRINT(i);
            tSOCDataPointsArrayStartIndex = i;
            break;
        }
    }

    DEBUG_PRINT(F(" even="));
    DEBUG_PRINTLN(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
    SOCDataPointsInfo.ArrayStartIndex = tSOCDataPointsArrayStartIndex;
    SOCDataPointsInfo.ArrayLength = tSOCDataPointsArrayLength;
}

/*
 * Read and print SOC EEPROM data for Arduino Plotter
 * Compute ESR.
 */
void readAndPrintSOCData() {
    if (SOCDataPointsInfo.ArrayLength == 0) {
        return;
    }

    while (true) {
        SOCDataPointMinMaxStruct tMinimumSOCData;
        tMinimumSOCData.SOCPercent = UINT8_MAX;
        tMinimumSOCData.VoltageDifferenceToEmpty50Millivolt = INT16_MAX;
        tMinimumSOCData.CapacityAmpereHour = INT16_MAX;
        tMinimumSOCData.AverageAmpere = INT8_MAX;

        SOCDataPointMinMaxStruct tMaximumSOCData;
        tMaximumSOCData.SOCPercent = 0;
        tMaximumSOCData.VoltageDifferenceToEmpty50Millivolt = INT16_MIN;
        tMaximumSOCData.CapacityAmpereHour = INT16_MIN;
        tMaximumSOCData.AverageAmpere = INT8_MIN;

// Read first block
        SOCDataPointDeltaStruct tCurrentSOCDataPoint;
        auto tSOCDataPointsArrayIndex = SOCDataPointsInfo.ArrayStartIndex;
        int16_t tCurrentCapacityAmpereHour;
        int16_t tCurrentCapacity100MilliampereHour;
        int16_t tVoltageDifferenceToEmpty50Millivolt;

        /*
         * For automatic ESR computation
         */
        uint16_t tVoltToEmptyAccumulatedDeltasESR = 0;
        uint16_t tVoltToEmptyAccumulatedDeltasESRPlus1 = 0;
        uint16_t tVoltToEmptyAccumulatedDeltasESRMinus1 = 0;
        int16_t tLastVoltageDifferenceToEmpty50MillivoltESR;
        int16_t tLastVoltageDifferenceToEmpty50MillivoltESRPlus1;
        int16_t tLastVoltageDifferenceToEmpty50MillivoltESRMinus1;

        uint8_t tLastSOCPercent = tCurrentSOCDataPoint.SOCPercent;

//    float tDeltaTimeMinutes = 0;

        /*
         * Print more than 500 data points in order to shift unwanted entries out to left of plotter window
         */
        Serial.println(F("Print SOC data with each entry printed twice"));
        if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) == LOW) {
            Serial.println(F("No battery ESR compensation for voltage"));
        }

        uint16_t i;
        for (i = 0; i < SOCDataPointsInfo.ArrayLength; ++i) {
            eeprom_read_block(&tCurrentSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayIndex],
                    sizeof(tCurrentSOCDataPoint));
            tCurrentSOCDataPoint.SOCPercent &= ~SOC_EVEN_EEPROM_PAGE_INDICATION_BIT; // remove indication bit

            /*
             * This printout will crash the graph!
             */
            DEBUG_PRINT(tSOCDataPointsArrayIndex);
            DEBUG_PRINT(F(" even="));
            DEBUG_PRINT((bool) (tLastSOCPercent & SOC_EVEN_EEPROM_PAGE_INDICATION_BIT));
            DEBUG_PRINT(F(", SOC="));
            DEBUG_PRINT(tCurrentSOCDataPoint.SOCPercent);
            DEBUG_PRINT(F(", Diff50mV="));
            DEBUG_PRINT(tCurrentSOCDataPoint.VoltageDifferenceToEmpty50Millivolt);
            DEBUG_PRINT(F(", AverageA="));
            DEBUG_PRINT(tCurrentSOCDataPoint.AverageAmpere);
            DEBUG_PRINT(F(", Delta100mAh="));
            DEBUG_PRINTLN(tCurrentSOCDataPoint.Delta100MilliampereHour);

            if (i == 0) {
                // initialize capacity with a reasonable value
                tCurrentCapacityAmpereHour = (tCurrentSOCDataPoint.SOCPercent * JKComputedData.TotalCapacityAmpereHour) / 100;
                tCurrentCapacity100MilliampereHour = (tCurrentSOCDataPoint.SOCPercent * JKComputedData.TotalCapacityAmpereHour)
                        / 10;
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

            tVoltageDifferenceToEmpty50Millivolt = tCurrentSOCDataPoint.VoltageDifferenceToEmpty50Millivolt;
            if (tVoltageDifferenceToEmpty50Millivolt > 240) {
                // Interpret values > 12 V as negative ones
                tVoltageDifferenceToEmpty50Millivolt = (int8_t) tCurrentSOCDataPoint.VoltageDifferenceToEmpty50Millivolt;
            }

            /*
             * Compensate voltage with Current * ESR
             */
            if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) != LOW) {
                /*
                 * Compute sum of delta values for (sBatteryESRMilliohm + 1)
                 */
                int16_t tVoltageDifferenceToEmpty50MillivoltPlus1 = tVoltageDifferenceToEmpty50Millivolt
                        - (((int16_t) tCurrentSOCDataPoint.AverageAmpere * (sBatteryESRMilliohm + 1)) / 50);
                if (i != 0) {
                    tVoltToEmptyAccumulatedDeltasESRPlus1 += abs(
                            tVoltageDifferenceToEmpty50MillivoltPlus1 - tLastVoltageDifferenceToEmpty50MillivoltESRPlus1);
                }
                tLastVoltageDifferenceToEmpty50MillivoltESRPlus1 = tVoltageDifferenceToEmpty50MillivoltPlus1;

                /*
                 * Compute sum of delta values for (sBatteryESRMilliohm - 1)
                 */
                int16_t tVoltageDifferenceToEmpty50MillivoltMinus1 = tVoltageDifferenceToEmpty50Millivolt
                        - (((int16_t) tCurrentSOCDataPoint.AverageAmpere * (sBatteryESRMilliohm - 1)) / 50);
                if (i != 0) {
                    tVoltToEmptyAccumulatedDeltasESRMinus1 += abs(
                            tVoltageDifferenceToEmpty50MillivoltMinus1 - tLastVoltageDifferenceToEmpty50MillivoltESRMinus1);
                }
                tLastVoltageDifferenceToEmpty50MillivoltESRMinus1 = tVoltageDifferenceToEmpty50MillivoltMinus1;

                /*
                 * Compute sum of delta values for sBatteryESRMilliohm
                 */
                tVoltageDifferenceToEmpty50Millivolt -= (((int16_t) tCurrentSOCDataPoint.AverageAmpere * sBatteryESRMilliohm) / 50);
                if (i != 0) {
                    tVoltToEmptyAccumulatedDeltasESR += abs(
                            tVoltageDifferenceToEmpty50Millivolt - tLastVoltageDifferenceToEmpty50MillivoltESR);
                }
                tLastVoltageDifferenceToEmpty50MillivoltESR = tVoltageDifferenceToEmpty50Millivolt;
            }

            if (tMinimumSOCData.VoltageDifferenceToEmpty50Millivolt > tVoltageDifferenceToEmpty50Millivolt) {
                tMinimumSOCData.VoltageDifferenceToEmpty50Millivolt = tVoltageDifferenceToEmpty50Millivolt;
            } else if (tMaximumSOCData.VoltageDifferenceToEmpty50Millivolt < tVoltageDifferenceToEmpty50Millivolt) {
                tMaximumSOCData.VoltageDifferenceToEmpty50Millivolt = tVoltageDifferenceToEmpty50Millivolt;
            }

            if (i < SOCDataPointsInfo.ArrayLength - 1) {
                for (uint_fast8_t j = 0; j < 2; ++j) {
                    Serial.print(tCurrentSOCDataPoint.SOCPercent);
                    Serial.print(' ');
                    Serial.print(tCurrentCapacityAmpereHour); // print capacity in Ah
                    Serial.print(' ');
                    Serial.print(tVoltageDifferenceToEmpty50Millivolt / 2);
                    Serial.print(' ');
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
                    Serial.print(tMinimumSOCData.VoltageDifferenceToEmpty50Millivolt / 20.0, 2);
                    Serial.print(F("V->"));
                    Serial.print(tMaximumSOCData.VoltageDifferenceToEmpty50Millivolt / 20.0, 2);
                    Serial.print(F("V_[0.1V]:"));
                    Serial.print(tVoltageDifferenceToEmpty50Millivolt / 2);
                    Serial.print(F(" Current_"));
                    Serial.print(tMinimumSOCData.AverageAmpere);
                    Serial.print(F("A->"));
                    Serial.print(tMaximumSOCData.AverageAmpere);
                    Serial.print(F("A:"));
                    Serial.print(tCurrentSOCDataPoint.AverageAmpere);

                    if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) != LOW) {
                        Serial.print(F(" ESR_"));
                        Serial.print(sBatteryESRMilliohm);
//#if defined(LOCAL_DEBUG)
                        Serial.print(F("mOhm_"));
                        Serial.print(tVoltToEmptyAccumulatedDeltasESR);
                        Serial.print('_');
                        Serial.print(tVoltToEmptyAccumulatedDeltasESRPlus1);
                        Serial.print('_');
                        Serial.print(tVoltToEmptyAccumulatedDeltasESRMinus1);
                        Serial.print(F(":0"));
//#else
//                        Serial.print(F("mOhm:0"));
//#endif
                    } else {
                        Serial.print(F(" 0"));
                    }

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

        if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) == LOW) {
            break; // no automatic ESR detection here
        }
        /*
         * Check if BatteryESR + 1 or BatteryESR - 1 deltas are at least 3 smaller than current value,
         * then run loop again with the ESR of the smaller value.
         */
        if (tVoltToEmptyAccumulatedDeltasESR - 3 >= tVoltToEmptyAccumulatedDeltasESRPlus1) {
            JK_INFO_PRINT(F("Delta of +1="));
            JK_INFO_PRINTLN(tVoltToEmptyAccumulatedDeltasESRPlus1);
            sBatteryESRMilliohm++;
        } else if (tVoltToEmptyAccumulatedDeltasESR - 3 >= tVoltToEmptyAccumulatedDeltasESRMinus1) {
            JK_INFO_PRINT(F("Delta of -1="));
            JK_INFO_PRINTLN(tVoltToEmptyAccumulatedDeltasESRPlus1);
            sBatteryESRMilliohm--;
        } else {
            break; // Current BatteryESR deltas is smaller than BatteryESR + 1 and BatteryESR - 1 deltas.
        }
        // This is NOT printed for last graph :-)
        JK_INFO_PRINT(F("Set new ESR to "));
        JK_INFO_PRINTLN(sBatteryESRMilliohm);
    };
    eeprom_update_byte(&sBatteryESRMilliohm_EEPROM, sBatteryESRMilliohm); // write final value to eeprom
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
    SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty10Millivolt +=
            JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt;
    SOCDataPointsInfo.NumberOfSamples++; // For one sample each 2 seconds, we can store up to 36.4 hours here.
    TRACE_PRINT(F("NumberOfSamples="));
    TRACE_PRINT(SOCDataPointsInfo.NumberOfSamples);
    TRACE_PRINT(F(", DeltaAccumulator10Milliampere="));
    TRACE_PRINTLN(SOCDataPointsInfo.DeltaAccumulator10Milliampere);
    /*
     * Check for transition from 0 to 1, where we do not write values, but reset all accumulators
     */
    auto tCurrentSOCPercent = sJKFAllReplyPointer->SOCPercent;

    if (lastJKReply.SOCPercent == 0 && tCurrentSOCPercent == 1) {
        Serial.println(F("SOC 0 -> 1 -> clear internal accumulator values"));
        SOCDataPointsInfo.DeltaAccumulator10Milliampere = 0;
        SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty10Millivolt = 0;
        SOCDataPointsInfo.AverageAccumulator10Milliampere = 0;
        SOCDataPointsInfo.NumberOfSamples = 0;
#if !defined(DISABLE_MONITORING)
        // Special monitoring output to generate capacity to cell voltage graphs e.g. with excel
        printCSVLine('#');
        Serial.println();
#endif
        return;
    }

    uint8_t tSOCDataPointsArrayLastWriteIndex = (SOCDataPointsInfo.ArrayStartIndex + SOCDataPointsInfo.ArrayLength - 1)
            % NUMBER_OF_SOC_DATA_POINTS;
    auto tLastWrittenSOCPercent = eeprom_read_byte(&SOCDataPointsEEPROMArray[tSOCDataPointsArrayLastWriteIndex].SOCPercent);
    bool tLastWritingOnAnEvenPage = tLastWrittenSOCPercent & SOC_EVEN_EEPROM_PAGE_INDICATION_BIT;
    tLastWrittenSOCPercent &= ~SOC_EVEN_EEPROM_PAGE_INDICATION_BIT;

    if (tCurrentSOCPercent > tLastWrittenSOCPercent || tCurrentSOCPercent < (tLastWrittenSOCPercent - 1)) {
        /*
         * Insert new entry
         * First check for buffer wrap around and toggle currentlyWritingOnAnEvenPage flag
         */
        if (tSOCDataPointsArrayLastWriteIndex == (NUMBER_OF_SOC_DATA_POINTS - 1)) {
            // Here we write next entry at index 0, i.e. we start at next page, so we must switch even / odd indicator
            SOCDataPointsInfo.currentlyWritingOnAnEvenPage = !(tLastWritingOnAnEvenPage);
            JK_INFO_PRINT(F("Buffer wrap around detected, set even to "));
            JK_INFO_PRINTLN(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
        }

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

        /*
         * Compute current value and adjust accumulator
         * Use integer as intermediate value and restrict the written value to the values of a signed char
         * Adjust accumulator with the value written. We can have a residual of up to 18000 after write
         * and bigger if we have an overflow, but this only happens at the learning phase of the BMS
         */
        int tDelta100MilliampereHour = SOCDataPointsInfo.DeltaAccumulator10Milliampere / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10); // / 18000
        tDelta100MilliampereHour = constrain(tDelta100MilliampereHour, SCHAR_MIN, SCHAR_MAX);
        tSOCDataPoint.Delta100MilliampereHour = tDelta100MilliampereHour;
        SOCDataPointsInfo.DeltaAccumulator10Milliampere -= tDelta100MilliampereHour * (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10);

        // compute average volt to empty
        long tNumberOfSamplesTimes5 = SOCDataPointsInfo.NumberOfSamples * 5L;
        tSOCDataPoint.VoltageDifferenceToEmpty50Millivolt = SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty10Millivolt
                / tNumberOfSamplesTimes5;

        /*
         * Compute average ampere
         * Must use 32 bit, since number of samples can be as high as 4000
         */
        tSOCDataPoint.AverageAmpere = SOCDataPointsInfo.AverageAccumulator10Milliampere / (tNumberOfSamplesTimes5 * 20L);

        /*
         * Write to eeprom
         */
        uint8_t tSOCDataPointsArrayNextWriteIndex = (tSOCDataPointsArrayLastWriteIndex + 1) % NUMBER_OF_SOC_DATA_POINTS;
        eeprom_write_block(&tSOCDataPoint, &SOCDataPointsEEPROMArray[tSOCDataPointsArrayNextWriteIndex], sizeof(tSOCDataPoint));

#if !defined(STANDALONE_TEST)
        JK_INFO_PRINT(F("EEPROM write of "));
        JK_INFO_PRINT(SOCDataPointsInfo.NumberOfSamples);
        JK_INFO_PRINT(F(" samples to "));
        JK_INFO_PRINT(tSOCDataPointsArrayNextWriteIndex);
        if (SOCDataPointsInfo.ArrayLength != NUMBER_OF_SOC_DATA_POINTS) {
            JK_INFO_PRINT(F(", length="));
            JK_INFO_PRINT(SOCDataPointsInfo.ArrayLength);
        }
        JK_INFO_PRINT(F(", SOC="));
        JK_INFO_PRINT(tSOCDataPoint.SOCPercent & (~SOC_EVEN_EEPROM_PAGE_INDICATION_BIT));
        JK_INFO_PRINT(F("%, VoltageDifferenceToEmpty50Millivolt="));
        JK_INFO_PRINT(tSOCDataPoint.VoltageDifferenceToEmpty50Millivolt);
        JK_INFO_PRINT(F(", AverageAmpere="));
        JK_INFO_PRINT(tSOCDataPoint.AverageAmpere);
        JK_INFO_PRINT(F(", Delta100MilliampereHour="));
        JK_INFO_PRINT(tSOCDataPoint.Delta100MilliampereHour);
        DEBUG_PRINT(F(", Left100MilliampereHour=")); // correlates to number of samples
        DEBUG_PRINT(SOCDataPointsInfo.DeltaAccumulator10Milliampere / (CAPACITY_ACCUMULATOR_1_AMPERE_HOUR / 10.0), 2);
        DEBUG_PRINT(F(", even="));
        DEBUG_PRINT(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
        JK_INFO_PRINTLN();
#endif

        SOCDataPointsInfo.AverageAccumulator10Milliampere = 0;
        SOCDataPointsInfo.AverageAccumulatorVoltageDifferenceToEmpty10Millivolt = 0;
        SOCDataPointsInfo.NumberOfSamples = 0;
#if !defined(DISABLE_MONITORING)
        // Special monitoring output to generate capacity to cell voltage graphs e.g. with excel
        printCSVLine('#');
        Serial.println();
#endif
    }
}

#include "LocalDebugLevelEnd.h"
#endif // _JK_BMS_ANALYTICS_HPP
