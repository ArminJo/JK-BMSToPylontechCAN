/*
 * JK-BMS_Analytics.h
 *
 * Definitions of the data structures used by JK-BMS_Analytics
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
#ifndef _JK_BMS_ANALYTICS_H
#define _JK_BMS_ANALYTICS_H

#  if !defined(USE_NO_LCD)
#define SIZE_OF_COMPUTED_CAPACITY_ARRAY         4 // LCD_ROWS
#  else
#define SIZE_OF_COMPUTED_CAPACITY_ARRAY         16
#  endif

struct JKComputedCapacityStruct {
    uint8_t StartSOCPercent;
    uint8_t Start100MilliVoltToEmpty; // 250 bytes program memory incl. display
    uint8_t EndSOCPercent;
    uint8_t End100MilliVoltToEmpty;
    uint16_t Capacity;
    uint16_t TotalCapacity;
};

#define CAPACITY_COMPUTATION_MODE_IDLE                  0
#define CAPACITY_COMPUTATION_MODE_CHARGE                1
#define CAPACITY_COMPUTATION_MODE_DISCHARGE             2
#define CAPACITY_COMPUTATION_MAX_WRONG_CHARGE_DIRECTION 4 // If we have 5 wrong directions, we end computation
/*
 * 60 * 60 * 1000L / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS is number of samples in 1 hour, -> 1800 at 1 sample / 2 seconds
 * 100 is factor for 10 mA to 1 A
 */
#define CAPACITY_ACCUMULATOR_1_AMPERE_HOUR  (100L * 60L * 60L * 1000L / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) // 180000

struct CapacityComputationInfoStruct {
    uint8_t WrongDirectionCount = 0;
    uint8_t Mode = CAPACITY_COMPUTATION_MODE_IDLE;
    uint32_t Accumulator10Milliampere = 0;
    uint8_t LastDeltaSOC = 0;
};
extern CapacityComputationInfoStruct sCapacityComputationInfo;

extern struct JKComputedCapacityStruct JKComputedCapacity[SIZE_OF_COMPUTED_CAPACITY_ARRAY]; // The last 4 values
void checkAndStoreCapacityComputationValues();
void printComputedCapacity(uint8_t aCapacityArrayIndex);

struct SOCDataPointStruct {
    uint8_t SOCPercent;
    uint16_t VoltageDifferenceToEmpty40Millivolt;
    int16_t Capacity100MilliampereHour; // -12 to 12 Ah per 1% SOC
    int8_t AverageAmpere;
};

/*
 * This structure is stored to EEPROM
 */
#define SOC_EVEN_EEPROM_PAGE_INDICATION_BIT 0x80 // Set in SOCPercent if we currently write on an even page. Required to find the end of current data in cyclic buffer.
struct SOCDataPointDeltaStruct {
    uint8_t SOCPercent;
    uint8_t VoltageDifferenceToEmpty40Millivolt; // 1 = 40 mV, 255 = 10.200 V
    int8_t AverageAmpere;
    int8_t Delta100MilliampereHour; // -12 to 12 Ah per 1% SOC
};
#define NUMBER_OF_SOC_DATA_POINTS   ((E2END + 1) / sizeof(SOCDataPointDeltaStruct)) // 0x100

struct SOCDataPointsInfoStruct {
    uint8_t ArrayStartIndex;   // Index of first entry in cyclic SOCDataPointsEEPROMArray, index of next value to be written.
    uint16_t ArrayLength;      // Length of valid data in Array. Required if not fully written. Maximum is NUMBER_OF_SOC_DATA_POINTS
    bool currentlyWritingOnAnEvenPage; // If true SOC_EVEN_EEPROM_PAGE_INDICATION_BIT is set in SOCPercent.
    uint8_t lastSOCPercent;     // for detecting transition from 0 to 1.
    long MillisOfLastValidEntry;
    uint16_t NumberOfSamples = 0; // For one sample each 2 seconds, we can store up to 36.4 hours here.
    long Accumulator10Milliampere = 0; // Serves as accumulator for AverageAmpere
    long DeltaAccumulator10Milliampere = 0; // Serves as accumulator to avoid rounding errors for consecutive data points of Delta100MilliampereHour. We can have a residual of up to 18000 after write.
};
extern SOCDataPointsInfoStruct SOCDataPointsInfo;

void updateEEPROMTo_FF();
void computeCapacity();
void writeSOCData();
void findFirstSOCDataPointIndex();
void readAndPrintSOCData();

#endif // _JK_BMS_ANALYTICS_H
