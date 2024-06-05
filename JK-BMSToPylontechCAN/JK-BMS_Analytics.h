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

/*
 * The value of AverageAccumulator10Milliampere for 1 Ah is:
 * (60 * 60 * 1000L / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) is number of samples in 1 hour -> 1800 at 1 sample / 2 seconds
 * 100 is factor for 10 mA to 1 A
 */
#define CAPACITY_10_mA_ACCUMULATOR_1_AMPERE_HOUR  (100L * 60L * 60L * 1000L / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) // 180,000

/*
 * This structure is stored to EEPROM
 */
#define SOC_EVEN_EEPROM_PAGE_INDICATION_BIT 0x80 // Set in SOCPercent if we currently write on an even page. Required to find the end of current data in cyclic buffer.
struct SOCDataPointDeltaStruct {
    uint8_t SOCPercent;
    uint8_t VoltageDifferenceToEmpty50Millivolt; // 1 = 50 mV, 255 = 12.75 V. Values > 240 to 255 / 12 V to 12.7 V are taken as negative ones, just in case it happens.
    int8_t AverageAmpere;
    int8_t Delta100MilliampereHour; // at a capacity of 320 Ah we have 3.2 Ah per 1% SOC
};
// First place of size SOCDataPointDeltaStruct is used for sBatteryESRMilliohm_EEPROM + 3 filler bytes
#define NUMBER_OF_SOC_DATA_POINTS   (((E2END + 1) - sizeof(SOCDataPointDeltaStruct)) / sizeof(SOCDataPointDeltaStruct)) // 0xFE for 1k EEPROM, 0x1FE for 2kEEPROM

struct SOCDataPointsInfoStruct {
    /*
     * Index of next value to be written is ArrayStartIndex + ArrayLength % NUMBER_OF_SOC_DATA_POINTS
     * => if array is full i.e. ArrayLength == NUMBER_OF_SOC_DATA_POINTS, index of next value to be written is ArrayStartIndex.
     */
    uint16_t ArrayStartIndex;   // Index of first data entry in cyclic SOCDataPointsEEPROMArray. Index of next value to be written if ArrayLength == NUMBER_OF_SOC_DATA_POINTS.
    uint16_t ArrayLength;       // Length of valid data in Array. Required if not fully written. Maximum is NUMBER_OF_SOC_DATA_POINTS
    bool currentlyWritingOnAnEvenPage; // If true SOC_EVEN_EEPROM_PAGE_INDICATION_BIT is set in SOCPercent.
    uint16_t NumberOfSamples = 0; // For one sample each 2 seconds, we can store up to 36.4 hours here.
    long AverageAccumulatorVoltageDifferenceToEmpty10Millivolt = 0; // Serves as accumulator to enable a more smooth graph.
    long AverageAccumulator10Milliampere = 0; // Serves as accumulator for AverageAmpere
    long DeltaAccumulator10Milliampere = 0; // Serves as accumulator to avoid rounding errors for consecutive data points of Delta100MilliampereHour. 1 Ah is 180,000 => Can hold values of +/-11930 Ah. We can have a residual of up to 18,000 (100 mAh) after write.
    long lastWrittenBatteryCapacityAccumulator10Milliampere = 0;
};
extern SOCDataPointsInfoStruct SOCDataPointsInfo;

struct SOCDataPointMinMaxStruct {
    uint8_t SOCPercent;
    int16_t VoltageDifferenceToEmpty50Millivolt;
    int16_t CapacityAmpereHour;
    int8_t AverageAmpere;
};

void initializeAnalytics();
void updateEEPROMTo_FF();
void writeSOCData();
void findFirstSOCDataPointIndex();
void readBatteryESRfromEEPROM();
void readAndPrintSOCData();

#endif // _JK_BMS_ANALYTICS_H
