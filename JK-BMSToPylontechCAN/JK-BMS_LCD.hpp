/*
 *  JK-BMS_LCD.hpp
 *
 *  Contains LCD related variables and functions
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

#ifndef _JK_BMS_LCD_HPP
#define _JK_BMS_LCD_HPP

#include "JK-BMS_LCD.h"
#include "LCDPrintUtils.hpp"

bool sSerialLCDAvailable;

#if !defined(ENABLE_MONITORING) && defined(NO_ANALYTICS)
char sStringBuffer[LCD_COLUMNS + 1];    // Only for rendering a LCD row with snprintf_P()
#endif

//#define DISPLAY_ALWAYS_ON   // Activate this, if you want the display to be always on.
#  if !defined(DISPLAY_ALWAYS_ON)
void doLCDBacklightTimeoutHandling();
bool checkAndTurnLCDOn();
bool sSerialLCDIsSwitchedOff = false;
uint16_t sFrameCounterForLCDTAutoOff = 0;
#  endif

LiquidCrystal_I2C myLCD(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

/*
 * Big numbers for LCD JK_BMS_PAGE_BIG_INFO page
 */
#define USE_SERIAL_2004_LCD             // required by LCDBigNumbers.hpp
#include "LCDBigNumbers.hpp"            // Include sources for LCD big number generation
//LCDBigNumbers bigNumberLCD(&myLCD, BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_1); // Use 2x3 numbers, 1. variant
//#define UNITS_ROW_FOR_BIG_INFO  2
LCDBigNumbers bigNumberLCD(&myLCD, BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_2); // Use 2x3 numbers, 2. variant
#define UNITS_ROW_FOR_BIG_INFO  1
const uint8_t bigNumbersTopBlock[8] PROGMEM = { 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // char 1: top block for maximum cell voltage marker
const uint8_t bigNumbersBottomBlock[8] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F }; // char 2: bottom block for minimum cell voltage marker

uint32_t sLastPageChangeMillis;

//uint8_t sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW; // Start with Overview page
uint8_t sLCDDisplayPageNumber = JK_BMS_START_PAGE; // Start with Big Info page
uint8_t sToggleDisplayCounter;            // counter for cell statistics page to determine max or min page and for capacity page

/*
 * Since over and undervoltage is indicated by O or U in state info, it is not necessary to switch to Overview page
 */
//#define ENABLE_OVER_AND_UNDER_VOLTAGE_WARNING_ON_LCD // Enables switching to Overview page and showing over- and undervoltage data. Does not suppress the beeps for it!
bool sShowAlarmInsteadOfOverview = false; // is reset on button press

void setupLCD() {

    /*
     * Initialize I2C and check for bus lockup
     */
    if (!i2c_init()) {
        Serial.println(F("I2C init failed"));
    }

    /*
     * Check for LCD connected
     */
    sSerialLCDAvailable = i2c_start(LCD_I2C_ADDRESS << 1);
    i2c_stop();

    if (sSerialLCDAvailable) {
        /*
         * Print program, version and date on the upper two LCD lines
         */
        myLCD.init();
        myLCD.clear();
        myLCD.backlight(); // Switch backlight LED on
        myLCD.setCursor(0, 0);
        myLCD.print(F("JK-BMS to CAN conv."));
        myLCD.setCursor(0, 1);
        myLCD.print(F(VERSION_EXAMPLE " " __DATE__));
        bigNumberLCD.begin(); // This creates the custom character used for printing big numbers

    } else {
        Serial.println(F("No I2C LCD connected at address " STR(LCD_I2C_ADDRESS)));
    }
}

void printDebugInfoOnLCD() {
    /*
     * Print debug pin info
     */

    if (sSerialLCDAvailable) {
#  if !defined(DISPLAY_ALWAYS_ON)
        myLCD.setCursor(0, 0);
#    if defined(DISPLAY_ON_TIME_STRING)
        myLCD.print(F("Screen timeout " DISPLAY_ON_TIME_STRING));
#    else
        myLCD.print(F("Screen timeout "));
        myLCD.print(DISPLAY_ON_TIME_STRING / 60);
        myLCD.print(F(" min"));
#    endif
#  endif
        myLCD.setCursor(0, 1);
        myLCD.print(F("Long press = debug"));
        myLCD.setCursor(0, 2);
#  if defined(STANDALONE_TEST)
        myLCD.print(F("Test: fixed BMS data"));
#  else
        myLCD.print(F("Get  BMS every " SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS " s  "));
#  endif
        myLCD.setCursor(0, 3);
        myLCD.print(F("Send CAN every " SECONDS_BETWEEN_CAN_FRAME_SEND " s  "));
        delay (LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the messages
        myLCD.clear();
    }
}

#  if !defined(DISPLAY_ALWAYS_ON)
/*
 * Called on button press, BMS communication timeout and new error
 * Always reset timeout counter!
 * @return true if LCD was switched off before
 */
bool checkAndTurnLCDOn() {
    sFrameCounterForLCDTAutoOff = 0; // Always start again to enable backlight switch off after 5 minutes

    if (sSerialLCDIsSwitchedOff) {
        /*
         * If backlight LED off, switch it on, but do not select next page, except if debug button was pressed.
         */
        myLCD.backlight();
        sSerialLCDIsSwitchedOff = false;
        JK_INFO_PRINT(F("Switch on LCD display, triggered by ")); // to be continued by caller
        return true;
    }
    return false;
}

/*
 * Display backlight handling
 */
void doLCDBacklightTimeoutHandling() {
    sFrameCounterForLCDTAutoOff++;
    if (sFrameCounterForLCDTAutoOff == 0) {
        sFrameCounterForLCDTAutoOff--; // To avoid overflow, we have an unsigned integer here
    }

    if (sFrameCounterForLCDTAutoOff == (DISPLAY_ON_TIME_SECONDS * 1000U) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
        myLCD.noBacklight(); // switch off backlight after 5 minutes
        sSerialLCDIsSwitchedOff = true;
        if (sLCDDisplayPageNumber != JK_BMS_PAGE_CAPACITY_INFO) {
            JK_INFO_PRINTLN(F("Switch off LCD display, triggered by LCD \"ON\" timeout reached."));
        }
    }
}
#  endif

void printShortEnableFlagsOnLCD() {
    if (JK_BMS_1.JKAllReplyPointer->ChargeIsEnabled) {
        myLCD.print('C');
    } else {
        myLCD.print(' ');
    }
    if (JK_BMS_1.JKAllReplyPointer->ChargeIsEnabled) {
        myLCD.print('D');
    } else {
        myLCD.print(' ');
    }
    if (JK_BMS_1.JKAllReplyPointer->BalancingIsEnabled) {
        myLCD.print('B');
    }
}

/*
 * Prints Charge, Discharge and Balancing flag
 * If Overvoltage, C is replaced by O
 * If Undervoltage, D is Replaced by U
 */
void printShortStateOnLCD() {

    if (JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.ChargeOvervoltageAlarm) {
        myLCD.print('O');
    } else if (JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive) {
        myLCD.print('C');
    } else {
        myLCD.print(' ');
    }

    if (JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.DischargeUndervoltageAlarm) {
        myLCD.print('U');
    } else if (JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusBits.DischargeMosFetActive) {
        myLCD.print('D');
    } else {
        myLCD.print(' ');
    }

    if (JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
        myLCD.print('B');
    } else {
        myLCD.print(' ');
    }
}

void printLongStateOnLCD() {
    if (JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive) {
        myLCD.print(F("CH "));
    } else {
        myLCD.print(F("   "));
    }

    if (JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusBits.DischargeMosFetActive) {
        myLCD.print(F("DC "));
    } else {
        myLCD.print(F("   "));
    }

    if (JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
        myLCD.print(F("BAL"));
    }
}

/*
 * Print the actual HEX alarm bits in the last 4 characters
 * or state, if no alarm or under or overvoltage alarm, which is printed also in state
 */
void printAlarmHexOrStateOnLCD() {
    if ((JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord
            & ~(MASK_OF_CHARGING_OVERVOLTAGE_ALARM_UNSWAPPED | MASK_OF_DISCHARGING_UNDERVOLTAGE_ALARM_UNSWAPPED)) == 0) {
        myLCD.setCursor(17, 3); // Last 3 characters are the actual states
        printShortStateOnLCD();
    } else {
        myLCD.setCursor(16, 3); // Last 4 characters are the actual HEX alarm bits
        uint16_t tAlarms = swap(JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord);
        if (tAlarms < 0x100) {
            myLCD.print(F("0x"));
            if (tAlarms < 0x10) {
                myLCD.print('0'); // leading 0
            }
        } else if (tAlarms < 0x1000) {
            myLCD.print('0'); // leading 0
        }
        myLCD.print(tAlarms, HEX);
    }
}

/*
 * We can display only up to 16 cell values on the LCD :-(
 * Print Info in last 3 columns
 *  SOC
 *  Current
 *  " A  " or " A B"
 *  Voltage difference
 */
void printCellInfoOnLCD() {
    uint_fast8_t tRowNumber;
    auto tNumberOfCellInfoEntries = JK_BMS_1.JKConvertedCellInfo.ActualNumberOfCellInfoEntries;
//#define SHOW_SHORT_CELL_VOLTAGES // Print 3 digits cell voltage (value - 3.0 V) on Cell Info page. Enables display of up to 20 voltages or additional information.
#if defined(SHOW_SHORT_CELL_VOLTAGES)
    if (tNumberOfCellInfoEntries > 15) {
        tRowNumber = 0;
        if (tNumberOfCellInfoEntries > 20) {
            tNumberOfCellInfoEntries = 20;
        }
    } else {
        myLCD.print(F("    -CELL INFO-"));
        tRowNumber = 1;
    }

    for (uint8_t i = 0; i < tNumberOfCellInfoEntries; ++i) {
        if ((tNumberOfCellInfoEntries <= 16 && i % 4 == 0) || (tNumberOfCellInfoEntries > 16 && i % 5 == 0)) {
            if (tNumberOfCellInfoEntries <= 16) {
                /*
                 * Print info in last 3 columns before printing next line with cell voltages
                 */
                if (i == 4) {
                    // print SOC
                    snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%3u%%"), JK_BMS_1.JKAllReplyPointer->SOCPercent);
                    myLCD.print(sStringBuffer);
                } else if (i == 8) {
                    // Print current in the last 4 characters
                    printFloatValueRightAlignedOnLCD(JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat, 4);
                } else if (i == 12) {
                    // print " A  " or " A B"
                    myLCD.print(F(" A "));
                    if (JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
                        myLCD.print('B');
                    } else {
                        myLCD.print(' ');
                    }
                }
            }
            myLCD.setCursor(0, tRowNumber);
            tRowNumber++;
        }

        // print maximum or minimum indicator
        if (JK_BMS_1.JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MAXIMUM) {
            myLCD.print((char) (0x1));
        } else if (JK_BMS_1.JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MINIMUM) {
            myLCD.print((char) (0x2));
        } else {
            myLCD.print(' ');
        }
        // print fix format 3 character value
        snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%3d"), JK_BMS_1.JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt - 3000); // Value can be negative!
        myLCD.print(sStringBuffer);
    }
    if (tNumberOfCellInfoEntries > 0 && tNumberOfCellInfoEntries <= 16) {
        // print voltage difference
        myLCD.setCursor(17, 3);
        printFloatValueRightAlignedOnLCD(JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 100.0, 3, true); // true = no leading space
    }
#else
    if (tNumberOfCellInfoEntries > 12) {
        tRowNumber = 0;
        if (tNumberOfCellInfoEntries > 16) {
            tNumberOfCellInfoEntries = 16;
        }
    } else {
        myLCD.print(F("    -CELL INFO-"));
        tRowNumber = 1;
    }

    for (uint8_t i = 0; i < tNumberOfCellInfoEntries; ++i) {
        if (i % 4 == 0) {
            myLCD.setCursor(0, tRowNumber);
            tRowNumber++;
        }

        // print maximum or minimum indicator
        if (JK_BMS_1.JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MAXIMUM) {
            myLCD.print((char) (0x1));
        } else if (JK_BMS_1.JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MINIMUM) {
            myLCD.print((char) (0x2));
        } else {
            myLCD.print(' ');
        }

        myLCD.print(JK_BMS_1.JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt);
    }
#endif
}

#if !defined(NO_CELL_STATISTICS)
/*
 * Switch between display of minimum and maximum at each 4. call
 * The sum of percentages may not give 100% because of rounding errors
 */
void printCellStatisticsOnLCD() {
// check if minimum or maximum is to be displayed
    bool tDisplayCellMinimumStatistics = sToggleDisplayCounter & CELL_STATISTICS_COUNTER_MASK; // 0x04
    sToggleDisplayCounter++;

    auto tNumberOfCellInfoEntries = JK_BMS_1.JKConvertedCellInfo.ActualNumberOfCellInfoEntries;
    uint_fast8_t tRowNumber;
    if (tNumberOfCellInfoEntries > 12) {
        tRowNumber = 0;
        if (tNumberOfCellInfoEntries > 16) {
            tNumberOfCellInfoEntries = 16;
        }
    } else {
        if (tDisplayCellMinimumStatistics) {
            myLCD.print(F("CELL Minimum Percent"));
        } else {
            myLCD.print(F("CELL Maximum Percent"));
        }
        tRowNumber = 1;
    }

    char *tBalancingTimeStringPtr = &sBalancingTimeString[0];
    for (uint8_t i = 0; i < tNumberOfCellInfoEntries; ++i) {
        uint8_t tPercent;
        if (tDisplayCellMinimumStatistics) {
            tPercent = JK_BMS_1.CellStatistics.CellMinimumPercentageArray[i];
        } else {
            tPercent = JK_BMS_1.CellStatistics.CellMaximumPercentageArray[i];
        }
        if (tPercent < 10) {
            myLCD.print(' ');
        }
        myLCD.print(tPercent);
        myLCD.print(F("% "));
        if (i % 4 == 3) {
            /*
             * Use the last 4 characters of a LCD row for information
             * about min or max and the total balancing time
             */
            if (i == 3) {
                // first line
                if (tDisplayCellMinimumStatistics) {
                    myLCD.print(F("MIN"));
                } else {
                    myLCD.print(F("MAX"));
                }
            } else {
                if (i == 7) {
                    // Second line print 4 characters days
                    myLCD.print(*tBalancingTimeStringPtr++);
                } else {
                    myLCD.print(' ');
                }
                // print 3 character for hours or minutes
                myLCD.print(*tBalancingTimeStringPtr++);
                myLCD.print(*tBalancingTimeStringPtr++);
                myLCD.print(*tBalancingTimeStringPtr++);
            }
            tRowNumber++;
            myLCD.setCursor(0, tRowNumber);
        }
    }
}
#endif // !defined(NO_CELL_STATISTICS)

#if !defined(NO_ANALYTICS)
/*
 * Display the last measured capacities
 * Percentage: "100%->30%  101 130Ah"
 * Voltage: "0.5->4.5mV 101 130Ah"
 */
void printCapacityInfoOnLCD() {
    myLCD.setCursor(0, 0);
    myLCD.print(F("Print plotter graph"));
    if (SOCDataPointsInfo.ArrayLength > 1) {
        myLCD.setCursor(0, 1);
        myLCD.print(F("You can clear EEPROM"));
        myLCD.setCursor(0, 2);
        myLCD.print(F("by long press"));
        myLCD.setCursor(0, 3);
        myLCD.print(F("instead of short one"));
    }
}
#endif

void printBigInfoOnLCD() {
    bigNumberLCD.setBigNumberCursor(0, 0);
    bigNumberLCD.print(JK_BMS_1.JKAllReplyPointer->SOCPercent);
    uint8_t tColumn;
    if (JK_BMS_1.JKAllReplyPointer->SOCPercent < 10) {
        tColumn = 3;
    } else if (JK_BMS_1.JKAllReplyPointer->SOCPercent < 100) {
        tColumn = 6;
    } else {
        tColumn = 8; // 100%
    }

    myLCD.setCursor(tColumn, UNITS_ROW_FOR_BIG_INFO); // 3, 6 or 8
    myLCD.print('%'); // print "small" percent sign
    /*
     * Here we can start the big number power string at column 4, 7 or 9
     */
    uint8_t tAvailableColumns = (LCD_COLUMNS - 2) - tColumn; // 14, 11, 9. -2 for the trailing W or KW
    char tKiloWattChar = ' ';
    int16_t tBatteryLoadPower = JK_BMS_1.JKComputedData.BatteryLoadPower;
    /*
     * First print string to buffer
     */
    if (tBatteryLoadPower >= 1000 || tBatteryLoadPower <= -1000) {
        tKiloWattChar = 'k';
        float tBatteryLoadPowerFloat = tBatteryLoadPower * 0.001; // convert to kW
        dtostrf(tBatteryLoadPowerFloat, 5, 2, sStringBuffer);
    } else {
        snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%d"), JK_BMS_1.JKComputedData.BatteryLoadPower);
    }

    /*
     * Then compute maximum possible string length
     */
    uint8_t tColumnsRequiredForString = 0;
    uint8_t i = 0;
    while (true) {
        char tChar = sStringBuffer[i];
        uint8_t tCharacterWidth;
        if (tChar == '\0') {
            break; // end of string
        } else if (tChar == '.') {
            tCharacterWidth = 1; // decimal point
        } else if (tChar == '-') {
            tCharacterWidth = 2; // minus sign
        } else {
            tCharacterWidth = 3; // plain number
        }

        /*
         * Check if next character can be rendered
         */
        if (tAvailableColumns >= tCharacterWidth) {
            tColumnsRequiredForString += tCharacterWidth;
            tAvailableColumns -= tCharacterWidth;
        } else {
            /*
             * Next character cannot be rendered here
             */
            if (sStringBuffer[i - 1] == '.') {
                // do not render trailing decimal point
                tColumnsRequiredForString--;
                tAvailableColumns++;
                i--;
            }
            // Terminate string and exit
            sStringBuffer[i] = '\0';
            break;
        }
        i++;
    }
    bigNumberLCD.setBigNumberCursor((LCD_COLUMNS - 1) - tColumnsRequiredForString, 0);
    bigNumberLCD.print(sStringBuffer);

    // Print units
    myLCD.setCursor(19, UNITS_ROW_FOR_BIG_INFO - 1);
    myLCD.print(tKiloWattChar);
    myLCD.setCursor(19, UNITS_ROW_FOR_BIG_INFO);
    myLCD.print('W');

    /*
     * Bottom row: Max temperature, current, voltage difference and the actual states
     */
    myLCD.setCursor(0, 3);
    myLCD.print(JK_BMS_1.JKComputedData.TemperatureMaximum);
    myLCD.print(F(DEGREE_SIGN_STRING " "));

    myLCD.setCursor(4, 3);
    printFloatValueRightAlignedOnLCD(JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat, 5);
    myLCD.print('A');

    printFloatValueRightAlignedOnLCD(JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 100.0, 5);
    myLCD.print('V');

    myLCD.setCursor(17, 3); // Last 3 characters are the actual states
    printShortStateOnLCD();
}

void printCANInfoOnLCD() {
    /*
     * sLCDDisplayPageNumber == JK_BMS_PAGE_CAN_INFO
     */
    if (!sCANDataIsInitialized || JK_BMS_1.JKComputedData.BMSIsStarting) {
        myLCD.print(F("No CAN data are sent"));
        myLCD.setCursor(0, 1);
        if (JK_BMS_1.JKComputedData.BMSIsStarting) {
            myLCD.print(F("BMS is starting"));
        } else {
            myLCD.print(F("No BMS data received"));
        }
    } else {
        CANFrameStruct *tCANFrameDataPointer = reinterpret_cast<struct CANFrameStruct*>(&PylontechCANErrorsWarningsFrame359);
        if (tCANFrameDataPointer->FrameData.ULong.LowLong == 0) {
            /*
             * Caption in row 1 and "No errors / warnings" in row 2
             */
            myLCD.print(F(" -CAN INFO- "));
            if (PylontechCANSohSocFrame355.FrameData.SOCPercent < 100) {
                myLCD.print(' ');
            }
            myLCD.print(F("SOC="));
            myLCD.print(PylontechCANSohSocFrame355.FrameData.SOCPercent);
            myLCD.print('%');
            myLCD.setCursor(0, 1);
            myLCD.print(F("No errors / warnings"));
        } else {
            /*
             * Errors in row 1 and warnings in row 2
             */
            myLCD.print(F("Errors: 0x"));
            myLCD.print(tCANFrameDataPointer->FrameData.UBytes[0], HEX);
            myLCD.print(F(" 0x"));
            myLCD.print(tCANFrameDataPointer->FrameData.UBytes[1], HEX);
            myLCD.setCursor(0, 1);
            myLCD.print(F("Warnings: 0x"));
            myLCD.print(tCANFrameDataPointer->FrameData.UBytes[2], HEX);
            myLCD.print(F(" 0x"));
            myLCD.print(tCANFrameDataPointer->FrameData.UBytes[3], HEX);
        }
        /*
         * Voltage, current and maximum temperature in row 3
         */
        myLCD.setCursor(0, 2);
// Voltage
        printFloatValueRightAlignedOnLCD(PylontechCANCurrentValuesFrame356.FrameData.Voltage10Millivolt / 100.0, 5, true);
        myLCD.print(F("V "));

// Current
        printFloatValueRightAlignedOnLCD(PylontechCANCurrentValuesFrame356.FrameData.Current100Milliampere / 10.0, 5);
        myLCD.print(F("A "));

// Temperature, we have only a 1 degree resolution here
        myLCD.print(PylontechCANCurrentValuesFrame356.FrameData.Temperature100Millicelsius / 10);
        myLCD.print(F(DEGREE_SIGN_STRING "C "));

        /*
         * Request flags in row 4
         */
        myLCD.setCursor(0, 3);
// Charge enable
        if (PylontechCANBatteryRequestFrame35C.FrameData.ChargeEnable) {
            myLCD.print(F("CH "));
        } else {
            myLCD.print(F("   "));
        }
        if (PylontechCANBatteryRequestFrame35C.FrameData.DischargeEnable) {
            myLCD.print(F("DC "));
        } else {
            myLCD.print(F("   "));
        }
        if (PylontechCANBatteryRequestFrame35C.FrameData.ForceChargeRequestI) {
            myLCD.print(F("FORCEI "));
        } else {
            myLCD.print(F("       "));
        }
        if (PylontechCANBatteryRequestFrame35C.FrameData.ForceChargeRequestII) {
            myLCD.print(F("FORCEII"));
        } else {
            myLCD.print(F("       "));
        }

// Currently constant 0
//    myLCD.setCursor(10, 3);
//    if (PylontechCANBatteryRequestFrame35C.FrameData.FullChargeRequest) {
//        myLCD.print(F("FULL"));
//    }
    }
}

/*
 * Row 3 - Voltage, Current and Power
 */
void printVoltageCurrentAndPowerOnLCD() {
    myLCD.setCursor(0, 2);
    // Voltage
//    myLCD.print(JK_BMS_1.JKComputedData.BatteryVoltageFloat, 2); // currently requires more programming space
    printFloatValueRightAlignedOnLCD(JK_BMS_1.JKComputedData.BatteryVoltageFloat, 5, true); // true -> do not print leading space
    myLCD.print(F("V "));

    // Current
    printFloatValueRightAlignedOnLCD(JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat, 5);
    myLCD.print('A');

    // Power
    snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%6d"), JK_BMS_1.JKComputedData.BatteryLoadPower); // force use of 6 columns
    myLCD.print(sStringBuffer);
    myLCD.print('W');
}

/*
 * Row 4 - 1. Voltage difference, MOS temperature, maximum temperature of 2 external sensors and 3 enable states
 * Row 4 - 2. MOS temperature, temperature of 2 external sensors and 3 enable states
 */
void printVoltageDifferenceAndTemperature() {
    myLCD.setCursor(0, 3);
    /*
     * If one of the temperatures is negative or they difference is more than 25%
     * show all 3 temperatures instead of voltage difference and 2 temperatures
     */
    bool tShowBothExternalTemperatures = true;
    if (JK_BMS_1.JKComputedData.TemperatureSensor1 > 0 && JK_BMS_1.JKComputedData.TemperatureSensor2 > 0
            && abs(JK_BMS_1.JKComputedData.TemperatureSensor1 - JK_BMS_1.JKComputedData.TemperatureSensor2)
                    < (min(JK_BMS_1.JKComputedData.TemperatureSensor1, JK_BMS_1.JKComputedData.TemperatureSensor1) / 4)) {
        tShowBothExternalTemperatures = false;
        printFloatValueRightAlignedOnLCD(JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt / 100.0, 5, true); // true = no leading space
        myLCD.print(F("V "));
    }
    myLCD.print(JK_BMS_1.JKComputedData.TemperaturePowerMosFet);
    myLCD.print(F(DEGREE_SIGN_STRING "C "));
    if (tShowBothExternalTemperatures) {
// show both external sensors
        myLCD.print(JK_BMS_1.JKComputedData.TemperatureSensor1);
        myLCD.print(F(DEGREE_SIGN_STRING "C "));
        myLCD.print(JK_BMS_1.JKComputedData.TemperatureSensor2);
        myLCD.print(F(DEGREE_SIGN_STRING "C "));
    } else {
// show maximum of the 2 external sensors
        myLCD.print(max(JK_BMS_1.JKComputedData.TemperatureSensor1, JK_BMS_1.JKComputedData.TemperatureSensor2));
        myLCD.print(F(DEGREE_SIGN_STRING "C "));
    }
}

/*
 * Print timeout message with uptime
 */
void printTimeoutMessageOnLCD() {
    myLCD.clear();
    myLCD.setCursor(0, 0);
    myLCD.print(F("Receive timeout ")); // 16 char
#if defined(HANDLE_MULTIPLE_BMS)
    myLCD.print(F("at "));
    myLCD.print(sCurrentBMS->NumberOfThisBMS); // 4 char
#endif
    myLCD.setCursor(LCD_COLUMNS - LENGTH_OF_UPTIME_STRING, 1);
    myLCD.print(sUpTimeString);
    myLCD.setCursor(0, 2);
    myLCD.print(F("Is BMS switched off?"));
}

/*
 * Print alarm info only once
 * Only the alarm bits on row 4 are updated each time
 * Line 3 and 4 are like Overview page
 */
void printAlarmInfoOnLCD() {
    myLCD.clear();
    uint8_t tAlarmIndexToShowOnLCD = JK_BMS_1.AlarmIndexToShowOnLCD;

    // Copy alarm message from flash, but not more than 20 characters
    const char *tLastAlarmString = (char*) (pgm_read_word(&JK_BMSAlarmStringsArray[tAlarmIndexToShowOnLCD]));
    strncpy_P(sStringBuffer, tLastAlarmString, LCD_COLUMNS);
    sStringBuffer[LCD_COLUMNS] = '\0';
    myLCD.print(sStringBuffer);

    /*
     * Row 2
     * index of min or max cell and cell voltage and uptime
     * or remainder of alarm string and uptime
     */
    myLCD.setCursor(0, 1);
    if (tAlarmIndexToShowOnLCD == INDEX_OF_DISCHARGING_UNDERVOLTAGE_ALARM
            || tAlarmIndexToShowOnLCD == INDEX_OF_CHARGING_OVERVOLTAGE_ALARM) {
        uint16_t tCellMillivoltToPrint;
        uint8_t tCellIndexToPrint;
        if (tAlarmIndexToShowOnLCD == INDEX_OF_DISCHARGING_UNDERVOLTAGE_ALARM) {
            // Index of minimum cell
            tCellIndexToPrint = JK_BMS_1.JKConvertedCellInfo.IndexOfMinimumCellMillivolt;
            tCellMillivoltToPrint = JK_BMS_1.JKConvertedCellInfo.MinimumCellMillivolt;
        } else if (tAlarmIndexToShowOnLCD == INDEX_OF_CHARGING_OVERVOLTAGE_ALARM) {
            // Index of maximum cell
            tCellIndexToPrint = JK_BMS_1.JKConvertedCellInfo.IndexOfMaximumCellMillivolt;
            tCellMillivoltToPrint = JK_BMS_1.JKConvertedCellInfo.MaximumCellMillivolt;
        }
// print millivolt with fix format 3 character value
        snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%2d %3dmV "), tCellIndexToPrint + 1, tCellMillivoltToPrint - 3000);
        myLCD.print(sStringBuffer);

    } else if (strlen_P(tLastAlarmString) > LCD_COLUMNS) {
// remainder of alarm string
        strncpy_P(sStringBuffer, &tLastAlarmString[LCD_COLUMNS], LCD_COLUMNS - LENGTH_OF_UPTIME_STRING); // 11 = Length of uptime string
        sStringBuffer[LCD_COLUMNS - LENGTH_OF_UPTIME_STRING] = '\0';
        myLCD.print(sStringBuffer);
    }

    // Uptime
    myLCD.setCursor(LCD_COLUMNS - LENGTH_OF_UPTIME_STRING, 1);
    myLCD.print(sUpTimeString);

    /*
     * Row 3 - Voltage, Current and Power
     */
    printVoltageCurrentAndPowerOnLCD();

    /*
     * Row 4 - 1. Voltage difference, MOS temperature, maximum temperature of 2 external sensors and 3 enable states
     * Row 4 - 2. MOS temperature, temperature of 2 external sensors and 3 enable states
     */
    printVoltageDifferenceAndTemperature();
}

/*
 * If AlarmIndexToShowOnLCD != INDEX_NO_ALARM show only actual HEX alarm bits in row 4
 */
void printOverwiewOrAlarmInfoOnLCD() {
    if (!sShowAlarmInsteadOfOverview) {
        /*
         * Top row 1 - Up time
         */
        myLCD.print(F("Uptime:  "));
        myLCD.print(sUpTimeString);

        /*
         * Row 2 - SOC and remaining capacity and Charge / Discharge / Balancing enable flags
         */
        myLCD.setCursor(0, 1);
// Percent of charge
        myLCD.print(JK_BMS_1.JKAllReplyPointer->SOCPercent);
        myLCD.print(F("% "));
// Remaining capacity
        snprintf_P(sStringBuffer, sizeof(sStringBuffer), PSTR("%3d"), JK_BMS_1.JKComputedData.RemainingCapacityAmpereHour);
        myLCD.print(sStringBuffer);
        myLCD.print(F("Ah "));
// Last 3 characters are the enable states
        myLCD.setCursor(14, 1);
        myLCD.print(F("En:"));
        printShortEnableFlagsOnLCD();

        /*
         * Row 3 - Voltage, Current and Power
         */
        printVoltageCurrentAndPowerOnLCD();

        /*
         * Row 4 - 1. Voltage difference, MOS temperature, maximum temperature of 2 external sensors and 3 enable states
         * Row 4 - 2. MOS temperature, temperature of 2 external sensors and 3 enable states
         * In case of alarm, print alarm hex instead of 3 enable states
         */
        printVoltageDifferenceAndTemperature();
    }

    // print it always, even for alarm display
    printAlarmHexOrStateOnLCD();
}

void printBMSDataOnLCD() {
    if (sSerialLCDAvailable
#  if !defined(DISPLAY_ALWAYS_ON)
            && !sSerialLCDIsSwitchedOff
#  endif
    ) {
        /*
         * Check for alarm info, which is not yet reset by main loop at time of calling
         */
        if (JK_BMS_1.AlarmJustGetsActive) {
#  if !defined(ENABLE_OVER_AND_UNDER_VOLTAGE_WARNING_ON_LCD)
            if (JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord
                    & ~MASK_OF_CHARGING_AND_DISCHARGING_OVERVOLTAGE_ALARM_UNSWAPPED) {
                // Other than over  / undervoltage alarm bit is active
#  endif
                /*
                 * fill main part of LCD alarm page only once
                 */
                sShowAlarmInsteadOfOverview = true;
                sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW; // Set current page
                printAlarmInfoOnLCD(); // print info only once
#  if !defined(DISPLAY_ALWAYS_ON)
            if (checkAndTurnLCDOn()) {
                JK_INFO_PRINTLN(F("alarm status changing")); // Reason for switching on LCD display
            }
    #  endif
#  if !defined(ENABLE_OVER_AND_UNDER_VOLTAGE_WARNING_ON_LCD)
            }
#  endif
        }

// do not clear alarm info, which is only printed once
        if (!sShowAlarmInsteadOfOverview) {
            myLCD.clear();
        }

        if (sLCDDisplayPageNumber == JK_BMS_PAGE_OVERVIEW) {
            printOverwiewOrAlarmInfoOnLCD();
        }

        else if (sLCDDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
            printCellInfoOnLCD();

#if !defined(NO_CELL_STATISTICS)
        } else if (sLCDDisplayPageNumber == JK_BMS_PAGE_CELL_STATISTICS) {
            printCellStatisticsOnLCD();
#endif
        } else if (sLCDDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {
            printBigInfoOnLCD();

#if !defined(NO_ANALYTICS)
        } else if (sLCDDisplayPageNumber == JK_BMS_PAGE_CAPACITY_INFO) {
            printCapacityInfoOnLCD();
#endif

        } else { //sLCDDisplayPageNumber == JK_BMS_PAGE_CAN_INFO
            printCANInfoOnLCD();
        }

    }
}

/*
 * Synchronously handle page button press (called from loop)
 */
void checkButtonPressForLCD() {
    if (sSerialLCDAvailable) {
        uint8_t tLCDDisplayPageNumber = sLCDDisplayPageNumber;

        if (sPageButtonJustPressed) {
            /*
             * New press here
             */
            sPageButtonJustPressed = false;
            /*
             * If alarm is active, only reset it and do no other action, except switching the display on, if display is off
             */
            if (sAlarmOrTimeoutBeepActive) {
                sAlarmOrTimeoutBeepActive = false; // disable further alarm beeps
#  if defined(DISPLAY_ALWAYS_ON)
                return; // No further action, just reset flag / beep
#  else
                if (!sSerialLCDIsSwitchedOff) {
                    return; // No switching on LCD, just reset flag / beep
                }
#  endif
            }
#  if !defined(DISPLAY_ALWAYS_ON)
            /*
             * If backlight LED off, switch it on, but do not select next page
             */
            if (checkAndTurnLCDOn()) {
                JK_INFO_PRINTLN(F("button press")); // Reason for switching on LCD display
                sPageButtonJustPressed = false; // avoid switching pages if page button was pressed.
            } else
#  endif
            {
                if (millis() - PageSwitchButtonAtPin2.ButtonReleaseMillis > 60000 && tLCDDisplayPageNumber != JK_BMS_START_PAGE) {
                    /*
                     * More than 1 minute since last button press -> go to directly to start page if not already there
                     */
                    tLCDDisplayPageNumber = JK_BMS_START_PAGE;
                } else {
                    /*
                     * Switch display pages to next page
                     */
                    tLCDDisplayPageNumber++;
                    if (tLCDDisplayPageNumber == JK_BMS_PAGE_MAX + 1 || tLCDDisplayPageNumber == JK_BMS_DEBUG_PAGE_MAX + 1) {
                        // Wrap around
                        tLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
                    }
                }
                setLCDDisplayPage(tLCDDisplayPageNumber);
            }

        } else if (PageSwitchButtonAtPin2.readDebouncedButtonState() == BUTTON_IS_ACTIVE
                && PageSwitchButtonAtPin2.checkForLongPress(LONG_PRESS_BUTTON_DURATION_MILLIS) == EASY_BUTTON_LONG_PRESS_DETECTED) {
            /*
             * Here long press detected i.e. button was not just pressed in the loop before and button is still active for longer than LONG_PRESS_BUTTON_DURATION_MILLIS.
             */
            if (tLCDDisplayPageNumber == JK_BMS_PAGE_CAN_INFO) {
                // Button is still pressed on CAN Info page -> enable serial debug output as long as button is pressed
                sCommunicationDebugModeActivated = true; // Is set to false in loop

#if !defined(NO_ANALYTICS)
            } else if (tLCDDisplayPageNumber == JK_BMS_PAGE_CAPACITY_INFO) {
                if (SOCDataPointsInfo.ArrayLength > 1) {
                    // EEPROM data not already cleared here
                    myLCD.setCursor(0, 0);
                    myLCD.print(F("Clear EEPROM data in"));
                    LCDClearLine(1);
                    myLCD.print(F("2 seconds"));
                    LCDClearLine(2);
                    LCDClearLine(3);
                    delay(1000);
                    if (PageSwitchButtonAtPin2.readDebouncedButtonState() == BUTTON_IS_ACTIVE) { // Check again, if still pressed
                        myLCD.setCursor(0, 1);
                        myLCD.print('1');
                        delay(1000); // To wait for eventual button release
                        if (PageSwitchButtonAtPin2.readDebouncedButtonState() == BUTTON_IS_ACTIVE) { // Check again, if still pressed
                            LCDClearLine(1);
                            myLCD.print(F("now")); // is visible for the time EEPROM needs for erasing (+200 ms)
                            delay(200); // To wait for eventual button release
                        }
                    }
                    myLCD.setCursor(0, 0);
                    if (PageSwitchButtonAtPin2.readDebouncedButtonState() == BUTTON_IS_ACTIVE) { // Check again, if still pressed
                        updateCompleteEEPROMTo_FF(); // Clear EEPROM
                        JK_INFO_PRINTLN(F("Long press detected -> clear EEPROM"));
                        myLCD.print(F("EEPROM data cleared "));
                    } else {
                        myLCD.print(F("Clear EEPROM aborted"));
                    }
                    myLCD.setCursor(0, 1);
                    LCDPrintSpaces(9); // overwrite 2. line containing 2 seconds

                    delay (LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the messages
                }
#endif
            } else {
                /*
                 * Not page JK_BMS_PAGE_CAN_INFO or JK_BMS_PAGE_CAPACITY_INFO -> switch to CAN Info page
                 */
                sCommunicationDebugModeActivated = true; // Is set to false in loop
                if (sSerialLCDAvailable) {
                    JK_INFO_PRINTLN();
                    JK_INFO_PRINTLN(F("Long press detected -> switch to CAN page and activate one time debug print"));
                    setLCDDisplayPage(JK_BMS_PAGE_CAN_INFO);
                }
            }

        } else if (tLCDDisplayPageNumber == JK_BMS_PAGE_CAPACITY_INFO
                && (millis() - sLastPageChangeMillis) > JK_BMS_PAGE_CAPACITY_INFO_PAGE_TIMEOUT_MILLIS) {
            /*
             * Handle timeout for Capacity Info page
             */
            setLCDDisplayPage(JK_BMS_START_PAGE, true);
        }        // PageSwitchButtonAtPin2.ButtonStateHasJustChanged
    }
}

/*
 * Exclusively called by checkButtonPressForLCD()
 */
void setLCDDisplayPage(uint8_t aLCDDisplayPageNumber, bool aDoNotPrint) {
    sLCDDisplayPageNumber = aLCDDisplayPageNumber;
    sLastPageChangeMillis = millis();
    tone(BUZZER_PIN, 2200, 30);
    if (!aDoNotPrint) {
        JK_INFO_PRINT(F("Set LCD display page to: "));
        JK_INFO_PRINTLN(aLCDDisplayPageNumber);
    }

    if (aLCDDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
// Create symbols character for maximum and minimum
        bigNumberLCD._createChar(1, bigNumbersTopBlock);
        bigNumberLCD._createChar(2, bigNumbersBottomBlock);
#if !defined(NO_CELL_STATISTICS)
// Prepare for statistics page here display max first but for half the regular time
        sToggleDisplayCounter = (CELL_STATISTICS_COUNTER_MASK >> 1) - 1;
#endif
    }

#if !defined(NO_CELL_STATISTICS)
    else if (aLCDDisplayPageNumber == JK_BMS_PAGE_CAPACITY_INFO) {
        sToggleDisplayCounter = 0;
    }
#endif

    else if (aLCDDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {
        bigNumberLCD.begin(); // Creates custom character used for generating big numbers
    }

    // On button press, reset any alarm display
    sShowAlarmInsteadOfOverview = false;

    /*
     * Show new page
     */
    printBMSDataOnLCD();

#if !defined(NO_ANALYTICS)
    if (aLCDDisplayPageNumber == JK_BMS_PAGE_CAPACITY_INFO) {
// do it even if we have timeout
        sCommunicationDebugModeActivated = false; // Disable every debug output after entering this page
        printCapacityInfoOnLCD(); // First update LCD before printing the plotter data
        readAndPrintSOCData(); // this takes a while...
    }
#endif
}

#if defined(STANDALONE_TEST)
void testLCDPages() {
    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    printBMSDataOnLCD();
    delay (LCD_MESSAGE_PERSIST_TIME_MILLIS);

    sLCDDisplayPageNumber = JK_BMS_PAGE_CELL_INFO;
// Create symbols character for maximum and minimum
    bigNumberLCD._createChar(1, bigNumbersTopBlock);
    bigNumberLCD._createChar(2, bigNumbersBottomBlock);
    printBMSDataOnLCD();
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    sLCDDisplayPageNumber = JK_BMS_PAGE_CAN_INFO;
    printBMSDataOnLCD();
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    Serial.println(F("testLCDPages: Test alarms"));
    /*
     * Test alarms
     */
    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.ChargeOvervoltageAlarm = true;
    JK_BMS_1.detectAndPrintAlarmInfo(); // this sets the LCD alarm string
    printBMSDataOnLCD();
    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.ChargeOvervoltageAlarm = false;
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.DischargeUndervoltageAlarm = true;
    JK_BMS_1.detectAndPrintAlarmInfo(); // this sets the LCD alarm string
    printBMSDataOnLCD();
    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.DischargeUndervoltageAlarm = false;
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    /*
     * PowerMosFetOvertemperatureAlarm
     */
    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.PowerMosFetOvertemperatureAlarm = true;
    JK_BMS_1.JKComputedData.TemperaturePowerMosFet = 90;
    JK_BMS_1.JKComputedData.TemperatureSensor1 = 25;
    JK_BMS_1.detectAndPrintAlarmInfo(); // this sets the LCD alarm string
    printBMSDataOnLCD();
    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.PowerMosFetOvertemperatureAlarm = false;
    JK_BMS_1.JKComputedData.TemperaturePowerMosFet = 33;
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    // reset alarm
    JK_BMS_1.AlarmIndexToShowOnLCD = INDEX_NO_ALARM;
    JK_BMS_1.AlarmJustGetsActive = false;
    sShowAlarmInsteadOfOverview = false;

    Serial.println(F("testLCDPages: Test maximum values"));
    /*
     * Check display of maximum values
     */
    JK_BMS_1.JKAllReplyPointer->SOCPercent = 100;
    JK_BMS_1.JKComputedData.BatteryLoadPower = -11000;
    JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat = JK_BMS_1.JKComputedData.BatteryLoadPower
            / JK_BMS_1.JKComputedData.BatteryVoltageFloat;
    JK_BMS_1.JKComputedData.TemperaturePowerMosFet = 111;
    JK_BMS_1.JKComputedData.TemperatureSensor1 = 100;
    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    printBMSDataOnLCD();
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    /*
     * maximum values at Big Info
     */
    sLCDDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;
    myLCD.clear();
    bigNumberLCD.begin();
    printBMSDataOnLCD();
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    Serial.println(F("testLCDPages: Test other values"));
    /*
     * Test other values
     */
    JK_BMS_1.JKAllReplyPointer->SOCPercent = 1;
    JK_BMS_1.JKComputedData.BatteryLoadPower = 12345;
    JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat = JK_BMS_1.JKComputedData.BatteryLoadPower
            / JK_BMS_1.JKComputedData.BatteryVoltageFloat;

    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    printBMSDataOnLCD();
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    sLCDDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;
    myLCD.clear();
    printBMSDataOnLCD();
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    JK_BMS_1.JKAllReplyPointer->SOCPercent = 100;
    JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat = -100;
    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    printBMSDataOnLCD();
    Serial.println(F("testLCDPages: End"));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
}

void testBigNumbers() {
    Serial.println(F("Test BigNumbers"));

    sLCDDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;

    for (int j = 0; j < 3; ++j) {
        // Test with 100 %  and 42 %

        /*
         * test with positive numbers
         */
        JK_BMS_1.JKComputedData.BatteryLoadPower = 12345;
        JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat = JK_BMS_1.JKComputedData.BatteryLoadPower
                / JK_BMS_1.JKComputedData.BatteryVoltageFloat;

        for (int i = 0; i < 5; ++i) {
            delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS);
            myLCD.clear();
            printBMSDataOnLCD();
            JK_BMS_1.JKComputedData.BatteryLoadPower /= 10; // 1234 -> 12
            JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat = JK_BMS_1.JKComputedData.BatteryLoadPower
                    / JK_BMS_1.JKComputedData.BatteryVoltageFloat;
        }
        /*
         * test with negative numbers
         */
        JK_BMS_1.JKComputedData.BatteryLoadPower = -12345;
        JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat = JK_BMS_1.JKComputedData.BatteryLoadPower
                / JK_BMS_1.JKComputedData.BatteryVoltageFloat;

        for (int i = 0; i < 5; ++i) {
            delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS);
            myLCD.clear();
            printBMSDataOnLCD();
            JK_BMS_1.JKComputedData.BatteryLoadPower /= 10; // 1234 -> 12
            JK_BMS_1.JKComputedData.BatteryLoadCurrentFloat = JK_BMS_1.JKComputedData.BatteryLoadPower
                    / JK_BMS_1.JKComputedData.BatteryVoltageFloat;
        }

        JK_BMS_1.JKAllReplyPointer->SOCPercent /= 10;
    }
    Serial.println(F("Test BigNumbers: end"));

}
#endif // STANDALONE_TEST

#endif // _JK_BMS_LCD_HPP
