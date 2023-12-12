/*
 *  JK-BMS_LCD.hpp
 *
 *  Contains LCD related variables and functions
 *
 *  Copyright (C) 2023  Armin Joachimsmeyer
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

#define USE_SERIAL_2004_LCD // Parallel or 1604 LCD not yet supported

/*
 * LCD hardware stuff
 */
#define LCD_COLUMNS     20
#define LCD_ROWS         4
#define LCD_I2C_ADDRESS 0x27     // Default LCD address is 0x27 for a 20 chars and 4 line / 2004 display
bool sSerialLCDAvailable;

/*
 * Display timeouts, may be adapted to your requirements
 */
#  if defined(STANDALONE_TEST)
#define DISPLAY_ON_TIME_STRING               "30 s"
#define DISPLAY_ON_TIME_SECONDS              30L // L to avoid overflow at macro processing
//#define NO_MULTIPLE_BEEPS_ON_TIMEOUT           // Activate it if you do not want multiple beeps
#define BEEP_ON_TIME_SECONDS_IF_TIMEOUT      10L // 10 s
#  else
#define DISPLAY_ON_TIME_STRING              "5 min" // Only for display on LCD
#define DISPLAY_ON_TIME_SECONDS             300L // 5 minutes. L to avoid overflow at macro processing
#  endif // DEBUG

//#define DISPLAY_ALWAYS_ON   // Activate this, if you want the display to be always on.
#  if !defined(DISPLAY_ALWAYS_ON)
void doLCDBacklightTimeoutHandling();
bool checkAndTurnLCDOn();
bool sSerialLCDIsSwitchedOff = false;
uint16_t sFrameCounterForLCDTAutoOff = 0;
#  endif

#include "LiquidCrystal_I2C.hpp" // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use only the modified version delivered with this program!
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

/*
 * LCD display pages
 */
#define JK_BMS_PAGE_OVERVIEW            0 // is displayed in case of BMS error message
#define JK_BMS_PAGE_CELL_INFO           1
#define JK_BMS_PAGE_CELL_STATISTICS     2
#define CELL_STATISTICS_COUNTER_MASK 0x04 // must be a multiple of 2 and determines how often one page (min or max) is displayed.
#define JK_BMS_PAGE_BIG_INFO            3
#define JK_BMS_PAGE_CAN_INFO            4 // If debug was pressed
#define JK_BMS_PAGE_CAPACITY_INFO       5 // If debug was pressed
#define CELL_CAPACITY_COUNTER_VOLTAGE   0x06 // If counter "anded" with mask is true show delta voltages instead of percents.

#define JK_BMS_PAGE_MAX                 JK_BMS_PAGE_BIG_INFO
#define JK_BMS_DEBUG_PAGE_MAX           JK_BMS_PAGE_CAPACITY_INFO
#define JK_BMS_START_PAGE               JK_BMS_PAGE_BIG_INFO
//uint8_t sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW; // Start with Overview page
uint8_t sLCDDisplayPageNumber = JK_BMS_START_PAGE; // Start with Big Info page
uint8_t sToggleDisplayCounter;            // counter for CELL_STATISTICS_COUNTER_MASK, to determine max or min page
#if !defined(ENABLE_MONITORING) && defined(NO_INTERNAL_STATISTICS)
char sStringBuffer[LCD_COLUMNS + 1];      // For rendering a LCD row with sprintf_P()
#endif
void setDisplayPage(uint8_t aDisplayPageNumber);

void printBMSDataOnLCD();
void printCANInfoOnLCD();
void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint);
void LCDClearLine(uint8_t aLineNumber);

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
        myLCD.print(F("Screen timeout " DISPLAY_ON_TIME_STRING));
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
        delay(2000); // To see the messages
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
        Serial.print(F("Switch on LCD display, triggered by ")); // to be continued by caller
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
        Serial.println(F("Switch off LCD display, triggered by LCD \"ON\" timeout reached."));
    }
}
#  endif

void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint) {
    for (uint_fast8_t i = 0; i < aNumberOfSpacesToPrint; ++i) {
        myLCD.print(' ');
    }
}

void LCDClearLine(uint8_t aLineNumber) {
    myLCD.setCursor(0, aLineNumber);
    LCDPrintSpaces(20);
    myLCD.setCursor(0, aLineNumber);
}

/*
 * Global timeout message
 */
void printTimeoutMessageOnLCD() {
    if (sSerialLCDAvailable
#  if !defined(DISPLAY_ALWAYS_ON)
            && !sSerialLCDIsSwitchedOff
#  endif
            && sLCDDisplayPageNumber != JK_BMS_PAGE_CAN_INFO) {
        myLCD.clear();
        myLCD.setCursor(0, 0);
        myLCD.print(F("Receive timeout "));
        myLCD.print(sTimeoutFrameCounter);
        myLCD.setCursor(0, 1);
        myLCD.print(F("Is BMS switched off?"));
    }
}

void printShortEnableFlagsOnLCD() {
    if (sJKFAllReplyPointer->ChargeIsEnabled) {
        myLCD.print('C');
    } else {
        myLCD.print(' ');
    }
    if (sJKFAllReplyPointer->ChargeIsEnabled) {
        myLCD.print('D');
    } else {
        myLCD.print(' ');
    }
    if (sJKFAllReplyPointer->BalancingIsEnabled) {
        myLCD.print('B');
    }
}

/*
 * Prints state of Charge Overvoltage 'F'ull warning, and Charge, Discharge and Balancing flag
 */
void printShortStateOnLCD() {
    if (sJKFAllReplyPointer->AlarmUnion.AlarmBits.ChargeOvervoltageAlarm
            && !sJKFAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive) {
        // we can replace C by F here
        myLCD.print(' ');
        myLCD.print('F');

    } else {
        if (sJKFAllReplyPointer->AlarmUnion.AlarmBits.ChargeOvervoltageAlarm) {
            myLCD.print('F');
        } else {
            myLCD.print(' ');
        }
        if (sJKFAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive) {
            myLCD.print('C');
        } else {
            myLCD.print(' ');
        }
    }
    if (sJKFAllReplyPointer->BMSStatus.StatusBits.DischargeMosFetActive) {
        myLCD.print('D');
    } else {
        myLCD.print(' ');
    }
    if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
        myLCD.print('B');
    }
}

void printLongStateOnLCD() {
    if (sJKFAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive) {
        myLCD.print(F("CH "));
    } else {
        myLCD.print(F("   "));
    }

    if (sJKFAllReplyPointer->BMSStatus.StatusBits.DischargeMosFetActive) {
        myLCD.print(F("DC "));
    } else {
        myLCD.print(F("   "));
    }

    if (sJKFAllReplyPointer->BMSStatus.StatusBits.BalancerActive) {
        myLCD.print(F("BAL"));
    }
}

/*
 * Print current right aligned as 5 character including sign and trailing "A "
 * -9.99
 * -99.9
 *  -999
 */
void printCurrent5CharacterRightAlignedOnLCD() {
    int16_t tBattery10MilliAmpere = JKComputedData.Battery10MilliAmpere;
    if (tBattery10MilliAmpere >= 0) {
        myLCD.print(' '); // handle not printed + sign
    }
    tBattery10MilliAmpere = abs(tBattery10MilliAmpere); // remove sign for length computation
    uint8_t tNumberOfDecimalPlaces;
    if (tBattery10MilliAmpere < 1000) {
        // less than 10 A (1000 * 10mA)
        tNumberOfDecimalPlaces = 2; // -9.99
    } else if (tBattery10MilliAmpere < 10000) {
        // less than 100 A (10000 * 10mA)
        tNumberOfDecimalPlaces = 1; // -99.9
    } else {
        myLCD.print(' ');           // Handle not printed decimal point . for -999 - Maximum current is 327 A
        tNumberOfDecimalPlaces = 0; // -999
    }
    myLCD.print(JKComputedData.BatteryLoadCurrentFloat, tNumberOfDecimalPlaces);
    myLCD.print(F("A "));
}

/*
 * Print 3 characters 2.4 or .45
 */
void printVoltageDifference3CharactersOnLCD() {
    uint16_t tBatteryToFullDifference10Millivolt = JKComputedData.BatteryFullVoltage10Millivolt
            - JKComputedData.BatteryVoltage10Millivolt;
    if (tBatteryToFullDifference10Millivolt < 100) {
        // Print small values as ".43" instead of "0.4"
        sprintf_P(sStringBuffer, PSTR(".%02d"), tBatteryToFullDifference10Millivolt);
        myLCD.print(sStringBuffer);
    } else {
        myLCD.print(((float) (tBatteryToFullDifference10Millivolt)) / 100.0, 1);
    }
}

/*
 * Print current right aligned as 4 character including sign
 * -9.9
 *  -99
 * -999
 */
void printCurrent4CharacterRightAlignedOnLCD() {
    int16_t tBattery10MilliAmpere = JKComputedData.Battery10MilliAmpere;
    if (tBattery10MilliAmpere >= 0) {
        myLCD.print(' '); // handle not printed + sign
    }
    tBattery10MilliAmpere = abs(tBattery10MilliAmpere); // remove sign for length computation
    uint8_t tNumberOfDecimalPlaces = 0;
    if (tBattery10MilliAmpere < 1000) {
        // less than 10 A (1000 * 10mA)
        tNumberOfDecimalPlaces = 1; // -9.9
    } else if (tBattery10MilliAmpere < 10000) {
        // less than 100 A (10000 * 10mA)
        myLCD.print(' '); // handle not printed decimal point . for -99
    }
    myLCD.print(JKComputedData.BatteryLoadCurrentFloat, tNumberOfDecimalPlaces);
}

/*
 * We can display only up to 16 cell values on the LCD :-(
 */
void printCellInfoOnLCD() {
    uint_fast8_t tRowNumber;
    auto tNumberOfCellInfoEntries = JKConvertedCellInfo.ActualNumberOfCellInfoEntries;
    //#define SHOW_SHORT_CELL_VOLTAGES        // Show cell voltage -3.0. This reduces the voltage string length from 4 to 3.
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
                 * Print current in the last 4 characters
                 */
                if (i == 4) {
                    sprintf_P(sStringBuffer, PSTR("%3u%%"), sJKFAllReplyPointer->SOCPercent);
                    myLCD.print(sStringBuffer);
                } else if (i == 8) {
                    printCurrent4CharacterRightAlignedOnLCD();
                } else if (i == 12) {
                    myLCD.print(F("   A"));
                }

            }
            myLCD.setCursor(0, tRowNumber);
            tRowNumber++;
        }

        // print maximum or minimum indicator
        if (JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MAXIMUM) {
            myLCD.print((char) (0x1));
        } else if (JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MINIMUM) {
            myLCD.print((char) (0x2));
        } else {
            myLCD.print(' ');
        }
        // print fix format 3 character value
        sprintf_P(sStringBuffer, PSTR("%3d"), JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt - 3000); // Value can be negative!
        myLCD.print(sStringBuffer);
    }
    if (tNumberOfCellInfoEntries <= 16) {
        myLCD.setCursor(17, 3);
        printVoltageDifference3CharactersOnLCD();
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
        if (JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MAXIMUM) {
            myLCD.print((char) (0x1));
        } else if (JKConvertedCellInfo.CellInfoStructArray[i].VoltageIsMinMaxOrBetween == VOLTAGE_IS_MINIMUM) {
            myLCD.print((char) (0x2));
        } else {
            myLCD.print(' ');
        }

        myLCD.print(JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt);
    }
#endif
}

/*
 * Switch between display of minimum and maximum at each 4. call
 * The sum of percentages may not give 100% because of rounding errors
 */
void printCellStatisticsOnLCD() {
// check if minimum or maximum is to be displayed
    bool tDisplayCellMinimumStatistics = sToggleDisplayCounter & CELL_STATISTICS_COUNTER_MASK; // 0x04
    sToggleDisplayCounter++;

    auto tNumberOfCellInfoEntries = JKConvertedCellInfo.ActualNumberOfCellInfoEntries;
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
            tPercent = CellMinimumPercentageArray[i];
        } else {
            tPercent = CellMaximumPercentageArray[i];
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

/*
 * Display the last measured capacities
 * Long: "100%->30%  111|130Ah"
 * Short: "99 30 111 "
 */
void printCapacityInfoOnLCD() {
    if (JKComputedCapacity[0].Capacity == 0 && JKComputedCapacity[1].Capacity == 0) {
        myLCD.print(F("No capacity computed"));
    } else {
        // check if minimum or maximum is to be displayed
        bool tDisplayDeltaVoltages = (sToggleDisplayCounter & CELL_CAPACITY_COUNTER_VOLTAGE) == CELL_CAPACITY_COUNTER_VOLTAGE; // 0x06
        sToggleDisplayCounter++;
        for (uint8_t i = 0; i < LCD_ROWS; ++i) {
            if (JKComputedCapacity[i].Capacity != 0) {
                myLCD.setCursor(0, i);
                if (tDisplayDeltaVoltages) {
                    snprintf_P(sStringBuffer, LCD_COLUMNS + 1, PSTR("%1u.%1u->%1u.%1umV %3u %3uAh"),
                            JKComputedCapacity[i].Start100MilliVoltToFull / 10, JKComputedCapacity[i].Start100MilliVoltToFull % 10,
                            JKComputedCapacity[i].End100MilliVoltToFull / 10, JKComputedCapacity[i].End100MilliVoltToFull % 10,
                            JKComputedCapacity[i].Capacity, JKComputedCapacity[i].TotalCapacity);
                } else {
                    snprintf_P(sStringBuffer, LCD_COLUMNS + 1, PSTR("%2d%%->%2d%% "), JKComputedCapacity[i].StartSOC,
                            JKComputedCapacity[i].EndSOC);
                    myLCD.print(sStringBuffer);
                    // If we have 100% as SOC value, we end up one column later, so we start at fixed position here
                    myLCD.setCursor(9, i);
                    snprintf_P(sStringBuffer, LCD_COLUMNS + 1, PSTR("  %3u %3uAh"), JKComputedCapacity[i].Capacity,
                            JKComputedCapacity[i].TotalCapacity);
                }
                myLCD.print(sStringBuffer);
            }
        }
    }
}

void printBigInfoOnLCD() {
    bigNumberLCD.setBigNumberCursor(0, 0);
    bigNumberLCD.print(sJKFAllReplyPointer->SOCPercent);
    uint8_t tColumn;
    if (sJKFAllReplyPointer->SOCPercent < 10) {
        tColumn = 3;
    } else if (sJKFAllReplyPointer->SOCPercent < 100) {
        tColumn = 6;
    } else {
        tColumn = 8; // 100%
    }

    myLCD.setCursor(tColumn, UNITS_ROW_FOR_BIG_INFO); // 3, 6 or 8
    myLCD.print('%');
    /*
     * Here we can start the power string at column 4, 7 or 9
     */
    uint8_t tAvailableColumns = (LCD_COLUMNS - 2) - tColumn;
    char tKiloWattChar = ' ';
    int16_t tBatteryLoadPower = JKComputedData.BatteryLoadPower;
    /*
     * First print string to buffer
     */
    if (tBatteryLoadPower >= 1000 || tBatteryLoadPower <= -1000) {
        tKiloWattChar = 'k';
        float tBatteryLoadPowerFloat = tBatteryLoadPower * 0.001; // convert to kW
        dtostrf(tBatteryLoadPowerFloat, 5, 2, sStringBuffer);
    } else {
        sprintf_P(sStringBuffer, PSTR("%d"), JKComputedData.BatteryLoadPower);
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
    myLCD.print(JKComputedData.TemperatureMaximum);
    myLCD.print(F("\xDF "));

    myLCD.setCursor(4, 3);
    printCurrent5CharacterRightAlignedOnLCD();

    myLCD.setCursor(11, 3);
    myLCD.print(((float) (JKComputedData.BatteryFullVoltage10Millivolt - JKComputedData.BatteryVoltage10Millivolt)) / 100.0, 2);
    myLCD.print('V');

    myLCD.setCursor(16, 3);
    printShortStateOnLCD();
}

void printCANInfoOnLCD() {
    /*
     * sLCDDisplayPageNumber == JK_BMS_PAGE_CAN_INFO
     */
    if (!sCANDataIsInitialized || JKComputedData.BMSIsStarting) {
        myLCD.print(F("No CAN data are sent"));
        myLCD.setCursor(0, 1);
        if (JKComputedData.BMSIsStarting) {
            myLCD.print(F("BMS is starting"));
        } else {
            myLCD.print(F("No BMS data received"));
        }
    } else {
        PylontechCANFrameStruct *tCANFrameDataPointer =
                reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANErrorsWarningsFrame);
        if (tCANFrameDataPointer->FrameData.ULong.LowLong == 0) {
            /*
             * Caption in row 1 and "No errors / warnings" in row 2
             */
            myLCD.print(F(" -CAN INFO- "));
            if (PylontechCANSohSocFrame.FrameData.SOCPercent < 100) {
                myLCD.print(' ');
            }
            myLCD.print(F("SOC="));
            myLCD.print(PylontechCANSohSocFrame.FrameData.SOCPercent);
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
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Voltage10Millivolt / 100);
        myLCD.print('.');
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Voltage10Millivolt % 100);
        myLCD.print(F("V "));
        // Current
        int16_t tCurrent100Milliampere = PylontechCANCurrentValuesFrame.FrameData.Current100Milliampere;
        myLCD.print(tCurrent100Milliampere / 10);
        if (tCurrent100Milliampere < 0) {
            // avoid negative numbers after decimal point
            tCurrent100Milliampere = -tCurrent100Milliampere;
        }
        if (tCurrent100Milliampere < 100) {
            // Print fraction if value < 100
            myLCD.print('.');
            myLCD.print(tCurrent100Milliampere % 10);
        }
        myLCD.print(F("A "));
        // Temperature
        myLCD.setCursor(14, 2);
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Temperature100Millicelsius / 10);
        if (PylontechCANCurrentValuesFrame.FrameData.Temperature100Millicelsius < 100) {
            // Print fraction if value < 100
            myLCD.print('.');
            myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Temperature100Millicelsius % 10);
        }
        myLCD.print(F("\xDF" "C "));

        /*
         * Request flags in row 4
         */
        myLCD.setCursor(0, 3);
        // Charge enable
        if (PylontechCANBatteryRequestFrame.FrameData.ChargeEnable) {
            myLCD.print(F("CH "));
        } else {
            myLCD.print(F("   "));
        }
        if (PylontechCANBatteryRequestFrame.FrameData.DischargeEnable) {
            myLCD.print(F("DC"));
        }
        myLCD.setCursor(6, 3);
        if (PylontechCANBatteryRequestFrame.FrameData.ForceChargeRequestI) {
            myLCD.print(F("FORCEI"));
        }
        myLCD.setCursor(13, 3);
        if (PylontechCANBatteryRequestFrame.FrameData.ForceChargeRequestII) {
            myLCD.print(F("FORCEII"));
        }

        // Currently constant 0
//    myLCD.setCursor(10, 3);
//    if (PylontechCANBatteryRequestFrame.FrameData.FullChargeRequest) {
//        myLCD.print(F("FULL"));
//    }
    }
}

void printOverwiewInfoOnLCD() {
    /*
     * Top row 1 - Error message or up time
     */
    if (sErrorStringForLCD != NULL) {
        // Copy error message from flash, but not more than 20 characters
        memcpy_P(sStringBuffer, sErrorStringForLCD, LCD_COLUMNS);
        sStringBuffer[LCD_COLUMNS] = '\0';
        myLCD.print(sStringBuffer);
    } else {
        myLCD.print(F("Uptime:  "));
        myLCD.print(sUpTimeString);
    }
    /*
     * Row 2 - SOC and remaining capacity and state of MosFets or Error
     */
    myLCD.setCursor(0, 1);
// Percent of charge
    myLCD.print(sJKFAllReplyPointer->SOCPercent);
    myLCD.print(F("% "));
// Remaining capacity
    myLCD.print(JKComputedData.RemainingCapacityAmpereHour);
    myLCD.print(F("Ah "));
    if (JKComputedData.RemainingCapacityAmpereHour < 100) {
        myLCD.print(' ');
    }
// Last 3 characters are the enable states
    myLCD.setCursor(14, 1);
    myLCD.print(F("En:"));
    printShortEnableFlagsOnLCD();

    /*
     * Row 3 - Voltage, Current and Power
     */
    myLCD.setCursor(0, 2);
// Voltage
    myLCD.print(JKComputedData.BatteryVoltageFloat, 2);
    myLCD.print(F("V "));
// Current
    printCurrent5CharacterRightAlignedOnLCD();
// Power
    if (JKComputedData.BatteryLoadPower < -10000) {
        // over 10 kW
        myLCD.setCursor(13, 2);
        myLCD.print(JKComputedData.BatteryLoadPower); // requires 6 columns
    } else {
        myLCD.setCursor(14, 2);
        sprintf_P(sStringBuffer, PSTR("%5d"), JKComputedData.BatteryLoadPower); // force use of 5 columns
        myLCD.print(sStringBuffer);
    }
    myLCD.print('W');
    /*
     * Row 4 - 3 Temperatures and 3 enable states
     */
    myLCD.setCursor(0, 3);
// 3 temperatures
    myLCD.print(JKComputedData.TemperaturePowerMosFet);
    myLCD.print(F("\xDF" "C "));
    myLCD.print(JKComputedData.TemperatureSensor1);
    myLCD.print(F("\xDF" "C "));
    myLCD.print(JKComputedData.TemperatureSensor2);
    myLCD.print(F("\xDF" "C "));
// Last 4 characters are the actual states
    myLCD.setCursor(16, 3);
    printShortStateOnLCD();

}

void printBMSDataOnLCD() {
    if (sSerialLCDAvailable
#  if !defined(DISPLAY_ALWAYS_ON)
            && !sSerialLCDIsSwitchedOff
#  endif
    ) {
        myLCD.clear();
        myLCD.setCursor(0, 0);

        if (sLCDDisplayPageNumber == JK_BMS_PAGE_OVERVIEW) {
            printOverwiewInfoOnLCD();

        } else if (sLCDDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
            printCellInfoOnLCD();

        } else if (sLCDDisplayPageNumber == JK_BMS_PAGE_CELL_STATISTICS) {
            printCellStatisticsOnLCD();

        } else if (sLCDDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {
            printBigInfoOnLCD();

        } else if (sLCDDisplayPageNumber == JK_BMS_PAGE_CAN_INFO) {
            printCANInfoOnLCD();

        } else { //sLCDDisplayPageNumber == JK_BMS_PAGE_CAPACITY_INFO
            printCapacityInfoOnLCD();
        }
    }
}

void checkButtonPressForLCD() {
    if (sSerialLCDAvailable) {
        /*
         * Handle Page button
         */
        uint8_t tDisplayPageNumber = sLCDDisplayPageNumber;
        if (sPageButtonJustPressed) {
            sPageButtonJustPressed = false;
#  if !defined(DISPLAY_ALWAYS_ON)
            /*
             * If backlight LED off, switch it on, but do not select next page
             */
            if (checkAndTurnLCDOn()) {
                Serial.println(F("button press")); // Switch on LCD display, triggered by button press
                sPageButtonJustPressed = false; // avoid switching pages if page button was pressed.
            } else
#  endif
            {

                if (sJKBMSFrameHasTimeout || tDisplayPageNumber == JK_BMS_PAGE_MAX || tDisplayPageNumber == JK_BMS_DEBUG_PAGE_MAX) {
                    // Receive timeout or wrap around here
                    tDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;

                } else {
                    /*
                     * Switch display pages to next page
                     */
                    tDisplayPageNumber++;
                    if (tDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
                        // Create symbols character for maximum and minimum
                        bigNumberLCD._createChar(1, bigNumbersTopBlock);
                        bigNumberLCD._createChar(2, bigNumbersBottomBlock);

                        // Prepare for statistics page here display max first but for half the regular time
                        sToggleDisplayCounter = (CELL_STATISTICS_COUNTER_MASK >> 1) - 1;

                    } else if (tDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {
                        bigNumberLCD.begin(); // Creates custom character used for generating big numbers
                    } else if (tDisplayPageNumber == JK_BMS_PAGE_CAPACITY_INFO) {
                        sToggleDisplayCounter = 0;
                    }
                }
                setDisplayPage(tDisplayPageNumber);
            }

        } else if (PageSwitchButtonAtPin2.readDebouncedButtonState()) {
            /*
             * Here button was not pressed in the loop before. Check if button is still active.
             */
            if (tDisplayPageNumber == JK_BMS_PAGE_CAN_INFO) {
                // Button is still pressed on CAN Info page -> enable serial debug output as long as button is pressed
                sDebugModeActivated = true; // Is set to false in loop
            } else if (PageSwitchButtonAtPin2.checkForLongPress(LONG_PRESS_BUTTON_DURATION_MILLIS)
                    == EASY_BUTTON_LONG_PRESS_DETECTED) {
                /*
                 * Long press detected -> switch to CAN Info page
                 */
                sDebugModeActivated = true; // Is set to false in loop
                if (sSerialLCDAvailable) {
                    Serial.println();
                    Serial.println(F("Long press detected -> switch to CAN page and activate one time debug print"));
                    setDisplayPage(JK_BMS_PAGE_CAN_INFO);
                }
            }
        } // PageSwitchButtonAtPin2.ButtonStateHasJustChanged
    }
}

void setDisplayPage(uint8_t aDisplayPageNumber) {
    sLCDDisplayPageNumber = aDisplayPageNumber;
    tone(BUZZER_PIN, 2200, 30);
    Serial.print(F("Set LCD display page to: "));
    Serial.println(aDisplayPageNumber);

// If BMS communication timeout, only timeout message or CAN Info page can be displayed.
    if (!sJKBMSFrameHasTimeout || aDisplayPageNumber == JK_BMS_PAGE_CAN_INFO) {
        /*
         * Show new page
         */
        printBMSDataOnLCD();
    }
}

#if defined(STANDALONE_TEST)
void testLCDPages() {
    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    printBMSDataOnLCD();

    sLCDDisplayPageNumber = JK_BMS_PAGE_CELL_INFO;
// Create symbols character for maximum and minimum
    bigNumberLCD._createChar(1, bigNumbersTopBlock);
    bigNumberLCD._createChar(2, bigNumbersBottomBlock);
    delay(2000);
    printBMSDataOnLCD();

    sLCDDisplayPageNumber = JK_BMS_PAGE_CAN_INFO;
    delay(2000);
    printBMSDataOnLCD();

    /*
     * Check display of maximum values
     */
    sJKFAllReplyPointer->SOCPercent = 100;
    JKComputedData.BatteryLoadPower = -11000;
    JKComputedData.BatteryLoadCurrentFloat = JKComputedData.BatteryLoadPower / JKComputedData.BatteryVoltageFloat;
    JKComputedData.TemperaturePowerMosFet = 111;
    JKComputedData.TemperatureSensor1 = 100;

    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    delay(2000);
    printBMSDataOnLCD();

    sLCDDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;
    bigNumberLCD.begin();
    delay(2000);
    printBMSDataOnLCD();

    /*
     * Test other values
     */
    sJKFAllReplyPointer->AlarmUnion.AlarmBits.PowerMosFetOvertemperatureAlarm = true;
    JKComputedData.TemperaturePowerMosFet = 90;
    JKComputedData.TemperatureSensor1 = 25;
    handleAndPrintAlarmInfo(); // this sets the LCD alarm string

    sJKFAllReplyPointer->SOCPercent = 1;
    JKComputedData.BatteryLoadPower = 12345;
    JKComputedData.BatteryLoadCurrentFloat = JKComputedData.BatteryLoadPower / JKComputedData.BatteryVoltageFloat;

    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    delay(2000);
    printBMSDataOnLCD();

    sLCDDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;
    delay(2000);
    printBMSDataOnLCD();

    lastJKReply.AlarmUnion.AlarmBits.PowerMosFetOvertemperatureAlarm = true; // to enable reset of LCD alarm string
    sJKFAllReplyPointer->AlarmUnion.AlarmBits.PowerMosFetOvertemperatureAlarm = false;
    JKComputedData.TemperaturePowerMosFet = 33;
    handleAndPrintAlarmInfo(); // this resets the LCD alarm string

    sJKFAllReplyPointer->SOCPercent = 100;
    JKComputedData.BatteryLoadCurrentFloat = -100;

    sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    delay(2000);
    printBMSDataOnLCD();
}

void testBigNumbers() {
    sLCDDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;

    for (int j = 0; j < 3; ++j) {
        // Test with 100 %  and 42 %

        /*
         * test with positive numbers
         */
        JKComputedData.BatteryLoadPower = 12345;
        JKComputedData.BatteryLoadCurrentFloat = JKComputedData.BatteryLoadPower / JKComputedData.BatteryVoltageFloat;

        for (int i = 0; i < 5; ++i) {
            delay(4000);
            printBMSDataOnLCD();
            JKComputedData.BatteryLoadPower /= 10; // 1234 -> 12
            JKComputedData.BatteryLoadCurrentFloat = JKComputedData.BatteryLoadPower / JKComputedData.BatteryVoltageFloat;
        }
        /*
         * test with negative numbers
         */
        JKComputedData.BatteryLoadPower = -12345;
        JKComputedData.BatteryLoadCurrentFloat = JKComputedData.BatteryLoadPower / JKComputedData.BatteryVoltageFloat;

        for (int i = 0; i < 5; ++i) {
            delay(4000);
            printBMSDataOnLCD();
            JKComputedData.BatteryLoadPower /= 10; // 1234 -> 12
            JKComputedData.BatteryLoadCurrentFloat = JKComputedData.BatteryLoadPower / JKComputedData.BatteryVoltageFloat;
        }

        sJKFAllReplyPointer->SOCPercent /= 10;
    }
}
#endif // STANDALONE_TEST

#endif // _JK_BMS_LCD_HPP
