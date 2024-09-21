/*
 *  JK-BMS_LCD.h
 *
 *  Contains declarations for LCD related variables and functions
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

#ifndef _JK_BMS_LCD_H
#define _JK_BMS_LCD_H

/*
 * LCD hardware stuff
 */
#define LCD_I2C_DEFAULT_ADDRESS     0x27     // Default LCD address is 0x27 for a I2C adaptor with PCF8574
#define LCD_COLUMNS                 20
#define LCD_ROWS                    4
#define LCD_I2C_ADDRESS             LCD_I2C_DEFAULT_ADDRESS // 0x27 Default LCD address for a 20 chars and 4 line / 2004 display
extern bool sSerialLCDAvailable;

#if !defined(ENABLE_MONITORING) && defined(NO_ANALYTICS)
extern char sStringBuffer[LCD_COLUMNS + 1];    // Only for rendering a LCD row with sprintf_P()
#endif
/*
 * Display timeouts, may be adapted to your requirements
 */
#  if defined(STANDALONE_TEST)
#define DISPLAY_ON_TIME_STRING               "30 s"
#define DISPLAY_ON_TIME_SECONDS              30L // L to avoid overflow at macro processing
//#define NO_MULTIPLE_BEEPS_ON_TIMEOUT           // Activate it if you do not want multiple beeps
#define BEEP_ON_TIME_SECONDS_IF_TIMEOUT      10L // 10 s
#  else
#    if !defined(DISPLAY_ON_TIME_SECONDS)
#define DISPLAY_ON_TIME_SECONDS             300L // 5 minutes. L to avoid overflow at macro processing
#define DISPLAY_ON_TIME_STRING              "5 min" // Only for display on LCD
#    endif
#  endif

//#define DISPLAY_ALWAYS_ON   // Activate this, if you want the display to be always on.
#  if !defined(DISPLAY_ALWAYS_ON)
void doLCDBacklightTimeoutHandling();
bool checkAndTurnLCDOn();
extern bool sSerialLCDIsSwitchedOff;
extern uint16_t sFrameCounterForLCDTAutoOff;
#  endif

/*
 * LCD display pages
 */
#define JK_BMS_PAGE_OVERVIEW            0 // is selected in case of BMS alarm message
#define JK_BMS_PAGE_CELL_INFO           1
#if defined(NO_CELL_STATISTICS)
#define JK_BMS_PAGE_BIG_INFO            2
#define JK_BMS_PAGE_CAN_INFO            3 // Enter on long press
#define JK_BMS_PAGE_CAPACITY_INFO       4
#else
#define JK_BMS_PAGE_CELL_STATISTICS     2
#define CELL_STATISTICS_COUNTER_MASK 0x04 // must be a multiple of 2 and determines how often one page (min or max) is displayed.
#define JK_BMS_PAGE_BIG_INFO            3
#define JK_BMS_PAGE_CAN_INFO            4 // Enter on long press
#define JK_BMS_PAGE_CAPACITY_INFO       5
#endif

// Switch to start page after 10 seconds of JK_BMS_PAGE_CAPACITY_INFO. 10 seconds to allow EEROM clearing by long press.
#define JK_BMS_PAGE_CAPACITY_INFO_PAGE_TIMEOUT_MILLIS  10000
#define CELL_CAPACITY_COUNTER_VOLTAGE   0x06 // If counter "anded" with mask is true show delta voltages instead of percents.

#define JK_BMS_PAGE_MAX                 JK_BMS_PAGE_BIG_INFO
#define JK_BMS_DEBUG_PAGE_MAX           JK_BMS_PAGE_CAPACITY_INFO
#define JK_BMS_START_PAGE               JK_BMS_PAGE_BIG_INFO
//uint8_t sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW; // Start with Overview page
extern uint8_t sLCDDisplayPageNumber; // Start with Big Info page

void setLCDDisplayPage(uint8_t aLCDDisplayPageNumber, bool aDoNotPrint = false);

void printBMSDataOnLCD();
void printCANInfoOnLCD();
void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint);
void LCDClearLine(uint8_t aLineNumber);

void setupLCD();
void printDebugInfoOnLCD();

#  if !defined(DISPLAY_ALWAYS_ON)
bool checkAndTurnLCDOn();
void doLCDBacklightTimeoutHandling();
#  endif

void printShortEnableFlagsOnLCD();
void printShortStateOnLCD();
void printLongStateOnLCD();
void printAlarmHexOrStateOnLCD();
void printCellInfoOnLCD();

#if !defined(NO_CELL_STATISTICS)
void printCellStatisticsOnLCD();
#endif // !defined(NO_CELL_STATISTICS)

#if !defined(NO_ANALYTICS)
void printCapacityInfoOnLCD();
#endif

void printBigInfoOnLCD();
void printCANInfoOnLCD();
void printVoltageCurrentAndPowerOnLCD();
void printVoltageDifferenceAndTemperature();
void printTimeoutMessageOnLCD();
void printAlarmInfoOnLCD();
void printOverwiewOrAlarmInfoOnLCD();
void printBMSDataOnLCD();
void checkButtonPressForLCD();
void setLCDDisplayPage(uint8_t aLCDDisplayPageNumber, bool aDoNotPrint);

#if defined(STANDALONE_TEST)
void testLCDPages();
void testBigNumbers();
#endif // STANDALONE_TEST

#endif // _JK_BMS_LCD_H
