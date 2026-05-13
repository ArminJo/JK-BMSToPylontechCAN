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

#if !defined(ENABLE_MONITORING) && defined(NO_SOC_HISTORY)
extern char sStringBuffer[LCD_COLUMNS + 1];    // Only for rendering a LCD row with snprintf_P()
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
typedef enum {
    PageBigInfo = 0,
    PageOverview, /*selected in case of BMS alarm message*/
    PageCellInfo,
#if !defined(NO_CELL_STATISTICS)
    PageCellStatistics,
#endif
    DummyPageWrapAround1,
    PageCANInfo, /*Enter on long press*/
#if !defined(NO_SOC_HISTORY)
    PageSOCHistory,
#endif
#if !defined(NO_CAPACITY_INFO)
    PageCapacityInfo,
#endif
    DummyPageWrapAround2
} sPageNummberEnum;
#define JK_BMS_START_PAGE   PageBigInfo

#define CELL_STATISTICS_COUNTER_MASK 0x04 // must be a multiple of 2 and determines how often one page (min or max) is displayed.

// Switch to capacity page after 10 seconds of PageSOCHistory. 10 seconds to allow EEROM clearing by long press.
#define JK_BMS_PAGE_SOC_HISTORY_TIMEOUT_MILLIS    10000
#define CAPACITY_INFO_COUNTER_MASK_FOR_VOLTAGE_DISPLAY  0x06 // If counter "anded" with mask is true show delta voltages instead of percents.

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

#if !defined(NO_SOC_HISTORY)
void printSOCDataInfoOnLCD();
#endif
#if !defined(NO_CAPACITY_INFO)
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
void setLCDDisplayPage(sPageNummberEnum aLCDDisplayPageNumber, bool aDoNotPrint = false);

#if defined(STANDALONE_TEST)
void testLCDPages();
void testBigNumbers();
#endif // STANDALONE_TEST

#endif // _JK_BMS_LCD_H
