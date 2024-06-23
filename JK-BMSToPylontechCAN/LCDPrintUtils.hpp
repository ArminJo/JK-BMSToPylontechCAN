/*
 *  LCDPrintUtils.hpp
 *
 *  Contains LCD related variables and functions
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

#ifndef _LCD_PRINT_UTILS_HPP
#define _LCD_PRINT_UTILS_HPP

//#define LOCAL_DEBUG // This enables debug output only for this file - only for development

/*
 * Helper macro for getting a macro definition as string
 */
#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

#define LCD_I2C_DEFAULT_ADDRESS 0x27     // Default LCD address is 0x27 for a I2C adaptor with PCF8574

#define DEGREE_SIGN_STRING "\xDF"

#if !defined(LCD_COLUMNS)
#define LCD_COLUMNS     20
#endif
extern char sStringBuffer[];    // For rendering a LCD row with sprintf_P()

#include "LiquidCrystal_I2C.hpp" // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use only the modified version delivered with this program!
extern LiquidCrystal_I2C myLCD;

void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint);
void LCDClearLine(uint8_t aLineNumber);
uint8_t getNumberOfDecimalsFor16BitValues(uint16_t a16BitValue);
void printFloatValueRightAlignedOnLCD(float aFloatValue, uint8_t aNumberOfCharactersToPrint, bool aNoLeadingSpaceForPositiveValues =
        false);
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
 * !!! We internally use uint16_t, for bigger values we have an overflow.
 */
uint8_t getNumberOfDecimalsFor16BitValues(uint16_t a16BitValue) {
    uint16_t tCompareValue = 1;
    /*
     * Check for 10, 100, 1000
     */
    for (uint_fast8_t tNumberOfDecimals = 0; tNumberOfDecimals < 5; ++tNumberOfDecimals) {
        if (a16BitValue < tCompareValue) {
            return tNumberOfDecimals;
        }
        tCompareValue *= 10;
    }
    // here we have values >= 10000
    return 5;
}

/*
 * !!! we internally use uint32_t, for bigger values we have an overflow.
 * Requires 26 bytes more program space than getNumberOfDecimalsFor16BitValues()
 */
uint8_t getNumberOfDecimalsFor32BitValues(uint32_t a32BitValue) {
    uint_fast8_t tNumberOfDecimals = 1;
    uint32_t tCompareValue = 10;
    /*
     * Check for 10, 100, 1000 up to 100,000,000
     */
    for (; tNumberOfDecimals < 10; ++tNumberOfDecimals) {
        if (a32BitValue < tCompareValue) {
            return tNumberOfDecimals;
        }
        tCompareValue *= 10;
    }
    // here we have values >= 1,000,000
    return 10;
}
/*
 * !!! We internally use uint16_t, for bigger values (> 65,536 or < -65,536) we have an overflow.
 *
 * @param aNumberOfCharactersToPrint - The characters to be used for the most negative value to show.
 *   I.e. 4 => max negative value is "-999" max positive value is " 999".
 *   Values below 10 and -10 are displayed as floats with decimal point " 9.9" and "-9.9".
 *   Values below 1 and -1 are displayed as floats without 0 before decimal point " .9" and "-.9".
 *   If positive, there is a leading space, which improves readability if directly concatenated to another value.
 *     e.g. "71V1.189A" is not readable, "71V 1.18A" is as well as "71V-1.18A".
 *
 * Saves programming space if used more than 1 times for printing floats if used instead of myLCD.print(JKComputedData.BatteryVoltageFloat, 2);
 */
void printFloatValueRightAlignedOnLCD(float aFloatValue, uint8_t aNumberOfCharactersToPrint,
        bool aNoLeadingSpaceForPositiveValues) {

    uint16_t tAbsValue = abs(aFloatValue); // remove sign for length computation
    uint8_t tNumberOfDecimals = getNumberOfDecimalsFor16BitValues(tAbsValue);
    int8_t tNumberOfDecimalPlaces = (aNumberOfCharactersToPrint - 2) - tNumberOfDecimals;
    if (aNoLeadingSpaceForPositiveValues && aFloatValue >= 0) {
        tNumberOfDecimalPlaces++; // Use this increased value internally, since we do not eventually print the '-'
    }
#if defined(LOCAL_DEBUG)
    Serial.print(F("NumberOfDecimalPlaces("));
    Serial.print(tAbsValue);
    Serial.print(F(", "));
    Serial.print(aNumberOfCharactersToPrint);
    Serial.print(F(")="));
    Serial.print(tNumberOfDecimalPlaces);
    Serial.print(F(" NumberOfDecimals="));
    Serial.println(tNumberOfDecimals);
#endif
    if (tNumberOfDecimalPlaces < 0) {
        tNumberOfDecimalPlaces = 0;
    }

    char *tStartOfString = sStringBuffer;
    if (tNumberOfDecimals == 0 && tNumberOfDecimalPlaces > 0) {
        if (aFloatValue >= 0) {
            if (!aNoLeadingSpaceForPositiveValues) {
                myLCD.print(' ');
            }
            tStartOfString = &sStringBuffer[1];
        } else {
            myLCD.print('-');
            tStartOfString = &sStringBuffer[2];
        }
    }
    dtostrf(aFloatValue, aNumberOfCharactersToPrint, tNumberOfDecimalPlaces, sStringBuffer);
    myLCD.print(tStartOfString);
}

void testPrintFloatValueRightAlignedOnLCD() {
    myLCD.clear();
    float tTestValue = 123.45;
    printFloatValueRightAlignedOnLCD(tTestValue, 6);
    printFloatValueRightAlignedOnLCD(tTestValue, 5);
    printFloatValueRightAlignedOnLCD(tTestValue, 4); // no leading space here
    printFloatValueRightAlignedOnLCD(tTestValue, 3); // no leading space here
    // Result=" 123.4  123 123123"

    myLCD.setCursor(0, 1);
    printFloatValueRightAlignedOnLCD(-tTestValue, 6);
    printFloatValueRightAlignedOnLCD(-tTestValue, 5);
    printFloatValueRightAlignedOnLCD(-tTestValue, 4);
    printFloatValueRightAlignedOnLCD(-tTestValue, 3); // requires also 5 character
    // Result="-123.4 -123-123-123"

    myLCD.setCursor(0, 2);
    tTestValue = -1.234;
    printFloatValueRightAlignedOnLCD(tTestValue, 6);
    printFloatValueRightAlignedOnLCD(tTestValue, 5);
    printFloatValueRightAlignedOnLCD(tTestValue, 4);
    printFloatValueRightAlignedOnLCD(tTestValue, 3);
    printFloatValueRightAlignedOnLCD(tTestValue, 2);
    // Result="-1.234-1.23-1.2 -1-1"

    myLCD.setCursor(0, 3);
    tTestValue = -0.1234;
    printFloatValueRightAlignedOnLCD(tTestValue, 6);
    printFloatValueRightAlignedOnLCD(tTestValue, 5);
    printFloatValueRightAlignedOnLCD(tTestValue, 4);
    printFloatValueRightAlignedOnLCD(tTestValue, 3);
    printFloatValueRightAlignedOnLCD(tTestValue, 2);
    // Result="-.1234-.123-.12-.1-0"

    delay(4000);

    myLCD.clear();
    tTestValue = 123.45;
    // no leading space here
    printFloatValueRightAlignedOnLCD(tTestValue, 6, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 5, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 4, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 3, true);
    // Result="123.45123.4 123123"

    myLCD.setCursor(0, 1);
    printFloatValueRightAlignedOnLCD(-tTestValue, 6, true);
    printFloatValueRightAlignedOnLCD(-tTestValue, 5, true);
    printFloatValueRightAlignedOnLCD(-tTestValue, 4, true);
    printFloatValueRightAlignedOnLCD(-tTestValue, 3, true);
    // Result="-123.4 -123-123-123"

    tTestValue = 1.2344; // .12345 leads to rounding up for .1235
    myLCD.setCursor(0, 2);
    printFloatValueRightAlignedOnLCD(tTestValue, 6, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 5, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 4, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 3, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 2, true);
    // Result="1.23441.2341.231.2 1"

    tTestValue = 0.12344; // .12345 leads to rounding up for .1235
    myLCD.setCursor(0, 3);
    printFloatValueRightAlignedOnLCD(tTestValue, 6, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 5, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 4, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 3, true);
    printFloatValueRightAlignedOnLCD(tTestValue, 2, true);
    // Result=".12344.1234.123.12.1"

    delay(4000);
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _LCD_PRINT_UTILS_HPP
