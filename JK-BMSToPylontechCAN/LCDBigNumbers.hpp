/*
 *  LCDBigNumbers.hpp
 *
 *  Arduino library to write big numbers on a 1602 or 2004 LCD.
 *
 *  Copyright (C) 2022-2023  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of LCDBigNumbers https://github.com/ArminJo/LCDBigNumbers.
 *
 *  LCDBigNumbers is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#ifndef _LCD_BIG_NUMBERS_HPP
#define _LCD_BIG_NUMBERS_HPP

#include <Arduino.h>

#define ONE_COLUMN_SPACE_CHARACTER      '|' // This input character is printed as a one column space. Normal spaces are printed as a space with the width of the number.
#define ONE_COLUMN_SPACE_STRING         "|" // This input string is printed as a one column space. Normal spaces are printed as a space with the width of the number.

#define ONE_COLUMN_HYPHEN_CHARACTER      '_' // This input character is printed as a one column hyphen. Normal hyphen / minus are printed as a hyphen with the width of the number - 1.
#define ONE_COLUMN_HYPHEN_STRING         "_" // This input string is printed as a one column hyphen. Normal hyphen / minus are printed as a hyphen with the width of the number - 1.

//#define USE_PARALLEL_2004_LCD // Is default
//#define USE_PARALLEL_1602_LCD
//#define USE_SERIAL_2004_LCD
//#define USE_SERIAL_1602_LCD

#if !defined(USE_PARALLEL_2004_LCD) && !defined(USE_PARALLEL_1602_LCD) && !defined(USE_SERIAL_2004_LCD) && !defined(USE_SERIAL_1602_LCD)
#define USE_PARALLEL_2004_LCD    // Use parallel 2004 LCD as default
#endif

#if defined(USE_PARALLEL_2004_LCD) || defined(USE_PARALLEL_1602_LCD)
#  if defined(USE_PARALLEL_2004_LCD)
#define LCD_COLUMNS     20
#define LCD_ROWS        4
#  else
#define LCD_COLUMNS     16
#define LCD_ROWS        2
#  endif
#define USE_PARALLEL_LCD
#include <LiquidCrystal.h>
#else
#  if defined(USE_SERIAL_2004_LCD)
#define LCD_COLUMNS     20
#define LCD_ROWS        4
#  else
#define LCD_COLUMNS     16
#define LCD_ROWS        2
#  endif
#include "LiquidCrystal_I2C.h"  // Use an up to date library version which has the init method
#endif

#define DEFAULT_TEST_DELAY  3000
#define NUMBER_OF_SPECIAL_CHARACTERS_IN_FONT_ARRAY  3

#define COLUMN_MASK     0x0C // Number of columns = shifted masked value + 1
#define ROW_MASK        0x03 // Number of rows = masked value + 1
#define VARIANT_MASK    0x30
// Numbers are created by using the above masks
#define BIG_NUMBERS_FONT_1_COLUMN_2_ROWS_VARIANT_1  0x01
#define BIG_NUMBERS_FONT_2_COLUMN_2_ROWS_VARIANT_1  0x05
#define BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_1  0x09
#define BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_2  0x19
#define BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_3  0x29
#define BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_1  0x06
#define BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_2  0x16
#define BIG_NUMBERS_FONT_3_COLUMN_3_ROWS_VARIANT_1  0x0A
#define BIG_NUMBERS_FONT_3_COLUMN_4_ROWS_VARIANT_1  0x0B
#define BIG_NUMBERS_FONT_3_COLUMN_4_ROWS_VARIANT_2  0x1B

#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // To debug/understand the writeBigNumber() function - only for development
#endif

// !!! Must be without comment and closed by @formatter:on
// @formatter:off

// http://www.picbasic.co.uk/forum/showthread.php?t=13376
// 8 custom characters for 1 column font
const uint8_t bigNumbers1x2CustomPatterns_1[][8] PROGMEM = {
 { B11110, B10010, B10010, B10010, B10010, B10010, B10010, B11110 }, // 0 Closed rectangle - 8
 { B11110, B10010, B10010, B10010, B10010, B10010, B10010, B10010 }, // 1 Rectangle - open at bottom - 0
 { B10010, B10010, B10010, B10010, B10010, B10010, B10010, B11110 }, // 2 Rectangle - open at top - 0
 { B11110, B00010, B00010, B00010, B00010, B00010, B00010, B11110 }, // 3 Rectangle - open at left
 { B11110, B10000, B10000, B10000, B10000, B10000, B10000, B11110 }, // 4 Rectangle - open at right
 { B00010, B00010, B00010, B00010, B00010, B00010, B00010, B00010 }, // 5 Right bar - 1
 { B11110, B00010, B00010, B00010, B00010, B00010, B00010, B00010 }, // 6 Top right - 7
 { B00010, B00010, B00010, B00010, B00010, B00010, B00010, B11110 }  // 7 Right bottom - 3,5,9
};
const uint8_t bigNumbers1x2_1[2][13] PROGMEM = {                   // 2-line numbers
//    "-"   "."   ":"    0     1     2     3     4     5     6     7     8     9
    { 0x5F, 0xFE, 0xA5, 0x01, 0x05, 0x06, 0x03, 0x02, 0x04, 0x04, 0x06, 0x00, 0x00 },
    { 0xFE, 0x2E, 0xA5, 0x02, 0x05, 0x04, 0x07, 0x05, 0x07, 0x02, 0x05, 0x02, 0x07 }
};

// https://www.alpenglowindustries.com/blog/the-big-numbers-go-marching-2x2#/
// https://github.com/AlpenglowIndustries/Alpenglow_BigNums2x2
// 8 custom characters for Trek font
// Requires 1 0xFF Block for the special "0"
const uint8_t bigNumbers2x2CustomPatterns_1[][8] PROGMEM = {
 { B11111, B11111, B00000, B00000, B00000, B00000, B00000, B00000 }, // 0
 { B11000, B11000, B11000, B11000, B11000, B11000, B11000, B11000 }, // 1
 { B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111 }, // 2
 { B11111, B11111, B00011, B00011, B00011, B00011, B11111, B11111 }, // 3
 { B11111, B11111, B11000, B11000, B11000, B11000, B11111, B11111 }, // 4
 { B11111, B11111, B11000, B11000, B11000, B11000, B11000, B11000 }, // 5
 { B00011, B00011, B00011, B00011, B00011, B00011, B11111, B11111 }, // 6
 { B11000, B11000, B11000, B11000, B11000, B11000, B11111, B11111 } // 7
};
const uint8_t bigNumbers2x2_1[2][23] PROGMEM = {                   // 2-line numbers
//    "-"   "."   ":"       0          1          2          3          4          5          6          7          8          9
    { 0xFE, 0xFE, 0xA5, 0x05,0xFF, 0x00,0x01, 0x00,0x03, 0x00,0x03, 0x01,0x01, 0x04,0x00, 0x05,0x00, 0x00,0x03, 0x04,0x03, 0x04,0x03},
    { 0x00, 0x2E, 0xA5, 0x07,0x06, 0x02,0x07, 0x04,0x02, 0x02,0x03, 0x00,0x05, 0x02,0x03, 0x04,0x03, 0xFE,0x01, 0x04,0x03, 0x02,0x06}
};


// 3x2 https://liudr.wordpress.com/2011/03/21/big-font/
// 3x2 http://www.netzmafia.de/skripten/hardware/Arduino/LCD/index.html
// Requires 0xFF Blocks, but character 6 could be used for it
const uint8_t bigNumbers3x2CustomPatterns_1[6][8] PROGMEM = {
  { B11111,B11111,B00000,B00000,B00000,B00000,B00000,B00000 }, // 0 Upper bar
  { B00000,B00000,B00000,B00000,B00000,B00000,B11111,B11111 }, // 1 Lower bar
  { B11111,B11111,B00000,B00000,B00000,B00000,B11111,B11111 }, // 2 Upper and lower bar
  { B00000,B00000,B00000,B11111,B11111,B00000,B00000,B00000 }, // 3 Minus sign
  { B00000,B00000,B00000,B00000,B00000,B01110,B01110,B01110 }, // 4 Decimal point
  { B00000,B00000,B01110,B01110,B01110,B00000,B00000,B00000 }  // 5 Colon
};
const uint8_t bigNumbers3x2_1[2][33] PROGMEM = {               // 2-line numbers
//    "-"   "."   ":"         0               1               2               3               4               5               6               7               8               9
    { 0x01, 0xFE, 0x05, 0xFF,0x00,0xFF, 0x00,0xFF,0xFE, 0x02,0x02,0xFF, 0x00,0x02,0xFF, 0xFF,0x01,0xFF, 0xFF,0x02,0x02, 0xFF,0x02,0x02, 0x00,0x00,0xFF, 0xFF,0x02,0xFF, 0xFF,0x02,0xFF},
    { 0xFE, 0x04, 0x05, 0xFF,0x01,0xFF, 0x01,0xFF,0x01, 0xFF,0x01,0x01, 0x01,0x01,0xFF, 0xFE,0xFE,0xFF, 0x01,0x01,0xFF, 0xFF,0x01,0xFF, 0xFE,0xFE,0xFF, 0xFF,0x01,0xFF, 0x01,0x01,0xFF}
};

// 3x2 https://forum.arduino.cc/t/display-3-character-wide-big-digits-on-16x2-lcd/905360 bottom of page
// Requires 0xFF Blocks
const uint8_t bigNumbers3x2CustomPatterns_2[8][8] PROGMEM = {
  { B11111,B11111,B11111,B00000,B00000,B00000,B00000,B00000 }, // 0 Upper bar
  { B00000,B00000,B00000,B00000,B00000,B11111,B11111,B11111 }, // 1 Lower bar
  { B11111,B11111,B11111,B00000,B00000,B00000,B11111,B11111 }, // 2 Upper and lower bar for 5,6
  { B11100,B11100,B11100,B11100,B11100,B11100,B11100,B11100 }, // 3 Left bar
  { B00000,B00000,B00000,B00000,B00000,B11100,B11100,B11100 }, // 4 Left lower bar for 2
  { B11100,B11100,B11100,B00000,B00000,B00000,B11100,B11100 }, // 5 Left upper and lower bar for 5,6
  { B00000,B00000,B00000,B00000,B00000,B01110,B01110,B01110 }, // 6 Decimal point
  { B00000,B00000,B01110,B01110,B01110,B00000,B00000,B00000 }  // 7 Colon
};
const uint8_t bigNumbers3x2_2[2][33] PROGMEM = {               // 2-line numbers
//    "-"   "."   ":"         0               1               2               3               4               5               6               7               8               9
    { 0x01, 0xFE, 0x07, 0xFF,0x00,0x03, 0x00,0x03,0xFE, 0x02,0x02,0x03, 0x02,0x02,0x03, 0xFF,0x01,0x03, 0xFF,0x02,0x05, 0xFF,0x02,0x05, 0x00,0x00,0x03, 0xFF,0x02,0x03, 0xFF,0x02,0x03},
    { 0xFE, 0x06, 0x07, 0xFF,0x01,0x03, 0xFE,0x03,0xFE, 0xFF,0x01,0x04, 0x01,0x01,0x03, 0xFE,0xFE,0x03, 0x01,0x01,0x03, 0xFF,0x01,0x03, 0xFE,0xFE,0x03, 0xFF,0x01,0x03, 0xFE,0xFE,0x03}
};

//3x2 https://exploreembedded.com/wiki/Distance_Meter_with_Big_Fonts
const uint8_t bigNumbers3x2CustomPatterns_3[8][8] PROGMEM = {
{ B11100, B11110, B11110, B11110, B11110, B11110, B11110, B11100}, // 0 left bar
{ B00111, B01111, B01111, B01111, B01111, B01111, B01111, B00111}, // 1 right bar
{ B11111, B11111, B00000, B00000, B00000, B00000, B11111, B11111}, // 2 upper and lower bar
{ B11110, B11100, B00000, B00000, B00000, B00000, B11000, B11100}, // 3 left upper and lower rounded
{ B01111, B00111, B00000, B00000, B00000, B00000, B00011, B00111}, // 4 right upper and lower rounded
{ B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111}, // 5 right lower
{ B00000, B00000, B00000, B00000, B00000, B00000, B00111, B01111}, // 6 right lower rounded
{ B11111, B11111, B00000, B00000, B00000, B00000, B00000, B00000}  // 7 upper bar
};
const uint8_t bigNumbers3x2_3[2][33] PROGMEM = {                   // 2-line numbers
//    "-"   "."   ":"         0               1               2               3               4               5               6               7               8               9
    { 0xFE, 0xFE, 0xA5, 0x01,0x07,0x00, 0xFE,0x00,0xFE, 0x04,0x02,0x00, 0x04,0x02,0x00, 0x01,0x05,0x00, 0x01,0x02,0x03, 0x01,0x02,0x03, 0x01,0x07,0x00, 0x01,0x02,0x00, 0x01,0x02,0x00},
    { 0x07, 0x06, 0xA5, 0x01,0x05,0x00, 0xFE,0x00,0xFE, 0x01,0x05,0x05, 0x06,0x05,0x00, 0xFE,0xFE,0x00, 0x06,0x05,0x00, 0x01,0x05,0x00, 0xFE,0xFE,0x00, 0x01,0x05,0x00, 0x06,0x05,0x00}
};

// http://woodsgood.ca/projects/2015/01/16/large-numbers-on-small-displays/
// 2x3 - Version 1 with space above
const uint8_t bigNumbers2x3CustomPatterns_1[8][8] PROGMEM = { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07 }, // char 0: bottom right
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x1C, 0x1C },     // char 1: bottom left
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F },     // char 2: bottom block
        { 0x07, 0x07, 0x07, 0x07, 0x07, 0x1F, 0x1F, 0x1F },     // char 3: right bottom block
        { 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1F, 0x1F, 0x1F },     // char 4: left bottom block
        { 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },     // char 5: left bar
        { 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07 },     // char 6: right bar
        { 0x00, 0x00, 0x0E, 0x0E, 0x0E, 0x00, 0x00, 0x00 }      // char 7: 3x3 block for colon
};
const uint8_t bigNumbers2x3_1[3][23] PROGMEM = {                   // 3-line numbers
//    "-"   "."   ":"       0          1          2          3          4          5          6          7          8          9
    { 0xFE, 0xFE, 0xFE, 0x02,0x02, 0x00,0x01, 0x02,0x02, 0x02,0x02, 0x01,0x00, 0x02,0x02, 0x01,0xFE, 0x02,0x02, 0x02,0x02, 0x02,0x02},
    { 0x02, 0xFE, 0x07, 0x05,0x06, 0xFE,0x05, 0x02,0x03, 0x00,0x03, 0x04,0x03, 0x04,0x02, 0x04,0x02, 0xFE,0x06, 0x04,0x03, 0x04,0x03},
    { 0xFE, 0x01, 0x07, 0x04,0x03, 0x00,0x04, 0x04,0x02, 0x02,0x03, 0xFE,0x06, 0x02,0x03, 0x04,0x03, 0xFE,0x06, 0x04,0x03, 0xFE,0x06}
};

// 2x3 - Version 2 with space below
const uint8_t bigNumbers2x3CustomPatterns_2[][8] PROGMEM = { { 0x07, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00  }, // char 0: top right
        { 0x1C, 0x1C, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00 },     // char 1: top left
        { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00 },     // char 2: top block
        { 0x1F, 0x1F, 0x1F, 0x07, 0x07, 0x07, 0x07, 0x07 },     // char 3: right top block
        { 0x1F, 0x1F, 0x1F, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },     // char 4: left top block
        { 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },     // char 5: left bar
        { 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07 },     // char 6: right bar
        { 0x00, 0x00, 0x0E, 0x0E, 0x0E, 0x00, 0x00, 0x00 }      // char 7: 3x3 block for colon
};

const uint8_t bigNumbers2x3_2[][23] PROGMEM = {                   // 3-line numbers
//    "-"   "."   ":"       0          1          2          3          4          5          6          7          8          9
    { 0xFE, 0xFE, 0x07, 0x04,0x03, 0x00,0x05, 0x02,0x03, 0x02,0x03, 0x05,0x06, 0x04,0x02, 0x05,0xFE, 0x02,0x03, 0x04,0x03, 0x04,0x03},
    { 0x02, 0xFE, 0x07, 0x05,0x06, 0xFE,0x05, 0x04,0x02, 0x00,0x03, 0x02,0x03, 0x02,0x03, 0x04,0x03, 0xFE,0x06, 0x04,0x03, 0x02,0x03},
    { 0xFE, 0x01, 0xFE, 0x02,0x02, 0x00,0x02, 0x02,0x02, 0x02,0x02, 0xFE,0x00, 0x02,0x02, 0x02,0x02, 0xFE,0x00, 0x02,0x02, 0xFE,0x00}
};

// 3x4 Font custom patterns http://woodsgood.ca/projects/2015/03/06/3-4-line-big-font-numerals/
// Requires 0xFF Blocks
const uint8_t bigNumbers3x3And3x4CustomPatterns_1[][8] PROGMEM = { { 0x01, 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }, // char 0: bottom right triangle
        { 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F },     // char 1: bottom block
        { 0x10, 0x1C, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },     // char 2: bottom left triangle
        { 0x1F, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00 },     // char 3: top right triangle
        { 0x1F, 0x1E, 0x1C, 0x10, 0x00, 0x00, 0x00, 0x00 },     // char 4: top left triangle
        { 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00 },     // char 5: upper block
        { 0x10, 0x1C, 0x1E, 0x1F, 0x00, 0x00, 0x00, 0x00 },     // char 6: full top left triangle Used only once in 7
//        { 0x1F, 0x1F, 0x1E, 0x1C, 0x18, 0x10, 0x00, 0x00 },     // char 6: full top left triangle Used only once in 7
        { 0x01, 0x07, 0x0F, 0x1F, 0x00, 0x00, 0x00, 0x00 }      // char 7: top right triangle
};

const uint8_t bigNumbers3x3_1[3][33] PROGMEM = {   // 3-line numbers
//    "-"   "."   ":"         0               1               2               3               4              5               6                7               8               9
    { 0xFE, 0xFE, 0x01, 0x00,0x05,0x02, 0x07,0xFF,0xFE, 0x07,0x05,0x02, 0x07,0x05,0x02, 0xFF,0xFE,0xFF, 0xFF,0x05,0x05, 0x00,0x05,0x06, 0x05,0x05,0xFF, 0x00,0x05,0x02, 0x00,0x05,0x02},
    { 0x05, 0xFE, 0x01, 0xFF,0xFE,0xFF, 0xFE,0xFF,0xFE, 0x00,0x05,0x05, 0xFE,0x05,0xFF, 0x05,0x05,0xFF, 0x05,0x05,0xFF, 0xFF,0x05,0x02, 0xFE,0x00,0x04, 0xFF,0x05,0xFF, 0x03,0x05,0xFF},
    { 0xFE, 0x05, 0xFE, 0x03,0x05,0x04, 0xFE,0x05,0xFE, 0x05,0x05,0x05, 0x03,0x05,0x04, 0xFE,0xFE,0x05, 0x03,0x05,0x04, 0x03,0x05,0x04, 0xFE,0x05,0xFE, 0x03,0x05,0x04, 0x03,0x05,0x04}
};

// 3x4 Font variant 1
const uint8_t bigNumbers3x4_1[4][33] PROGMEM = {                   // 4-line numbers
//    "-"   "."   ":"         0               1               2               3               4              5               6                7               8               9
    { 0xFE, 0xFE, 0xFE, 0x00,0x05,0x02, 0x07,0xFF,0xFE, 0x07,0x05,0x02, 0x07,0x05,0x02, 0xFF,0xFE,0xFF, 0xFF,0x05,0x05, 0x00,0x05,0x02, 0x05,0x05,0xFF, 0x00,0x05,0x02, 0x00,0x05,0x02},
    { 0x01, 0xFE, 0x05, 0xFF,0xFE,0xFF, 0xFE,0xFF,0xFE, 0x01,0x01,0xFF, 0xFE,0x01,0xFF, 0xFF,0x01,0xFF, 0xFF,0x01,0x01, 0xFF,0x01,0x01, 0xFE,0x00,0x04, 0xFF,0x01,0xFF, 0xFF,0x01,0xFF},
    { 0xFE, 0xFE, 0x01, 0xFF,0xFE,0xFF, 0xFE,0xFF,0xFE, 0xFF,0xFE,0xFE, 0xFE,0xFE,0xFF, 0xFE,0xFE,0xFF, 0xFE,0xFE,0xFF, 0xFF,0xFE,0xFF, 0xFE,0xFF,0xFE, 0xFF,0xFE,0xFF, 0xFE,0xFE,0xFF},
    { 0xFE, 0x05, 0xFE, 0x03,0x05,0x04, 0xFE,0x05,0xFE, 0x05,0x05,0x05, 0x03,0x05,0x04, 0xFE,0xFE,0x05, 0x03,0x05,0x04, 0x03,0x05,0x04, 0xFE,0x05,0xFE, 0x03,0x05,0x04, 0x03,0x05,0x04}
};

// 3x4 Font variant 2
// https://forum.arduino.cc/t/wie-bekommt-man-solch-grosse-zahlen-hin/986148/12
// Requires 0xFF Blocks
const uint8_t bigNumbers3x4CustomPatterns_2[][8] PROGMEM = {
{ B00000, B00000, B00000, B00000, B00001, B00111, B01111, B11111 }, // char 0: bottom right triangle
{ B00000, B00000, B00000, B00000, B11111, B11111, B11111, B11111 }, // char 1: bottom block
{ B00000, B00000, B00000, B00000, B10000, B11100, B11110, B11111 }, // char 2: bottom left triangle
{ B11111, B01111, B00111, B00001, B00000, B00000, B00000, B00000 }, // char 3: top right triangle
{ B11111, B11111, B11111, B11111, B00000, B00000, B00000, B00000 }, // char 4: upper block
{ B11111, B11110, B11100, B10000, B00000, B00000, B00000, B00000 }, // char 5: top left triangle
{ B11111, B11111, B11111, B11111, B11111, B01111, B00111, B00001 }, // char 6: full top right triangle
{ B11111, B11111, B11111, B11111, B11111, B11110, B11100, B10000 }  // char 7: full top left triangle
};

const uint8_t bigNumbers3x4_2[4][33] PROGMEM = {                         // 4-line numbers
//    "-"   "."   ":"         0               1               2               3               4              5               6                7               8               9
    { 0xFE, 0xFE, 0xFE, 0x00,0x01,0x02, 0x00,0x01,0xFE, 0x00,0x01,0x02, 0x00,0x01,0x02, 0x00,0xFE,0x01, 0x01,0x01,0x01, 0x00,0x01,0x02, 0x01,0x01,0x01, 0x00,0x01,0x02, 0x00,0x01,0x02},
    { 0x01, 0xFE, 0x04, 0xFF,0x00,0xFF, 0x05,0xFF,0xFE, 0x04,0x00,0x07, 0x04,0x00,0x07, 0xFF,0xFE,0xFF, 0x06,0x01,0x02, 0xFF,0x01,0x02, 0xFE,0x00,0x07, 0x06,0x01,0x07, 0x06,0x01,0xFF},
    { 0xFE, 0xFE, 0x01, 0xFF,0x05,0xFF, 0xFE,0xFF,0xFE, 0xFF,0x05,0xFE, 0x01,0x03,0xFF, 0x04,0x04,0xFF, 0x01,0xFE,0xFF, 0xFF,0xFE,0xFF, 0xFF,0x05,0xFE, 0xFF,0xFE,0xFF, 0xFE,0xFE,0xFF},
    { 0xFE, 0x04, 0xFE, 0x03,0x04,0x05, 0xFE,0x04,0xFE, 0x04,0x04,0x04, 0x03,0x04,0x05, 0xFE,0xFE,0x04, 0x03,0x04,0x05, 0x03,0x04,0x05, 0x04,0xFE,0xFE, 0x03,0x04,0x05, 0x03,0x04,0x05}
};

// 4x4: https://github.com/wa1hco/BigFont
// @formatter:on
class LCDBigNumbers: public Print {

public:
#if defined(USE_PARALLEL_LCD)
    LiquidCrystal *LCD;
#else
    LiquidCrystal_I2C *LCD;
#endif
    uint8_t NumberWidth;
    uint8_t NumberHeight;
    uint8_t FontVariant;
    const uint8_t (*bigNumbersCustomPatterns)[8];
    uint8_t NumberOfCustomPatterns;
    const uint8_t *bigNumbersFont;
    bool forceGapBetweenNumbers;    // The default depends on the font used
    uint8_t upperLeftColumnIndex;   // Start of the next character
    uint8_t maximumColumnIndex; // Maximum of columns to be written. Used to not clear the gap after a number which ends at the last column. ( 44 bytes program space)
    uint8_t upperLeftRowIndex;      // Start of the next character

    /*
     *
     */
    void setBigNumberCursor(uint8_t aUpperLeftColumnIndex, uint8_t aUpperLeftRowIndex = 0) {
        upperLeftColumnIndex = aUpperLeftColumnIndex;
        upperLeftRowIndex = aUpperLeftRowIndex;
    }

    size_t write(uint8_t aBigNumberValue) {
        return writeBigNumber(aBigNumberValue);
    }

    /*
     * Creates custom character used for generating big numbers
     */
    void begin() {
        maximumColumnIndex = 0;
        // create maximum 8 custom characters
        for (uint_fast8_t i = 0; i < NumberOfCustomPatterns; i++) {
            _createChar(i, bigNumbersCustomPatterns[i]);
        }
    }

    void enableGapBetweenNumbers() {
        forceGapBetweenNumbers = true;
    }
    void disableGapBetweenNumbers() {
        forceGapBetweenNumbers = false;
    }
    void setGapBetweenNumbers(bool aForceGapBetweenNumbers) {
        forceGapBetweenNumbers = aForceGapBetweenNumbers;
    }

    /*
     * Internal function to select the appropriate font arrays
     */
    void init(const uint8_t aBigNumberFontIdentifier) {
        setBigNumberCursor(0);
        NumberWidth = ((aBigNumberFontIdentifier & COLUMN_MASK) >> 2) + 1;
        NumberHeight = (aBigNumberFontIdentifier & ROW_MASK) + 1;
        FontVariant = ((aBigNumberFontIdentifier & VARIANT_MASK) >> 4) + 1;
        NumberOfCustomPatterns = 8;
        forceGapBetweenNumbers = true;
        switch (aBigNumberFontIdentifier) {
        case BIG_NUMBERS_FONT_1_COLUMN_2_ROWS_VARIANT_1:
            bigNumbersCustomPatterns = bigNumbers1x2CustomPatterns_1;
            bigNumbersFont = (const uint8_t*) bigNumbers1x2_1;
            forceGapBetweenNumbers = false;
            break;
        case BIG_NUMBERS_FONT_2_COLUMN_2_ROWS_VARIANT_1:
            bigNumbersCustomPatterns = bigNumbers2x2CustomPatterns_1;
            bigNumbersFont = (const uint8_t*) bigNumbers2x2_1;
            break;
        case BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_1:
            bigNumbersCustomPatterns = bigNumbers3x2CustomPatterns_1;
            bigNumbersFont = (const uint8_t*) bigNumbers3x2_1;
            NumberOfCustomPatterns = 6;
            break;
        case BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_2:
            bigNumbersCustomPatterns = bigNumbers3x2CustomPatterns_3;
            bigNumbersFont = (const uint8_t*) bigNumbers3x2_3;
            forceGapBetweenNumbers = false;
            break;
        case BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_3:
            bigNumbersCustomPatterns = bigNumbers3x2CustomPatterns_2;
            bigNumbersFont = (const uint8_t*) bigNumbers3x2_2;
            forceGapBetweenNumbers = false;
            break;
#if LCD_ROWS <= 2
            default:
                // ERROR: NumberHeight is greater than 2 for a 2 line display -> fallback to 2x2 font
                bigNumbersCustomPatterns = bigNumbers2x2CustomPatterns_1;
                bigNumbersFont = (const uint8_t*) bigNumbers2x2_1;
                break;
#else
        case BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_1:
            bigNumbersCustomPatterns = bigNumbers2x3CustomPatterns_1;
            bigNumbersFont = (const uint8_t*) bigNumbers2x3_1;
            break;
        case BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_2:
            bigNumbersCustomPatterns = bigNumbers2x3CustomPatterns_2;
            bigNumbersFont = (const uint8_t*) bigNumbers2x3_2;
            break;
        case BIG_NUMBERS_FONT_3_COLUMN_3_ROWS_VARIANT_1:
            bigNumbersCustomPatterns = bigNumbers3x3And3x4CustomPatterns_1;
            bigNumbersFont = (const uint8_t*) bigNumbers3x3_1;
            break;
        case BIG_NUMBERS_FONT_3_COLUMN_4_ROWS_VARIANT_1:
            bigNumbersCustomPatterns = bigNumbers3x3And3x4CustomPatterns_1;
            bigNumbersFont = (const uint8_t*) bigNumbers3x4_1;
            break;
        case BIG_NUMBERS_FONT_3_COLUMN_4_ROWS_VARIANT_2:
            bigNumbersCustomPatterns = bigNumbers3x4CustomPatterns_2;
            bigNumbersFont = (const uint8_t*) bigNumbers3x4_2;
            break;
#endif

        }
    }

#if defined(USE_PARALLEL_LCD)
    LCDBigNumbers(LiquidCrystal *aLCD, const uint8_t aBigNumberFontIdentifier) :
#else
    LCDBigNumbers(LiquidCrystal_I2C *aLCD, const uint8_t aBigNumberFontIdentifier) :
#endif
                    LCD(aLCD) {
        init(aBigNumberFontIdentifier);
    }

    //createChar with PROGMEM input
    void _createChar(uint8_t location, const uint8_t *charmap) {
        location &= 0x7; // we only have 8 locations 0-7
        LCD->command(LCD_SETCGRAMADDR | (location << 3));
        for (int i = 0; i < 8; i++) {
            LCD->write(pgm_read_byte(charmap++));
        }
    }

    /**
     * Draws a big digit of size aNumberWidth x aNumberHeight at cursor position
     * Special characters always have the width of 1!
     * After each number one column gap is inserted. The gap is cleared, if not at the (last + 1) column!
     * @param aNumber - byte 0x00 to 0x09 or ASCII number or one of ' ', '|', '-', '_', '.' and ':' special characters to display
     * @return  The number of columns written (1 to 4 currently)
     */
    size_t writeBigNumber(uint8_t aNumberOrSpecialCharacter) {
        uint_fast8_t tFontArrayOffset = 0;
        uint_fast8_t tCharacterWidth = 1;
        /*
         * First 3 entries are the special characters
         * All non characters not compared with here, are mapped to a space with the width of the number
         */
        if (aNumberOrSpecialCharacter == '-' || aNumberOrSpecialCharacter == ONE_COLUMN_HYPHEN_CHARACTER) {
            // here we have the initial values: tFontArrayOffset = 0; and tCharacterWidth = 1;
        } else if (aNumberOrSpecialCharacter == '.') {
            tFontArrayOffset = 1;
        } else if (aNumberOrSpecialCharacter == ':') {
            tFontArrayOffset = 2;
        } else if (aNumberOrSpecialCharacter == ' ') {
            tCharacterWidth = NumberWidth;
        } else if (aNumberOrSpecialCharacter == ONE_COLUMN_SPACE_CHARACTER) {
            // print a one column space
            aNumberOrSpecialCharacter = ' ';
        } else {
            if (aNumberOrSpecialCharacter > 9) {
                // if not byte 0x00 to 0x09, convert number character to ASCII
                aNumberOrSpecialCharacter -= '0'; // convert ASCII value to number
            }
            if (aNumberOrSpecialCharacter > 9) {
                // If we have a non number character now, we convert it to a space with the width of the number
                aNumberOrSpecialCharacter = ' ';
            }
            tCharacterWidth = NumberWidth;
            tFontArrayOffset = NUMBER_OF_SPECIAL_CHARACTERS_IN_FONT_ARRAY + (aNumberOrSpecialCharacter * tCharacterWidth);
        }
#if defined(LOCAL_DEBUG)
        Serial.print(F("Number="));
        Serial.print(aNumberOrSpecialCharacter);
        Serial.print(F(" CharacterWidth="));
        Serial.print(tCharacterWidth);
        Serial.print(F(" FontArrayOffset="));
        Serial.print(tFontArrayOffset);
        Serial.print(F(" ColunmOffset="));
        Serial.println(upperLeftColumnIndex);
#endif
        const uint8_t *tArrayPtr = bigNumbersFont + tFontArrayOffset;
        for (uint_fast8_t tRow = 0; tRow < NumberHeight; tRow++) {
            LCD->setCursor(upperLeftColumnIndex, upperLeftRowIndex + tRow);
            for (uint_fast8_t i = 0; i < tCharacterWidth; i++) {
                uint8_t tCharacterIndex;
                if (aNumberOrSpecialCharacter == ' ') {
                    tCharacterIndex = ' '; // Blank
                } else {
                    tCharacterIndex = pgm_read_byte(tArrayPtr);
                }
                LCD->write(tCharacterIndex);
                tArrayPtr++; // next number column
#if defined(LOCAL_DEBUG)
                Serial.print(F(" 0x"));
                Serial.print(tCharacterIndex, HEX);
                Serial.print(F(" 0x"));
                Serial.print((uint16_t) tArrayPtr, HEX);
#endif
            }
            tArrayPtr += NUMBER_OF_SPECIAL_CHARACTERS_IN_FONT_ARRAY + (NumberWidth - tCharacterWidth) + (9 * NumberWidth); // Next array row
#if defined(LOCAL_DEBUG)
            Serial.print('|');
#endif
        }
        upperLeftColumnIndex += tCharacterWidth;

        if (maximumColumnIndex < upperLeftColumnIndex) {
            // find maximum column at runtime
            maximumColumnIndex = upperLeftColumnIndex;
        }

        /*
         * Implement the gap after the character
         */
        if (forceGapBetweenNumbers && (NumberWidth == 1 || tCharacterWidth > 1 || aNumberOrSpecialCharacter == '-')) {
            if (maximumColumnIndex != upperLeftColumnIndex) {
                // We are not at the last column, so clear the gap after the number
                for (uint_fast8_t tRow = 0; tRow < NumberHeight; tRow++) {
                    LCD->setCursor(upperLeftColumnIndex + 1, upperLeftRowIndex + tRow);
                    LCD->write(' '); // Blank
                }
                tCharacterWidth++;
            }
            upperLeftColumnIndex++; // This provides one column gap between big numbers, but not between special characters.
        }

#if defined(LOCAL_DEBUG)
        Serial.println();
#endif
        return tCharacterWidth;
    }

    /**
     * Draws a big digit of size aNumberWidth x aNumberHeight
     * @param aNumber - Number to display, if > 9 a blank character is drawn
     * @param aUpperLeftColumnIndex - Starts with 0, no check!
     * @param aStartRowIndex - Starts with 0, no check!
     */
    size_t writeAt(uint8_t aNumber, uint8_t aUpperLeftColumnIndex, uint8_t aUpperLeftRowIndex = 0) {
        setBigNumberCursor(aUpperLeftColumnIndex, aUpperLeftRowIndex);
        return writeBigNumber(aNumber);
    }

};

#if defined(USE_PARALLEL_LCD)
void printSpaces(LiquidCrystal *aLCD, uint_fast8_t aNumberOfSpacesToPrint)
#else
void printSpaces(LiquidCrystal_I2C *aLCD, uint_fast8_t aNumberOfSpacesToPrint)
#endif
        {
    for (uint_fast8_t i = 0; i < aNumberOfSpacesToPrint; ++i) {
        aLCD->print(' ');
    }
}

#if defined(USE_PARALLEL_LCD)
void clearLine(LiquidCrystal *aLCD, uint_fast8_t aLineNumber)
#else
void clearLine(LiquidCrystal_I2C *aLCD, uint_fast8_t aLineNumber)
#endif
        {
    aLCD->setCursor(0, aLineNumber);
    printSpaces(aLCD, LCD_COLUMNS);
    aLCD->setCursor(0, aLineNumber);
}

#if defined(USE_PARALLEL_LCD)
size_t printHex(LiquidCrystal *aLCD, uint16_t aHexByteValue)
#else
size_t printHex(LiquidCrystal_I2C *aLCD, uint16_t aHexByteValue)
#endif
        {
    aLCD->print(F("0x"));
    size_t tPrintSize = 2;
    if (aHexByteValue < 0x10 || (aHexByteValue > 0x100 && aHexByteValue < 0x1000)) {
        aLCD->print('0'); // leading 0
        tPrintSize++;
    }
    return aLCD->print(aHexByteValue, HEX) + tPrintSize;
}

/*
 * On my 2004 LCD the custom characters are available under 0 to 7 and mirrored to 8 to 15
 * The characters 0x80 to 0x8F are blanks
 */
#if defined(USE_PARALLEL_LCD)
void showSpecialCharacters(LiquidCrystal *aLCD)
#else
void showSpecialCharacters(LiquidCrystal_I2C *aLCD)
#endif
        {
    aLCD->setCursor(0, 0);
    // 0 to 7 are mirrored to 8 to 15 as described in datasheet
    for (uint_fast8_t i = 0; i < 0x8; ++i) {
        aLCD->write(i);
    }
    // Print some interesting characters
    aLCD->write(0xA1);
    aLCD->write(0xA5);
    aLCD->write(0xB0);
    aLCD->write(0xDB);
    aLCD->write(0xDF);

    aLCD->setCursor(0, 1);
    // The characters 0x10 to 0x1F seem to be all blanks => ROM Code: A00
    for (uint_fast8_t i = 0x10; i < 0x20; ++i) {
        aLCD->write(i);
    }
    aLCD->setCursor(0, 2);
    // The characters 0x80 to 0x8F seem to be all blanks => ROM Code: A00
    for (uint_fast8_t i = 0x80; i < 0x90; ++i) {
        aLCD->write(i);
    }
    aLCD->setCursor(0, 3);
    // The characters 0x90 to 0x9F seem to be all blanks => ROM Code: A00
    for (uint_fast8_t i = 0x90; i < 0xA0; ++i) {
        aLCD->write(i);
    }
    delay(2000);
}

#if defined(USE_PARALLEL_LCD)
void showCustomCharacters(LiquidCrystal *aLCD)
#else
void showCustomCharacters(LiquidCrystal_I2C *aLCD)
#endif
        {
    aLCD->setCursor(0, 0);
    for (uint_fast8_t i = 0; i < 0x08; ++i) {
        aLCD->write(i);
    }
}

#if defined(USE_PARALLEL_LCD)
/*
 * Print all fonts, used in screenshots, using one object
 */
void testBigNumbers(LiquidCrystal *aLCD)
#else
void testBigNumbers(LiquidCrystal_I2C *aLCD)
#endif
        {
    /*
     * 1 X 2
     */
    aLCD->clear(); // Clear display
    // Allocate object
    LCDBigNumbers bigNumberLCD(aLCD, BIG_NUMBERS_FONT_1_COLUMN_2_ROWS_VARIANT_1);
    bigNumberLCD.begin(); // Generate font symbols in LCD controller
    bigNumberLCD.print(F("0123456789 -.:")); // no special space required, we have an 1 column font

    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0, 2);
    bigNumberLCD.print(F("-42.38"));
    delay(DEFAULT_TEST_DELAY);

    /*
     * 2 X 2
     */
    aLCD->clear(); // Clear display
    // Reconfigure existing object to hold another font
    bigNumberLCD.init(BIG_NUMBERS_FONT_2_COLUMN_2_ROWS_VARIANT_1);
    bigNumberLCD.begin(); // Generate font symbols in LCD controller
    bigNumberLCD.print(F("01234"));
#if LCD_ROWS <= 2
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("56789"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-.: "));
#else
    bigNumberLCD.setBigNumberCursor(0, 2);
    bigNumberLCD.print(F("56789" ONE_COLUMN_SPACE_STRING "-.:"));
#endif
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-42.38"));
    delay(DEFAULT_TEST_DELAY);

    /*
     * 3 X 2 1. variant
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init(BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_1);
    bigNumberLCD.begin();

#if LCD_ROWS <= 2
    bigNumberLCD.print(F("0123"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("4567"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("89" ONE_COLUMN_SPACE_STRING "-.: "));
#else
    bigNumberLCD.print(F("01234"));
    bigNumberLCD.setBigNumberCursor(0, 2);
    bigNumberLCD.print(F("56789"));
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-42.38:"));
#endif

    delay(DEFAULT_TEST_DELAY);

    /*
     * 3 X 2 2. variant
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init( BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_2);
    bigNumberLCD.begin();
    bigNumberLCD.print(F("01234"));
#if LCD_ROWS <= 2
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("56789"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-.: "));
#else
    bigNumberLCD.setBigNumberCursor(0, 2);
    bigNumberLCD.print(F("56789")); // we have a space between this characters, i.e. forceGapBetweenNumbers is true
#endif
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-42.38:"));
    delay(DEFAULT_TEST_DELAY);

    /*
     * 3 X 2 3. variant
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init( BIG_NUMBERS_FONT_3_COLUMN_2_ROWS_VARIANT_3);
    bigNumberLCD.begin();
    bigNumberLCD.print(F("01234"));
#if LCD_ROWS <= 2
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("56789"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-.: "));
#else
    bigNumberLCD.setBigNumberCursor(0, 2);
    bigNumberLCD.print(F("56789" ONE_COLUMN_SPACE_STRING "-.:"));
#endif
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-42.38"));
    delay(DEFAULT_TEST_DELAY);

#if LCD_ROWS > 2
    /****************
     * 3 line numbers
     ****************/
    /*
     * 2 X 3 Space above
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init(BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_1);
    bigNumberLCD.begin();
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("01234"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("56789" ONE_COLUMN_SPACE_STRING "-.:"));
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("-42.38"));
    delay(DEFAULT_TEST_DELAY);

    /*
     * 2 X 3 Space below
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init( BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_2);
    bigNumberLCD.begin();
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("01234"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("56789" ONE_COLUMN_SPACE_STRING "-.:"));
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("-42.38"));
    delay(DEFAULT_TEST_DELAY);

    /*
     * 3 X 3 Space below
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init(BIG_NUMBERS_FONT_3_COLUMN_3_ROWS_VARIANT_1);
    bigNumberLCD.begin();
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("01234"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("56789"));
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0, 1);
    bigNumberLCD.print(F("-42.38:"));
    delay(DEFAULT_TEST_DELAY);

    /****************
     * 4 line numbers
     ****************/
    /*
     * 3 X 4
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init( BIG_NUMBERS_FONT_3_COLUMN_4_ROWS_VARIANT_1);
    bigNumberLCD.begin();
    bigNumberLCD.print(F("01234"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("56789"));
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-42.38:"));
    delay(DEFAULT_TEST_DELAY);

    /*
     * Variant 2
     */
    aLCD->clear(); // Clear display
    bigNumberLCD.init( BIG_NUMBERS_FONT_3_COLUMN_4_ROWS_VARIANT_2);
    bigNumberLCD.begin();
    bigNumberLCD.print(F("01234"));
    delay(DEFAULT_TEST_DELAY);
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("56789"));
    delay(DEFAULT_TEST_DELAY);

    aLCD->clear(); // Clear display
    // Print "-42.38 :"
    bigNumberLCD.setBigNumberCursor(0);
    bigNumberLCD.print(F("-42.38:"));
    delay(DEFAULT_TEST_DELAY);
#endif // LCD_ROWS > 2
}

#if defined(LOCAL_DEBUG)
#undef LOCAL_DEBUG
#endif
#endif // _LCD_BIG_NUMBERS_HPP
