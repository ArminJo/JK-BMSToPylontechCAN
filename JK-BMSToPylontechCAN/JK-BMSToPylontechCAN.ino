/*
 *  JK-BMSToPylontechCAN.cpp
 *
 *  Converts the JK-BMS RS485 data to Pylontech CAN data for inverters
 *  which are not compatible with JK-BMS protocol but with Pylontech protocol, like Deye inverters.
 *  It displays many BMS information and alarms on a locally attached 2004 LCD.
 *  The JK-BMS data are provided as RS232 TTL.
 *
 *  The software TX and hardware RX lines are connected to the JK BMS and run with 115200 baud.
 *  CAN is connected to the inverter which must accept Pylontech low voltage protocol, which runs with 500 kBit/s.
 *  This protocol is used e.g by the Pylontech US2000 battery.
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  !!! 8 MHz crystal and 500 kBit/s does not work with MCP2515                 !!!
 *  !!! So you must replace the crystal of the module with a 16 (or 20) MHz one !!!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *  Internal operation (every n seconds):
 *  1. A request to deliver all informations is sent to the BMS.
 *  2. The BMS reply frame is stored in a buffer and parity and other plausi checks are made.
 *  3. The cell data are converted and enhanced to fill the JKConvertedCellInfoStruct.
 *     Other frame data are mapped to a C structure.
 *     But all words and longs in this structure are filled with big endian and thus cannot be read directly but must be swapped on reading.
 *  4. Some other frame data are converted and enhanced to fill the JKComputedDataStruct.
 *  5. The content of the result frame is printed. After reset, all info is printed once, then only dynamic info is printed.
 *  6. The required CAN data is filled in the according PylontechCANFrameInfoStruct.
 *  7. Dynamic data and errors are displayed on the optional 2004 LCD if attached.
 *  8. CAN data is sent.
 *
 *  The LCD has 3 "pages" showing overview data, up to 16 cell voltages, or SOC and current with big numbers.
 *  The pages can be switched by the button at pin 2.
 *
 *  On timeout, the last BMS data is kept.
 *
 *  It uses the following libraries, which are included in this repository:
 *  SoftwareSerialTX for sending Serial to JK-BMS
 *  Modified LiquidCrystal_I2C for I2C connected LCD
 *  LCDBigNumbers for LCD big number generation
 *  EasyButtonAtInt01 for LCD page switching button
 *  SoftI2CMaster for minimal I2C functions
 *  modified mcp_can_dfs.h file from Seed-Studio Seeed_Arduino_CAN
 *
 *  Based on https://github.com/syssi/esphome-jk-bms and https://github.com/maxx-ukoo/jk-bms2pylontech
 *
 *  Copyright (C) 2023  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ArduinoUtils https://github.com/ArminJo/PVUtils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
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
 * UART-TTL
 *  __________                  _________            _________             _________
 * |          |<----- RX ----->|         |<-- SPI ->|         |           |         |
 * |  JK-BMS  |<----- TX ----->|  UNO/   |          | MCP2515 |           |         |
 * |          |                |  NANO   |<-- 5V -->|   CAN   |<-- CAN -->|  DEYE   |
 * |          |<----- GND ---->|         |<-- GND-->|         |           |         |
 * |__________|                |_________|          |_________|           |_________|
 *
 * # UART-TTL socket (4 Pin, JST 1.25mm pitch)
 *  ___ ________ ___
 * |                |
 * | O   O   O   O  |
 * |GND  RX  TX VBAT|
 * |________________|
 *   |   |   |
 *   |   |   ----- RX
 *   |   --------- D4 (or other)
 *   --------------GND
 */

#include <Arduino.h>

#if !defined(LOCAL_DEBUG)
//#define LOCAL_DEBUG
#endif

#define VERSION_EXAMPLE "1.0"

#define BUZZER_PIN                  A2 // To signal errors

// Debug stuff
#define DEBUG_PIN                   8 // If low, print additional info
bool sDebugModeActive = false;

/*
 * Program timing
 */
#if defined(LOCAL_DEBUG)
#define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     5000
#define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             5000
#else
#define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     2000
#define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             2000
#endif
#define TIMEOUT_MILLIS_FOR_FRAME_REPLY                  100 // I measured 15 ms between request end and end of received 273 byte result
#if TIMEOUT_MILLIS_FOR_FRAME_REPLY > MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS
#error "TIMEOUT_MILLIS_FOR_FRAME_REPLY must be smaller than MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS to detect timeouts"
#endif
bool sStaticInfoWasSent = false; // Flag to send static Info only once after reset.

/*
 * JK-BMS stuff
 */
#define MAXIMUM_NUMBER_OF_CELLS     24 // must be before #include "JK-BMS.h"
#include "JK-BMS.h"

#include "SoftwareSerialTX.h"
SoftwareSerialTX TxToJKBMS(4);          // Use a 115200 baud software serial for the short request frame
bool sFrameIsRequested = false;         // If true, request was recently sent so now check for serial input
uint32_t sMillisOfLastRequestedJKDataFrame = -MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS; // Initial value to start first request immediately
uint32_t sMillisOfLastReceivedByte = 0; // For timeout
bool sFrameHasTimeout = false;          // If true BMS is likely switched off.
uint16_t sFrameTimeoutCounter = 0;

/*
 * CAN stuff
 */
#include "Pylontech_CAN.h" // Must be before #include "MCP2515_TX.hpp"
//#define CRYSTAL_20MHZ_ASSEMBLED  // Otherwise a 16 MHz crystal is assumed. Must be specified before #include "MCP2515_TX.hpp"
// Pin 9 is the default pin for the Arduino CAN bus shield. Alternately you can use pin 10 on this shield.
//#define SPI_CS_PIN                  10 // Otherwise pin 9 is assumed. Must be specified before #include "MCP2515_TX.hpp"
#include "MCP2515_TX.hpp" // my reduced tx only driver
bool sCanDataIsInitialized = false;
uint32_t sMillisOfLastCANFrameSent = 0; // For CAN timing

/*
 * LCD stuff
 */
#define LCD_COLUMNS     20
#define LCD_I2C_ADDRESS 0x27            // Default LCD address is 0x27 for a 20 chars and 4 line / 2004 display
#include "LiquidCrystal_I2C.hpp" // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use only the modified version delivered with this program!
LiquidCrystal_I2C myLCD(LCD_I2C_ADDRESS, LCD_COLUMNS, 4);
bool sSerialLCDAvailable;
/*
 * Big numbers for LCD JK_BMS_PAGE_BIG_INFO page
 */
#define USE_SERIAL_2004_LCD             // required by LCDBigNumbers.hpp
#include "LCDBigNumbers.hpp"            // Include sources for LCD big number generation
LCDBigNumbers bigNumberLCD(&myLCD, BIG_NUMBERS_FONT_2_COLUMN_3_ROWS_VARIANT_1); // Use 2x3 numbers, 2. variant
const uint8_t bigNumbersTopBlock[8] PROGMEM = { 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };     // char 1: top block
const uint8_t bigNumbersBottomBlock[8] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F };     // char 2: bottom block
/*
 * Button for switching LCD pages
 */
#define USE_BUTTON_0  // Enable code for 1. button at INT0 / D2
#include "EasyButtonAtInt01.hpp"
EasyButton Button0AtPin2; // Only 1. button (USE_BUTTON_0) enabled -> button is connected to INT0
#define JK_BMS_PAGE_OVERVIEW    0 // is displayed in case of error
#define JK_BMS_PAGE_CELL_INFO   1
#define JK_BMS_PAGE_BIG_INFO    2
#define JK_BMS_PAGE_CAN_INFO    3 // Only if debug is enabled
#define JK_BMS_PAGE_MAX         JK_BMS_PAGE_BIG_INFO
#define JK_BMS_PAGE_MAX_DEBUG   JK_BMS_PAGE_CAN_INFO
//uint8_t sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW; // Start with Overview page
uint8_t sDisplayPageNumber = JK_BMS_PAGE_BIG_INFO; // Start with Big Info page

void printBMSDataOnLCD();
void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint);
void LCDClearLine(uint8_t aLineNumber);

void processReceivedData();

//#define STANDALONE_TEST
#if defined(STANDALONE_TEST) //
uint8_t TestJKReplyStatusFrame[] PROGMEM = { /* Header*/0x4E, 0x57, 0x01, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01,
/*Cell voltages*/0x79, 0x3C, 0x01, 0x0E, 0xED, 0x02, 0x0E, 0xFA, 0x03, 0x0E, 0xF7, 0x04, 0x0E, 0xEC, 0x05, 0x0E, 0xF8, 0x06, 0x0E,
        0xFA, 0x07, 0x0E, 0xF1, 0x08, 0x0E, 0xF8, 0x09, 0x0E, 0xE3, 0x0A, 0x0E, 0xFA, 0x0B, 0x0E, 0xF1, 0x0C, 0x0E, 0xFB, 0x0D,
        0x0E, 0xFB, 0x0E, 0x0E, 0xF2, 0x0F, 0x0E, 0xF2, 0x10, 0x0E, 0xF2, 0x11, 0x0E, 0xF2, 0x12, 0x0E, 0xF2, 0x13, 0x0E, 0xF2,
        0x14, 0x0E, 0xF2,
        /*JKFrameAllDataStruct*/0x80, 0x00, 0x16, 0x81, 0x00, 0x15, 0x82, 0x00, 0x15, 0x83, 0x0F, 0xF8, 0x84, 0x80, 0xD0, 0x85,
        0x0F, 0x86, 0x02, 0x87, 0x00, 0x04, 0x89, 0x00, 0x00, 0x01, 0xE0, 0x8A, 0x00, 0x0E, 0x8B, 0x00, 0x00, 0x8C, 0x00, 0x07,
        0x8E, 0x16, 0x26, 0x8F, 0x10, 0xAE, 0x90, 0x0F, 0xD2, 0x91, 0x0F, 0xA0, 0x92, 0x00, 0x05, 0x93, 0x0B, 0xEA, 0x94, 0x0C,
        0x1C, 0x95, 0x00, 0x05, 0x96, 0x01, 0x2C, 0x97, 0x00, 0x07, 0x98, 0x00, 0x03, 0x99, 0x00, 0x05, 0x9A, 0x00, 0x05, 0x9B,
        0x0C, 0xE4, 0x9C, 0x00, 0x08, 0x9D, 0x01, 0x9E, 0x00, 0x5A, 0x9F, 0x00, 0x46, 0xA0, 0x00, 0x64, 0xA1, 0x00, 0x64, 0xA2,
        0x00, 0x14, 0xA3, 0x00, 0x46, 0xA4, 0x00, 0x46, 0xA5, 0xFF, 0xEC, 0xA6, 0xFF, 0xF6, 0xA7, 0xFF, 0xEC, 0xA8, 0xFF, 0xF6,
        0xA9, 0x0E, 0xAA, 0x00, 0x00, 0x00, 0x0E, 0xAB, 0x01, 0xAC, 0x01, 0xAD, 0x04, 0x11, 0xAE, 0x01, 0xAF, 0x01, 0xB0, 0x00,
        0x0A, 0xB1, 0x14, 0xB2, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x00, 0x00, 0x00, 0x00, 0xB3, 0x00, 0xB4, 0x49, 0x6E, 0x70,
        0x75, 0x74, 0x20, 0x55, 0x73, 0xB5, 0x32, 0x31, 0x30, 0x31, 0xB6, 0x00, 0x00, 0xE2, 0x00, 0xB7, 0x31, 0x31, 0x2E, 0x58,
        0x57, 0x5F, 0x53, 0x31, 0x31, 0x2E, 0x32, 0x36, 0x5F, 0x5F, 0x5F, 0xB8, 0x00, 0xB9, 0x00, 0x00, 0x04, 0x00, 0xBA, 0x49,
        0x6E, 0x70, 0x75, 0x74, 0x20, 0x55, 0x73, 0x65, 0x72, 0x64, 0x61, 0x4A, 0x4B, 0x5F, 0x42, 0x32, 0x41, 0x32, 0x30, 0x53,
        0x32, 0x30, 0x50, 0xC0, 0x01,
        /*End*/0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x51, 0xC2 };
#endif

/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

void setup() {
// LED_BUILTIN pin is used as SPI Clock !!!
//    pinMode(LED_BUILTIN, OUTPUT);
//    digitalWrite(LED_BUILTIN, LOW);

    pinMode(DEBUG_PIN, INPUT_PULLUP);

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

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
        myLCD.backlight();
        myLCD.setCursor(0, 0);
        myLCD.print(F("JK-BMS to CAN conv."));
        myLCD.setCursor(0, 1);
        myLCD.print(F(VERSION_EXAMPLE " " __DATE__));
        bigNumberLCD.begin(); // This creates the custom character used for printing big numbers

    } else {
        Serial.println(F("No I2C LCD connected at address " STR(LCD_I2C_ADDRESS)));
    }

    /*
     * 115200 baud soft serial to JK-BMS. For serial from BMS we use the hardware Serial RX.
     */
    TxToJKBMS.begin(115200);
    Serial.println(F("Serial to JK-BMS started with 115200 bit/s!"));
    if (sSerialLCDAvailable) {
        myLCD.setCursor(0, 2);
        myLCD.print(F("BMS serial started"));
    }

    /*
     * CAN initialization
     */
//    if (CAN.begin(500000)) { // Resets the device and start the CAN bus at 500 kbps
    byte tCanInitResult = initializeCAN();
    if (tCanInitResult == MCP2515_RETURN_OK) { // Resets the device and start the CAN bus at 500 kbps
        Serial.println(F("CAN started with 500 kbit/s!"));
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("CAN started"));
        }
    } else {
        Serial.print(F("Starting CAN failed with result="));
        Serial.println(tCanInitResult);
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("Starting CAN failed!"));
        }
        while (1)
            ;
    }

    /*
     * Print debug pin info
     */
    Serial.println(F("If you connect debug pin " STR(DEBUG_BUTTON_PIN) " to ground, additional debug data is printed"));
    Serial.println();

    if (sSerialLCDAvailable) {
        delay(1000); // To see the date
        myLCD.setCursor(0, 1);
        myLCD.print(F("Debug pin = " STR(DEBUG_PIN) "  "));

        delay(2000); // To see the messages
        myLCD.clear();
    }
}

void loop() {

    sDebugModeActive = !digitalRead(DEBUG_PIN);
    if (sSerialLCDAvailable) {

        /*
         * Manually check button state change
         */
        if (sFrameTimeoutCounter == 0 && Button0AtPin2.ButtonStateHasJustChanged) {
// reset flag in order to do call digitalWrite() only once per button press
            Button0AtPin2.ButtonStateHasJustChanged = false;
            if (Button0AtPin2.ButtonStateIsActive) {
                sDisplayPageNumber++;

                if (sDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
                    // Create symbols character for maximum and minimum
                    bigNumberLCD._createChar(1, bigNumbersTopBlock);
                    bigNumberLCD._createChar(2, bigNumbersBottomBlock);

                } else if (sDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {
                    // Creates custom character used for generating big numbers
                    bigNumberLCD.begin();

                } else if ((sDebugModeActive && sDisplayPageNumber > JK_BMS_PAGE_MAX_DEBUG)
                        || (!sDebugModeActive && sDisplayPageNumber > JK_BMS_PAGE_MAX)) {
                    // wrap around
                    sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
                    sForcePrintUpTime = true; // force printing uptime
                }

                Serial.print(F("Set LCD display page to: "));
                Serial.println(sDisplayPageNumber);
                myLCD.clear();
            }
        }
    }
    /*
     * Request status frame every n seconds
     */
    if (millis() - sMillisOfLastRequestedJKDataFrame >= MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
        sMillisOfLastRequestedJKDataFrame = millis();
        /*
         * Flush input buffer and send request to JK-BMS
         */
        while (Serial.available()) {
            Serial.read();
        }
        requestJK_BMSStatusFrame(&TxToJKBMS, sDebugModeActive);
        sFrameIsRequested = true; // enable check for serial input
        initJKReplyFrameBuffer();
        sMillisOfLastReceivedByte = millis(); // initialize reply timeout
    }

    if (sFrameIsRequested) {
        if (Serial.available()) {
            sMillisOfLastReceivedByte = millis();

            uint8_t tReceiveResultCode = readJK_BMSStatusFrameByte();
            if (tReceiveResultCode != JK_BMS_RECEIVE_OK) {
                if (tReceiveResultCode == JK_BMS_RECEIVE_FINISHED) {
                    /*
                     * All JK-BMS frame data received
                     */
                    sFrameIsRequested = false;
                    if (sDebugModeActive) {
                        Serial.println();
                        Serial.print(sReplyFrameBufferIndex + 1);
                        Serial.println(F(" bytes received"));
                        printJKReplyFrameBuffer();
                        Serial.println();
                    }
                    sFrameHasTimeout = false;
                    sFrameTimeoutCounter = 0;
                    sForcePrintUpTime = true; // to force printing on LCD
                    processReceivedData();
                } else {
                    /*
                     * Error here
                     */
                    Serial.print(F("Receive error="));
                    Serial.print(tReceiveResultCode);
                    Serial.print(F(" at index"));
                    Serial.println(sReplyFrameBufferIndex);

                    sFrameIsRequested = false;
                    printJKReplyFrameBuffer();
                }
            }

        } else if (millis() - sMillisOfLastReceivedByte >= TIMEOUT_MILLIS_FOR_FRAME_REPLY) {
            /*
             * Timeout for one byte
             * If complete timeout, print it only once
             */
            if (sReplyFrameBufferIndex != 0 || sFrameTimeoutCounter == 0 || sDebugModeActive) {
                Serial.print(F("Receive timeout at sReplyFrameBufferIndex="));
                Serial.println(sReplyFrameBufferIndex);
                if (sSerialLCDAvailable) {
                    myLCD.clear();
                    myLCD.setCursor(0, 0);
                    myLCD.print(F("Receive timeout"));
                }
            }

            if (sSerialLCDAvailable) {
                myLCD.setCursor(16, 0);
                myLCD.print(sFrameTimeoutCounter);
            }

            sFrameHasTimeout = true;
            sFrameTimeoutCounter++;

            sFrameIsRequested = false; // do not try to receive more

#if defined(STANDALONE_TEST)
            /*
             * Copy test data to receive buffer, if no data was received previously
             */
            if (!sCanDataIsInitialized) {
                Serial.println(F("Use fixed demo data"));
                memcpy_P(JKReplyFrameBuffer, TestJKReplyStatusFrame, sizeof(TestJKReplyStatusFrame));
                sReplyFrameBufferIndex = sizeof(TestJKReplyStatusFrame) - 1;
                printJKReplyFrameBuffer();
                Serial.println();
                processReceivedData();
            }
#endif
        }
    }

    /*
     * Send CAN frame independently of the period of JK-BMS data requests
     * 0.5 MB/s
     * Inverter reply every second: 0x305: 00-00-00-00-00-00-00-00
     */
    if (sCanDataIsInitialized && millis() - sMillisOfLastCANFrameSent >= MILLISECONDS_BETWEEN_CAN_FRAME_SEND) {
        sMillisOfLastCANFrameSent = millis();
        if (sDebugModeActive) {
            Serial.println(F("Send CAN"));
        }
        sendPylontechAllCANFrames(sDebugModeActive);

        /*
         * Signaling errors
         */
        if (ErrorStringForLCD != NULL) {
            tone(BUZZER_PIN, 2200, 50);
            delay(200);
            tone(BUZZER_PIN, 2200, 50);
        }
    }

}

void processReceivedData() {
    /*
     * Set the static pointer to the start of the reply data which depends on the number of cell voltage entries
     * The JKFrameAllDataStruct starts behind the header + cell data header 0x79 + CellInfoSize + the variable length cell data (CellInfoSize is contained in JKReplyFrameBuffer[12])
     */
    sJKFAllReplyPointer = reinterpret_cast<JKReplyStruct*>(&JKReplyFrameBuffer[JK_BMS_FRAME_HEADER_LENGTH + 2
            + JKReplyFrameBuffer[JK_BMS_FRAME_INDEX_OF_CELL_INFO_LENGTH]]);

    fillJKConvertedCellInfo();
    fillJKComputedData();

    if (!sStaticInfoWasSent) {
        sStaticInfoWasSent = true;
        printJKStaticInfo();
    }
    printJKDynamicInfo();
    Serial.println();
    fillAllCANData(sJKFAllReplyPointer);
    sCanDataIsInitialized = true;
    if (sSerialLCDAvailable) {
        printBMSDataOnLCD();
    }
}
/*
 * Print selected data on a 2004 LCD display
 */
void printBMSDataOnLCD() {

    myLCD.setCursor(0, 0);
    if (ErrorStringForLCD != NULL) {
        // print not more than 20 characters
        char t20CharacterString[LCD_COLUMNS + 1];
        memcpy_P(t20CharacterString, ErrorStringForLCD, LCD_COLUMNS);
        t20CharacterString[LCD_COLUMNS] = '\0';
        // Allow error flags to be seen on page CAN info
        if (sDisplayPageNumber != JK_BMS_PAGE_CAN_INFO) {
            sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
            myLCD.print(t20CharacterString);
        }
    }

    if (sDisplayPageNumber == JK_BMS_PAGE_OVERVIEW) {
        /*
         * Top line 0 - Error message or up time, which is only printed if changed
         */
        if (ErrorStringForLCD == NULL && sForcePrintUpTime) {
            sForcePrintUpTime = false;
            myLCD.print(F("Uptime:  "));
            myLCD.print(&sUpTimeString[4]);
        }

        /*
         * Line 1 - SOC and remaining capacity and state of MosFets or Error
         */
        myLCD.setCursor(0, 1);
// Percent of charge
        myLCD.print(sJKFAllReplyPointer->SOCPercent);
        myLCD.print(F("% "));

// Remaining capacity
        myLCD.print(JKComputedData.RemainingCapacityAmpereHour);
        myLCD.print(F("Ah  "));

        if (sJKFAllReplyPointer->StatusUnion.StatusBits.ChargeMosFetActive) {
            myLCD.print(F("CH "));
        } else {
            myLCD.print(F("   "));
        }

        if (sJKFAllReplyPointer->StatusUnion.StatusBits.DischargeMosFetActive) {
            myLCD.print(F("-CH "));
        } else {
            myLCD.print(F("    "));
        }

        if (sJKFAllReplyPointer->StatusUnion.StatusBits.BalancerActive) {
            myLCD.print(F("BAL"));
        } else {
            myLCD.print(F("   "));
        }

        /*
         * Line 2 - Voltage, Current and Power
         */
        myLCD.setCursor(0, 2);
// Voltage
        uint_fast8_t tCharactersPrinted = myLCD.print(JKComputedData.BatteryVoltageFloat, 2);
        myLCD.print(F("V "));

// Current
        tCharactersPrinted += myLCD.print(JKComputedData.BatteryLoadCurrentFloat, 2);
        myLCD.print(F("A "));

// Power
        tCharactersPrinted += myLCD.print(JKComputedData.BatteryLoadPower);
        myLCD.print('W');
        LCDPrintSpaces((LCD_COLUMNS - 5) - tCharactersPrinted);

        /*
         * Line 3 - 3 Temperatures
         */
        myLCD.setCursor(0, 3);
        // 3 temperatures
        myLCD.print(JKComputedData.TemperaturePowerMosFet);
        myLCD.print(F("\xDF" "C "));
        myLCD.print(JKComputedData.TemperatureSensor1);
        myLCD.print(F("\xDF" "C "));
        myLCD.print(JKComputedData.TemperatureSensor2);
        myLCD.print(F("\xDF" "C "));

    } else if (sDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
        uint_fast8_t tRowNumber;
        if (JKConvertedCellInfo.NumberOfCellInfoEnties > 12) {
            tRowNumber = 0;
        } else {
            myLCD.print(F("    -CELL INFO-"));
            tRowNumber = 1;
        }
        for (uint8_t i = 0; i < JKConvertedCellInfo.NumberOfCellInfoEnties; ++i) {
            if (i % 4 == 0) {
                myLCD.setCursor(0, tRowNumber);
                tRowNumber++;
            }
            if (i == JKConvertedCellInfo.MaximumVoltagCellIndex) {
                myLCD.print((char) 0x1);
            } else if (i == JKConvertedCellInfo.MinimumVoltagCellIndex) {
                myLCD.print((char) 0x2);
            } else {
                myLCD.print(' ');
            }
            myLCD.print(JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt);

        }
    } else if (sDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {

        bigNumberLCD.setBigNumberCursor(0, 0);
        bigNumberLCD.print(sJKFAllReplyPointer->SOCPercent);
        myLCD.setCursor(6, 2);
        myLCD.print('%');

        if (JKComputedData.BatteryLoadCurrentFloat < 0.0) {
            // position the '-' with a space before the number
            bigNumberLCD.writeAt('-', 7, 0);
            bigNumberLCD.setBigNumberCursor(9, 0);
            if (JKComputedData.BatteryLoadCurrentFloat < -10.0) {
                bigNumberLCD.print(-JKComputedData.BatteryLoadCurrentFloat, 1);
            } else {
                bigNumberLCD.print(-JKComputedData.BatteryLoadCurrentFloat, 2);
            }

        } else {
            bigNumberLCD.setBigNumberCursor(9, 0);
            if (JKComputedData.BatteryLoadCurrentFloat > 10.0) {
                bigNumberLCD.print(JKComputedData.BatteryLoadCurrentFloat, 1);
            } else {
                bigNumberLCD.print(JKComputedData.BatteryLoadCurrentFloat, 2);
            }
        }
        myLCD.setCursor(19, 2);
        myLCD.print('A');

    } else {
        /*
         * sDisplayPageNumber == JK_BMS_PAGE_CAN_INFO
         */
        PylontechCANFrameStruct *tCANFrameDataPointer =
                reinterpret_cast<struct PylontechCANFrameStruct*>(&PylontechCANErrorsWarningsFrame);
        if (tCANFrameDataPointer->FrameData.ULong.LowLong == 0) {
            /*
             * Caption and "No error" in row 1 and 2
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
             * Errors and Warnings in row 1 and 2
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
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Voltage100Millivolt / 10);
        myLCD.print('.');
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Voltage100Millivolt % 10);
        myLCD.print(F("V "));

        // Current
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Current100Milliampere / 10);
        myLCD.print('.');
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Current100Milliampere % 10);
        myLCD.print(F("A "));

        // Current
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Temperature100Millicelsius / 10);
        myLCD.print('.');
        myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Temperature100Millicelsius % 10);
        myLCD.print(F("\xDF" "C "));

        /*
         * Request flags in row 4
         */
        myLCD.setCursor(0, 3);
        // Charge enable
        if (PylontechCANBatteryRequesFrame.FrameData.ChargeEnable) {
            myLCD.print(F("CH "));
        } else {
            myLCD.print(F("   "));
        }
        if (PylontechCANBatteryRequesFrame.FrameData.DischargeEnable) {
            myLCD.print(F("-CH "));
        } else {
            myLCD.print(F("    "));
        }
        if (PylontechCANBatteryRequesFrame.FrameData.FullChargeRequest) {
            myLCD.print(F("FULL "));
        } else {
            myLCD.print(F("     "));
        }
        if (PylontechCANBatteryRequesFrame.FrameData.ForceChargeRequestI
                || PylontechCANBatteryRequesFrame.FrameData.ForceChargeRequestII) {
            myLCD.print(F("FORCE "));
        } else {
            myLCD.print(F("      "));
        }

    }
}

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
