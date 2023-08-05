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
 *  1. A request to deliver all informations is sent to the BMS (1.85 ms).
 *  2. Wait and receive the The BMS reply frame (0.18 to 1 ms + 25.5 ms).
 *  3. The BMS reply frame is stored in a buffer and parity and other plausi checks are made.
 *  4. The cell data are converted and enhanced to fill the JKConvertedCellInfoStruct.
 *     Other frame data are mapped to a C structure.
 *     But all words and longs in this structure are filled with big endian and thus cannot be read directly but must be swapped on reading.
 *  5. Other frame data are converted and enhanced to fill the JKComputedDataStruct.
 *  6. The content of the result frame is printed. After reset, all info is printed once, then only dynamic info is printed.
 *  7. The required CAN data is filled in the according PylontechCANFrameInfoStruct.
 *  8. Dynamic data and errors are displayed on the optional 2004 LCD if attached.
 *  9. CAN data is sent..
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
 *  Tested with SUN-5K-SG03LP1-EU
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
 *
 * # Connection schematic
 * A schottky diode is inserted into the RX line to allow programming the AVR with the JK-BMS still connected.
 *
 *  __________ Schottky diode  _________            _________             _________
 * |        TX|----|<|-- RX ->|RX       |<-- SPI ->|         |           |         |
 * |        RX|<-------- TX --|4  Uno/  |          | MCP2515 |           |         |
 * |  JK-BMS  |               |   Nano  |<-- 5V -->|   CAN   |<-- CAN -->|  DEYE   |
 * |          |<------- GND ->|         |<-- GND-->|         |           |         |
 * |__________|               |_________|          |_________|           |_________|
 *
 * # UART-TTL socket (4 Pin, JST 1.25mm pitch)
 *  ___ ________ ___
 * |                |
 * | O   O   O   O  |
 * |GND  RX  TX VBAT|
 * |________________|
 *   |   |   |
 *   |   |   ----- RX of Uno / Nano
 *   |   --------- D4 (or other)
 *   --------------GND
 */

/*
 * Ideas:
 * Balancing time per day / week / month etc.
 * Maximum, minimum cell while balancing
 */
#include <Arduino.h>

#define VERSION_EXAMPLE "1.2"

#if !defined(LOCAL_DEBUG)
//#define LOCAL_DEBUG
#endif

/*
 * Pin layout, may be adapted to your requirements
 */
#define BUZZER_PIN                 A2 // To signal errors
#define BUTTON_PIN                  2
// The standard RX of the Arduino is used for the JK_BMS connection.
#define JK_BMS_RX_PIN               0 // We use the Serial RX pin. Not used in program, only for documentation
#define JK_BMS_TX_PIN               4
/*
 * The SPI pins for connection to CAN converter and the I2C / TWI pins for the LCD are determined by hardware.
 * For Uno / Nano:
 *   SPI: MOSI - 11, MISO - 12, SCK - 13.
 *   I2C: SDA - A4, SCL - A5.
 */
#define SPI_CS_PIN                  9 // Pin 9 is the default pin for the Arduino CAN bus shield. Alternately you can use pin 10 on this shield.
//#define SPI_CS_PIN                 10 // Must be specified before #include "MCP2515_TX.hpp"

#define DEBUG_PIN                   8 // If low, print additional info

/*
 * Program timing, may be adapted to your requirements
 */
#if defined(LOCAL_DEBUG)
#define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     5000
#define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             5000
#define SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS          "5" // Only for display on LCD
#define SECONDS_BETWEEN_CAN_FRAME_SEND                  "5" // Only for display on LCD
#else
#define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     2000
#define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             2000
#define SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS          "2" // Only for display on LCD
#define SECONDS_BETWEEN_CAN_FRAME_SEND                  "2" // Only for display on LCD
#endif

/*
 * Display timeouts, may be adapted to your requirements
 */
//#define DISPLAY_ALWAYS_ON   // Activate this, if you want the display to be always on
#define DISPLAY_ON_TIME_STRING              "5 min" // 5 minutes. L to avoid overflow at macro processing
#define DISPLAY_ON_TIME_SECONDS             300L // 5 minutes. L to avoid overflow at macro processing
#define DISPLAY_ON_TIME_SECONDS             300L // 5 minutes. L to avoid overflow at macro processing
#define DISPLAY_ON_TIME_SECONDS_IF_TIMEOUT  180L // 3 minutes
//#define NO_MULTIPLE_BEEPS_ON_TIMEOUT           // Activate it if you do not want beeps for 1 minute
#define BEEP_ON_TIME_SECONDS_IF_TIMEOUT      60L // 1 minute

/*
 * Sleep stuff
 */
#include "AVRUtils.h"
bool sEnableSleep = false;             // If true, we can do a sleep at the end of the loop

/*
 * JK-BMS stuff
 */
#define MAXIMUM_NUMBER_OF_CELLS     24 // must be before #include "JK-BMS.h"
#include "JK-BMS.h"

#include "SoftwareSerialTX.h"
/*
 * Use a 115200 baud software serial for the short request frame.
 * If available, we also can use a second hardware serial here :-).
 */
SoftwareSerialTX TxToJKBMS(JK_BMS_TX_PIN);
bool sFrameIsRequested = false;             // If true, request was recently sent so now check for serial input
uint32_t sMillisOfLastRequestedJKDataFrame = -MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS; // Initial value to start first request immediately
uint32_t sMillisOfLastReceivedByte = 0;     // For timeout
bool sFrameHasTimeout = false;              // If true BMS is likely switched off.

/*
 * CAN stuff
 */
#include "Pylontech_CAN.h" // Must be before #include "MCP2515_TX.hpp"
#define CAN_BAUDRATE    500000  // 500 kB
#if !defined(MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE)
// Must be specified before #include "MCP2515_TX.hpp"
#define MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE  16 // 16 MHz is default for the Arduino CAN bus shield
//#define MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE   8 // 8 MHz is default for the Chinese breakout board. !!! This does not work with 500 kB !!!
#endif
#include "MCP2515_TX.hpp" // my reduced tx only driver
bool sCanDataIsInitialized = false;
uint32_t sMillisOfLastCANFrameSent = 0; // For CAN timing

/*
 * LCD stuff
 */
#define LCD_COLUMNS     20
#define LCD_ROWS         4
#define LCD_I2C_ADDRESS 0x27     // Default LCD address is 0x27 for a 20 chars and 4 line / 2004 display
#include "LiquidCrystal_I2C.hpp" // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use only the modified version delivered with this program!
LiquidCrystal_I2C myLCD(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
bool sSerialLCDAvailable;
bool sSerialLCDIsSwitchedOff;
uint16_t sFrameTimeoutCounterForLCDTAutoOff = 0; // counts frame timeouts, (every 2 seconds)
uint16_t sFrameCounterForLCDTAutoOff = 0;
void printBMSDataOnLCD();
void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint);
void LCDClearLine(uint8_t aLineNumber);

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
 * Button at INT0 / D2 for switching LCD pages
 */
#define USE_BUTTON_0              // Enable code for 1. button at INT0 / D2
#include "EasyButtonAtInt01.hpp"
EasyButton Button0AtPin2;         // Only 1. button (USE_BUTTON_0) enabled -> button is connected to INT0
#define JK_BMS_PAGE_OVERVIEW    0 // is displayed in case of error
#define JK_BMS_PAGE_CELL_INFO   1
#define JK_BMS_PAGE_BIG_INFO    2
#define JK_BMS_PAGE_CAN_INFO    3 // Only if debug is enabled
#define JK_BMS_PAGE_MAX         JK_BMS_PAGE_BIG_INFO
#define JK_BMS_PAGE_MAX_DEBUG   JK_BMS_PAGE_CAN_INFO
#define JK_BMS_START_PAGE       JK_BMS_PAGE_BIG_INFO
//uint8_t sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW; // Start with Overview page
uint8_t sDisplayPageNumber = JK_BMS_START_PAGE; // Start with Big Info page
void checkButtonStateChange();

/*
 * Miscellaneous
 */
#define TIMEOUT_MILLIS_FOR_FRAME_REPLY                  100 // I measured 26 ms between request end and end of received 273 byte result
#if TIMEOUT_MILLIS_FOR_FRAME_REPLY > MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS
#error "TIMEOUT_MILLIS_FOR_FRAME_REPLY must be smaller than MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS to detect timeouts"
#endif
bool sStaticInfoWasSent = false; // Flag to send static Info only once after reset.

bool sDebugModeActive = false;
void processReceivedData();
void printReceivedData();

//#define STANDALONE_TEST
#if defined(STANDALONE_TEST)
//#define LCD_PAGES_TEST
const uint8_t TestJKReplyStatusFrame[] PROGMEM = { /* Header*/0x4E, 0x57, 0x01, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01,
/*Cell voltages*/0x79, 0x3C, 0x01, 0x0E, 0xED, 0x02, 0x0E, 0xFA, 0x03, 0x0E, 0xF7, 0x04, 0x0E, 0xEC, 0x05, 0x0E, 0xF8, 0x06, 0x0E,
        0xFA, 0x07, 0x0E, 0xF1, 0x08, 0x0E, 0xF8, 0x09, 0x0E, 0xD8, 0x0A, 0x0E, 0xFA, 0x0B, 0x0E, 0xF1, 0x0C, 0x0E, 0x0F, 0xA0,
        0x0E, 0xFB, 0x0E, 0x0E, 0xF2, 0x0F, 0x0E, 0xF2, 0x10, 0x0E, 0xF2, 0x11, 0x0E, 0xF2, 0x12, 0x0E, 0xF0, 0x13, 0x0E, 0xF3,
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

void testLCDPages();
void testBigNumbers();
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

    tone(BUZZER_PIN, 2200, 50);

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
        sSerialLCDIsSwitchedOff = false;
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
    if (initializeCAN(CAN_BAUDRATE, MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE, &Serial) == MCP2515_RETURN_OK) { // Resets the device and start the CAN bus at 500 kbps
        Serial.println(F("CAN started with 500 kbit/s!"));
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("CAN started"));
        }
    } else {
        Serial.print(F("Starting CAN failed!"));
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("Starting CAN failed!"));
        }
#if !defined(STANDALONE_TEST)
        while (1)
            ;
#endif
    }

    /*
     * Print debug pin info
     */
    Serial.println(F("If you connect debug pin " STR(DEBUG_BUTTON_PIN) " to ground, additional debug data is printed"));
    Serial.println(F(STR(MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) " ms between 2 BMS requests"));
    Serial.println(F(STR(MILLISECONDS_BETWEEN_CAN_FRAME_SEND) " ms between 2 CAN transmissions"));
    Serial.println();

    if (sSerialLCDAvailable) {
        delay(4000); // To see the date
#if !defined(DISPLAY_ALWAYS_ON)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Screen timeout " DISPLAY_ON_TIME_STRING));
#endif
        myLCD.setCursor(0, 1);
        myLCD.print(F("Debug pin = " STR(DEBUG_PIN) "  "));
        myLCD.setCursor(0, 2);
        myLCD.print(F("Get  BMS every " SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS " s  "));
        myLCD.setCursor(0, 3);
        myLCD.print(F("Send CAN every " SECONDS_BETWEEN_CAN_FRAME_SEND " s  "));
        delay(2000); // To see the messages
        myLCD.clear();
    }
    initSleep(SLEEP_MODE_PWR_SAVE); // The timer 0 (required for millis() is disabled here
    pinMode(8, OUTPUT);
}

void loop() {
    sDebugModeActive = !digitalRead(DEBUG_PIN);

    if (sSerialLCDAvailable) {
        checkButtonStateChange();
    } // if (sSerialLCDAvailable)

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
        requestJK_BMSStatusFrame(&TxToJKBMS, sDebugModeActive); // 1.85 ms
        sFrameIsRequested = true; // enable check for serial input
        initJKReplyFrameBuffer();
        sMillisOfLastReceivedByte = millis(); // initialize reply timeout
    }

    /*
     * Get reply from BMS and check timeout
     */
    if (sFrameIsRequested) {
        if (Serial.available()) {
            sMillisOfLastReceivedByte = millis();

            uint8_t tReceiveResultCode = readJK_BMSStatusFrameByte();
            if (tReceiveResultCode == JK_BMS_RECEIVE_FINISHED) {
                /*
                 * All JK-BMS frame data received
                 */
                sFrameIsRequested = false;
                sEnableSleep = true;
                if (sDebugModeActive) {
                    Serial.println();
                    Serial.print(sReplyFrameBufferIndex + 1);
                    Serial.println(F(" bytes received"));
                    printJKReplyFrameBuffer();
                    Serial.println();
                }

#if !defined(DISPLAY_ALWAYS_ON)
                sFrameCounterForLCDTAutoOff++;
                if (sFrameCounterForLCDTAutoOff == (DISPLAY_ON_TIME_SECONDS * 1000U) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS
                        && !sDebugModeActive) {
                    myLCD.noBacklight(); // switch off backlight after 5 minutes
                    sSerialLCDIsSwitchedOff = true;
                    Serial.println(F("Switch off LCD display, triggered by LCD \"ON\" timeout reached."));
                    sFrameCounterForLCDTAutoOff = 0; // start again to enable switch off after 5 minutes
                }
#endif
                if (sSerialLCDIsSwitchedOff && sFrameTimeoutCounterForLCDTAutoOff != 0) {
                    myLCD.backlight(); // Switch backlight LED on after timeout
                    sSerialLCDIsSwitchedOff = false;
                    Serial.println(F("Switch on LCD display, triggered by successfully receiving a BMS info frame after timeout"));
                }

                sFrameHasTimeout = false;
                sFrameTimeoutCounterForLCDTAutoOff = 0;
                processReceivedData();
                printReceivedData();

            } else if (tReceiveResultCode != JK_BMS_RECEIVE_OK) {
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

        } else if (millis() - sMillisOfLastReceivedByte >= TIMEOUT_MILLIS_FOR_FRAME_REPLY) {
            /*
             * Here we have timeout at one byte
             * If complete timeout, print it only once
             */
            sFrameHasTimeout = true;
            sFrameIsRequested = false; // do not try to receive more
            sEnableSleep = true;

#if !defined(NO_MULTIPLE_BEEPS_ON_TIMEOUT)
            if (sFrameTimeoutCounterForLCDTAutoOff
                    < (BEEP_ON_TIME_SECONDS_IF_TIMEOUT * 1000U) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS || sDebugModeActive) {
                tone(BUZZER_PIN, 2200); // beep at least first minute
                delay(10);
                noTone(BUZZER_PIN); // to avoid Tone interrupts waking us up from sleep
            }
#endif

            if (sReplyFrameBufferIndex != 0 || sFrameTimeoutCounterForLCDTAutoOff == 0 || sDebugModeActive) {
                /*
                 * No byte received, BMS may be off or disconnected
                 */
                Serial.print(F("Receive timeout at sReplyFrameBufferIndex="));
                Serial.println(sReplyFrameBufferIndex);
#if defined(NO_MULTIPLE_BEEPS_ON_TIMEOUT)
                tone(BUZZER_PIN, 2200); // beep once
                delay(10);
                noTone(BUZZER_PIN); // to avoid Tone interrupts waking us up from sleep
#endif
#if !defined(STANDALONE_TEST)
                if (sSerialLCDAvailable) {
                    if (!sSerialLCDIsSwitchedOff) {
                        myLCD.clear();
                        myLCD.setCursor(0, 0);
                        myLCD.print(F("Receive timeout"));
                    }
                }
#endif
            }

            if (sSerialLCDAvailable) {
                if (sFrameTimeoutCounterForLCDTAutoOff
                        == ((DISPLAY_ON_TIME_SECONDS_IF_TIMEOUT * 1000) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS)
                        && !sDebugModeActive) {
                    myLCD.noBacklight(); // switch off backlight after 3 minutes
                    sSerialLCDIsSwitchedOff = true;
                    Serial.println(F("Switch off LCD display now, triggered by receive timeouts."));
                }
#if !defined(STANDALONE_TEST)
                if (!sSerialLCDIsSwitchedOff) {
                    myLCD.setCursor(16, 0);
                    myLCD.print(sFrameTimeoutCounterForLCDTAutoOff);
                }
#endif
            }

            sFrameTimeoutCounterForLCDTAutoOff++;

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
#if defined(LCD_PAGES_TEST)
                if (sSerialLCDAvailable) {
                    testLCDPages();
                    delay(2000);
                    testBigNumbers();
                }
#endif
            }
#endif
        } // millis() - sMillisOfLastReceivedByte >= TIMEOUT_MILLIS_FOR_FRAME_REPLY
    } // if (sFrameIsRequested)

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
        if (sErrorStringForLCD != NULL) {
            tone(BUZZER_PIN, 2200, 50);
            delay(200);
            tone(BUZZER_PIN, 2200, 50);
        }
    }

    /*
     * Sleep instead of checking millis(). This saves only 5 mA during sleep.
     */
#if MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS == 2000 && MILLISECONDS_BETWEEN_CAN_FRAME_SEND == 2000
    // currently not in the mood to compute it more general ;-)
    if (sEnableSleep) {
        sEnableSleep = false;
        Serial.flush();
        // Interrupts, like button press will wake us up early, but millis will be incremented anyway :-(
        sleepWithWatchdog(WDTO_1S, true); // I have seen clock deviation of + 30 % :-(
        sleepWithWatchdog(WDTO_500MS, true);
        sleepWithWatchdog(WDTO_250MS, true); // assume maximal 250 ms for BMS, LCD and CAN communication
    }
#endif
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

    handleAndPrintAlarmInfo();
    computeUpTimeString();

    fillAllCANData(sJKFAllReplyPointer);
    sCanDataIsInitialized = true;
}

void printReceivedData() {
    if (!sStaticInfoWasSent) {
        sStaticInfoWasSent = true;
        printJKStaticInfo();
    }
    printJKDynamicInfo();
    sCanDataIsInitialized = true;
    if (sSerialLCDAvailable && !sSerialLCDIsSwitchedOff) {
        printBMSDataOnLCD();
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

void printShortStateOnLCD() {
    if (sJKFAllReplyPointer->BMSStatus.StatusBits.ChargeMosFetActive) {
        myLCD.print('C');
    } else {
        myLCD.print(' ');
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

void printCurrentOnLCD() {
    uint16_t tBatteryLoadCurrentInt = JKComputedData.BatteryLoadCurrentFloat;
    tBatteryLoadCurrentInt = abs(tBatteryLoadCurrentInt);
    if (tBatteryLoadCurrentInt < 10) {
        myLCD.print(JKComputedData.BatteryLoadCurrentFloat, 2);
    } else if (tBatteryLoadCurrentInt < 100) {
        myLCD.print(JKComputedData.BatteryLoadCurrentFloat, 1);
    } else {
        myLCD.print(JKComputedData.BatteryLoadCurrentFloat, 0);
    }
    myLCD.print(F("A "));
}

/*
 * Print selected data on a 2004 LCD display
 */
char sStringBuffer[7]; // For rendering numbers with sprintf()
void printBMSDataOnLCD() {
    myLCD.clear();

    myLCD.setCursor(0, 0);

    if (sDisplayPageNumber == JK_BMS_PAGE_OVERVIEW) {
        /*
         * Top row 1 - Error message or up time
         */
        if (sErrorStringForLCD != NULL) {
            // print not more than 20 characters
            char t20CharacterString[LCD_COLUMNS + 1];
            memcpy_P(t20CharacterString, sErrorStringForLCD, LCD_COLUMNS);
            t20CharacterString[LCD_COLUMNS] = '\0';
            myLCD.print(t20CharacterString);
        } else {
            myLCD.print(F("Uptime:  "));
            myLCD.print(&sUpTimeString[4]);
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

        myLCD.setCursor(11, 1);
        printLongStateOnLCD();

        /*
         * Row 3 - Voltage, Current and Power
         */
        myLCD.setCursor(0, 2);
// Voltage
        myLCD.print(JKComputedData.BatteryVoltageFloat, 2);
        myLCD.print(F("V "));

// Current
        printCurrentOnLCD();

// Power
        if (JKComputedData.BatteryLoadPower < -10000) {
            // over 10 kW
            myLCD.setCursor(13, 2);
            myLCD.print(JKComputedData.BatteryLoadPower); // requires 6 columns
        } else {
            myLCD.setCursor(14, 2);
            sprintf(sStringBuffer, "%5d", JKComputedData.BatteryLoadPower); // force use of 5 columns
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

        // Last 3 characters are the enable states
        myLCD.setCursor(17, 3);
        printShortEnableFlagsOnLCD();

        /**************************
         *  CELL INFO
         **************************/
    } else if (sDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
        /*
         * We can display only up to 16 cell values on the LCD :-(
         */
        uint_fast8_t tRowNumber;
        auto tNumberOfCellInfoEnties = JKConvertedCellInfo.NumberOfCellInfoEnties;
        if (tNumberOfCellInfoEnties > 12) {
            tRowNumber = 0;
            if (tNumberOfCellInfoEnties > 16) {
                tNumberOfCellInfoEnties = 16;
            }
        } else {
            myLCD.print(F("    -CELL INFO-"));
            tRowNumber = 1;
        }
        for (uint8_t i = 0; i < tNumberOfCellInfoEnties; ++i) {
            if (i % 4 == 0) {
                myLCD.setCursor(0, tRowNumber);
                tRowNumber++;
            }

            // print maximum or minimum indicator
            if (i == JKConvertedCellInfo.MaximumVoltagCellIndex) {
                myLCD.print((char) 0x1);
            } else if (i == JKConvertedCellInfo.MinimumVoltagCellIndex) {
                myLCD.print((char) 0x2);
            } else {
                myLCD.print(' ');
            }
            myLCD.print(JKConvertedCellInfo.CellInfoStructArray[i].CellMillivolt);
        }

        /**************************
         *  BIG INFO
         **************************/
    } else if (sDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {
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
            sprintf(sStringBuffer, "%d", JKComputedData.BatteryLoadPower);
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

        // Row 3: Max temperature, the actual states
        myLCD.setCursor(0, 3);
        myLCD.print(JKComputedData.TemperatureMaximum);
        myLCD.print(F("\xDF" "C "));

        myLCD.setCursor(7, 3);
        printCurrentOnLCD();

        myLCD.setCursor(17, 3);
        printShortStateOnLCD();

        /**************************
         *  CAN INFO
         **************************/
    } else {
        /*
         * sDisplayPageNumber == JK_BMS_PAGE_CAN_INFO
         */
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
        if (PylontechCANCurrentValuesFrame.FrameData.Current100Milliampere < 100) {
            // Print fraction if value < 100
            myLCD.print('.');
            myLCD.print(PylontechCANCurrentValuesFrame.FrameData.Current100Milliampere % 10);
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
        if (PylontechCANBatteryRequesFrame.FrameData.ChargeEnable) {
            myLCD.print(F("CH "));
        } else {
            myLCD.print(F("   "));
        }
        if (PylontechCANBatteryRequesFrame.FrameData.DischargeEnable) {
            myLCD.print(F("DC"));
        }

        myLCD.setCursor(10, 3);
        if (PylontechCANBatteryRequesFrame.FrameData.FullChargeRequest) {
            myLCD.print(F("FULL"));
        }

        myLCD.setCursor(15, 3);
        if (PylontechCANBatteryRequesFrame.FrameData.ForceChargeRequestI
                || PylontechCANBatteryRequesFrame.FrameData.ForceChargeRequestII) {
            myLCD.print(F("FORCE "));
        }

    }
}

/*
 * Manually check button state change
 */
void checkButtonStateChange() {
    if (Button0AtPin2.ButtonStateHasJustChanged) {
        // reset flag in order to do this only once per button press
        Button0AtPin2.ButtonStateHasJustChanged = false;
        sFrameCounterForLCDTAutoOff = 0; // reset display timeout

        if (Button0AtPin2.ButtonStateIsActive) {
            if (sSerialLCDIsSwitchedOff) {
                /*
                 * If off, switch backlight LED on, but do not select next page
                 */
                myLCD.backlight();
                sFrameTimeoutCounterForLCDTAutoOff = 0; // start again to enable switch off after 3 minutes
                sFrameCounterForLCDTAutoOff = 0; // start again to enable switch off after 5 minutes
                sSerialLCDIsSwitchedOff = false;
                Serial.println(F("Switch on LCD display, triggered by button press"));
            } else if (sFrameTimeoutCounterForLCDTAutoOff == 0) {
                /*
                 * Switch display pages only if no timeout happened
                 */
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
                }

                Serial.print(F("Set LCD display page to: "));
                Serial.println(sDisplayPageNumber);

            }

        } // if (Button0AtPin2.ButtonStateIsActive)
    } // Button0AtPin2.ButtonStateHasJustChanged
    if (sErrorStatusJustChanged == true && sErrorStringForLCD != NULL) {
        /*
         * Switch to overview page once, to show the error
         */
        sErrorStatusJustChanged = false;
        sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    }
}

void testLCDPages() {
    sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    printBMSDataOnLCD();

    sDisplayPageNumber = JK_BMS_PAGE_CELL_INFO;
// Create symbols character for maximum and minimum
    bigNumberLCD._createChar(1, bigNumbersTopBlock);
    bigNumberLCD._createChar(2, bigNumbersBottomBlock);
    delay(2000);
    printBMSDataOnLCD();

    sDisplayPageNumber = JK_BMS_PAGE_CAN_INFO;
    delay(2000);
    printBMSDataOnLCD();

    /*
     * Check display of maximum values
     */
    sJKFAllReplyPointer->SOCPercent = 100;
    JKComputedData.BatteryLoadCurrentFloat = -210.98;
    JKComputedData.BatteryLoadPower = -11000;
    JKComputedData.TemperaturePowerMosFet = 111;
    JKComputedData.TemperatureSensor1 = 99;

    sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    delay(2000);
    printBMSDataOnLCD();

    sDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;
    bigNumberLCD.begin();
    delay(2000);
    printBMSDataOnLCD();

    /*
     * Test other values
     */
    sJKFAllReplyPointer->AlarmUnion.AlarmBits.PowerMosFetOvertemperatureAlarm = true;
    handleAndPrintAlarmInfo(); // this sets the LCD alarm string
    sJKFAllReplyPointer->SOCPercent = 1;
    JKComputedData.BatteryLoadCurrentFloat = -10;
    JKComputedData.BatteryLoadPower = 12345;

    sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    delay(2000);
    printBMSDataOnLCD();

    sDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;
    delay(2000);
    printBMSDataOnLCD();

    sJKFAllReplyPointer->AlarmUnion.AlarmBits.PowerMosFetOvertemperatureAlarm = false;
    handleAndPrintAlarmInfo(); // this sets the LCD alarm string
    sJKFAllReplyPointer->SOCPercent = 100;
    JKComputedData.BatteryLoadCurrentFloat = -100;

    sDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;
    delay(2000);
    printBMSDataOnLCD();
}

void testBigNumbers() {
    sDisplayPageNumber = JK_BMS_PAGE_BIG_INFO;

    for (int j = 0; j < 3; ++j) {
        // Test with 100 %  and 42 %

        /*
         * test with positive numbers
         */
        JKComputedData.BatteryLoadPower = 12345;
        for (int i = 0; i < 5; ++i) {
            delay(4000);
            printBMSDataOnLCD();
            JKComputedData.BatteryLoadPower /= 10; // 1234 -> 12
        }
        /*
         * test with negative numbers
         */
        JKComputedData.BatteryLoadPower = -12345;
        for (int i = 0; i < 5; ++i) {
            delay(4000);
            printBMSDataOnLCD();
            JKComputedData.BatteryLoadPower /= 10; // 1234 -> 12
        }

        sJKFAllReplyPointer->SOCPercent /= 10;
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
