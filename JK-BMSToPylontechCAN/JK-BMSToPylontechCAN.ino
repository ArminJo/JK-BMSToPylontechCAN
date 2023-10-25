/*
 *  JK-BMSToPylontechCAN.cpp
 *
 *  Converts the JK-BMS RS485 data to Pylontech CAN data for inverters
 *  which are not compatible with JK-BMS protocol but with Pylontech protocol, like Deye inverters.
 *  It displays many BMS information and alarms on a locally attached 2004 LCD.
 *  The JK-BMS data are provided as RS232 TTL.
 *
 *  Data from JK-BMS is received by the Hardware USART, and sent by the SoftwareSerialTX library.
 *  Data for serial monitor is sent by Hardware USART (connected to USB).
 *
 *  The software TX and hardware RX lines are connected to the JK-BMS and run with 115200 baud.
 *  CAN is connected to the inverter which must accept Pylontech low voltage protocol, which runs with 500 kBit/s.
 *  This protocol is used e.g by the Pylontech US2000 battery.
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  !!! 8 MHz crystal and 500 kBit/s does not work with MCP2515                 !!!
 *  !!! So you must replace the crystal of the module with a 16 (or 20) MHz one !!!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *  Internal operation (default every 2 seconds):
 *  1. A request to deliver all informations is sent to the BMS (1.85 ms).
 *  2. Wait and receive the BMS status frame (0.18 to 1 ms + 25.5 ms).
 *  3. The BMS status frame is stored in a buffer and parity and other plausi checks are made.
 *  4. The cell data are converted and enhanced to fill the JKConvertedCellInfoStruct.
 *     Other frame data are mapped to a C structure.
 *     But all words and longs in this structure are filled with big endian and thus cannot be read directly but must be swapped on reading.
 *  5. Other frame data are converted and enhanced to fill the JKComputedDataStruct.
 *  6. The content of the status frame is printed. After reset, all info is printed once, then only dynamic info is printed.
 *  7. The required CAN data is filled in the according PylontechCANFrameInfoStruct.
 *  8. Dynamic data and errors are displayed on the optional 2004 LCD if attached.
 *  9. CAN data is sent..
 *
 *  The LCD has 4 "pages" showing overview data, up to 16 cell voltages, up to 16 cell minimum and maximum statistics, or SOC and current with big numbers.
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
 *                                             ___ 78L05    Schottky diode
 * External 6.6 V from Battery #2 >--o--------|___|-------o----|<|---O From Uno / Nano 5 V to avoid
 *                                   |          |         | 5V         parasitic supply of CAN module
 *  __________ Schottky diode    ____|____     ---    ____|____             _________
 * |        TX|----|<|-- RX --->|RX Vin   |<-- SPI ->|   5V    |           |         |
 * |        RX|<-------- TX ----|4  Uno/  |          | MCP2515 |           |         |
 * |  JK-BMS  |                 |   Nano  |          |   CAN   |<-- CAN -->|  DEYE   |
 * |          |<------- GND --->|         |<-- GND-->|         |           |         |
 * |__________|                 |_________|          |_________|           |_________|
 *
 * # UART-TTL socket (4 Pin, JST 1.25mm pitch)
 *  ___ ________ ___
 * |                |
 * | O   O   O   O  |
 * |GND  RX  TX VBAT|
 * |________________|
 *   |   |   |
 *   |   |   --|>|-- RX of Uno / Nano
 *   |   ----------- D4 (or other)
 *   --------------- GND
 */

/*
 * Ideas:
 * Balancing time per day / week / month etc.
 * Maximum, minimum cell while balancing
 */
#include <Arduino.h>

/*
 * If battery SOC is below this value, the inverter is forced to charge the battery from any available power source regardless of inverter settings.
 */
#define SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I        5
//#define SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I        0 // This disables the setting if the force charge request, even if battery SOC is 0.
const uint8_t sSOCThresholdForForceCharge = SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I;

#define VERSION_EXAMPLE "2.3.0"

/*
 * Pin layout, may be adapted to your requirements
 */
#define BUZZER_PIN                 A2 // To signal errors
#define LCD_PAGE_BUTTON_PIN         2 // Just for documentation
#define DEBUG_PIN                   3 // Just for documentation. If pressed, print additional info and switch to LCD CAN page.
// The standard RX of the Arduino is used for the JK_BMS connection.
#define JK_BMS_RX_PIN               0 // We use the Serial RX pin. Not used in program, only for documentation
#if !defined(JK_BMS_TX_PIN)           // Allow override by global symbol
#define JK_BMS_TX_PIN               4
#endif
/*
 * The SPI pins for connection to CAN converter and the I2C / TWI pins for the LCD are determined by hardware.
 * For Uno / Nano:
 *   SPI: MOSI - 11, MISO - 12, SCK - 13. CS cannot be replaced by constant ground.
 *   I2C: SDA - A4, SCL - A5.
 */
#if !defined(SPI_CS_PIN)              // Allow override by global symbol
#define SPI_CS_PIN                  9 // Pin 9 is the default pin for the Arduino CAN bus shield. Alternately you can use pin 10 on this shield.
//#define SPI_CS_PIN                 10 // Must be specified before #include "MCP2515_TX.hpp"
#endif

//#define STANDALONE_TEST

//#define TIMING_TEST
#define TIMING_TEST_PIN             7

/*
 * Program timing, may be adapted to your requirements
 */
#if defined(STANDALONE_TEST)
#define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     1000
#define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             1000
#define SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS          "1" // Only for display on LCD
#define SECONDS_BETWEEN_CAN_FRAME_SEND                  "1" // Only for display on LCD
#else
#define MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS     2000
#define MILLISECONDS_BETWEEN_CAN_FRAME_SEND             2000
#define SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS          "2" // Only for display on LCD
#define SECONDS_BETWEEN_CAN_FRAME_SEND                  "2" // Only for display on LCD
#endif

/*
 * Error beep behavior
 * Overvoltage error cannot be suppressed by a macro!
 * If no NO_BEEP_ON_ERROR, ONE_BEEP_ON_ERROR or MULTIPLE_BEEPS_WITH_TIMEOUT are activated, we beep forever until error vanishes.
 */
//#define NO_BEEP_ON_ERROR              // If activated, Do not beep on error or timeout.
//#define ONE_BEEP_ON_ERROR             // If activated, only beep once if error was detected.
#define BEEP_TIMEOUT_SECONDS        60L // 1 minute, Maximum is 254 seconds = 4 min 14 s
#define MULTIPLE_BEEPS_WITH_TIMEOUT     // If activated, beep for 1 minute if error was detected. Timeout is disabled if debug is active.
bool sLastDoErrorBeep = false;          // required for ONE_BEEP_ON_ERROR
bool sDoErrorBeep = false;              // If true, we do an error beep at the end of the loop
uint8_t sBeepTimeoutCounter;
uint16_t sTimeoutFrameCounter = 0;      // Counts BMS frame timeouts, (every 2 seconds)

//#define SUPPRESS_LIFEPO4_PLAUSI_WARNING   // Disables warning on Serial out about using LiFePO4 beyond 3.0 v to 3.45 V.

//#define USE_NO_LCD
#if !defined(USE_NO_LCD)
#define USE_SERIAL_LCD
#endif

#if defined(USE_SERIAL_LCD) || defined(USE_PARALLEL_LCD)
#define USE_LCD
#endif

#if defined(USE_LCD)
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
#endif // defined(USE_LCD)

/*
 * Page button stuff
 *
 * Button at INT0 / D2 for switching LCD pages
 */
#define USE_BUTTON_0              // Enable code for 1. button at INT0 / D2
#define BUTTON_DEBOUNCING_MILLIS 80 // With this you can adapt to the characteristic of your button. Default is 50.
#define NO_BUTTON_RELEASE_CALLBACK  // Disables the code for release callback. This saves 2 bytes RAM and 64 bytes program memory.
#include "EasyButtonAtInt01.hpp"

volatile bool sPageButtonJustPressed = false;
void handlePageButtonPress(bool aButtonToggleState);     // The button press callback function sets just a flag.
EasyButton PageSwitchButtonAtPin2(&handlePageButtonPress);   // Button is connected to INT0
#define LONG_PRESS_BUTTON_DURATION_MILLIS   1000
bool sDebugModeActivated = false; // Is activated on long press
void checkButtonPress();

bool readJK_BMSStatusFrame();
void processJK_BMSStatusFrame();
void handleFrameReceiveTimeout();

/*
 * Software serial for JK-BMS stuff
 */
#if !defined(MAXIMUM_NUMBER_OF_CELLS)
#define MAXIMUM_NUMBER_OF_CELLS     24 // Maximum number of cell info which can be converted. Must be before #include "JK-BMS.hpp".
#endif
#include "JK-BMS.hpp"

/*
 * Software serial for JK-BMS request frame sending
 */
#include "SoftwareSerialTX.h"
/*
 * Use a 115200 baud software serial for the short request frame.
 * If available, we also can use a second hardware serial here :-).
 */
SoftwareSerialTX TxToJKBMS(JK_BMS_TX_PIN);
bool sFrameIsRequested = false;             // If true, request was recently sent so now check for serial input
uint32_t sMillisOfLastRequestedJKDataFrame = -MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS; // Initial value to start first request immediately
uint32_t sMillisOfLastReceivedByte = 0;     // For timeout

/*
 * CAN stuff
 */
#if !defined(NO_SNA_EXTENSIONS) // SNA
#define SNA_EXTENSIONS // Add frame 0x379 for total capacity of the battery to be used with SNA5000 extension (from LuxPowerTech), which is no problem for Deye inverters.
#endif
#include "Pylontech_CAN.hpp" // Must be before #include "MCP2515_TX.hpp"
#define CAN_BAUDRATE    500000  // 500 kB
#if !defined(MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE)
// Must be specified before #include "MCP2515_TX.hpp"
#define MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE  16 // 16 MHz is default for the Arduino CAN bus shield
//#define MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE   8 // 8 MHz is default for the Chinese breakout board. !!! 8MHz does not work with 500 kB !!!
#endif
#include "MCP2515_TX.hpp"                   // my reduced tx only driver
bool sCANDataIsInitialized = false;         // One time flag, it is never set to false again.
uint32_t sMillisOfLastCANFrameSent = 0;     // For CAN timing

/*
 * Optional sleep stuff
 */
//#define USE_SLEEP
#if defined(USE_SLEEP) && MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS == 2000 && MILLISECONDS_BETWEEN_CAN_FRAME_SEND == 2000
// currently not in the mood to compute it more general ;-)
void LoopDelayWithSleep();
#include "AVRUtils.h"
#endif

bool sBMSFrameProcessingComplete = false; // True if one status frame was received and processed or timeout happened. Then we can do a sleep at the end of the loop.

/*
 * Optional LCD hardware stuff
 */
#if defined(USE_SERIAL_LCD)
#define LCD_COLUMNS     20
#define LCD_ROWS         4
#define LCD_I2C_ADDRESS 0x27     // Default LCD address is 0x27 for a 20 chars and 4 line / 2004 display
bool sSerialLCDAvailable;

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
#define JK_BMS_PAGE_MAX                 JK_BMS_PAGE_BIG_INFO
#define JK_BMS_START_PAGE               JK_BMS_PAGE_BIG_INFO
//uint8_t sLCDDisplayPageNumber = JK_BMS_PAGE_OVERVIEW; // Start with Overview page
uint8_t sLCDDisplayPageNumber = JK_BMS_START_PAGE; // Start with Big Info page
uint8_t sCellStatisticsDisplayCounter;    // counter for CELL_STATISTICS_COUNTER_MASK, to determine max or min page
char sStringBuffer[7];                    // For rendering numbers with sprintf_P()
void setDisplayPage(uint8_t aDisplayPageNumber);

void printBMSDataOnLCD();
void printCANInfoOnLCD();
void LCDPrintSpaces(uint8_t aNumberOfSpacesToPrint);
void LCDClearLine(uint8_t aLineNumber);
#endif // defined(USE_SERIAL_LCD)

/*
 * Miscellaneous
 */
#define TIMEOUT_MILLIS_FOR_FRAME_REPLY                  100 // I measured 26 ms between request end and end of received 273 byte result
#if TIMEOUT_MILLIS_FOR_FRAME_REPLY > MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS
#error "TIMEOUT_MILLIS_FOR_FRAME_REPLY must be smaller than MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS to detect timeouts"
#endif
bool sStaticInfoWasSent = false; // Flag to send static Info only once after reset.

#if defined(TIMING_TEST)
#include "digitalWriteFast.h"
#endif

void processReceivedData();
void printReceivedData();
bool isVCCTooHighSimple();
void handleOvervoltage();

#if defined(STANDALONE_TEST)
//#define LCD_PAGES_TEST
#  if defined(LCD_PAGES_TEST)
//#define BIG_NUMBER_TEST
#  endif
const uint8_t TestJKReplyStatusFrame[] PROGMEM = { /* Header*/0x4E, 0x57, 0x01, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01,
/*Length of Cell voltages*/
0x79, 0x30,
/*Cell voltages*/
0x01, 0x0C, 0xC6, 0x02, 0x0C, 0xBE, 0x03, 0x0C, 0xC7, 0x04, 0x0C, 0xC7, 0x05, 0x0C, 0xC7, 0x06, 0x0C, 0xC5, 0x07, 0x0C, 0xC6, 0x08,
        0x0C, 0xC7, 0x09, 0x0C, 0xC2, 0x0A, 0x0C, 0xC2, 0x0B, 0x0C, 0xC2, 0x0C, 0x0C, 0xC2, 0x0D, 0x0C, 0xC1, 0x0E, 0x0C, 0xBE,
        0x0F, 0x0C, 0xC1, 0x10, 0x0C, 0xC1,
        /*JKFrameAllDataStruct*/
        0x80, 0x00, 0x16, 0x81, 0x00, 0x15, 0x82, 0x00, 0x15, /*Voltage*/0x83, 0x14, 0x6C, /*Current*/0x84, 0x80, 0xD0, /*SOC*/0x85,
        0x47, 0x86, 0x02, 0x87, 0x00, 0x04, 0x89, 0x00, 0x00, 0x01, 0xE0, 0x8A, 0x00, 0x0E, /*Warnings*/0x8B, 0x00, 0x00, 0x8C,
        0x00, 0x07, 0x8E, 0x16, 0x26, 0x8F, 0x10, 0xAE, 0x90, 0x0F, 0xD2, 0x91, 0x0F, 0xA0, 0x92, 0x00, 0x05, 0x93, 0x0B, 0xEA,
        0x94, 0x0C, 0x1C, 0x95, 0x00, 0x05, 0x96, 0x01, 0x2C, 0x97, 0x00, 0x07, 0x98, 0x00, 0x03, 0x99, 0x00, 0x05, 0x9A, 0x00,
        0x05, 0x9B, 0x0C, 0xE4, 0x9C, 0x00, 0x08, 0x9D, 0x01, 0x9E, 0x00, 0x5A, 0x9F, 0x00, 0x46, 0xA0, 0x00, 0x64, 0xA1, 0x00,
        0x64, 0xA2, 0x00, 0x14, 0xA3, 0x00, 0x46, 0xA4, 0x00, 0x46, 0xA5, 0xFF, 0xEC, 0xA6, 0xFF, 0xF6, 0xA7, 0xFF, 0xEC, 0xA8,
        0xFF, 0xF6, 0xA9, 0x0E, 0xAA, 0x00, 0x00, 0x01, 0x40, 0xAB, 0x01, 0xAC, 0x01, 0xAD, 0x04, 0x11, 0xAE, 0x01, 0xAF, 0x01,
        0xB0, 0x00, 0x0A, 0xB1, 0x14, 0xB2, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x00, 0x00, 0x00, 0x00, 0xB3, 0x00, 0xB4, 0x49,
        0x6E, 0x70, 0x75, 0x74, 0x20, 0x55, 0x73, 0xB5, 0x32, 0x31, 0x30, 0x31, 0xB6, 0x00, 0x00, 0xE2, 0x00, 0xB7, 0x31, 0x31,
        0x2E, 0x58, 0x57, 0x5F, 0x53, 0x31, 0x31, 0x2E, 0x32, 0x36, 0x5F, 0x5F, 0x5F, 0xB8, 0x00, 0xB9, 0x00, 0x00, 0x04, 0x00,
        0xBA, 0x49, 0x6E, 0x70, 0x75, 0x74, 0x20, 0x55, 0x73, 0x65, 0x72, 0x64, 0x61, 0x4A, 0x4B, 0x5F, 0x42, 0x32, 0x41, 0x32,
        0x30, 0x53, 0x32, 0x30, 0x50, 0xC0, 0x01,
        /*Trailer*/
        0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x51, 0xC2 };

void doStandaloneTest();
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
#if defined(TIMING_TEST)
    pinMode(TIMING_TEST_PIN, OUTPUT);
#endif

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    tone(BUZZER_PIN, 2200, 50);

#if defined(USE_SERIAL_LCD)
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
#endif

    /*
     * 115200 baud soft serial to JK-BMS. For serial from BMS we use the hardware Serial RX.
     */
    TxToJKBMS.begin(115200);
    Serial.println(F("Serial to JK-BMS started with 115200 bit/s!"));
#if defined(USE_LCD)
    if (sSerialLCDAvailable) {
        myLCD.setCursor(0, 2);
        myLCD.print(F("BMS serial started"));
    }
#endif
    /*
     * CAN initialization
     */
    //    if (CAN.begin(500000)) { // Resets the device and start the CAN bus at 500 kbps
    if (initializeCAN(CAN_BAUDRATE, MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE, &Serial) == MCP2515_RETURN_OK) { // Resets the device and start the CAN bus at 500 kbps
        Serial.println(F("CAN started with 500 kbit/s!"));
#if defined(USE_LCD)
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("CAN started"));
            delay(4000); // To see the info
        }
#endif
    } else {
        Serial.println(F("Starting CAN failed!"));
#if defined(USE_LCD)
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("Starting CAN failed!"));
#  if defined(STANDALONE_TEST)
            delay(2000);
#  else
            delay(8000); // To see the info
#  endif
        }
#endif

    }

    /*
     * Print debug pin info
     */
#if defined(USE_LCD)
    Serial.println(F("Page switching button is at pin " STR(LCD_PAGE_BUTTON_PIN)));
    Serial.println(F("At long press, CAN Info page is entered and additional debug data is printed as long as button is pressed"));
#else
    Serial.println(F("Debug button is at pin " STR(LCD_PAGE_BUTTON_PIN)));
    Serial.println(F("Additional debug data is printed as long as button is pressed"));
#endif
    Serial.println(F(STR(MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) " ms between 2 BMS requests"));
    Serial.println(F(STR(MILLISECONDS_BETWEEN_CAN_FRAME_SEND) " ms between 2 CAN transmissions"));
#if defined(USE_LCD) && !defined(DISPLAY_ALWAYS_ON)
    Serial.println(F("LCD Backlight timeout is " DISPLAY_ON_TIME_STRING));
#endif
    Serial.println();

#if defined(USE_LCD)
    if (sSerialLCDAvailable) {
#  if !defined(DISPLAY_ALWAYS_ON)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Screen timeout " DISPLAY_ON_TIME_STRING));
#  endif
        myLCD.setCursor(0, 1);
        myLCD.print(F("Long press = debug"));
        myLCD.setCursor(0, 2);
#if defined(STANDALONE_TEST)
        myLCD.print(F("Test -fixed BMS data"));
#else
        myLCD.print(F("Get  BMS every " SECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS " s  "));
#endif
        myLCD.setCursor(0, 3);
        myLCD.print(F("Send CAN every " SECONDS_BETWEEN_CAN_FRAME_SEND " s  "));
        delay(2000); // To see the messages
        myLCD.clear();
    }
#endif

#if defined(USE_SLEEP)
    initSleep(SLEEP_MODE_PWR_SAVE);
#endif

#if defined(STANDALONE_TEST)
    /*
     * Copy test data to receive buffer
     */
    Serial.println(F("Standalone test. Use fixed demo data"));
    Serial.println();
    memcpy_P(JKReplyFrameBuffer, TestJKReplyStatusFrame, sizeof(TestJKReplyStatusFrame));
    sReplyFrameBufferIndex = sizeof(TestJKReplyStatusFrame) - 1;
    printJKReplyFrameBuffer();
    Serial.println();
    processReceivedData();
    printReceivedData();
    /*
     * Copy complete reply and computed values for change determination
     */
    lastJKComputedData = JKComputedData;
    lastJKReply = *sJKFAllReplyPointer; // 221 bytes
    doStandaloneTest();
#endif
}

void loop() {

    checkButtonPress();

    /*
     * Request status frame every 2 seconds
     */
    if (millis() - sMillisOfLastRequestedJKDataFrame >= MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
        sMillisOfLastRequestedJKDataFrame = millis(); // set for next check
        /*
         * Flush input buffer and send request to JK-BMS
         */
        while (Serial.available()) {
            Serial.read();
        }
#if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, HIGH);
#endif
        requestJK_BMSStatusFrame(&TxToJKBMS, sDebugModeActivated); // 1.85 ms
#if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, LOW);
#endif
        sFrameIsRequested = true; // enable check for serial input
        initJKReplyFrameBuffer();
        sMillisOfLastReceivedByte = millis(); // initialize reply timeout
    }

#if defined(STANDALONE_TEST)
    sBMSFrameProcessingComplete = true; // for LCD timeout etc.
    processReceivedData(); // for statistics
    printBMSDataOnLCD(); // for switching between MAX and MIN display
    delay(MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS); // do it simple :-)
#else
    /*
     * Get reply from BMS and check timeout
     */
    if (sFrameIsRequested) {
        if (Serial.available()) {
#  if defined(TIMING_TEST)
            digitalWriteFast(TIMING_TEST_PIN, HIGH);
#  endif
            if (readJK_BMSStatusFrame()) {
                processJK_BMSStatusFrame(); // Process the complete receiving of the status frame and set the appropriate flags
            }
#  if defined(TIMING_TEST)
            digitalWriteFast(TIMING_TEST_PIN, LOW);
#  endif

        } else if (millis() - sMillisOfLastReceivedByte >= TIMEOUT_MILLIS_FOR_FRAME_REPLY) {
            /*
             * Here we have requested frame, but serial was not available fore a longer time => timeout at receiving
             * If no bytes received before (because of BMS disconnected), print it only once
             */
            handleFrameReceiveTimeout();
        }
    }
#endif // !defined(STANDALONE_TEST)

    /*
     * Send CAN frame independently of the period of JK-BMS data requests
     * 0.5 MB/s
     * Inverter reply every second: 0x305: 00-00-00-00-00-00-00-00
     * Do not send, if BMS is starting up, the 0% SOC during this time will trigger a deye error beep.
     */
    if (sCANDataIsInitialized && !JKComputedData.BMSIsStarting
            && millis() - sMillisOfLastCANFrameSent >= MILLISECONDS_BETWEEN_CAN_FRAME_SEND) {
        sMillisOfLastCANFrameSent = millis();

        if (sDebugModeActivated) {
            Serial.println(F("Send CAN"));
        }
        sendPylontechAllCANFrames(sDebugModeActivated);
    }

    /*
     * Do this once after each complete status frame or timeout
     */
    if (sBMSFrameProcessingComplete) {
        sDebugModeActivated = false; // reset flag here. It may be set again at start of next loop.

        /*
         * Check for overvoltage
         */
        while (isVCCTooHighSimple()) {
            handleOvervoltage();
        }

        /*
         * Checking for BMS error flags
         */
        if (sErrorStringForLCD != NULL && sErrorStatusIsError) {
            sDoErrorBeep = true;
            if (sErrorStatusJustChanged) {
                /*
                 * Switch to overview page once, to show the error
                 * Not required for non errors
                 */
                sErrorStatusJustChanged = false;
#if defined(USE_LCD)
                setDisplayPage(JK_BMS_PAGE_OVERVIEW);
#  if !defined(DISPLAY_ALWAYS_ON)
                if (checkAndTurnLCDOn()) {
                    Serial.println(F("error status changing")); // Switch on LCD display, triggered by error status changing
                }
#  endif
#endif
            }
        }

#if defined(USE_LCD) && !defined(DISPLAY_ALWAYS_ON)
        if (sSerialLCDAvailable) {
            doLCDBacklightTimeoutHandling();
        }
#endif

        /*
         * Beep handling
         */
#if !defined(NO_BEEP_ON_ERROR)      // Beep enabled
#  if defined(ONE_BEEP_ON_ERROR)    // Beep once
        if (sDoErrorBeep) {
            if (!sLastDoErrorBeep) {
                // First error here
                sLastDoErrorBeep = sDoErrorBeep; // Set sLastDoErrorBeep
                Serial.println(F("Beep only once, suppress consecutive error beeps"));
            } else {
                sDoErrorBeep = false; // Suppress consecutive error beeps
            }
        } else {
            sLastDoErrorBeep = sDoErrorBeep; // Reset sLastDoErrorBeep
        }
#  endif
        if (sDoErrorBeep) {
            sDoErrorBeep = false;
#  if defined(MULTIPLE_BEEPS_WITH_TIMEOUT) && !defined(ONE_BEEP_ON_ERROR)   // Beep one minute
            sBeepTimeoutCounter++;
            if (sBeepTimeoutCounter == (BEEP_TIMEOUT_SECONDS * 1000U) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
                Serial.println(F("Timeout reached, suppress consecutive error beeps"));
            } else if (sBeepTimeoutCounter > (BEEP_TIMEOUT_SECONDS * 1000U) / MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
                sBeepTimeoutCounter--; // To avoid overflow
            } else
#  endif
            {
                tone(BUZZER_PIN, 2200);
                tone(BUZZER_PIN, 2200, 50);
                delay(200);
                tone(BUZZER_PIN, 2200, 50);
                delay(200);
                noTone(BUZZER_PIN); // to avoid tone interrupts waking us up from sleep
            }
        }
#endif // NO_BEEP_ON_ERROR

        sBMSFrameProcessingComplete = false; // prepare for next loop

#if defined(USE_SLEEP)
        /*
         * Sleep instead of checking millis(). This saves only 5 mA during sleep.
         */
        void LoopDelayWithSleep();
#endif // defined(USE_SLEEP)
    } // if (sBMSFrameProcessingComplete)
}

/*
 * Process the complete receiving of the status frame and set the appropriate flags
 */
void processJK_BMSStatusFrame() {
    if (sDebugModeActivated) {
        /*
         * Do it once at every debug start
         */
        if (sReplyFrameBufferIndex == 0) {
            Serial.println(F("sReplyFrameBufferIndex is 0"));
        } else {
            Serial.print(sReplyFrameBufferIndex + 1);
            Serial.println(F(" bytes received"));
            printJKReplyFrameBuffer();
        }
        Serial.println();
    }

    sFrameIsRequested = false; // Everything OK, do not try to receive more
    sBMSFrameProcessingComplete = true;
    sJKBMSFrameHasTimeout = false;
    if (sTimeoutFrameCounter > 0) {
        // First frame after timeout
        sTimeoutFrameCounter = 0;
#if defined(USE_LCD) && !defined(DISPLAY_ALWAYS_ON)
        if (checkAndTurnLCDOn()) {
            Serial.println(F("successfully receiving first BMS status frame after BMS communication timeout")); // Switch on LCD display, triggered by successfully receiving first BMS status frame
        }
#endif
    }
    processReceivedData();
    printReceivedData();
    /*
     * Copy complete reply and computed values for change determination
     */
    lastJKComputedData = JKComputedData;
    lastJKReply = *sJKFAllReplyPointer; // 221 bytes
}

/*
 * Reads all bytes of the requested frame into buffer and prints errors
 * Sets sFrameIsRequested to false, if frame was complete and manages other flags too
 * @return true if frame was completely received
 */
bool readJK_BMSStatusFrame() {
    sMillisOfLastReceivedByte = millis();
    uint8_t tReceiveResultCode = readJK_BMSStatusFrameByte();
    if (tReceiveResultCode == JK_BMS_RECEIVE_FINISHED) {
        /*
         * All JK-BMS status frame data received
         */
        return true;

    } else if (tReceiveResultCode != JK_BMS_RECEIVE_OK) {
        /*
         * Error here
         */
        Serial.print(F("Receive error="));
        Serial.print(tReceiveResultCode);
        Serial.print(F(" at index"));
        Serial.println(sReplyFrameBufferIndex);
        sFrameIsRequested = false; // do not try to receive more
        sBMSFrameProcessingComplete = true;
        printJKReplyFrameBuffer();
    }
    return false;
}

/*
 * Here we have requested frame, but serial was not available fore a longer time => timeout at receiving
 * If no bytes received before (because of BMS disconnected), print it only once
 */
void handleFrameReceiveTimeout() {
    sDoErrorBeep = true;
    sFrameIsRequested = false; // Do not try to receive more
    sBMSFrameProcessingComplete = true;
    sJKBMSFrameHasTimeout = true;
    if (sReplyFrameBufferIndex != 0 || sTimeoutFrameCounter == 0) {
        /*
         * No byte received here -BMS may be off or disconnected
         * Do it only once if we receive 0 bytes
         */
        Serial.print(F("Receive timeout at ReplyFrameBufferIndex="));
        Serial.println(sReplyFrameBufferIndex);
        if (sReplyFrameBufferIndex != 0) {
            printJKReplyFrameBuffer();
        }
        modifyAllCanDataToInactive();
#if defined(USE_LCD)
        if (sSerialLCDAvailable && sLCDDisplayPageNumber == JK_BMS_PAGE_CAN_INFO) {
            // Update the changed values on LCD
            myLCD.clear();
            myLCD.setCursor(0, 0);
            printCANInfoOnLCD();
        }
#  if !defined(DISPLAY_ALWAYS_ON)
        if (checkAndTurnLCDOn()) {
            Serial.println(F("BMS communication timeout")); // Switch on LCD display, triggered by BMS communication timeout
        }
#  endif
#endif
    }
    sTimeoutFrameCounter++;
    if (sTimeoutFrameCounter == 0) {
        sTimeoutFrameCounter--; // To avoid overflow, we have an unsigned integer here
    }

#if defined(USE_LCD)
    /*
     * Global timeout message
     */
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
    sCANDataIsInitialized = true; // One time flag
}

void printReceivedData() {
    if (!sStaticInfoWasSent) {
        sStaticInfoWasSent = true;
        printJKStaticInfo();
    }
    printJKDynamicInfo();
#if defined(USE_LCD)
    if (sSerialLCDAvailable
#  if !defined(DISPLAY_ALWAYS_ON)
            && !sSerialLCDIsSwitchedOff
#  endif
    ) {
        printBMSDataOnLCD();
    }
#endif
}

#if defined(USE_LCD)
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
 * Prints state of Charge Overvoltage warning, and Charge, Discharge and Balancing flag
 */
void printShortStateOnLCD() {
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
 * Print current as 5 character including sign
 */
void printCurrentOnLCD() {
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
        tNumberOfDecimalPlaces = 1; // -99.9
    } else {
        tNumberOfDecimalPlaces = 0; // -9999
    }
    myLCD.print(JKComputedData.BatteryLoadCurrentFloat, tNumberOfDecimalPlaces);
    myLCD.print(F("A "));
}

/*
 * We can display only up to 16 cell values on the LCD :-(
 */
void printCellInfoOnLCD() {
    uint_fast8_t tRowNumber;
    auto tNumberOfCellInfoEntries = JKConvertedCellInfo.ActualNumberOfCellInfoEntries;
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
}

/*
 * Switch between display of minimum and maximum at each 4. call
 * The sum of percentages may not give 100% because of rounding errors
 */
void printCellStatisticsOnLCD() {
    // check if minimum or maximum is to be displayed
    bool tDisplayCellMinimumStatistics = sCellStatisticsDisplayCounter & CELL_STATISTICS_COUNTER_MASK; // 0x04
    sCellStatisticsDisplayCounter++;

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

// Bottom row: Max temperature, current and the actual states
    myLCD.setCursor(0, 3);
    myLCD.print(JKComputedData.TemperatureMaximum);
    myLCD.print(F("\xDF "));

    myLCD.setCursor(4, 3);
    printCurrentOnLCD();

    myLCD.setCursor(11, 3);
    uint16_t tBatteryToFullDifference10Millivolt = JKComputedData.BatteryFullVoltage10Millivolt
            - JKComputedData.BatteryVoltage10Millivolt;
    if (tBatteryToFullDifference10Millivolt < 100) {
        // Print small values as ".43" instead of "0.4"
        sprintf_P(sStringBuffer, PSTR(".%02d"), tBatteryToFullDifference10Millivolt);
        myLCD.print(sStringBuffer);
    } else {
        myLCD.print(((float) tBatteryToFullDifference10Millivolt) / 100.0, 1);
    }
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
        // print not more than 20 characters
        char t20CharacterString[LCD_COLUMNS + 1];
        memcpy_P(t20CharacterString, sErrorStringForLCD, LCD_COLUMNS);
        t20CharacterString[LCD_COLUMNS] = '\0';
        myLCD.print(t20CharacterString);
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
    printCurrentOnLCD();
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

    } else { //sLCDDisplayPageNumber == JK_BMS_PAGE_CAN_INFO
        printCANInfoOnLCD();
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
#endif // #if defined(USE_LCD)

/*
 * Callback handlers for button press
 * Just set flags for evaluation in checkButtonPress(), otherwise readButtonState() may again be false when checkButtonPress() is called
 */
void handlePageButtonPress(bool aButtonToggleState __attribute__((unused))) {
    sPageButtonJustPressed = true;
}

/*
 * Manually handle button press
 */
void checkButtonPress() {
#if defined(USE_LCD)
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
                /*
                 * Switch display pages to next page
                 */
                tDisplayPageNumber++;

                if (sJKBMSFrameHasTimeout || tDisplayPageNumber > JK_BMS_PAGE_MAX) {
                    // Receive timeout or wrap around here
                    tDisplayPageNumber = JK_BMS_PAGE_OVERVIEW;

                } else if (tDisplayPageNumber == JK_BMS_PAGE_CELL_INFO) {
                    // Create symbols character for maximum and minimum
                    bigNumberLCD._createChar(1, bigNumbersTopBlock);
                    bigNumberLCD._createChar(2, bigNumbersBottomBlock);

                    // Prepare for statistics page here display max first but for half the regular time
                    sCellStatisticsDisplayCounter = (CELL_STATISTICS_COUNTER_MASK >> 1) - 1;

                } else if (tDisplayPageNumber == JK_BMS_PAGE_BIG_INFO) {
                    bigNumberLCD.begin(); // Creates custom character used for generating big numbers
                }
                setDisplayPage(tDisplayPageNumber);
            }
        } else if (PageSwitchButtonAtPin2.readDebouncedButtonState()) {
            if (tDisplayPageNumber == JK_BMS_PAGE_CAN_INFO) {
                // Button is still pressed
                sDebugModeActivated = true; // Is set to false in loop
            }
            if (PageSwitchButtonAtPin2.checkForLongPress(LONG_PRESS_BUTTON_DURATION_MILLIS) == EASY_BUTTON_LONG_PRESS_DETECTED) {
                /*
                 * Long press detected -> switch to CAN Info page
                 */
                sDebugModeActivated = true; // Is set to false in loop
                if (sSerialLCDAvailable && tDisplayPageNumber != JK_BMS_PAGE_CAN_INFO) {
                    Serial.println();
                    Serial.println(F("Long press detected -> switch to CAN page and activate one time debug print"));
                    setDisplayPage(JK_BMS_PAGE_CAN_INFO);
                }
            }
        } // PageSwitchButtonAtPin2.ButtonStateHasJustChanged
    }
#else
    /*
     * Treat Page button as Debug button
     */
    if (sPageButtonJustPressed) {
        sPageButtonJustPressed = false;
        sDebugModeActivated = true; // Is set to false in loop
        Serial.println(F("One time debug print just activated"));
    } else if (PageSwitchButtonAtPin2.readDebouncedButtonState()) {
        // Button is still pressed
        sDebugModeActivated = true; // Is set to false in loop
    }
#endif // defined(USE_LCD)

}

#if defined(STANDALONE_TEST)
void doStandaloneTest() {

#  if defined(LCD_PAGES_TEST)
    if (sSerialLCDAvailable) {
        testLCDPages();
        delay(2000);
#    if defined(BIG_NUMBER_TEST)
        testBigNumbers();
#    endif
        memcpy_P(JKReplyFrameBuffer, TestJKReplyStatusFrame, sizeof(TestJKReplyStatusFrame));
        processReceivedData(); // to clear every changes
    }
#  endif
}

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
#endif

void handleOvervoltage() {
#if defined(USE_LCD)
    if (sSerialLCDAvailable) {
#  if !defined(DISPLAY_ALWAYS_ON)
        if (sSerialLCDIsSwitchedOff) {
            myLCD.backlight();
            sSerialLCDIsSwitchedOff = false;
        }
#  endif
        myLCD.clear();
        myLCD.setCursor(0, 0);
        myLCD.print(F("VCC overvoltage"));
        myLCD.setCursor(0, 1);
        myLCD.print(F("VCC > 5.25 V"));
    }
#endif
// Do it as long as overvoltage happens
    tone(BUZZER_PIN, 1100, 300);
    delay(300);
    tone(BUZZER_PIN, 2200, 1000);
    delay(1000);
}

#ifndef _ADC_UTILS_HPP
/*
 * Recommended VCC is 1.8 V to 5.5 V, absolute maximum VCC is 6.0 V.
 * Check for 5.25 V, because such overvoltage is quite unlikely to happen during regular operation.
 * Raw reading of 1.1 V is 225 at 5 V.
 * Raw reading of 1.1 V is 221 at 5.1 V.
 * Raw reading of 1.1 V is 214 at 5.25 V (+5 %).
 * Raw reading of 1.1 V is 204 at 5.5 V (+10 %).
 * @return true if overvoltage reached
 */
bool isVCCTooHighSimple() {
    ADMUX = 14 | (DEFAULT << 6);
// ADCSRB = 0; // Only active if ADATE is set to 1.
// ADSC-StartConversion ADIF-Reset Interrupt Flag - NOT free running mode
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADIF) | 7); //  7 -> 104 microseconds per ADC conversion at 16 MHz --- Arduino default
// wait for single conversion to finish
    loop_until_bit_is_clear(ADCSRA, ADSC);

// Get value
    uint16_t tRawValue = ADCL | (ADCH << 8);

    return tRawValue < 214;
}
#endif

#if defined(USE_SLEEP)
void LoopDelayWithSleep() {
    Serial.flush();

    /*
     * Interrupts, like button press will wake us up early.
     * The millis() timer is disabled during sleep, but millis will be incremented by sleepWithWatchdog() function.
     */
    if (!sPageButtonJustPressed) { // skip next delays if button was pressed after loop button processing, to enable fast response
#  if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, HIGH);
#  endif
        sleepWithWatchdog(WDTO_1S, true); // I have seen clock deviation of + 30 % :-(
    }
    if (!sPageButtonJustPressed) { // skip next delays if button was pressed during sleep, which waked us up, to enable fast response
#  if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, LOW);
#  endif
        sleepWithWatchdog(WDTO_500MS, true);
    }
    if (!sPageButtonJustPressed) { // skip next delay if button was pressed during sleep, which waked us up, to enable fast response
#  if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, HIGH);
#  endif
        sleepWithWatchdog(WDTO_250MS, true); // assume maximal 250 ms for BMS, LCD and CAN communication
#  if defined(TIMING_TEST)
        digitalWriteFast(TIMING_TEST_PIN, LOW);
#  endif
    }
}
#endif
