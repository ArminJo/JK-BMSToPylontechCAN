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
 *  Available as Wokwi example https://wokwi.com/projects/371657348012321793
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
 *
 * # Connection schematic
 * A schottky diode is inserted into the RX line to allow programming the AVR with the JK-BMS still connected.
 *
 * ALTERNATIVE EXTERNAL POWER SUPPLY:
 *                                          78L05    Optional Schottky diode - From Uno/Nano 5 V
 *                                           ___                         to enable powering CAN
 * Optional 6.6 V from Battery #2 >-o-------|___|-------o-|<|-< Uno 5V   module by Nano USB, if
 *                                  |         |         |                battery is not attached
 *                                  |        ---        |
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 *                                  |                   |
 *  __________ Schottky diode   ____|____           ____|____             _________
 * |        TX|>---|<|-- RX -->|RX Vin   |<- SPI ->|   5V    |           |         |
 * |        RX|<-------- TX --<|4  Uno/  |         | MCP2515 |           |         |
 * |  JK-BMS  |                |   Nano  |         |   CAN   |<-- CAN -->|  DEYE   |
 * |          |<------- GND -->|         |<- GND ->|         |           |         |
 * |__________|                |_________|         |_________|           |_________|
 *
 * # Connection diagram for JK-BMS GPS / UART-TTL socket (4 Pin, JST 1.25mm pitch)
 *  ___ ________ ___
 * |                |
 * | O   O   O   O  |
 * |GND  RX  TX VBAT|
 * |________________|
 *   |   |   |
 *   |   |   --|<|-- RX of Uno / Nano
 *   |   ----------- D4 (or other pin, if specified)
 *   --------------- GND
 */

#include <Arduino.h>

// For full revision history see https://github.com/ArminJo/JK-BMSToPylontechCAN?tab=readme-ov-file#revision-history
#define VERSION_EXAMPLE "4.0.0"

//#define HANDLE_MULTIPLE_BMS
#if defined(HANDLE_MULTIPLE_BMS)
#define NUMBER_OF_SUPPORTED_BMS 2 // Currently not more than 2 are supported by this source
#endif

/*
 * If battery SOC is below this value, the inverter is forced to charge the battery from any available power source regardless of inverter settings.
 */
//#define SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I        5
#define SOC_THRESHOLD_FOR_FORCE_CHARGE_REQUEST_I        0 // This disables the setting if the force charge request, even if battery SOC is 0.

/*
 * Macros for CAN data modifications
 */
//#define CAN_DATA_MODIFICATION         // Currently enables the function to reduce max current at high SOC level
//#define USE_CCCV_MODIFY_FUNCTION      // Changes modification to CCCV method by Ngoc: https://github.com/ArminJo/JK-BMSToPylontechCAN/discussions/31
//#define USE_OWN_MODIFY_FUNCTION       // Use (currently empty) function which must be filled in at bottom of Pylontech_CAN.hpp
/*
 * Values for standard CAN data modification
 */
//#define MAX_CURRENT_MODIFICATION_LOWER_SOC_THRESHOLD_PERCENT        80  // Start SOC for linear reducing maximum current. Default 80
//#define MAX_CURRENT_MODIFICATION_MIN_CURRENT_TENTHS_OF_AMPERE       50  // Value of current at 100 % SOC. Units are 100 mA! Default 50
//#define DEBUG                 // This enables debug output for all files - only for development
#if defined(DEBUG)
#define NO_CELL_STATISTICS // to save program space required for debug
#endif

//#define STANDALONE_TEST       // If activated, fixed BMS data is sent to CAN bus and displayed on LCD.
#if defined(STANDALONE_TEST)
//#define ALARM_TIMEOUT_TEST
#define BEEP_TIMEOUT_SECONDS        20L     // 20 beeps
//#define LCD_PAGES_TEST // LCD pages layout tests
#  if defined(LCD_PAGES_TEST)
//#define BIG_NUMBER_TEST // Additional automatic tests, especially for rendering of JK_BMS_PAGE_BIG_INFO
#  endif
//#define ENABLE_MONITORING
#  if defined(USE_NO_LCD)
#undef USE_NO_LCD // LCD is activated for standalone test
#  endif
#endif

#if defined(__AVR_ATmega644P__)
#define USE_LAYOUT_FOR_644_BOARD
#endif

/*
 * Options to reduce program size / add optional features
 */
#if FLASHEND > 0x7FFF // for more than 32k
#define ENABLE_MONITORING // Requires additional 858 bytes program space
#define SERIAL_INFO_PRINT // Requires additional 1684 bytes program space
#endif
#define KEEP_ANALYTICS_ACCUMULATED_DATA_AT_RESET  // Requires additional 80 bytes program space

//#define DO_NOT_SHOW_SHORT_CELL_VOLTAGES // Saves 470 bytes program space
#if !defined(DO_NOT_SHOW_SHORT_CELL_VOLTAGES)
#define SHOW_SHORT_CELL_VOLTAGES // Print 3 digits cell voltage (value - 3.0 V) on Cell Info page. Enables display of up to 20 voltages or additional information.
#endif

#if defined(ENABLE_MONITORING) && !defined(MONOTORING_PERIOD_SECONDS)
//#define MONOTORING_PERIOD_FAST    // If active, then print CSV line every 2 seconds, else every minute
#define MONOTORING_PERIOD_SLOW    // If active, then print CSV line every hour, and CSV line every 10 minutes
#endif

#if !defined(SERIAL_INFO_PRINT) && !defined(STANDALONE_TEST) && FLASHEND <= 0x7FFF
#define NO_SERIAL_INFO_PRINT    // Disables writing some info to serial output. Saves 974 bytes program space.
#endif

#if defined(NO_SERIAL_INFO_PRINT)
#define JK_INFO_PRINT(...)      void();
#define JK_INFO_PRINTLN(...)    void();
#else
#define JK_INFO_PRINT(...)      Serial.print(__VA_ARGS__);
#define JK_INFO_PRINTLN(...)    Serial.println(__VA_ARGS__);
#endif

//#define NO_CAPACITY_35F_EXTENSIONS // Disables generating of frame 0x35F for total capacity. This additional frame is no problem for Deye inverters. Saves 56 bytes program space.
//#define NO_CAPACITY_379_EXTENSIONS // Disables generating of frame 0x379 for total capacity. This additional frame is no problem for Deye inverters. Saves 24 bytes program space.
//#define NO_BYD_LIMITS_373_EXTENSIONS // Disables generating of frame 0x373 for cell limits as sent by BYD battery. See https://github.com/dfch/BydCanProtocol/tree/main. This additional frame is no problem for Deye inverters. Saves 200 bytes program space.
//#define NO_CELL_STATISTICS    // Disables generating and display of cell balancing statistics. Saves 1628 bytes program space.
//#define NO_ANALYTICS          // Disables generating, storing and display of SOC graph for Arduino Serial Plotter. Saves 3856 bytes program space.
//#define USE_NO_LCD            // Disables the code for the LCD display. Saves 25% program space on a Nano.

//#define USE_NO_COMMUNICATION_STATUS_LEDS // The code for the BMS and CAN communication status LED is deactivated and the pins are not switched to output

#if !defined(ENABLE_LIFEPO4_PLAUSI_WARNING)
#define SUPPRESS_LIFEPO4_PLAUSI_WARNING     // Disables warning on Serial out about using LiFePO4 beyond 3.0 v to 3.45 V.
#endif

#if !defined(USE_NO_LCD)
#define USE_SERIAL_2004_LCD // Parallel or 1604 LCD not yet supported
#define LCD_MESSAGE_PERSIST_TIME_MILLIS 2000
#endif

#if defined(ARDUINO_AVR_NANO)
/*
 * On a unmodified nano, the fuse settings reserve 2 kB program space, to be backwards compatible.
 * Only 0,5 kB is required for the new bootloader.
 * You can change this by using the Arduino IDE, selecting the Uno as board connect your Nano
 * And then run Burn Bootloader. This will not only leave the 0.5 kB new bootloader
 * but additionally this sets the right fuse settings which reserve only 0.5 kB program space.
 * I regularly do this for all Nano boards I have!
 *
 * With the new fuse settings you can just compile and upload the source for an Uno board even if you have connected the Nano board.
 */
// Save space for an unmodified Nano
#  if !defined(NO_CELL_STATISTICS)
#define NO_CELL_STATISTICS          // No cell values, cell minimum, maximum and percentages.
#  endif
#endif

// sStringBuffer is defined in JK-BMS_LCD.hpp if not ENABLE_MONITORING and NO_ANALYTICS are defined
#if defined(ENABLE_MONITORING)
char sStringBuffer[100];                 // For cvs lines, "Store computed capacity" line and LCD rows
#elif !defined(NO_ANALYTICS)
char sStringBuffer[40];                 // for "Store computed capacity" line, printComputedCapacity() and LCD rows
#endif

/*
 * Pin layout, may be adapted to your requirements
 */
#define BUZZER_PIN                             A2 // To signal errors
#define PAGE_SWITCH_DEBUG_BUTTON_PIN_FOR_INFO   2 // Button at INT0 / D2 for switching LCD pages - definition is not used in program, only for documentation.

// The standard RX of the Arduino is used for the JK_BMS_1 connection.
#define JK_BMS_RX_PIN_FOR_INFO                  0 // We use the Serial RX pin - definition is not used in program, only for documentation.
#if !defined(JK_BMS_TX_PIN)                       // Allow override by global symbol
#  if defined(USE_LAYOUT_FOR_644_BOARD)
#define JK_BMS_TX_PIN                          12
#  else
#define JK_BMS_TX_PIN                           4 // 115200 baud soft serial to JK-BMS. For serial from BMS we use the hardware Serial RX
#  endif
#endif
#if !defined(JK_BMS_TX_PIN_2)                     // Allow override by global symbol
#  if defined(USE_LAYOUT_FOR_644_BOARD)
#define JK_BMS_TX_PIN_2                        12 // use same pin for tests
#  else
#define JK_BMS_TX_PIN_2                         5 // 115200 baud soft serial to JK-BMS
#  endif
#endif

#if defined(USE_NO_COMMUNICATION_STATUS_LEDS)
#define TURN_BMS_STATUS_LED_ON                  void()
#define TURN_BMS_STATUS_LED_OFF                 void()
#define TURN_CAN_STATUS_LED_ON                  void()
#define TURN_CAN_STATUS_LED_OFF                 void()
#else
// BMS and CAN communication status LEDs
#  if !defined(BMS_COMMUNICATION_STATUS_LED_PIN)
#    if defined(USE_LAYOUT_FOR_644_BOARD)
#define BMS_COMMUNICATION_STATUS_LED_PIN        14
#define CAN_COMMUNICATION_STATUS_LED_PIN        15
#    else
#define BMS_COMMUNICATION_STATUS_LED_PIN        6
#define CAN_COMMUNICATION_STATUS_LED_PIN        7
#    endif
#  endif
#define TURN_BMS_STATUS_LED_ON                  digitalWriteFast(BMS_COMMUNICATION_STATUS_LED_PIN, HIGH)
#define TURN_BMS_STATUS_LED_OFF                 digitalWriteFast(BMS_COMMUNICATION_STATUS_LED_PIN, LOW)
#define TURN_CAN_STATUS_LED_ON                  digitalWriteFast(CAN_COMMUNICATION_STATUS_LED_PIN, HIGH)
#define TURN_CAN_STATUS_LED_OFF                 digitalWriteFast(CAN_COMMUNICATION_STATUS_LED_PIN, LOW)
#endif

#if !defined(NO_ANALYTICS)
#define DISABLE_ESR_IN_GRAPH_OUTPUT_PIN         8 // If this pin is held low, the ESR value is not used to correct the voltage in the graph output.
#endif

//#define TIMING_TEST
#if defined(TIMING_TEST)
#  if defined(USE_LAYOUT_FOR_644_BOARD)
#define TIMING_TEST_PIN                        13
#  else
#define TIMING_TEST_PIN                        10 // is SS pin for SPI and must be used as OUTPUT (set by SPI.init())!
#  endif
#endif

/*
 * The SPI pins for connection to CAN converter and the I2C / TWI pins for the LCD are determined by hardware.
 * For Uno / Nano:
 *   SPI: MOSI - 11, MISO - 12, SCK - 13. CS cannot be replaced by constant ground.
 *   I2C: SDA - A4, SCL - A5.
 */
#if defined(USE_LAYOUT_FOR_644_BOARD)
#define SPI_CS_PIN                              4 // !SS Must be specified before #include "MCP2515_TX.hpp"
#define SPI_MOSI_PIN_FOR_INFO                   5 // Definition is not used in program, only for documentation.
#define SPI_MISO_PIN_FOR_INFO                   6 // Definition is not used in program, only for documentation.
#define SPI_SCK_PIN_FOR_INFO                    7 // Definition is not used in program, only for documentation.
#else
#  if !defined(SPI_CS_PIN)                          // Allow override by global symbol
#define SPI_CS_PIN                              9 // Pin 9 is the default pin for the Arduino CAN bus shield. Alternately you can use pin 10 on this shield.
//#define SPI_CS_PIN                             10 // Must be specified before #include "MCP2515_TX.hpp"
#define SPI_MOSI_PIN_FOR_INFO                  11 // Definition is not used in program, only for documentation.
#define SPI_MISO_PIN_FOR_INFO                  12 // Definition is not used in program, only for documentation.
#define SPI_SCK_PIN_FOR_INFO                   13 // Definition is not used in program, only for documentation.
#  endif
#endif // defined(USE_LAYOUT_FOR_644_BOARD)

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
uint32_t sMillisOfLastRequestedJKDataFrame = -MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS; // Initial value to start first request immediately
bool sResponseFrameBytesAreExpected = false; // If true, request was recently sent

/*
 * Error beep behavior
 * Overvoltage error cannot be suppressed by a macro!
 * If NO NO_BEEP_ON_ERROR, ONE_BEEP_ON_ERROR or MULTIPLE_BEEPS_WITH_TIMEOUT are activated, we beep forever until error vanishes.
 */
//#define NO_BEEP_ON_ERROR              // If activated, Do not beep on error or timeout.
//#define ONE_BEEP_ON_ERROR             // If activated, only beep once if error was detected.
//#define BEEP_TIMEOUT_SECONDS    10L   // 10 seconds, Maximum is 254 seconds = 4 min 14 s
#define SUPPRESS_CONSECUTIVE_SAME_ALARMS // If activated, recurring alarms are suppressed, e.g. undervoltage alarm tends to come and go in a minute period
#if defined(SUPPRESS_CONSECUTIVE_SAME_ALARMS)
uint16_t sLastActiveAlarmsAsWord;   // Used to disable new alarm beep on recurring of same alarm
bool sPrintAlarmInfoOnlyOnce = false; // This forces display on Alarm page
#define SUPPRESS_CONSECUTIVE_SAME_ALARMS_TIMEOUT_SECONDS    3600 // Allow consecutive same alarms after 1 hour of no alarm
uint16_t sNoConsecutiveAlarmTimeoutCounter; // Counts no alarms in order to reset suppression of consecutive alarms.
#endif
#if !defined(NO_BEEP_ON_ERROR) && !defined(ONE_BEEP_ON_ERROR)
#define MULTIPLE_BEEPS_WITH_TIMEOUT     // If activated, beep for 1 minute if error was detected. Timeout is disabled if debug is active.
#endif
bool sAlarmOrTimeoutBeepActive = false;     // If true, we do an error beep at each end of the loop until beep timeout
#if defined(MULTIPLE_BEEPS_WITH_TIMEOUT)  // Beep one minute
#  if !defined(BEEP_TIMEOUT_SECONDS)
#define BEEP_TIMEOUT_SECONDS        60L     // 1 minute, Maximum is 254 seconds = 4 min 14 s
#  endif
uint32_t sFirstBeepMillis = 0;
#endif
#if defined(NO_BEEP_ON_ERROR) && defined(ONE_BEEP_ON_ERROR)
#error NO_BEEP_ON_ERROR and ONE_BEEP_ON_ERROR are both defined, which makes no sense!
#endif

#define MILLIS_IN_ONE_SECOND 1000L

/*
 * Function declarations
 */
void printReceivedData();
bool isVCCTooHighSimple();
void handleOvervoltage();
void handleBeep();
void processJK_BMSStatusFrame();
void handleFrameReceiveTimeout();

/*
 * Page button stuff
 *
 * Button at INT0 / D2 for switching LCD pages
 */
#if defined(ARDUINO_AVR_ATmega644)
#define USE_INT2_FOR_BUTTON_0
#endif
#define USE_BUTTON_0                 // Enable code for 1. button at INT0 / D2
#define BUTTON_DEBOUNCING_MILLIS 100 // With this you can adapt to the characteristic of your button. Default is 50.
#define NO_BUTTON_RELEASE_CALLBACK   // Disables the code for release callback. This saves 2 bytes RAM and 64 bytes program memory.
#include "EasyButtonAtInt01.hpp"

volatile bool sPageButtonJustPressed = false;
void handlePageButtonPress(bool aButtonToggleState);        // The button press callback function sets just a flag.
EasyButton PageSwitchButtonAtPin2(&handlePageButtonPress);  // Button is connected to INT0
#define LONG_PRESS_BUTTON_DURATION_MILLIS   1000
bool sCommunicationDebugModeActivated = false; // Is true for JK_BMS_PAGE_CAN_INFO
void checkButtonPress();

#include "digitalWriteFast.h"

/*
 * JK-BMS communication stuff
 */
#if !defined(MAXIMUM_NUMBER_OF_CELLS)
#define MAXIMUM_NUMBER_OF_CELLS     24 // Maximum number of cell info which can be converted. Must be before #include "JK-BMS.hpp".
#endif
#include "JK-BMS.hpp"

JK_BMS JK_BMS_1;
#if defined(HANDLE_MULTIPLE_BMS)
JK_BMS JK_BMS_2;
JK_BMS *sCurrentBMS = &JK_BMS_1;
#else
JK_BMS *const sCurrentBMS = &JK_BMS_1;
#endif
/*
 * CAN stuff
 */
#if !defined(NO_CAPACITY_35F_EXTENSIONS) // SMA Sunny Island inverters
#define CAPACITY_35F_EXTENSIONS // Add frame 0x35F for total capacity for SMA Sunny Island inverters, which is no problem for Deye inverters.
#endif
#if !defined(NO_CAPACITY_379_EXTENSIONS) // Luxpower SNA inverters
#define CAPACITY_379_EXTENSIONS // Add frame 0x379 for total capacity for Luxpower SNA inverters, which is no problem for Deye inverters.
#endif
#if !defined(NO_BYD_LIMITS_373_EXTENSIONS) // BYD
#define BYD_LIMITS_373_EXTENSIONS // Add frame 0x373 for cell limits as sent by BYD battery, which is no problem for Deye inverters.
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
 * Optional analytics stuff
 */
#if !defined(NO_ANALYTICS)
#include "JK-BMS_Analytics.hpp"
#endif

/*
 * Optional LCD stuff
 */
#if defined(USE_SERIAL_2004_LCD)
#include "JK-BMS_LCD.hpp"
#endif

/*
 * Miscellaneous
 */
bool sInitialActionsPerformed = false; // Flag to send static info only once after reset.

#include "LocalDebugLevelCheck.h"
// This definition must be located after the includes of other *.hpp files
//#define LOCAL_DEBUG // This enables debug output only for this file - only for development
#include "LocalDebugLevelStart.h" // no include "LocalDebugLevelEnd.h" required :-)

#if defined(STANDALONE_TEST)
const uint8_t TestJKReplyStatusFrame[] PROGMEM = { /* Header*/0x4E, 0x57, 0x01, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x01,
/*Length of Cell voltages*/
0x79, 0x30,
/*Cell voltages*/
0x01, 0x0C, 0xC6, 0x02, 0x0C, 0xBE, 0x03, 0x0C, 0xC7, 0x04, 0x0C, 0xC7, 0x05, 0x0C, 0xC7, 0x06, 0x0C, 0xC5, 0x07, 0x0C, 0xC6, 0x08,
        0x0C, 0xC7, 0x09, 0x0C, 0xC2, 0x0A, 0x0C, 0xC2, 0x0B, 0x0C, 0xC2, 0x0C, 0x0C, 0xC2, 0x0D, 0x0C, 0xC1, 0x0E, 0x0C, 0xBE,
        0x0F, 0x0C, 0xC1, 0x10, 0x0C, 0xC1,
        /*JKFrameAllDataStruct*/
        0x80, 0x00, 0x16, 0x81, 0x00, 0x15, 0x82, 0x00, 0x15, /*Voltage*/0x83, 0x14, 0x6C, /*Current*/0x84, 0x08, 0xD0, /*SOC*/0x85,
//        0x80, 0x00, 0x16, 0x81, 0x00, 0x15, 0x82, 0x00, 0x15, /*Voltage*/0x83, 0x14, 0x6C, /*Current*/0x84, 0x80, 0xD0, /*SOC*/0x85,
        0x47, 0x86, 0x02, 0x87, 0x00, 0x04, 0x89, 0x00, 0x00, 0x01, 0xE0, 0x8A, 0x00, 0x0E, /*Warnings*/0x8B, 0x00, 0x00, 0x8C,
        0x00, 0x07, 0x8E, 0x16, 0x26, 0x8F, 0x10, 0xAE, /*CellOvervoltageProtection*/0x90, 0x0F, 0xD2, 0x91, 0x0F, 0xA0, 0x92, 0x00,
        0x05,
        /*CellOvervoltageProtection*/0x93, 0x0B, 0xEA, 0x94, 0x0C, 0x1C, 0x95, 0x00, 0x05, 0x96, 0x01, 0x2C, 0x97, 0x00, 0x07, 0x98,
        0x00, 0x03, 0x99, 0x00, 0x05, 0x9A, 0x00, 0x05, 0x9B, 0x0C, 0xE4, 0x9C, 0x00, 0x08, 0x9D, 0x01, 0x9E, 0x00, 0x5A, 0x9F,
        0x00, 0x46, 0xA0, 0x00, 0x64, 0xA1, 0x00, 0x64, 0xA2, 0x00, 0x14, /*ChargeOvertemperature*/0xA3, 0x00, 0x46, /*DischargeOvertemperature*/
        0xA4, 0x00, 0x46, /*ChargeUndertemperature*/0xA5, 0xFF, 0xEC, 0xA6, 0xFF, 0xF6, /*DischargeUndertemperature*/0xA7, 0xFF,
        0xEC, 0xA8, 0xFF, 0xF6, 0xA9, 0x0E, /*TotalCapacityAmpereHour*/0xAA, 0x00, 0x00, 0x01, 0x40, 0xAB, 0x01, 0xAC, 0x01, 0xAD,
        0x04, 0x11, 0xAE, 0x01, 0xAF, 0x01, 0xB0, 0x00, 0x0A, 0xB1, 0x14, 0xB2, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x00, 0x00,
        0x00, 0x00, 0xB3, 0x00, 0xB4, 0x49, 0x6E, 0x70, 0x75, 0x74, 0x20, 0x55, 0x73, 0xB5, 0x32, 0x31, 0x30, 0x31, 0xB6, 0x00,
        0x06, 0x07, 0x29, /*Uptime*/0xB7, 0x31, 0x31, 0x2E, 0x58, 0x57, 0x5F, 0x53, 0x31, 0x31, 0x2E, 0x32, 0x36, 0x5F, 0x5F, 0x5F,
        0xB8, 0x00, /*ActualBatteryCapacityAmpereHour*/
        0xB9, 0x00, 0x00, 0x01, 0x30, 0xBA, 0x49, 0x6E, 0x70, 0x75, 0x74, 0x20, 0x55, 0x73, 0x65, 0x72, 0x64, 0x61, 0x4A, 0x4B,
        0x5F, 0x42, 0x32, 0x41, 0x32, 0x30, 0x53, 0x32, 0x30, 0x50, 0xC0, 0x01,
        /*Trailer*/
        0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x51, 0xC2 };

void doStandaloneTest();
void initializeEEPROMForTest();
#endif

/*
 * Helper macro for getting a macro definition as string
 */
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

const char StringStartingCANFailed[] PROGMEM = "Starting CAN failed!";

void setup() {
// LED_BUILTIN pin is used as SPI Clock !!!
//    pinModeFast(LED_BUILTIN, OUTPUT);
//    digitalWriteFast(LED_BUILTIN, LOW);

#if !defined(USE_NO_COMMUNICATION_STATUS_LEDS)
    pinModeFast(BMS_COMMUNICATION_STATUS_LED_PIN, OUTPUT);
    pinModeFast(CAN_COMMUNICATION_STATUS_LED_PIN, OUTPUT);
#endif
#if defined(TIMING_TEST)
    pinModeFast(TIMING_TEST_PIN, OUTPUT);
#endif
#if !defined(NO_ANALYTICS)
    pinModeFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN, INPUT_PULLUP);
#endif

    Serial.begin(115200);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

#if defined(USE_LAYOUT_FOR_644_BOARD)
    JK_INFO_PRINTLN(F("Settings are for Andres 644 board"));
#endif

#if defined(ENABLE_MONITORING)
    JK_INFO_PRINTLN(F("Monitoring enabled"));
#else
    JK_INFO_PRINTLN(F("Monitoring disabled"));
#endif

#if defined(NO_CELL_STATISTICS)
    JK_INFO_PRINTLN(F("Cell statistics deactivated"));
#else
    JK_INFO_PRINTLN(F("Cell statistics activated"));
#endif

#if defined(NO_ANALYTICS)
    JK_INFO_PRINTLN(F("Analytics deactivated"));
#else
    JK_INFO_PRINTLN(F("Analytics activated"));
    readBatteryESRfromEEPROM();
    JK_INFO_PRINT(F("EEPROM BatteryESR="));
    JK_INFO_PRINTLN(sBatteryESRMilliohm);

    findFirstSOCDataPointIndex();
#if defined(USE_LAYOUT_FOR_644_BOARD)
    JK_INFO_PRINT(F("EEPROM SOC data start index="));
    JK_INFO_PRINT(SOCDataPointsInfo.ArrayStartIndex);
    JK_INFO_PRINT(F(" length="));
    JK_INFO_PRINT(SOCDataPointsInfo.ArrayLength);
    JK_INFO_PRINT(F(", even="));
    JK_INFO_PRINTLN(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
#else
    DEBUG_PRINT(F("EEPROM SOC data start index="));
    DEBUG_PRINT(SOCDataPointsInfo.ArrayStartIndex);
    DEBUG_PRINT(F(" length="));
    DEBUG_PRINT(SOCDataPointsInfo.ArrayLength);
    DEBUG_PRINT(F(", even="));
    DEBUG_PRINTLN(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
#endif
    JK_INFO_PRINTLN(F("Disable ESR compensation pin=" STR(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN)));
    JK_INFO_PRINT(F("Battery ESR compensation for voltage "));
    if (digitalReadFast(DISABLE_ESR_IN_GRAPH_OUTPUT_PIN) == LOW) {
        JK_INFO_PRINT(F("dis"));
    } else {
        JK_INFO_PRINT(F("en"));
    }
    JK_INFO_PRINT(F("abled. Specified ESR="));
    JK_INFO_PRINT(sBatteryESRMilliohm);
    JK_INFO_PRINTLN(F("mOhm"));
#endif

    tone(BUZZER_PIN, 2200, 50);

#if defined(USE_SERIAL_2004_LCD)
    setupLCD();
#else
    JK_INFO_PRINTLN(F("LCD code deactivated"));
#endif

    /*
     * 115200 baud soft serial to JK-BMS. For serial from BMS we use the hardware Serial RX.
     */
    JK_BMS_1.init(JK_BMS_TX_PIN);
#if defined(HANDLE_MULTIPLE_BMS)
    JK_BMS_2.init(JK_BMS_TX_PIN_2);
    Serial.println(F("Serial to JK-BMS 1 + 2 started with 115.200 kbit/s at pins " STR(JK_BMS_TX_PIN) " + " STR(JK_BMS_TX_PIN_2)));
#else
    Serial.println(F("Serial to JK-BMS started with 115.200 kbit/s at pin " STR(JK_BMS_TX_PIN)));
#endif
#if defined(USE_SERIAL_2004_LCD)
    if (sSerialLCDAvailable) {
        myLCD.setCursor(0, 2);
        myLCD.print(F("BMS serial started"));
    }
#endif

    /*
     * CAN initialization
     */
    if (initializeCAN(CAN_BAUDRATE, MHZ_OF_CRYSTAL_ASSEMBLED_ON_CAN_MODULE, &Serial) == MCP2515_RETURN_OK) { // Resets the device and start the CAN bus at 500 kbps
        Serial.println(F("CAN started with 500 kbit/s!"));
#if defined(USE_SERIAL_2004_LCD)
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(F("CAN started"));
            delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the info
        }
#endif
    } else {
        Serial.println(reinterpret_cast<const __FlashStringHelper*>(StringStartingCANFailed));
#if defined(USE_SERIAL_2004_LCD)
        if (sSerialLCDAvailable) {
            myLCD.setCursor(0, 3);
            myLCD.print(reinterpret_cast<const __FlashStringHelper*>(StringStartingCANFailed));
#  if defined(STANDALONE_TEST)
            delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#  else
            delay(4 * LCD_MESSAGE_PERSIST_TIME_MILLIS); // To see the info
#  endif
        }
#endif

    }

    /*
     * Print debug pin info
     */
#if defined(USE_SERIAL_2004_LCD)
    Serial.println(F("Page switching button is at pin " STR(PAGE_BUTTON_PIN)));
    Serial.println(F("At long press, CAN Info page is entered and additional debug data is printed as long as button is pressed"));
#else
    Serial.println(F("Debug button is at pin " STR(PAGE_BUTTON_PIN)));
    Serial.println(F("Additional debug data is printed as long as button is pressed"));
#endif
    Serial.println(F(STR(MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) " ms between 2 BMS requests"));
    Serial.println(F(STR(MILLISECONDS_BETWEEN_CAN_FRAME_SEND) " ms between 2 CAN transmissions"));
#if defined(USE_SERIAL_2004_LCD) && !defined(DISPLAY_ALWAYS_ON)
    Serial.println(F("LCD Backlight timeout is " DISPLAY_ON_TIME_STRING));
#else
    Serial.println(F("No LCD Backlight timeout"));
#endif
    Serial.println();

#if defined(USE_SERIAL_2004_LCD)
    printDebugInfoOnLCD();
#endif

#if defined(STANDALONE_TEST)
    /*
     * Copy test data to receive buffer
     */
    Serial.println(F("Standalone test. Use fixed demo data"));
    Serial.println();
    memcpy_P(JKReplyFrameBuffer, TestJKReplyStatusFrame, sizeof(TestJKReplyStatusFrame));
    JK_BMS_1.ReplyFrameBufferIndex = sizeof(TestJKReplyStatusFrame) - 1;
    JK_BMS_1.printJKReplyFrameBuffer();
    Serial.println();
    sResponseFrameBytesAreExpected = false; // Enable CAN data sending
    JK_BMS_1.processReceivedData(); // sets sCANDataIsInitialized to true
    printReceivedData();
    /*
     * Copy complete reply and computed values for change determination
     */
    JK_BMS_1.lastJKReply.SOCPercent = JK_BMS_1.JKAllReplyPointer->SOCPercent;
    JK_BMS_1.lastJKReply.BatteryAlarmFlags.AlarmsAsWord = JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord;
    JK_BMS_1.lastJKReply.BMSStatus.StatusAsWord = JK_BMS_1.JKAllReplyPointer->BMSStatus.StatusAsWord;
    JK_BMS_1.lastJKReply.SystemWorkingMinutes = JK_BMS_1.JKAllReplyPointer->SystemWorkingMinutes;

//    if (true) {
    // first value is still written by processReceivedData
    if (SOCDataPointsInfo.ArrayLength > 250) {
        Serial.println(F("EEPROM for standalone test is already initialized"));
    } else {
        initializeEEPROMForTest();
    }

    doStandaloneTest();
#endif
}

void loop() {

    checkButtonPress();
#if defined(LOCAL_DEBUG)
    sCommunicationDebugModeActivated = true;
#endif

    /*
     * Request status frame every 2 seconds and check for response
     * Avoid overlapping requests
     */
    if (!sResponseFrameBytesAreExpected
            && millis() - sMillisOfLastRequestedJKDataFrame >= MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS) {
        sMillisOfLastRequestedJKDataFrame = millis(); // set for next check
        sCurrentBMS->RequestStatusFrame(sCommunicationDebugModeActivated);
        sResponseFrameBytesAreExpected = true; // Enable check for serial input of response
    }

#if defined(STANDALONE_TEST)
    JK_BMS_1.processReceivedData(); // call it multiple times for statistics
    writeSOCData(); // for analytics tests
    printBMSDataOnLCD(); // for switching between MAX and MIN display
    fillAllCANData(&JK_BMS_1);
    sCANDataIsInitialized = true; // One time flag
    delay(MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS); // do it simple :-)

#  if !defined(USE_NO_COMMUNICATION_STATUS_LEDS)
    // Simulate BMS reading
    digitalWriteFast(BMS_COMMUNICATION_STATUS_LED_PIN, HIGH);// Turn on status LED. LED is turned off at end of loop.
    delay(20); // do it simple :-)
    digitalWriteFast(BMS_COMMUNICATION_STATUS_LED_PIN, LOW); // Turn on status LED. LED is turned off at end of loop.
    delay(20); // do it simple :-)
#  endif

#  if defined(USE_SERIAL_2004_LCD) && !defined(DISPLAY_ALWAYS_ON)
    if (sSerialLCDAvailable) {
        doLCDBacklightTimeoutHandling();
    }
#  endif
#  if !defined(NO_BEEP_ON_ERROR) // Beep enabled
    handleBeep(); // Alarm / beep handling
#  endif
#else // defined(STANDALONE_TEST)

    if (sResponseFrameBytesAreExpected) {
        uint8_t tBMSCommunicationReturnCode = sCurrentBMS->checkForReplyFromBMSOrTimeout();
        /*
         * Check for reply from BMS or timeout
         */
        if (tBMSCommunicationReturnCode != JK_BMS_RECEIVE_ONGOING) {
            sResponseFrameBytesAreExpected = false;

            if (sCurrentBMS->TimeoutJustDetected) {
                handleFrameReceiveTimeout();
            }

            /*
             * Process BMS reply
             */
            if (tBMSCommunicationReturnCode == JK_BMS_RECEIVE_FINISHED) {
                TURN_BMS_STATUS_LED_ON;
                /*
                 * First compute some internal variables
                 */
                sCurrentBMS->processReceivedData();
                processJK_BMSStatusFrame(); // Process the complete receiving of the status frame

                /*******************************************************************
                 * Do this once after each complete status frame or timeout or error
                 *******************************************************************/
                /*
                 * Check for overvoltage
                 */
                while (isVCCTooHighSimple()) {
                    handleOvervoltage();
                }

#  if defined(USE_SERIAL_2004_LCD) && !defined(DISPLAY_ALWAYS_ON)
                if (sSerialLCDAvailable) {
                    doLCDBacklightTimeoutHandling();
                }
#  endif

#  if defined(HANDLE_MULTIPLE_BMS)
                if (sCurrentBMS->NumberOfThisBMS != JK_BMS::sBMS_ArrayNextIndex) {
                    TURN_BMS_STATUS_LED_OFF;
                    delay(200); // a hack
                    /*
                     * Switch to next BMS, NumberOfThisBMS is 1 greater than index :-)
                     */
                    sCurrentBMS = JK_BMS::BMSArray[sCurrentBMS->NumberOfThisBMS];
                    sCurrentBMS->RequestStatusFrame(sCommunicationDebugModeActivated);
                    sResponseFrameBytesAreExpected = true; // Enable check for serial input of response

                } else {
                    /*
                     * Fill CAN data after all BMS are processed
                     * First BMS is determines the CAN data except current, capacity and flags, which are added / OR'ed.
                     * and switch back to first BMS
                     */
                    fillAllCANData(JK_BMS::BMSArray[0]);
                    sCANDataIsInitialized = true; // One time flag

#  if !defined(NO_BEEP_ON_ERROR) // Beep enabled
                    handleBeep(); // Alarm / beep handling one per period
#  endif
                    sCurrentBMS = JK_BMS::BMSArray[0]; // switch back to first BMS
                }
#  else
                /*
                 * Fill CAN data
                 */
                fillAllCANData(sCurrentBMS);
                sCANDataIsInitialized = true; // One time flag

                processJK_BMSStatusFrame();// Process the complete receiving of the status frame
#  if !defined(NO_BEEP_ON_ERROR) // Beep enabled
                handleBeep(); // Alarm / beep handling
#  endif

#  endif
                TURN_BMS_STATUS_LED_OFF;
                sCommunicationDebugModeActivated = false; // Reset flag here.
            }
        }
    }
#endif // else defined(STANDALONE_TEST)

    /*
     * Send CAN frame independently of the period of JK-BMS data requests,
     * but do not send, if frame is currently requested and not completely received and processed.
     * 0.5 MB/s
     * Inverter reply every second: 0x305: 00-00-00-00-00-00-00-00
     * Do not send, if BMS is starting up, the 0% SOC during this time will trigger a deye error beep.
     */
#if defined(TRACE)
    Serial.print(F("sCANDataIsInitialized="));
    Serial.print(sCANDataIsInitialized);
    Serial.print(F(" BMSIsStarting="));
    Serial.print(sCurrentBMS->JKComputedData.BMSIsStarting);
    Serial.print(F(", sResponseFrameBytesAreExpected="));
    Serial.println(sResponseFrameBytesAreExpected);
#endif
    if (sCANDataIsInitialized && !sCurrentBMS->JKComputedData.BMSIsStarting && !sResponseFrameBytesAreExpected
            && millis() - sMillisOfLastCANFrameSent >= MILLISECONDS_BETWEEN_CAN_FRAME_SEND) {
        sMillisOfLastCANFrameSent = millis();

        TURN_CAN_STATUS_LED_ON;
        if (sCommunicationDebugModeActivated) {
            Serial.println(F("Send CAN"));
        }
        sendAllCANFrames(sCommunicationDebugModeActivated);
        TURN_CAN_STATUS_LED_OFF;
    }
}

void handleBeep() {
    bool tDoBeep = false;
    bool tAnyAlarmActive;
    bool tAnyTimeoutActive;
#if defined(HANDLE_MULTIPLE_BMS)
    // loop through all instantiated BMS
    tAnyAlarmActive = JK_BMS::getAnyAlarm();
#else
    tAnyAlarmActive = sCurrentBMS->AlarmActive;
    tAnyTimeoutActive = sCurrentBMS->JKBMSFrameHasTimeout;
#endif

#if defined(SUPPRESS_CONSECUTIVE_SAME_ALARMS)
    if (!tAnyAlarmActive) {
        /*
         * Check for 1 hour of no alarm, then reset suppression of same alarm
         */
        sNoConsecutiveAlarmTimeoutCounter++; // integer overflow does not really matter here
        if (sNoConsecutiveAlarmTimeoutCounter == (SUPPRESS_CONSECUTIVE_SAME_ALARMS_TIMEOUT_SECONDS * MILLIS_IN_ONE_SECOND) / (
#if defined(HANDLE_MULTIPLE_BMS)
                NUMBER_OF_SUPPORTED_BMS *
#endif
                        MILLISECONDS_BETWEEN_JK_DATA_FRAME_REQUESTS)) {
            sLastActiveAlarmsAsWord = NO_ALARM_WORD_CONTENT; // Reset LastActiveAlarmsAsWord
        }
    }

    if (sCurrentBMS->AlarmJustGetsActive) {
        sNoConsecutiveAlarmTimeoutCounter = 0; // initialize timeout counter
        // If new alarm is equal old one, beep only once
        if (sLastActiveAlarmsAsWord == sCurrentBMS->JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord) {
            // do only one beep for recurring alarms
            sCurrentBMS->AlarmJustGetsActive = false; // avoid evaluation below
            tDoBeep = true; // Beep only once in this loop
        } else {
            /*
             * Here at least one alarm is active and it is NOT the last alarm processed
             */
            sLastActiveAlarmsAsWord = sCurrentBMS->JKAllReplyPointer->BatteryAlarmFlags.AlarmsAsWord;
        }
    }
#endif
    /*
     * Check for BMS alarm flags
     */
    if (sCurrentBMS->AlarmJustGetsActive || sCurrentBMS->TimeoutJustDetected) {
        sCurrentBMS->AlarmJustGetsActive = false;
        sCurrentBMS->TimeoutJustDetected = false;
#if defined(ONE_BEEP_ON_ERROR)
        tDoBeep = true; // Beep only once in this loop
#else
        sAlarmOrTimeoutBeepActive = true; // enable beep until beep timeout
#endif
#if defined(MULTIPLE_BEEPS_WITH_TIMEOUT)
        sFirstBeepMillis = millis();
#endif
    }

#if defined(HANDLE_MULTIPLE_BMS)
    tAnyTimeoutActive = JK_BMS::getAnyTimeout();
#else
    tAnyTimeoutActive = sCurrentBMS->JKBMSFrameHasTimeout;
#endif
    if (!tAnyAlarmActive && !tAnyTimeoutActive) {
        sAlarmOrTimeoutBeepActive = false; // No more error, stop beeping
    }

    /*
     * Is beep requested for this loop ?
     */
#if !defined(ONE_BEEP_ON_ERROR)
    if (sAlarmOrTimeoutBeepActive) {
        tDoBeep = true; // Beep only once in this loop
    }
#endif

    if (tDoBeep) {
#if defined(MULTIPLE_BEEPS_WITH_TIMEOUT) // Beep one minute
        if ((millis() - sFirstBeepMillis) > (BEEP_TIMEOUT_SECONDS * 1000U)) {
            JK_INFO_PRINTLN(F("Timeout reached, suppress consecutive error beeps"));
            sAlarmOrTimeoutBeepActive = false; // disable further alarm beeps
        } else
#endif
        {
            tone(BUZZER_PIN, 2200);
            tone(BUZZER_PIN, 2200, 50);
            delay(200);
            tone(BUZZER_PIN, 2200, 50);
            delay(200); // to avoid tone interrupts waking us up from sleep
        }
    }
}

/*
 * Process the complete receiving of the status frame
 * Print data on LCD and Serial
 * Do initial actions like printing static values after boot
 * Print received buffer, if debug enabled
 */
void processJK_BMSStatusFrame() {
    if (sCommunicationDebugModeActivated) {
        /*
         * Print received buffer, if debug enabled
         */
        if (sCurrentBMS->ReplyFrameBufferIndex == 0) {
            Serial.println(F("sReplyFrameBufferIndex is 0"));
        } else {
            sCurrentBMS->printJKReplyFrameBuffer();
        }
    }

    if (!sInitialActionsPerformed) {
        /*
         * Do initialization once here
         */
#  if defined(HANDLE_MULTIPLE_BMS)
        // Print static info for both BMS
        if (sCurrentBMS->NumberOfThisBMS == 2) {
            sInitialActionsPerformed = true;
        }
#  else
        sInitialActionsPerformed = true;

#  endif
        Serial.println();
        sCurrentBMS->printJKStaticInfo();
#if !defined(NO_ANALYTICS)
#  if defined(HANDLE_MULTIPLE_BMS)
        if (sCurrentBMS->NumberOfThisBMS == 1) {
            initializeAnalytics();
        }
#  else
        initializeAnalytics();

#  endif
        JK_INFO_PRINTLN();
#endif
    }

    printReceivedData();
#if !defined(NO_ANALYTICS)
#  if defined(HANDLE_MULTIPLE_BMS)
    if (sCurrentBMS->NumberOfThisBMS == 1) {
        writeSOCData();
    }
#  else
    writeSOCData();
#  endif
#endif
}

/*
 * Here we have requested frame, but serial was not available fore a longer time => timeout at receiving
 * We are called only once per timeout
 */
void handleFrameReceiveTimeout() {
    Serial.print(F("Receive timeout"));
#if defined(HANDLE_MULTIPLE_BMS)
    Serial.print(F(" at BMS "));
    Serial.print(sCurrentBMS->NumberOfThisBMS);
#endif

    if (sCurrentBMS->ReplyFrameBufferIndex != 0) {
        // Print bytes received so far
        Serial.print(F(" at ReplyFrameBufferIndex="));
        Serial.println(sCurrentBMS->ReplyFrameBufferIndex);
        if (sCurrentBMS->ReplyFrameBufferIndex != 0) {
            sCurrentBMS->printJKReplyFrameBuffer();
        }
    }
    Serial.println();

    modifyAllCanDataToInactive();

#if defined(USE_SERIAL_2004_LCD)
    /*
     * Switch on LCD and print timeout message once in case of timeout detection
     */
    if (sSerialLCDAvailable) {
#  if !defined(DISPLAY_ALWAYS_ON)
        if (checkAndTurnLCDOn()) {
            Serial.println(F("BMS communication timeout")); // Reason for switching on LCD display
        }
#  endif
        printTimeoutMessageOnLCD();
    }
#endif
}

/*
 * Called exclusively by processJK_BMSStatusFrame()
 */
void printReceivedData() {
#if defined(USE_SERIAL_2004_LCD)
    if (sLCDDisplayPageNumber != JK_BMS_PAGE_CAPACITY_INFO) {
        // Do not interfere with plotter output
        sCurrentBMS->printJKDynamicInfo(); // Prints newline before SOC[%]=
    }
    printBMSDataOnLCD();
#else
    sCurrentBMS->printJKDynamicInfo();
#endif
}

/*
 * Callback handlers for button press
 * Just set flags for evaluation in checkButtonPress(), otherwise readButtonState() may again be false when checkButtonPress() is called
 */
void handlePageButtonPress(bool aButtonToggleState __attribute__((unused))) {
    sPageButtonJustPressed = true;
}

/*
 * Synchronously handle button press (called from loop)
 */
void checkButtonPress() {
#if defined(USE_SERIAL_2004_LCD)
    checkButtonPressForLCD();
#else
    /*
     * Treat Page button as Debug button
     */
    if (sPageButtonJustPressed) {
        sPageButtonJustPressed = false;
        sCommunicationDebugModeActivated = true; // Is set to false in loop
        Serial.println(F("One time debug print just activated"));
    } else if (PageSwitchButtonAtPin2.readDebouncedButtonState()) {
        // Button is still pressed
        sCommunicationDebugModeActivated = true; // Is set to false in loop
    }
#endif // defined(USE_SERIAL_2004_LCD)
}

#if defined(STANDALONE_TEST)
void initializeEEPROMForTest() {
    // If EEPROM empty, fill in some values
    int8_t tIncrement = -1;
    Serial.println();
    Serial.println();
    Serial.println(F("Now write initial data to EEPROM"));
    Serial.print(F("SOC="));
    Serial.println(JK_BMS_1.JKAllReplyPointer->SOCPercent);
    Serial.print(F("JKComputedData.Battery10MilliAmpere="));
    Serial.println(JK_BMS_1.JKComputedData.Battery10MilliAmpere);
    Serial.print(F("JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt="));
    Serial.println(JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt);

    for (int i = 0; i < 260; ++i) { // force buffer wrap around
        JK_BMS_1.JKAllReplyPointer->SOCPercent += tIncrement;
        writeSOCData(); // write too EEPROM here.

        // change accumulated value
        JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt += tIncrement * 4;

        Serial.print(JK_BMS_1.JKAllReplyPointer->SOCPercent);
        Serial.print(F(", "));

        // Test negative values
        if (JK_BMS_1.JKAllReplyPointer->SOCPercent == 2 && tIncrement < 0) {
            JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt = -10; // -100 mV
            Serial.print(F(" -100 mV "));
        }

        /*
         * Generate EEPROM data by calling writeSOCData(); multiple times for each SOC value
         */
        for (int j = 0; j < (i + 4) * 2; ++j) { // 320 corresponds to 2 Ah
            // accumulate values
            writeSOCData();
            JK_BMS_1.lastJKReply.SOCPercent = JK_BMS_1.JKAllReplyPointer->SOCPercent; // for transition detection from 0 to 1
        }

        // Manage 0% and 100 %
        if (JK_BMS_1.JKAllReplyPointer->SOCPercent == 0 || JK_BMS_1.JKAllReplyPointer->SOCPercent == 100) {
            if (JK_BMS_1.JKAllReplyPointer->SOCPercent == 100) {
                // Add additional capacity after 100%
                for (int j = 0; j < 8000; ++j) {
                    writeSOCData();
                }
            } else {
                // set difference voltage to 0
                JK_BMS_1.JKComputedData.BatteryVoltageDifferenceToEmpty10Millivolt = 0;
            }
            // reverse values at 0 and 100
            JK_BMS_1.JKComputedData.Battery10MilliAmpere = -JK_BMS_1.JKComputedData.Battery10MilliAmpere;
            tIncrement = -tIncrement;
        }
    }
    Serial.println();
    findFirstSOCDataPointIndex();
    Serial.print(F("EEPROM SOC data start index="));
    Serial.print(SOCDataPointsInfo.ArrayStartIndex);
    Serial.print(F(" length="));
    Serial.print(SOCDataPointsInfo.ArrayLength);
    Serial.print(F(", even="));
    Serial.println(SOCDataPointsInfo.currentlyWritingOnAnEvenPage);
}

/*
 * Assume, that MULTIPLE_BEEPS_WITH_TIMEOUT is active
 */
void testAlarmTimeout() {
    Serial.println(F("Test alarm timeout"));
    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.ChargeOvervoltageAlarm = true;
    JK_BMS_1.detectAndPrintAlarmInfo(); // this sets the LCD alarm string and flags
    Serial.println(F("Test alarm timeout. You should hear " STR(BEEP_TIMEOUT_SECONDS) " double beeps"));
    for (int i = 0; i < BEEP_TIMEOUT_SECONDS + 10; ++i) {
        handleBeep();
        delay(100);
    }
    // reset alarm
    JK_BMS_1.JKAllReplyPointer->BatteryAlarmFlags.AlarmBits.ChargeOvervoltageAlarm = false;
    JK_BMS_1.detectAndPrintAlarmInfo(); // this sets the LCD alarm string and flags
    Serial.println(F("End of test alarm timeout"));

}

void doStandaloneTest() {

#  if defined(ALARM_TIMEOUT_TEST)
     testAlarmTimeout();
#endif
#  if defined(LCD_PAGES_TEST)
    if (sSerialLCDAvailable) {
        testLCDPages();
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        testPrintFloatValueRightAlignedOnLCD();
        delay(2*LCD_MESSAGE_PERSIST_TIME_MILLIS);
#    if defined(BIG_NUMBER_TEST)
        testBigNumbers();
#    endif
        memcpy_P(JKReplyFrameBuffer, TestJKReplyStatusFrame, sizeof(TestJKReplyStatusFrame));
        processReceivedData(); // to clear every changes
    }
#  endif
}

#endif

void handleOvervoltage() {
#if defined(USE_SERIAL_2004_LCD)
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

#if !defined(_ADC_UTILS_HPP)
#include "ADCUtils.h"
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
    ADMUX = ADC_1_1_VOLT_CHANNEL_MUX | (DEFAULT << SHIFT_VALUE_FOR_REFERENCE);
// ADCSRB = 0; // Only active if ADATE is set to 1.
// ADSC-StartConversion ADIF-Reset Interrupt Flag - NOT free running mode
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADIF) | ADC_PRESCALE128); //  128 -> 104 microseconds per ADC conversion at 16 MHz --- Arduino default
// wait for single conversion to finish
    loop_until_bit_is_clear(ADCSRA, ADSC);

// Get value
    uint16_t tRawValue = ADCL | (ADCH << 8);

    return tRawValue < 1126000 / VCC_OVERVOLTAGE_THRESHOLD_MILLIVOLT; // < 214
}
#endif // _ADC_UTILS_HPP
