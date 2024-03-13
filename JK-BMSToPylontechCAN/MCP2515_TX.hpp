/*
 * MCP2515_TX.hpp
 *
 * Functions to control send only functions for MCP2515 CAN controller
 *
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

#ifndef _MCP2515_TX_HPP
#define _MCP2515_TX_HPP

#include "SPI_.h"
#include "mcp2515_can_dfs.h"
#include "digitalWriteFast.h"

SPISettings sSPISettings(4000000, MSBFIRST, SPI_MODE0);
#if !defined SPI_CS_PIN
#define SPI_CS_PIN   9 // Pin 9 seems to be the default pin for the Arduino CAN bus shield. Alternately you can use pin 10 on this shield
#endif

#define MCP2515_RETURN_OK                           false

#define MCP2515_CAN_CONTROL_REGISTER_CONTENT        MODE_NORMAL // default mode
//#define MCP2515_CAN_CONTROL_REGISTER_CONTENT        MODE_ONESHOT | CLKOUT_ENABLE; // Alternative mode with no resending and clock output at pin 3

void resetMCP2515(void) {
    SPI.beginTransaction(sSPISettings);
    digitalWriteFast(SPI_CS_PIN, LOW);
    SPI.transfer(0xc0);
    digitalWriteFast(SPI_CS_PIN, HIGH);
    SPI.endTransaction();
    delayMicroseconds(10);
}

uint8_t readMCP2515Register(uint8_t address) {
    uint8_t value;

    SPI.beginTransaction(sSPISettings);
    digitalWriteFast(SPI_CS_PIN, LOW);
    SPI.transfer(0x03);
    SPI.transfer(address);
    value = SPI.transfer(0x00);
    digitalWriteFast(SPI_CS_PIN, HIGH);
    SPI.endTransaction();

    return value;
}

void modifyMCP2515Register(uint8_t address, uint8_t mask, uint8_t value) {
    SPI.beginTransaction(sSPISettings);
    digitalWriteFast(SPI_CS_PIN, LOW);
    SPI.transfer(0x05);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(value);
    digitalWriteFast(SPI_CS_PIN, HIGH);
    SPI.endTransaction();
}

void writeMCP2515Register(uint8_t address, uint8_t value) {
    SPI.beginTransaction(sSPISettings);
    digitalWriteFast(SPI_CS_PIN, LOW);
    SPI.transfer(0x02);
    SPI.transfer(address);
    SPI.transfer(value);
    digitalWriteFast(SPI_CS_PIN, HIGH);
    SPI.endTransaction();
}

const char StringAccessFailed[] PROGMEM = " access to MCP2515 config mode register failed";

/*
 * return true if error happens
 */
bool initializeCAN(uint32_t aBaudrate, uint8_t aCrystalMHz, Print *aSerial) { // Using Print class saves 95 bytes flash
    pinModeFast(SPI_CS_PIN, OUTPUT);

    SPI.begin(); // start SPI

    resetMCP2515(); // Reset MCP2515

    // Set Configuration mode
    writeMCP2515Register(MCP_CANCTRL, MODE_CONFIG);
    if (readMCP2515Register(MCP_CANCTRL) != MODE_CONFIG) {
        if(aSerial != NULL) {
            aSerial->print(F("First"));
            aSerial->println(reinterpret_cast<const __FlashStringHelper *>(StringAccessFailed));
        }
        return true;
    }

    /*
     * Set timing for 16 MHz crystal and 500 kBit/s
     */
    if (aBaudrate == 500000) {
        /*
         * Set timing for 16 MHz crystal and 500 kBit/s
         */
        if (aCrystalMHz == 20) {
            writeMCP2515Register(MCP_CNF1, MCP_20MHz_500kBPS_CFG1); // Baud Rate Prescaler
            writeMCP2515Register(MCP_CNF2, MCP_20MHz_500kBPS_CFG2); // 0x80 is BTLMODE and always set
            writeMCP2515Register(MCP_CNF3, MCP_20MHz_500kBPS_CFG3);
        } else if (aCrystalMHz == 16) {
            writeMCP2515Register(MCP_CNF1, MCP_16MHz_500kBPS_CFG1); // Baud Rate Prescaler
            writeMCP2515Register(MCP_CNF2, MCP_16MHz_500kBPS_CFG2); // 0x80 is BTLMODE and always set
            writeMCP2515Register(MCP_CNF3, MCP_16MHz_500kBPS_CFG3);
        } else {
            if(aSerial != NULL) {
                aSerial->println(F("500 kB is not working stable with 8 MHz crystal"));
            }
            return true;
        }
    } else if (aBaudrate == 250000) {
        if (aCrystalMHz == 16) {
            writeMCP2515Register(MCP_CNF1, MCP_16MHz_250kBPS_CFG1); // Baud Rate Prescaler
            writeMCP2515Register(MCP_CNF2, MCP_16MHz_250kBPS_CFG2); // 0x80 is BTLMODE and always set
            writeMCP2515Register(MCP_CNF3, MCP_16MHz_250kBPS_CFG3);
        } else {
            writeMCP2515Register(MCP_CNF1, MCP_8MHz_250kBPS_CFG1); // Baud Rate Prescaler
            writeMCP2515Register(MCP_CNF2, MCP_8MHz_250kBPS_CFG2); // 0x80 is BTLMODE and always set
            writeMCP2515Register(MCP_CNF3, MCP_8MHz_250kBPS_CFG3);
        }
        /*
         * Other baud rates and crystal combinations can be added like above
         * The compiler optimizer only uses the active code :-)
         */
    }

    // Reset Configuration mode
    writeMCP2515Register(MCP_CANCTRL, MCP2515_CAN_CONTROL_REGISTER_CONTENT);
    if (readMCP2515Register(MCP_CANCTRL) != MCP2515_CAN_CONTROL_REGISTER_CONTENT) {
        if(aSerial != NULL) {
            aSerial->println(F("Last"));
            aSerial->println(reinterpret_cast<const __FlashStringHelper *>(StringAccessFailed));
        }
        return true;
    }

    return false;
}

/*
 * return true if error happens
 */
bool sendCANMessage(uint16_t aCANId, uint8_t aLengthOfBuffer, const uint8_t *aSendDataBufferPointer) {

    /*
     * We use transmit buffer 0
     */
    writeMCP2515Register(MCP_TXB0SIDH, aCANId >> 3); // write bit 3:10 of ID
    writeMCP2515Register(MCP_TXB0SIDL, aCANId << 5); // write bit 0:2 and flag "no extended"
    writeMCP2515Register(MCP_TXB0DLC, aLengthOfBuffer);

    // Fill buffer
    for (uint_fast8_t i = 0; i < aLengthOfBuffer; i++) {
        writeMCP2515Register(MCP_TXB0D0 + i, aSendDataBufferPointer[i]);
    }

    writeMCP2515Register(MCP_TXB0CTRL, MCP_TXB_TXREQ_M);

    /*
     * Check for end of transmission, and if an error happened
     */
    while (readMCP2515Register(MCP_TXB0CTRL) & MCP_TXB_TXREQ_M) {
        if (readMCP2515Register(MCP_TXB0CTRL) & (MCP_TXB_TXERR_M | MCP_TXB_MLOA_M | MCP_TXB_ABTF_M)) {
            /*
             * Error happened here, abort transfer. First retransmit is still pending!
             */
            writeMCP2515Register(MCP_CANCTRL, ABORT_TX | MCP2515_CAN_CONTROL_REGISTER_CONTENT); // Set "Abort All Pending Transmissions" bit
            delayMicroseconds(10);
            writeMCP2515Register(MCP_CANCTRL, MCP2515_CAN_CONTROL_REGISTER_CONTENT); // Reset "Abort All Pending Transmissions" bit
            return true; // Error
        }
    }

    return false;
}
#endif // _MCP2515_TX_HPP
