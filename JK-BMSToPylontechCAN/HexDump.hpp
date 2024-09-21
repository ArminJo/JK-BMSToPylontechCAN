/*
 * HexDump.hpp
 * Hex memory dump utility functions for Arduino.
 * 0x00 and 0xFF are printed as spaces, values above 0x80 are printed as '.'.
 *
 * Sample output:
 * 0x0000:  0xF1 0x81 0x82 0x00 0x08 0x02 0x00 0x27 0xFF 0xFF 0x0E 0xB3 0x81 0xFC 0x9B 0x47  ... .. '  .....G
 * 0x0020:  0x00 0x00 0x00 0x00 0x20 0x65 0x00 0x0F 0xBE 0xEB 0x9B 0x98 0x2C 0xF1 0x08 0x2C       e .....,..,
 *
 *  Copyright (C) 2022-2024  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
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
 */

#ifndef _HEX_DUMP_HPP
#define _HEX_DUMP_HPP

#include <Arduino.h>

#define _16_BYTES_PER_LINE  16

#define HEX_DUMP_FORMAT_16_BIT_ABSOLUTE_ADDRESS 0x00 // Print 16 bit absolute address
#define HEX_DUMP_FORMAT_NO_ADDRESS_AT_ALL       0x01 // Bit 0: else print
#define HEX_DUMP_FORMAT_RELATIVE_ADDRESS        0x02 // Bit 1: else absolute address
#define HEX_DUMP_FORMAT_8_BIT_ADDRESS           0x04 // Bit 2: else 16 bit Address
#define HEX_DUMP_FORMAT_ASCII_VALUES            0x08 // default

void printBufferHex(uint8_t *aBufferAddress, uint16_t aNumberOfBytesToPrint);
void printBufferHexDump(uint8_t *aBufferAddress, uint16_t aNumberOfBytesToPrint);
void printBufferHexAndASCIIDump(uint8_t *aBufferAddress, uint16_t aNumberOfBytesToPrint);
void printMemoryHexDump(uint8_t *aMemory, uint16_t aSizeOfMemoryToPrint, uint8_t aBytesPerLine = _16_BYTES_PER_LINE,
        uint8_t aFormatFlags = HEX_DUMP_FORMAT_ASCII_VALUES);
void printBytePaddedHex(uint8_t aHexValueToPrint);
void printWordPaddedHex(uint16_t aHexValueToPrint);
/*
 * Print short address and hex bytes without ASCII representation
 */
void printBufferHex(uint8_t *aBufferAddress, uint16_t aNumberOfBytesToPrint) {
    printMemoryHexDump(aBufferAddress, aNumberOfBytesToPrint, _16_BYTES_PER_LINE, HEX_DUMP_FORMAT_NO_ADDRESS_AT_ALL);
}
void printBufferHexDump(uint8_t *aBufferAddress, uint16_t aNumberOfBytesToPrint) {
    printMemoryHexDump(aBufferAddress, aNumberOfBytesToPrint, _16_BYTES_PER_LINE,
    HEX_DUMP_FORMAT_8_BIT_ADDRESS | HEX_DUMP_FORMAT_RELATIVE_ADDRESS);
}
void printBufferHexAndASCIIDump(uint8_t *aBufferAddress, uint16_t aNumberOfBytesToPrint) {
    printMemoryHexDump(aBufferAddress, aNumberOfBytesToPrint, _16_BYTES_PER_LINE,
    HEX_DUMP_FORMAT_8_BIT_ADDRESS | HEX_DUMP_FORMAT_RELATIVE_ADDRESS | HEX_DUMP_FORMAT_ASCII_VALUES);
}
/**
 * Prints lines of memory content
 * 0x0000:  0xF1 0x81 0x82 0x00 0x08 0x02 0x00 0x27 0xFF 0xFF 0x0E 0xB3 0x81 0xFC 0x9B 0x47  ... .. '  .....G
 * @param aNumberOfBytesToPrint     Number of lines to print are: (aSizeOfMemoryToPrint/BYTES_PER_LINE) + 1
 * @param aBytesPerLine             Number of bytes in one line
 * @param aFormatFlags              See definitions above
 */
void printMemoryHexDump(uint8_t *aMemory, uint16_t aNumberOfBytesToPrint, uint8_t aBytesPerLine, uint8_t aFormatFlags) {
    uint16_t tIndex = 0;
    while (true) {
        if (aBytesPerLine > aNumberOfBytesToPrint) {
            // last line
            aBytesPerLine = aNumberOfBytesToPrint;
        }

        if (aBytesPerLine == 0) {
            break;
        } else {
            aNumberOfBytesToPrint -= aBytesPerLine;
            if ((aFormatFlags & HEX_DUMP_FORMAT_NO_ADDRESS_AT_ALL) == 0) {
                /*
                 * Print address in different formats
                 */
                uint16_t tAddress = tIndex;
                if ((aFormatFlags & HEX_DUMP_FORMAT_RELATIVE_ADDRESS) == 0) {
                    tAddress += (uint16_t) aMemory;
                }
                if ((aFormatFlags & HEX_DUMP_FORMAT_8_BIT_ADDRESS) != 0) {
                    printBytePaddedHex(tAddress);
                } else {
                    printWordPaddedHex(tAddress);
                }
                Serial.print(F(": "));
            }

            /*
             * print hex bytes
             */
            for (uint_fast8_t i = 0; i < aBytesPerLine; i++) {
                printBytePaddedHex(aMemory[tIndex + i]);
            }

            if ((aFormatFlags & HEX_DUMP_FORMAT_ASCII_VALUES) != 0) {
                /*
                 * print bytes ASCII representation
                 */
                Serial.print(F("  "));
                for (uint_fast8_t i = 0; i < aBytesPerLine; i++) {
                    uint8_t tCharacterToPrint = aMemory[tIndex + i];
//            if(isalnum(tIndex+i)){ // requires 40 bytes more program space
                    if (' ' <= tCharacterToPrint && tCharacterToPrint <= '~') {
                        Serial.print((char)tCharacterToPrint);
                    } else if (tCharacterToPrint != 0x00 && tCharacterToPrint != 0xFF) {
                        // for non printable characters except 0 and FF
                        Serial.print('.');
                    } else {
                        Serial.print(' ');
                    }
                }
            }
            Serial.println();
            tIndex += aBytesPerLine;
        }
    }
}

/*
 * Print with leading space and padded with 0
 */
void printBytePaddedHex(uint8_t aHexValueToPrint) {
    Serial.print(F(" 0x"));
    if (aHexValueToPrint < 0x10) {
        Serial.print('0');
    }
    Serial.print(aHexValueToPrint, HEX);
}

void printWordPaddedHex(uint16_t aHexValueToPrint) {
    Serial.print(F("0x"));
    if (aHexValueToPrint < 0x1000) {
        Serial.print('0');
    }
    if (aHexValueToPrint < 0x100) {
        Serial.print('0');
    }
    if (aHexValueToPrint < 0x10) {
        Serial.print('0');
    }
    Serial.print(aHexValueToPrint, HEX);

}
#endif // _HEX_DUMP_HPP
