/*
 * MCP2515_TX.h
 *
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
 */

#ifndef _MCP2515_TX_H
#define _MCP2515_TX_H

#include <inttypes.h>

bool initializeCAN(uint32_t aBaudrate, uint8_t aCrystalMHz); // Return true if error happens
bool sendCANMessage(uint16_t aCANId, uint8_t aLengthOfBuffer, const uint8_t *aSendDataBufferPointer); // Return true if error happens
#endif // _MCP2515_TX_H
