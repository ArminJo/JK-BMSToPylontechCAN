/*
SoftwareSerialTX.h (from SoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Transmit-only imoplementation
-- reduce footprint in code memory and RAM compared to SoftwareSerial
   ~ 686 byte code
   ~ 68 byte RAM

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

#ifndef SoftwareSerialTX_h
#define SoftwareSerialTX_h

#include <inttypes.h>

class SoftwareSerialTX
{
private:
  // per object data
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;
  uint16_t _tx_delay;

  // private methods
  void setTX(uint8_t transmitPin);

#ifndef ARDUINO_RASPBERRY_PI_PICO
  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);
#endif

public:
  // public methods
  SoftwareSerialTX(uint8_t transmitPin);
  void begin(long speed);
  size_t write(uint8_t byte);
  size_t write(const uint8_t *buffer, size_t size);
};

#endif
