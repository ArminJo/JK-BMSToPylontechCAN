/*
SoftwareSerialTX.h (from SoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Transmit-only imoplementation
-- reduce footprint in code memory and RAM compared to SoftwareSerial
   ~ 668 byte code
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

#if defined(__AVR_ATmega168__) ||defined(__AVR_ATmega168P__) ||defined(__AVR_ATmega328P__)
// 
// Includes
// 
#include <Arduino.h>
#include <util/delay_basic.h>
#include "SoftwareSerialTX.h"

//
// Constructor
//
SoftwareSerialTX::SoftwareSerialTX(uint8_t transmitPin) 
{
  setTX(transmitPin);
}

size_t SoftwareSerialTX::write(uint8_t b)
{
  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile uint8_t *reg = _transmitPortRegister;
  uint8_t reg_mask = _transmitBitMask;
  uint8_t inv_mask = ~_transmitBitMask;
  uint8_t oldSREG = SREG;
  uint16_t delay = _tx_delay;

  cli();  // turn off interrupts for a clean txmit

  // Write the start bit
  *reg &= inv_mask;

  _delay_loop_2(delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
      *reg |= reg_mask; // send 1
    else
      *reg &= inv_mask; // send 0

    _delay_loop_2(delay);
    b >>= 1;
  }

  // restore pin to natural state
  *reg |= reg_mask;

  SREG = oldSREG; // turn interrupts back on
  _delay_loop_2(delay);
  
  return 1;
}

void SoftwareSerialTX::begin(long speed)
{
  uint16_t bit_delay = (F_CPU / speed) / 4;
  //_rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;
  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  _tx_delay = subtract_cap(bit_delay, 15 / 4);
}

uint16_t SoftwareSerialTX::subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

void SoftwareSerialTX::setTX(uint8_t tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output high. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, HIGH);
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  _transmitPortRegister = portOutputRegister(digitalPinToPort(tx));
}

size_t SoftwareSerialTX::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--) {
    if (write(*buffer++)) n++;
    else break;
  }
  return n;
}
#else
#error unsupported platform
#endif