/*
 * AVRUtils.h
 *
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
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
 *
 */
#ifndef _AVR_UTILS_H
#define _AVR_UTILS_H

#if defined(__AVR__)
#include <stdint.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/*
 * storage for millis value to enable compensation for interrupt disable at signal acquisition etc.
 */
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)  || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#  if !defined(_MILLIS_UTILS_H)
#define timer0_millis millis_timer_millis // The ATTinyCore libraries use other variable name in wiring.c - copied from MillisUtils.h
#  endif
#  if !defined(DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN)
#define DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN  65
#  endif
#else
#  if !defined(DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN)
#define DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN  0   // default for Uno / Nano etc.
#  endif
#endif

extern volatile unsigned long timer0_millis;

void initSleep(uint8_t tSleepMode);
void initPeriodicSleepWithWatchdog(uint8_t tSleepMode, uint8_t aWatchdogPrescaler);
uint16_t computeSleepMillis(uint8_t aWatchdogPrescaler);
void sleepWithWatchdog(uint8_t aWatchdogPrescaler, bool aAdjustMillis = false);
extern volatile uint16_t sNumberOfSleeps;

#include <Print.h>

uint8_t* getHeapStart();
uint16_t getCurrentFreeHeapOrStack(void);
uint16_t getCurrentAvailableHeap(void);
void printHeapStart(Print *aSerial);
void printCurrentFreeHeap(Print *aSerial);
void printCurrentAvailableHeap(Print *aSerial);

#define HEAP_STACK_UNTOUCHED_VALUE 0x5A
void initStackFreeMeasurement();
uint16_t getStackUnusedBytes();
uint16_t getStackUsedBytes();
uint16_t getStackUnusedAndUsedBytes(uint16_t *aStackUsedBytesPointer);
void printStackUsedBytes(Print *aSerial);
void printStackUnusedAndUsedBytes(Print *aSerial);
void printStackUnusedAndUsedBytesIfChanged(Print *aSerial);

void printRAMInfo(Print *aSerial);

bool isAddressInRAM(void *aAddressToCheck);
bool isAddressBelowHeap(void *aAddressToCheck);

#endif //  defined(__AVR__)
#endif // _AVR_UTILS_H
