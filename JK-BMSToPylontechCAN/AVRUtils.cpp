/*
 *  AVRUtils.cpp
 *
 *  Stack, Ram and Heap utilities.
 *  Sleep utilities.
 *
 *  Copyright (C) 2016-2023  Armin Joachimsmeyer
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

#if defined(__AVR__) && !(defined(__AVR_ATtiny1616__)  || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny3217__))
#include "AVRUtils.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdlib.h> // for __malloc_margin
/*
 * The largest address just not allocated so far
 * Under Unix, the "break value" was the end of the data
 * segment as dynamically requested from the operating system.
 * Since we don't have an operating system, just make sure
 * that we don't collide with the stack.
 */
extern void *__brkval; // The largest address just not allocated so far

/*
 * Returns actual start of free heap
 * Usage for print:
 Serial.print(F("HeapStart=0x"));
 Serial.println((uintptr_t) getHeapStart(), HEX);
 */
uint8_t* getHeapStart(void) {
    if (__brkval == 0) {
        __brkval = __malloc_heap_start;
    }
    return (uint8_t*) __brkval;
}

/*
 * Initialize RAM between current stack and actual heap start (__brkval) with pattern 0x5A
 */
void initStackFreeMeasurement() {
    uint8_t tDummyVariableOnStack;
    uint8_t *tHeapPtr = getHeapStart();

// Fill / paint stack
    do {
        *tHeapPtr++ = HEAP_STACK_UNTOUCHED_VALUE;
    } while (tHeapPtr < &tDummyVariableOnStack);
}

/*
 * Returns the amount of stack/heap not used/touched since the last call to initStackFreeMeasurement()
 * by check for first touched pattern on the stack/heap, starting the UPWARD search at heap start.
 * Sets the variable aStackUsedBytesPointer points to with amount of used/touched bytes.
 *
 * This fails, if a HEAP_STACK_UNTOUCHED_VALUE was written in a former malloced block.
 */
uint16_t getStackUnusedAndUsedBytes(uint16_t *aStackUsedBytesPointer) {
    uint8_t tDummyVariableOnStack;
    uint8_t *tHeapPtr = getHeapStart();

// first search for first untouched value after current begin of heap, because malloc() and free() may be happened in between and overwrite low memory
    while (*tHeapPtr != HEAP_STACK_UNTOUCHED_VALUE && tHeapPtr < &tDummyVariableOnStack) {
        tHeapPtr++;
    }
// then count untouched patterns
    uint16_t tStackUnused = 0;
    while (*tHeapPtr == HEAP_STACK_UNTOUCHED_VALUE && tHeapPtr < &tDummyVariableOnStack) {
        tHeapPtr++;
        tStackUnused++;
    }
    *aStackUsedBytesPointer = ((RAMEND + 1) - (uint16_t) tHeapPtr);

    return tStackUnused;
}

/*
 * Returns the amount of stack/heap touched since the last call to initStackFreeMeasurement()
 * by check for first non touched pattern on the stack/heap, starting the DOWNWARD search at current stack pointer.
 *
 * If the end of former malloced and written memory was higher than current stackpointer,
 * the memory area is taken as overwritten stack. Therefore we may return values, which are too big.
 * But nevertheless, this constellation is likely a potential programming problem!
 */
uint16_t getStackUsedBytes() {
    uint8_t tDummyVariableOnStack;
    uint8_t *tHeapStart = getHeapStart();
    uint8_t *tSearchPtr = &tDummyVariableOnStack;

    // Search for first untouched value below current stackpointer
    while (*tSearchPtr != HEAP_STACK_UNTOUCHED_VALUE && tSearchPtr > tHeapStart) {
        tSearchPtr--;
    }

    return (RAMEND + 1) - (uint16_t) tSearchPtr;
}

/*
 * Returns the amount of stack/heap not touched since the last call to initStackFreeMeasurement()
 * by check for first touched pattern on the stack/heap, starting the search at heap start.
 */
uint16_t getStackUnusedBytes() {
    uint8_t tDummyVariableOnStack;
    uint8_t *tHeapPtr = getHeapStart();

// first search for first match after current begin of heap, because malloc() and free() may be happened in between and overwrite low memory
    while (*tHeapPtr != HEAP_STACK_UNTOUCHED_VALUE && tHeapPtr < &tDummyVariableOnStack) {
        tHeapPtr++;
    }
// then count untouched patterns
    uint16_t tStackUnused = 0;
    while (*tHeapPtr == HEAP_STACK_UNTOUCHED_VALUE && tHeapPtr < &tDummyVariableOnStack) {
        tHeapPtr++;
        tStackUnused++;
    }
    return tStackUnused;
}

/*
 * Get amount of free RAM = current stackpointer - heap end
 */
uint16_t getCurrentFreeHeapOrStack(void) {
    uint16_t tHeapStart = (uint16_t) getHeapStart();
    if (tHeapStart >= SP) {
        return 0;
    }
    return (SP - (uint16_t) getHeapStart());
}

/*
 * Get amount of maximum available memory for malloc()
 * FreeRam - __malloc_margin (128 for ATmega328)
 */
uint16_t getCurrentAvailableHeap(void) {
    if (getCurrentFreeHeapOrStack() <= __malloc_margin) {
        return 0;
    }
    return getCurrentFreeHeapOrStack() - __malloc_margin; // (128)
}

void printCurrentAvailableHeap(Print *aSerial) {
    aSerial->print(F("Currently max available Heap[bytes]="));
    aSerial->println(getCurrentAvailableHeap());
}

void printHeapStart(Print *aSerial) {
    aSerial->print(F("Heap start="));
    aSerial->println((uint16_t) getHeapStart());
}

/*
 * Prints the amount of stack used/touched since the last call to initStackFreeMeasurement()
 * Example: "Stack used 20 of 7" up to "Stack used 20 of 20" if stack runs into data
 */
void printStackUsedBytes(Print *aSerial) {
    aSerial->print(F("Stack used "));
    aSerial->print(getStackUsedBytes());
    aSerial->print(F(" of "));
    aSerial->println((RAMEND + 1) - (uint16_t) getHeapStart());
}

/*
 * Prints the amount of stack NOT used/touched and used/touched since the last call to initStackFreeMeasurement()
 * Example: "Stack unused=0, used=16" if stack runs into data
 */
void printStackUnusedAndUsedBytes(Print *aSerial) {
    uint16_t tStackUsedBytes;
    aSerial->print(F("Stack unused="));
    aSerial->print(getStackUnusedAndUsedBytes(&tStackUsedBytes));
    aSerial->print(F(", used="));
    aSerial->println(tStackUsedBytes);
}

/*
 * Prints the amount of stack NOT used/touched and used/touched since the last call to initStackFreeMeasurement()
 * Print only if value changed.
 */
void printStackUnusedAndUsedBytesIfChanged(Print *aSerial) {
    static uint16_t sStackUsedBytes = 0;

    uint16_t tOldStackUsedBytes = sStackUsedBytes;
    uint16_t tStackUnusedBytes = getStackUnusedAndUsedBytes(&sStackUsedBytes);
    if (tOldStackUsedBytes != sStackUsedBytes) {
        aSerial->print(F("Stack unused="));
        aSerial->print(tStackUnusedBytes);
        aSerial->print(F(", used="));
        aSerial->println(sStackUsedBytes);
    }
}

/*
 * RAM starts of variables initialized with values != 0,
 * followed by variables initialized with 0
 * and variables not initialized by using attribute "__attribute__((section(".noinit")))".
 * It ends with the heap and the stack.
 *
 * Sample output if stack runs into data:
 * Size of Data + BSS, Heap start, Stack end=2041
 * Stack used 20 of 7
 * Currently available Heap=0
 */
void printRAMInfo(Print *aSerial) {
    uint16_t tHeapStart = (uint16_t) getHeapStart();
    aSerial->print(F("Size of Data + BSS, Heap start, Stack end=0x"));
    aSerial->print(tHeapStart - RAMSTART, HEX);
    aSerial->print(F(" | "));
    aSerial->println(tHeapStart - RAMSTART);

    printStackUsedBytes(aSerial);

    aSerial->print(F("Currently available Heap="));
    aSerial->println(getCurrentAvailableHeap());
}

void printCurrentFreeHeap(Print *aSerial) {
    aSerial->print(F("Current free Heap / Stack[bytes]="));
    aSerial->println(getCurrentFreeHeapOrStack());
}

bool isAddressInRAM(void *aAddressToCheck) {
    return (aAddressToCheck <= (void*) RAMEND);
}

bool isAddressBelowHeap(void *aAddressToCheck) {
    return (aAddressToCheck < getHeapStart());
}

/********************************************
 * SLEEP AND WATCHDOG STUFF
 ********************************************/
volatile uint16_t sNumberOfSleeps = 0;

#ifndef _MILLIS_UTILS_H
// copied from MillisUtils.h
/*
 * storage for millis value to enable compensation for interrupt disable at signal acquisition etc.
 */
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)  || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#define timer0_millis millis_timer_millis // The ATTinyCore libraries use other variable name in wiring.c
#endif

extern volatile unsigned long timer0_millis;
#endif // MILLIS_UTILS_H_

/*
 * For sleep modes see sleep.h
 * SLEEP_MODE_IDLE
 * SLEEP_MODE_ADC
 * SLEEP_MODE_PWR_DOWN
 * SLEEP_MODE_PWR_SAVE
 * SLEEP_MODE_STANDBY
 * SLEEP_MODE_EXT_STANDBY
 */
// required only once
void initSleep(uint8_t tSleepMode) {
    sleep_enable();
    set_sleep_mode(tSleepMode);
}

/*
 * Watchdog wakes CPU periodically and all we have to do is call sleep_cpu();
 * aWatchdogPrescaler (see wdt.h) can be one of
 * WDTO_15MS, 30, 60, 120, 250, WDTO_500MS
 * WDTO_1S to WDTO_8S
 */
void initPeriodicSleepWithWatchdog(uint8_t tSleepMode, uint8_t aWatchdogPrescaler) {
    sleep_enable()
    ;
    set_sleep_mode(tSleepMode);
    MCUSR = ~_BV(WDRF); // Clear WDRF in MCUSR

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) \
    || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) \
    || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#define WDTCSR  WDTCR
#endif
    // Watchdog interrupt enable + reset interrupt flag -> needs ISR(WDT_vect)
    uint8_t tWDTCSR = _BV(WDIE) | _BV(WDIF) | (aWatchdogPrescaler & 0x08 ? _WD_PS3_MASK : 0x00) | (aWatchdogPrescaler & 0x07); // handles that the WDP3 bit is in bit 5 of the WDTCSR register,
    WDTCSR = _BV(WDCE) | _BV(WDE); // clear lock bit for 4 cycles by writing 1 to WDCE AND WDE
    WDTCSR = tWDTCSR; // set final Value
}

/*
 * @param aWatchdogPrescaler (see wdt.h) can be one of WDTO_15MS, 30, 60, 120, 250, WDTO_500MS, WDTO_1S to WDTO_8S
 *                           0 (15 ms) to 3(120 ms), 4 (250 ms) up to 9 (8000 ms)
 */
uint16_t computeSleepMillis(uint8_t aWatchdogPrescaler) {
    uint16_t tResultMillis = 8000;
    for (uint8_t i = 0; i < (9 - aWatchdogPrescaler); ++i) {
        tResultMillis = tResultMillis / 2;
    }
    return tResultMillis + DEFAULT_MILLIS_FOR_WAKEUP_AFTER_POWER_DOWN; // + for the (default) startup time. !!! But this depends from Clock Source and sleep mode !!!
}

/*
 * @param aWatchdogPrescaler (see wdt.h) can be one of WDTO_15MS, 30, 60, 120, 250, WDTO_500MS, WDTO_1S to WDTO_8S
 *                           0 (15 ms) to 3(120 ms), 4 (250 ms) up to 9 (8000 ms)
 *                           ! I have see + 30 % deviation from nominal WDT clock!
 * @param aAdjustMillis if true, adjust the Arduino internal millis counter the get quite correct millis()
 * results even after sleep, since the periodic 1 ms timer interrupt is disabled while sleeping.
 * Interrupts are enabled before sleep!
 * !!! Do not forget to call e.g. noTone() or  Serial.flush(); to wait for the last character to be sent, and/or disable interrupt sources before !!!
 */
void sleepWithWatchdog(uint8_t aWatchdogPrescaler, bool aAdjustMillis) {
    MCUSR = 0; // Clear MCUSR to enable a correct interpretation of MCUSR after reset
    ADCSRA &= ~ADEN; // disable ADC just before sleep -> saves 200 uA

    // use wdt_enable() since it handles that the WDP3 bit is in bit 5 of the WDTCSR register
    wdt_enable(aWatchdogPrescaler);

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#  if !defined(timer0_millis)
#define timer0_millis millis_timer_millis // The ATTinyCore + Digispark libraries use millis_timer_millis in wiring.c
#  endif
#define WDTCSR  WDTCR
#endif
    WDTCSR |= _BV(WDIE) | _BV(WDIF); // Watchdog interrupt enable + reset interrupt flag -> requires ISR(WDT_vect)
    sei();         // Enable interrupts, to get the watchdog interrupt, which will wake us up
    sleep_cpu();   // The watchdog interrupt will wake us up from sleep

    // We wake up here :-)
    wdt_disable(); // Because next interrupt will otherwise lead to a reset, since wdt_enable() sets WDE / Watchdog System Reset Enable
    ADCSRA |= ADEN;

    /*
     * Since timer clock may be disabled adjust millis only if not slept in IDLE mode (SM2...0 bits are 000)
     */
#if defined(SM2)
    if (aAdjustMillis && (SMCR & ((_BV(SM2) | _BV(SM1) | _BV(SM0)))) != 0) {
#elif ! defined(SMCR)
    if (aAdjustMillis && (MCUCR & ((_BV(SM1) | _BV(SM0)))) != 0) {
#else
    if (aAdjustMillis && (SMCR & ((_BV(SM1) | _BV(SM0)))) != 0) {
#endif
        timer0_millis += computeSleepMillis(aWatchdogPrescaler);
    }
}

/*
 * This interrupt wakes up the cpu from sleep
 */
ISR(WDT_vect) {
    sNumberOfSleeps++;
}

/*
 * 0 -> %1
 * _BV(CLKPS0) -> %2
 * _BV(CLKPS1) -> %4
 * _BV(CLKPS1) | _BV(CLKPS0) -> 8 etc. up to 256
 */
void setclockDivisionFactor(uint8_t aDivisionBits) {
    CLKPR = _BV(CLKPCE);
    CLKPR = aDivisionBits;
}

#endif // defined(__AVR__)
