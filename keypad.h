/*
 * keypad.h
 * 
 * Danica Fujiwara & Spencer Shaw
 * 
 * CPE 329-17/18 Spring 2019
 */

#ifndef __KEYPAD_H__
#define __KEYPAD_H__

#include <stdint.h>
#include "msp.h"
#include "my_msp.h"

/* Keypad Pins: NC  A     B     C     D     E     F     G    NC
 * MSP Pins:       P3.5  P4.4  P3.6  P4.5  P3.7  P4.7  P4.6
 * Pin Name:       COL1  ROW0  COL0  ROW3  COL2  ROW2  ROW1 
 */

#define COL0 P3_6
#define COL1 P3_5
#define COL2 P3_7

#define ROW0 P4_4
#define ROW1 P4_6
#define ROW2 P4_7
#define ROW3 P4_5

#define COL_MASK (COL2 | COL1 | COL0)
#define ROW_MASK (ROW3 | ROW2 | ROW1 | ROW0)

// Value when no key is pressed
#define KEYPAD_NO_KEY (12)

// Time to wait after bringing a row high
#define SETTLE_DELAY 1

// Initialize keypad
inline void keypad_init(void);

// Read the current value from the keypad (non-blocking)
uint8_t keypad_getkey(void);
uint8_t keypad_blocking_getkey(unsigned int hold_ms);
void keypad_await_keypress(unsigned int hold_ms);

//  Bringthe selected row high
inline static void row_all_select();
inline static void row_0_select();
inline static void row_1_select();
inline static void row_2_select();
inline static void row_3_select();

// Read to specified column
inline static uint8_t col_all_read();
inline static uint8_t col_0_read();
inline static uint8_t col_1_read();
inline static uint8_t col_2_read();

#endif
