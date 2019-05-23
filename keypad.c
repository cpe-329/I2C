/*
 * keypad.c
 * 
 * Danica Fujiwara & Spencer Shaw
 * 
 * CPE 329-17/18 Spring 2019
 */

#include "msp.h"
#include "my_msp.h"
#include "keypad.h"
#include "delay.h"
#include "led.h"

// Initialize keypad
inline void keypad_init(void){
    P3->SEL0 &= ~COL_MASK;  // Select GPIO mode
    P3->SEL1 &= ~COL_MASK;  // Select GPIO mode
    P3->DIR &= ~COL_MASK;  // Set columns to input mode
    P3->REN |= COL_MASK;  // Enable builtin resistors
    P3->OUT &= ~COL_MASK; // Set rows low  

    P4->SEL0 &= ~ROW_MASK;  // Select GPIO mode
    P4->SEL1 &= ~ROW_MASK;  // Select GPIO mode
    P4->DIR |= ROW_MASK;  // Set rows to output mode
    P4->OUT &= ~ROW_MASK; // Set rows low   
}

// Read the currently pressed key from the keypad
// Non-blocking, prioritizes some keys
uint8_t keypad_getkey(void){
    // Check all for no input
    row_all_select(); 
    delay_ms_auto(SETTLE_DELAY);
    if (col_all_read() == 0){
        // Nothing was pressed
        return KEYPAD_NO_KEY;
    }

    // Check first row
    row_0_select();  // put row high
    delay_ms(SETTLE_DELAY, FREQ_48_MHZ); //delay for 25 cycles
    if(col_0_read())
       return 1;
    else if (col_1_read())
       return 2;
    else if (col_2_read())
       return 3;

    // Check second row
    row_1_select();  // put row high
    delay_ms_auto(SETTLE_DELAY); //delay for 25 cycles
    if(col_0_read())
       return 4;
    else if (col_1_read())
       return 5;
    else if (col_2_read())
       return 6;

    // Check thrid row
    row_2_select();  // put row high
    delay_ms_auto(SETTLE_DELAY); //delay for 25 cycles
    if(col_0_read())
        return 7;
    else if (col_1_read())
        return 8;
    else if (col_2_read())
        return 9;

    // Check fourth row
    row_3_select();  // put row high
    delay_ms_auto(SETTLE_DELAY);  //delay for 25 cycles
    if(col_0_read())
        return 10;
    else if (col_1_read())
        return 0;
    else if (col_2_read())
        return 11;
    // Nothing was pressed
    return KEYPAD_NO_KEY;
}

// Blocking get key with some debounce
uint8_t keypad_blocking_getkey(unsigned int hold_ms){
    uint8_t val = keypad_getkey();
    while (val == KEYPAD_NO_KEY){
        val = keypad_getkey();
    }
    delay_ms_auto(hold_ms);
    while(keypad_getkey() != 12){}
    return val;
}

void keypad_await_keypress(unsigned int hold_ms){
    uint8_t val = keypad_getkey();
    while (val == KEYPAD_NO_KEY){
        val = keypad_getkey();
    }
    delay_ms_auto(hold_ms);
    while(keypad_getkey() != KEYPAD_NO_KEY){}
}


inline static void row_all_select(){
    P4->OUT |= ROW_MASK;
}

inline static void row_0_select(){
    P4->OUT |= ROW0;
    P4->OUT &= ~(ROW1 | ROW2 | ROW3);
}

inline static void row_1_select(){
    P4->OUT |= ROW1;
    P4->OUT &= ~(ROW0 | ROW2 | ROW3);
}

inline static void row_2_select(){
    P4->OUT |= ROW2;
    P4->OUT &= ~(ROW0 | ROW1 | ROW3);
}

inline static void row_3_select(){
    P4->OUT |= ROW3;
    P4->OUT &= ~(ROW0 | ROW1 | ROW2);
}

inline static uint8_t col_all_read(){
    return P3->IN & COL_MASK;
}

inline static uint8_t col_0_read(){
    return P3->IN & COL0;
}

inline static uint8_t col_1_read(){
    return P3->IN & COL1;
}

inline static uint8_t col_2_read(){
    return P3->IN & COL2;
}
