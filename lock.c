/*
 * lock.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */

#include <stdbool.h>

#include "delay.h"
#include "keypad.h"
#include "lcd.h"
#include "led.h"
#include "lock.h"

// Execute the lock routine
uint8_t lock(const passcode_t passcode) {
    // Stores the user's guess
    passcode_t guess;
    // Stores the last input value
    uint8_t new_dig;

    // construct to be empty
    guess = passcode_empty();
    // Default unset value
    new_dig = KEYPAD_NO_KEY;
    // Display message
    lock_message();
    // Restore blinking for input
    lcd_blink_on();

    // First digit
    new_dig = keypad_blocking_getkey(LOCK_HOLD_MS);
    if (new_dig == 10) {
        return false;
    }
    lcd_write(lcd_translate_keypad(new_dig));
    led_blink_ms(ONE_S_MS);
    guess.dig1 = new_dig;

    // Second digit
    new_dig = keypad_blocking_getkey(LOCK_HOLD_MS);
    if (new_dig == 10) {
        return false;
    }
    lcd_write(lcd_translate_keypad(new_dig));
    led_blink_ms(ONE_S_MS);
    guess.dig2 = new_dig;

    // Third digit
    new_dig = keypad_blocking_getkey(LOCK_HOLD_MS);
    if (new_dig == 10) {
        return false;
    }
    lcd_write(lcd_translate_keypad(new_dig));
    led_blink_ms(ONE_S_MS);
    guess.dig3 = new_dig;

    // Fourth digit
    new_dig = keypad_blocking_getkey(LOCK_HOLD_MS);
    if (new_dig == 10) {
        return false;
    }
    lcd_write(lcd_translate_keypad(new_dig));
    led_blink_ms(ONE_S_MS);
    guess.dig4 = new_dig;

    if (check_passcode(passcode, guess)) {
        return true;
    } else {
        return false;
    }
}

// Display the locked message
void lock_message() {
    lcd_cursor_off();
    lcd_clear();

    lcd_write('L');
    lcd_write('O');
    lcd_write('C');
    lcd_write('K');
    lcd_write('E');
    lcd_write('D');
    lcd_line_2();
    lcd_write('E');
    lcd_write('n');
    lcd_write('t');
    lcd_write('e');
    lcd_write('r');
    lcd_write(' ');
    lcd_write('k');
    lcd_write('e');
    lcd_write('y');
    lcd_write(':');
    lcd_write(' ');
}

// Display the unlocked message
void unlock_message() {
    lcd_cursor_off();
    lcd_clear();
    // lcd_home();  // ?

    lcd_write('H');
    lcd_write('e');
    lcd_write('l');
    lcd_write('l');
    lcd_write('o');
    lcd_write(' ');
    lcd_write('W');
    lcd_write('o');
    lcd_write('r');
    lcd_write('l');
    lcd_write('d');
}

// Check two passcodes for equality
uint8_t check_passcode(passcode_t actual, passcode_t guess) {
    return ((actual.dig1 == guess.dig1) & (actual.dig2 == guess.dig2) &
            (actual.dig3 == guess.dig3) & (actual.dig4 == guess.dig4));
}

// Construct a passcode
passcode_t passcode_init(uint8_t dig1,
                         uint8_t dig2,
                         uint8_t dig3,
                         uint8_t dig4) {
    passcode_t passcode;
    passcode.dig1 = dig1;
    passcode.dig2 = dig2;
    passcode.dig3 = dig3;
    passcode.dig4 = dig4;
    return passcode;
}

// Construct and empty passcode
passcode_t passcode_empty() {
    return passcode_init(12, 12, 12, 12);
}
