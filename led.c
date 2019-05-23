/*
 * led.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */

#include "led.h"
#include <stdint.h>
#include "delay.h"
#include "msp.h"
#include "my_msp.h"

// Setup LED1
inline void led_init(void) {
    P1->SEL0 &= ~LED1_PIN;  // Set sel0 bit low for GPIO
    P1->SEL1 &= ~LED1_PIN;  // Set sel1 bit low for GPIO
    P1->DIR |= LED1_PIN;    // Set P1.0 to output mode
    P1->OUT &= ~LED1_PIN;   //  Set LED1 state to off

    test_leds();
}

// Toggle LED1
inline void led_toggle(void) {
    P1->OUT ^= LED1_PIN;  // XOR LED1 state to toggle
}
inline void led_on(void) {
    P1->OUT |= LED1_PIN;
}
inline void led_off(void) {
    P1->OUT &= ~LED1_PIN;
}
inline void led_val(uint8_t val) {
    if (val != 0) {
        led_on();
    } else {
        led_off();
    }
}
// Blink LED on and off for given ms
void led_blink_ms(unsigned int ms) {
    led_on();
    delay_ms_auto(ms);
    led_off();
}

// Setup RGB LED
inline void rgb_init(void) {
    P1->SEL0 &= ~RGB_PINS;  // Set sel0 bits low for GPIO
    P1->SEL1 &= ~RGB_PINS;  // Set sel1 bits low for GPIO
    P2->DIR |= RGB_PINS;    // Set P2.0 - P2.2 to output mode
    P2->OUT &= ~RGB_PINS;   // Set RGB LED state to off
}

// Set 3 bits of RGB LED
inline void rgb_set(uint8_t value) {
    // P2->OUT &= ~RGB_PINS;           // Clear previous RGB LED state
    P2->OUT |= (value & RGB_PINS);  // Set RGB LED state
}

inline void rgb_clear(uint8_t value) {
    P2->OUT &= ~(value & RGB_PINS);  // Clear previous RGB LED state
}

inline void rgb_toggle(uint8_t value) {
    P2->OUT ^= (value & RGB_PINS);
}

// Blink all combinations of LEDs
inline void test_leds() {
    int i = 0;
    while (i < 13) {
        leds_display_keypad(i);
        delay_ms(50, FREQ_48_MHZ);
        i++;
    }
}

// Display keypad value on LEDs
inline void leds_display_keypad(uint8_t keypad_val) {
    switch (keypad_val) {
        case 0:
            led_on();
            rgb_set(RGB_WHITE);
            break;
        case 1:
            led_off();
            rgb_set(RGB_RED);
            break;
        case 2:
            led_off();
            rgb_set(RGB_GREEN);
            break;
        case 3:
            led_off();
            rgb_set(RGB_BLUE);
            break;
        case 4:
            led_off();
            rgb_set(RGB_YELLOW);
            break;
        case 5:
            led_off();
            rgb_set(RGB_PURPLE);
            break;
        case 6:
            led_off();
            rgb_set(RGB_TURQUOISE);
            break;
        case 7:
            led_off();
            rgb_set(RGB_WHITE);
            break;
        case 8:
            led_on();
            rgb_set(RGB_RED);
            break;
        case 9:
            led_on();
            rgb_set(RGB_GREEN);
            break;
        case 10:
            led_on();
            rgb_set(RGB_BLUE);
            break;
        case 11:
            led_on();
            rgb_set(RGB_YELLOW);
            break;
        default:
            led_off();
            rgb_set(RGB_OFF);
    }
}
