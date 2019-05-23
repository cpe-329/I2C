/*
 * led.h
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */

#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>
#include "msp.h"
#include "my_msp.h"

#define LED1_PIN P1_0

#define RGB_RED_PIN P2_0
#define RGB_GREEN_PIN P2_1
#define RGB_BLUE_PIN P2_2
#define RGB_PINS (RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN)

#define RGB_OFF (0b00000000)
#define RGB_RED (0b00000001)
#define RGB_GREEN (0b00000010)
#define RGB_BLUE (0b00000100)
#define RGB_YELLOW (0b00000011)
#define RGB_PURPLE (0b00000101)
#define RGB_TURQUOISE (0b00000110)
#define RGB_WHITE (0b00000111)

// Setup LED1
inline void led_init(void);

// Toggle LED1
inline void led_toggle(void);
inline void led_on(void);
inline void led_off(void);
inline void led_val(uint8_t val);
void led_blink_ms(unsigned int ms);

// Setup RGB LED
inline void rgb_init(void);

// Set 3 bits of RGB LED
inline void rgb_set(uint8_t value);
inline void rgb_clear(uint8_t value);
inline void rgb_toggle(uint8_t value);

// Blink all combinations of LEDs
inline void test_leds();

// Display keypad value on LEDs
inline void leds_display_keypad(uint8_t keypad_val);

#endif
