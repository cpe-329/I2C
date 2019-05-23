/*
 * my_msp.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */

#include <stdint.h>

#include "msp.h"

#include "adc.h"
#include "delay.h"
#include "keypad.h"
#include "lcd.h"
#include "led.h"
#include "my_msp.h"
#include "spi.h"
#include "timers.h"
#include "uart.h"
#include "button.h"

inline void init(const unsigned int freq) {
    DISABLE_WATCHDOG

    init_all_pins();

    init_dco();
    set_dco(freq);

    led_init();
    rgb_init();

    // button_init();

    // timer_init();

    // dac_init();
    // adc_init();
    uart_init();

    // lcd_init();
    // keypad_init();
}

inline void init_all_pins() {
    P1->SEL0 = 0x00;
    P1->SEL1 = 0x00;
    P1->DIR = 0xFF;
    P1->OUT = 0x00;

    P7->SEL0 = 0x00;
    P7->SEL1 = 0x00;
    P7->DIR = 0xFF;
    P7->OUT = 0x00;

    P8->SEL0 = 0x00;
    P8->SEL1 = 0x00;
    P8->DIR = 0xFF;
    P8->OUT = 0x00;

    P9->SEL0 = 0x00;
    P9->SEL1 = 0x00;
    P9->DIR = 0xFF;
    P9->OUT = 0x00;

    P10->SEL0 = 0x00;
    P10->SEL1 = 0x00;
    P10->DIR = 0xFF;
    P10->OUT = 0x00;
}
