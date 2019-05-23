/*
 * lcd.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */

#include "lcd.h"
#include <stdint.h>
#include "delay.h"
#include "msp.h"
#include "my_msp.h"

// Initialize LCD
void lcd_init() {
    // Setup GPIO pins
    P2->SEL0 &= ~LCD_CTL_PINS;
    P2->SEL1 &= ~LCD_CTL_PINS;
    P2->DIR |= LCD_CTL_PINS;
    P2->OUT &= ~LCD_CTL_PINS;

    P4->SEL0 &= ~LCD_DB_PINS;  // Initialize all P4 pins
    P4->SEL1 &= ~LCD_DB_PINS;  // as GPIO
    P4->DIR |= LCD_DB_PINS;    // Set pins to output mode
    P4->OUT &= ~LCD_DB_PINS;   // Clear pins

    // Power up delay
    delay_ms_auto(32);  // wait >30ms during power up

    // Function set
    lcd_command(CMD_STARTUP);    // Wake up signal?
    lcd_command(CMD_STARTUP);    // Wake up signal?
    lcd_command(CMD_SET_2L_4B);  // Select mode
    lcd_command(CMD_SET_2L_4B);  // Select mode

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);  // Datasheet delay, lies

    // Display ON/OFF control
    lcd_command(CMD_DISP_CTL_INIT);

    // Wait >39us
    delay_ms_auto(30);
    // delay_us_auto(40);  // Datasheet delay, lies

    // Display Clear
    lcd_command(CMD_DISP_CLR);

    // Wait >1.53ms
    delay_ms_auto(30);

    // Entry Mode Set
    lcd_command(CMD_ENTRY_MODE);

    delay_ms_auto(30);
}

void lcd_command(char i) {
    lcd_db_write(i >> 4);  // put shifted data on output Port
    clear_RS();            // D/I=LOW : send instruction
    clear_RW();            // R/W=LOW : Write
    Nybble();              // Send lower 4 bits
    lcd_db_write(i);       // put shifted data on output Port
    Nybble();              // Send upper 4 bits
}

void lcd_write(char i) {
    lcd_db_write(i >> 4);  // put data on output Port
    lcd_set_rs();          // D/I=HIGH : send data
    clear_RW();            // R/W=LOW : Write
    Nybble();              // Clock lower 4 bits
    lcd_db_write(i);       // put shifted data on output Port
    Nybble();              // Clock upper 4 bits
    delay_ms_auto(1);      // Wait >37us
    // delay_us_auto(40);  // Datasheet delay, lies
}

void lcd_write_str(char* s, const int len) {
    int i = 0;
    while (i < len) {
        lcd_write(s[i]);
        i--;
    }
}

// Wrapper for lcd_command with long delay
static inline void lcd_command_delay(unsigned char i) {
    lcd_command(i);
    delay_ms_auto(30);
}

inline void lcd_line_2() {
    lcd_command_delay(CMD_LINE_2);
}
// Set cursor to home, does not clear
inline void lcd_home() {
    lcd_command_delay(CMD_HOME);
}

// Set cursor to home, clears display
inline void lcd_clear() {
    lcd_command_delay(CMD_DISP_CLR);
    delay_ms_auto(25);
}

// Turn display on, without cursor or blinking
inline void lcd_disp_on() {
    lcd_command_delay(CMD_DISP_CTL_D);
}

// Turn display off, hides text
inline void lcd_disp_off() {
    lcd_command_delay(CMD_DISP_CTL_OFF);
}

// Turn display, cursor and blinking on
inline void lcd_blink_on() {
    lcd_command_delay(CMD_DISP_CTL_DCB);
}

// Turn display and cursor on, no blinking
inline void lcd_blink_off() {
    lcd_command_delay(CMD_DISP_CTL_DC);
}

// Turn display and cursor on, no blinking
inline void lcd_cursor_on() {
    lcd_command_delay(CMD_DISP_CTL_DC);
}

// Turn cursor and blinking off
inline void lcd_cursor_off() {
    lcd_command_delay(CMD_DISP_CTL_D);
}

// Write a char to lcd data bus
static inline void lcd_db_write(unsigned char i) {
    P4->OUT &= ~LCD_DB_PINS;
    P4->OUT |= i & LCD_DB_PINS;
}

// Clock data into lcd driver
static inline void Nybble() {
    P2->OUT |= EN;
    NOP
        // Delay(1);    // enable pulse width >= 300ns
        P2->OUT &= ~EN;  // Clock enable: falling edge
}

// Bring RS / D/I LCD pin HIGH
static inline void lcd_set_rs() {
    P2->OUT |= RS;
}

// Bring RS / D/I LCD pin LOW
static inline void clear_RS() {
    P2->OUT &= ~RS;
}

// Bring RW LCD pin HIGH
static inline void clear_RW() {
    P2->OUT &= ~RW;
}

// Translate keypad values to char
unsigned char lcd_translate_keypad(const uint8_t key) {
    switch (key) {
        case 0:
            return '0';
        case 1:
            return ('1');
        case 2:
            return ('2');
        case 3:
            return ('3');
        case 4:
            return ('4');
        case 5:
            return ('5');
        case 6:
            return ('6');
        case 7:
            return ('7');
        case 8:
            return '8';
        case 9:
            return '9';
        case 10:
            return '*';
        case 11:
            return '#';
        case 12:
            return ' ';
        default:
            return '&';
    }
}

// Display key from keypad
inline void lcd_display_keypad(const uint8_t key) {
    lcd_cursor_off();
    lcd_home();                            // Return to first position
    lcd_write(lcd_translate_keypad(key));  // Translate and write value
}
