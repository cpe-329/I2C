/*
 * main.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "msp.h"

//#include "adc.h"
//#include "button.h"
#include "delay.h"
#include "led.h"
#include "my_msp.h"
//#include "scope_data.h"
//#include "scope_term.h"
//#include "timers.h"
#include "uart.h"
#include "i2c.h"

#define FREQ FREQ_48_MHZ

unsigned char str_enter_addr[] = "Enter address: ";
unsigned char str_addr[] = "Address: ";
unsigned char str_enter_val[] = "Enter value: ";
unsigned char str_val[] = "Value: ";
unsigned char str_reading[] = "Reading value ";
unsigned char str_writing[] = "Writing value ";
unsigned char str_to_addr[] = " to address ";
unsigned char str_fron_addr[] = "from address";


volatile unsigned char char_data = '7';
volatile bool got_fresh_char = false;


int main(void) {
    unsigned int addr = 0;
    unsigned int value = 0;
    volatile uint32_t i = 0;
    finish_writing = 0;

    init(FREQ);

    while (1) {
        uart_write_nl();

        // Enter address
        uart_write_str(str_enter_addr, sizeof(str_enter_addr));
        addr = uart_get_int();
        uart_write_nl();
        uart_write_str(str_addr, sizeof(str_addr));
        uart_write_int(addr);
        uart_write_nl();

        // Enter Value
        uart_write_str(str_enter_val, sizeof(str_enter_val));
        value = uart_get_int();
        uart_write_nl();
        uart_write_str(str_val, sizeof(str_val));
        uart_write_int(value);
        uart_write_nl();

        // Output data
        uart_write_str(str_writing, sizeof(str_writing));
        uart_write_int(value);
        uart_write_str(str_to_addr, sizeof(str_to_addr));
        uart_write_int(addr);
        uart_write_nl();
        uart_write_nl();

        // Read or Write to I2C Memory
        rgb_set(RGB_GREEN);
        i2c_write(addr, value);
        //Waits until all bytes are written
        while(!finish_writing);
        finish_writing = 0;
        rgb_clear(RGB_GREEN);
    }
}

/*
int main(void) {


    while (1) {

        // // Don't wake up on exit from ISR
        // SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

        // // Ensures SLEEPONEXIT takes effect immediately
        // __DSB();

        // Arbitrary delay before transmitting the next byte
        for (i = 2000; i > 0; i--) {
        }

        rgb_set(RGB_GREEN);
        // Ensure stop condition got sent
        while (EUSCI_B3->CTLW0 & EUSCI_B_CTLW0_TXSTP) {
        }
        rgb_clear(RGB_GREEN);

        // I2C start condition
        EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

        // // Go to LPM0
        // __sleep();
        // __no_operation();  // for debug
    }
}
*/

// UART interrupt service routine
void EUSCIA0_IRQHandler(void) {
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG) {
        led_on();

        if (has_new) {
            return;
        }
        // Check if the TX buffer is empty first
        while (!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)) {
        }

        new_char = EUSCI_A0->RXBUF;
        has_new = true;
        // Echo the received character back
        EUSCI_A0->TXBUF = new_char;
        // delay_ms(10, FREQ);

        led_off();
    }
}


