/*
 * main.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */

//#include <stdbool.h>
//#include <stdint.h>
//#include "msp.h"
//
//#include "adc.h"
//#include "button.h"
//#include "delay.h"
//#include "led.h"
//#include "my_msp.h"
//#include "scope_data.h"
//#include "scope_term.h"
//#include "timers.h"
//#include "uart.h"
//
//#define FREQ FREQ_48_MHZ
//
//#define SECONDS_COUNT_MAX (4)
//
//volatile bool adc_data_fresh = false;
//volatile bool refresh_term = false;
//volatile bool repaint_term = true;
//
//volatile bool one_sec_interval = false;
//volatile bool two_second_interval = false;
//volatile uint8_t seconds_counter = 0;
//
//int main(void) {
//    init(FREQ);
//
//    term_clear_screen();
//    paint_terminal();
//    scope_switch_mode();
//
//    while (true) {
//        // Check button to switch mode
//        if (button_get()) {
//            scope_switch_mode();
//            repaint_term = true;
//        }
//
//        two_second_interval = ((seconds_counter % 2) == 0);
//
//        // Schedule repaint of entire term
//        if (seconds_counter >= SECONDS_COUNT_MAX) {
//            // repaint_term = true;
//            seconds_counter = 0;
//        }
//
//        // Repaint entire term only if needed
//        if (repaint_term) {
//            // Repaint UART VT100 terminal
//            led_on();
//            paint_terminal();
//            led_off();
//
//            scope_cycle_ac_data();
//            // Reset number of sample since last refresh
//            scope_reset_num_samples();
//
//            repaint_term = false;
//        } else if (refresh_term) {
//            // timer_stop_main();
//            // Refresh data displayed in term
//            scope_refresh_data();
//
//            led_on();
//            // Refresh UART VT100 terminal
//            scope_refresh_term();
//            led_off();
//            // timer_restart();
//
//            // Reset number of sample since last refresh
//            scope_reset_num_samples();
//            refresh_term = false;
//        }
//
//        if (one_sec_interval && (scope_get_mode() == SCOPE_MODE_AC)) {
//            // Refresh data displayed in term
//            scope_refresh_data();
//
//            led_on();
//            // Refresh UART VT100 terminal
//            scope_refresh_term();
//            led_off();
//
//            scope_cycle_ac_data();
//
//            // Reset number of sample since last refresh
//            scope_reset_num_samples();
//        }
//
//        one_sec_interval = false;
//
//        // Read data from scope
//        if (adc_data_fresh) {
//            scope_read_data();
//            adc_data_fresh = false;
//            adc_start_conversion();
//        }
//    }
//}
//
//// Timer A0_0 interrupt service routine
//// Every 1 second
//void TA0_0_IRQHandler(void) {
//    // rgb_set(RGB_RED);
//    rgb_toggle(RGB_RED);
//    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // Clear the CCR0 interrupt
//    seconds_counter += 1;
//    reset_refresh_delay();
//    one_sec_interval = true;
//    // rgb_clear(RGB_RED);
//}
//
//// Timer A0_N interrupt service routine for CCR1 - CCR4
//void TA0_N_IRQHandler(void) {
//    if (TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG)  // check for CCR1 interrupt
//    {
//        rgb_set(RGB_GREEN);
//        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;  // clear CCR1 interrupt
//        increment_refresh_delay();
//        // Action for ccr1 intr
//        refresh_term = true;
//        rgb_clear(RGB_GREEN);
//    }
//}
//
//// ADC14 interrupt service routine
//void ADC14_IRQHandler(void) {
//    rgb_set(RGB_BLUE);
//    adc_store_reading(ADC14->MEM[0]);
//    adc_data_fresh = true;
//    rgb_clear(RGB_BLUE);
//}
/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP432 CODE EXAMPLE DISCLAIMER
 *
 * MSP432 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see http://www.ti.com/tool/mspdriverlib for an API functional
 * library & https://dev.ti.com/pinmux/ for a GUI approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP432P401 Demo  - eUSCI_B0 I2C Master RX multiple bytes from MSP432 Slave
//
//  Description: This demo connects two MSP432's via the I2C bus. The master
//  reads 5 bytes from the slave. This is the MASTER CODE. The data from the slave
//  transmitter begins at 0 and increments with each transfer.
//  The USCI_B0 RX interrupt is used to know when new data has been received.
//
//    *****used with "msp432p401x_euscib0_i2c_11.c"****
//
//                                /|\  /|\
//               MSP432P401      10k  10k     MSP432P401
//                   slave         |    |        master
//             -----------------   |    |   -----------------
//            |     P1.6/UCB0SDA|<-|----|->|P1.6/UCB0SDA     |
//            |                 |  |       |                 |
//            |                 |  |       |                 |
//            |     P1.7/UCB0SCL|<-|------>|P1.7/UCB0SCL     |
//            |                 |          |             P1.0|--> LED
//
//   William Goh
//   Texas Instruments Inc.
//   June 2016 (updated) | June 2014 (created)
//   Built with CCSv6.1, IAR, Keil, GCC
//******************************************************************************
#include "msp.h"
#include <stdint.h>

uint8_t RXData[5] = {0};
uint8_t RXDataPointer;

int main(void)
{
    volatile uint32_t i;

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
            WDT_A_CTL_HOLD;

    // Configure GPIO
    P1->OUT &= ~BIT0;                       // Clear P1.0 output latch
    P1->DIR |= BIT0;                        // For LED
    P1->SEL0 |= BIT6 | BIT7;                // I2C pins

    // Initialize data variable
    RXDataPointer = 0;

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIB0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIB0_IRQn) & 31);

    // Configure USCI_B0 for I2C mode
    EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST;       // Software reset enabled
    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST |       // Remain eUSCI in reset mode
                      EUSCI_B_CTLW0_MODE_3 |      // I2C mode
                      EUSCI_B_CTLW0_MST |         // Master mode
                      EUSCI_B_CTLW0_SYNC |        // Sync mode
                      EUSCI_B_CTLW0_SSEL__SMCLK;  // SMCLK

    EUSCI_B0->CTLW1 |= EUSCI_B_CTLW1_ASTP_2;// Automatic stop generated
                                            // after EUSCI_B0->TBCNT is reached

    EUSCI_B0->BRW = 30;                     // baudrate = SMCLK / 30 = 100kHz
    EUSCI_B0->TBCNT = 0x0005;               // number of bytes to be received
    EUSCI_B0->I2CSA = 0x0048;               // Slave address
    EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;// Release eUSCI from reset

    EUSCI_B0->IE |= EUSCI_A_IE_RXIE |       // Enable receive interrupt
                    EUSCI_B_IE_NACKIE |     // Enable NACK interrupt
                    EUSCI_B_IE_BCNTIE;      // Enable byte counter interrupt

    while (1)
    {
        // Don't wake up on exit from ISR
        SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

        // Ensures SLEEPONEXIT takes effect immediately
        __DSB();

        // Arbitrary delay before transmitting the next byte
        for (i = 2000; i > 0; i--);

        // Ensure stop condition got sent
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

        // I2C start condition
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

        // Go to LPM0
        __sleep();
        __no_operation();                   // for debug
    }
}

// I2C interrupt service routine
void EUSCIB0_IRQHandler(void)
{
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)
    {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;

        // I2C start condition
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    }
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0)
    {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;

        // Get RX data
        RXData[RXDataPointer++] = EUSCI_B0->RXBUF;

        if (RXDataPointer > sizeof(RXData))
        {
            RXDataPointer = 0;
        }

        // Wake up on exit from ISR
        SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

        // Ensures SLEEPONEXIT takes effect immediately
        __DSB();
    }
    if (EUSCI_B0->IFG & EUSCI_B_IFG_BCNTIFG)
    {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_BCNTIFG;
    }
}
