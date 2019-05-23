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
//#include "delay.h"
//#include "led.h"
#include "my_msp.h"
//#include "scope_data.h"
//#include "scope_term.h"
//#include "timers.h"
//#include "uart.h"
//
#define FREQ FREQ_48_MHZ

#define I2C_SCL_PIN P6_5
#define I2C_SDA_PIN P6_4
#define I2C_PINS (I2C_SCL_PIN | I2C_SDA_PIN)

uint8_t RXData[5] = {0};
uint8_t RXDataPointer;

int main(void) {
    volatile uint32_t i;

    init(FREQ);

    P6->SEL0 |= I2C_PINS;  // I2C pins

    // Initialize data variable
    RXDataPointer = 0;

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIB3 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIB3_IRQn)&31);

    // Configure USCI_B3 for I2C mode
    EUSCI_B3->CTLW0 |= EUSCI_A_CTLW0_SWRST;       // Software reset enabled
    EUSCI_B3->CTLW0 = EUSCI_A_CTLW0_SWRST |       // Remain eUSCI in reset mode
                      EUSCI_B_CTLW0_MODE_3 |      // I2C mode
                      EUSCI_B_CTLW0_MST |         // Master mode
                      EUSCI_B_CTLW0_SYNC |        // Sync mode
                      EUSCI_B_CTLW0_SSEL__SMCLK;  // SMCLK

    EUSCI_B3->CTLW1 |=
        EUSCI_B_CTLW1_ASTP_2;  // Automatic stop generated
                               // after EUSCI_B3->TBCNT is reached

    EUSCI_B3->BRW = 30;                       // baudrate = SMCLK / 30 = 100kHz
    EUSCI_B3->TBCNT = 0x0005;                 // number of bytes to be received
    EUSCI_B3->I2CSA = 0x0048;                 // Slave address
    EUSCI_B3->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;  // Release eUSCI from reset

    EUSCI_B3->IE |= EUSCI_A_IE_RXIE |    // Enable receive interrupt
                    EUSCI_B_IE_NACKIE |  // Enable NACK interrupt
                    EUSCI_B_IE_BCNTIE;   // Enable byte counter interrupt

    while (1) {
        // Don't wake up on exit from ISR
        SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

        // Ensures SLEEPONEXIT takes effect immediately
        __DSB();

        // Arbitrary delay before transmitting the next byte
        for (i = 2000; i > 0; i--)
            ;

        // Ensure stop condition got sent
        while (EUSCI_B3->CTLW0 & EUSCI_B_CTLW0_TXSTP)
            ;

        // I2C start condition
        EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

        // Go to LPM0
        __sleep();
        __no_operation();  // for debug
    }
}

// I2C interrupt service routine
void EUSCIB3_IRQHandler(void) {
    if (EUSCI_B3->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B3->IFG &= ~EUSCI_B_IFG_NACKIFG;

        // I2C start condition
        EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
    }
    if (EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG0) {
        EUSCI_B3->IFG &= ~EUSCI_B_IFG_RXIFG0;

        // Get RX data
        RXData[RXDataPointer++] = EUSCI_B3->RXBUF;

        if (RXDataPointer > sizeof(RXData)) {
            RXDataPointer = 0;
        }

        // Wake up on exit from ISR
        SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

        // Ensures SLEEPONEXIT takes effect immediately
        __DSB();
    }
    if (EUSCI_B3->IFG & EUSCI_B_IFG_BCNTIFG) {
        EUSCI_B3->IFG &= ~EUSCI_B_IFG_BCNTIFG;
    }
}
