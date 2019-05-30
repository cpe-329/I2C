/*
 * i2c.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */

#include "i2c.h"
#include <stdint.h>
#include "delay.h"
#include "msp.h"
#include "led.h"

volatile uint8_t RXData[5] = {0};
volatile uint8_t RXDataPointer = 0;

volatile uint8_t TXData[I2C_TX_DATA_MAX_SIZE] = {0};
volatile uint8_t TXDataPointer = 0;
volatile uint8_t TXDataSize = 0;
volatile uint8_t ready = 0;

void i2c_init() {
    NVIC->ISER[0] = 1 << ((EUSCIB0_IRQn)&31);
    rgb_set(RGB_RED);

    P6->SEL0 |= I2C_PINS;  // I2C pins

    // Initialize data variable
    RXDataPointer = 0;

    // Enable eUSCIB3 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIB3_IRQn)&31);

    // Configure USCI_B3 for I2C mode
    EUSCI_B3->CTLW0 |= EUSCI_A_CTLW0_SWRST;       // Software reset enabled
    EUSCI_B3->CTLW0 = EUSCI_A_CTLW0_SWRST |       // Remain eUSCI in reset mode
                      EUSCI_B_CTLW0_MODE_3 |      // I2C mode
                      EUSCI_B_CTLW0_MST |         // Master mode
                      EUSCI_B_CTLW0_SYNC |        // Sync mode
                      EUSCI_B_CTLW0_SSEL__SMCLK;  // SMCLK

    EUSCI_B3->CTLW1 |= EUSCI_B_CTLW1_ASTP_2;
    // Automatic stop generated
    // after EUSCI_B3->TBCNT is reached

    EUSCI_B3->BRW = 30;                       // baudrate = SMCLK / 30 = 100kHz
    EUSCI_B3->TBCNT = 0x0;                    // number of bytes to be received
    EUSCI_B3->I2CSA = I2C_SLACE_ADDR;         // Slave address
    EUSCI_B3->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;  // Release eUSCI from reset

    EUSCI_B3->IE |= EUSCI_B_IE_RXIE |  // Enable receive interrupt
                    EUSCI_B_IE_TXIE |
                    EUSCI_B_IE_NACKIE;  // |  // Enable NACK interrupt
    // EUSCI_B_IE_BCNTIE;   // Enable byte counter interrupt

    delay_ms(500, FREQ_48_MHZ);
    rgb_clear(RGB_RED);
}

void i2c_write(uint8_t addr, uint8_t data) {
    TXData[0] = addr >> 8;
    TXData[1] = addr & 0x0FF;
    TXData[2] = data;

    // write mode
    EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TR;
    // I2C start condition
    EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTP;  // I2C stop condition
}


unsigned int i2c_read(uint8_t addr) {
    unsigned int data = 0;

    RXData[0] = addr >> 8;
    RXData[1] = addr & 0x0FF;
    //First write the address of memory
    // write mode
    EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TR;

    // I2C start condition
    EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    //wait til both parts of the address is sent;
    while(ready < 2);
    ready = 0;

    //Start reading
    // read mode
    EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_TR;

    // I2C start condition
    EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    // stop
    EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTP;


    return data;
}

// I2C interrupt service routine
void EUSCIB3_IRQHandler(void) {
    led_on();
    // If a NACK was received:
    if (EUSCI_B3->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B3->IFG &= ~EUSCI_B_IFG_NACKIFG;

        // I2C start condition
        EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

        //  Return to statrt state after receiving Nack,
        // in repsonse to last expected byte
    }
    // If data was received:
    if (EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG0) {
        EUSCI_B3->IFG &= ~EUSCI_B_IFG_RXIFG0;
        if(RXDataPointer < 2){
            EUSCI_B3->TXBUF = RXData[RXDataPointer++];
            ready++;
        }

        // Get RX data
        RXData[RXDataPointer++] = EUSCI_B3->RXBUF;

        if (RXDataPointer > sizeof(RXData)) {
            RXDataPointer = 0;
        }

        // Wake up on exit from ISR
        SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

        // // Ensures SLEEPONEXIT takes effect immediately
        // __DSB();
    }

    // TX buffer has been cleared
    if (EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG0) {
        rgb_set(RGB_RED);

        EUSCI_B3->IFG &= ~EUSCI_B_IFG_TXIFG0;

        EUSCI_B3->TXBUF = TXData[TXDataPointer++];

        // All data has been sent
        if (TXDataPointer >= TXDataSize |
            TXDataPointer >= I2C_TX_DATA_MAX_SIZE) {
            // Sent all the data, send Stop
            TXDataPointer = 0;
            // I2C start condition
            EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            finish_writing = 1;
        }
        rgb_clear(RGB_RED);
    }
    // if (EUSCI_B3->IFG & EUSCI_B_IFG_BCNTIFG) {
    //     EUSCI_B3->IFG &= ~EUSCI_B_IFG_BCNTIFG;
    // }
}
