/*
 * i2c.c
 *
 *  Created on: May 23, 2019
 *      Author: sfshaw
 */
#include i2c.h

void i2c_write(uint8_t addr, uint8_t data){

     // write mode
     EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TR;
     // I2C start condition
     EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

     EUSCI_B3 -> TXBUF = addr>>8;    // Address High byte

     EUSCI_B3 -> TXBUF = addr & 0xFF;    // Address low byte

     EUSCI_B3 -> TXBUF = data;      // send data

     EUSCI_B3 -> CTLW0 |= EUSCI_B_CTLW0_TXSTP;   // I2C stop condition
}

uint8_t i2c_read(uint8_t addr){
      uint8_t data;

      // write mode
      EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TR;

      // I2C start condition
      EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

      EUSCI_B3 -> TXBUF = addr >> 8    // Address high byte

      EUSCI_B3 -> TXBUF = LoAddress;    // Address low byte

      // read mode
      EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_TR;

      // I2C start condition
      EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

      // stop
      EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

      data = EUSCI_B3->RXBUF;    // Read byte

      return data;
    }
}
