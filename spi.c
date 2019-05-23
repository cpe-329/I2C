// #include "spi.h"
// #include "led.h"
// #include "delay.h"

// inline void dac_init(){
//     P5->SEL0 &= ~SPI_CS_PIN;
//     P5->SEL1 &= ~SPI_CS_PIN;
//     P5->DIR |= SPI_CS_PIN;

//     P6->SEL0 &= ~DAC_LATCH_PIN;
//     P6->SEL1 &= ~DAC_LATCH_PIN;
//     P6->DIR |= DAC_LATCH_PIN;

//     cs_high();
//     latch_high();
// }

// void dac_set(const unsigned int val){
//     int i = 0;
//     cs_low();
//     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)){}

//     EUSCI_B0->TXBUF = CONTORL_BITS | ((val & 0xF00) >> 8);
//     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)){}

//     EUSCI_B0->TXBUF = val & 0xFF;
//     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG)){}

//     for (i = 0; i < 10; i++){}
//     cs_high();
//     latch_low();
//     latch_high();
// }

// inline void cs_low(){
//     P5->OUT &= ~SPI_CS_PIN;
// }
// inline void cs_high(){
//     P5->OUT |= SPI_CS_PIN;
// }

// inline void latch_low(){
//     P6->OUT &= ~DAC_LATCH_PIN;
// }
// inline void latch_high(){
//     P6->OUT |= DAC_LATCH_PIN;
// }
