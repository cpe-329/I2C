// #include "msp.h"

// void main(void) {
//   init(FREQ_24_MHZ);

//   // setup TIMER_A0
//   TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // clear interrupt
//   TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;  // clear interrupt

//   TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;  // TACCR0 interrupt enabled
//   TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;  // TACCR1 interrupt enabled

//   TIMER_A0->CCR[0] = 120;  // set CCR0 count
//   TIMER_A0->CCR[1] = 30;   // set CCR1 count

//   TIMER_A0->CTL = TIMER_A_CTL_TASSEL_1 | TIMER_A_CTL_MC_1;  // SMCLK, UP mode

//   NVIC->ISER[0] = 1 << ((TA0_0_IRQn)&31);  // set NVIC interrupt
//   NVIC->ISER[0] = 1 << ((TA0_N_IRQn)&31);  // TA0_0 and TA0_N

//   __enable_irq();  // Enable global interrupt

//   while (1)
//     ;
// }
// // Timer A0_0 interrupt service routine
// void TA0_0_IRQHandler(void) {
//   TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // Clear the CCR0 interrupt
//   P1->OUT ^= BIT0;
// }

// // Timer A0_N interrupt service routine for CCR1 - CCR4
// void TA0_N_IRQHandler(void) {
//   if (TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG)  // check for CCR1 interrupt
//   {
//     TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;  // clear CCR1 interrupt
//     P2->OUT ^= BIT0;
//   }
// }
