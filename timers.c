/*
 * main.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 *  MCLK = = DCO ~48MHz
 *  SMCLK = MCLK / 8
 *  TIMER_A0 utilize SMCLK 3MHz
 *  TIMER_A0 CCR0 and CCR1 to create 2 timing events
 */

#include "timers.h"
#include <stdint.h>
#include "delay.h"
#include "led.h"
#include "msp.h"
#include "my_msp.h"

#define TIMER_ONE_SECOND_DELAY (2051)
#define REFRESH_DELAY_DELTA (256)

void timer_init(void) {
    rgb_set(RGB_YELLOW);

    // P4->SEL0 |= P4_3;
    // P4->SEL1 &= ~P4_3;
    // P4->DIR |= P4_3;

    // setup TIMER_A0
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // clear interrupt
    TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;  // clear interrupt

    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;  // TACCR0 interrupt enabled
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;  // TACCR1 interrupt enabled

    TIMER_A0->CCR[0] = TIMER_ONE_SECOND_DELAY;  // set CCR0 count
    TIMER_A0->CCR[1] = REFRESH_DELAY_DELTA;     // set CCR1 count

    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_1 |  // ACLK,
                    TIMER_A_CTL_MC_1;       // UP mode, count up to CCR[0]

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn)&31);  // set NVIC interrupt
    NVIC->ISER[0] = 1 << ((TA0_N_IRQn)&31);  // TA0_0 and TA0_N

    __enable_irq();  // Enable global interrupt

    rgb_set(RGB_OFF);
}

inline void timer_stop_main(){
    TIMER_A0->CCR[0] = 0;
}

inline void timer_restart(){
    TIMER_A0->CCR[0] = TIMER_ONE_SECOND_DELAY;
}

inline void increment_refresh_delay() {
    TIMER_A0->CCR[1] += REFRESH_DELAY_DELTA;  // increment CCR1 count
}

inline void reset_refresh_delay() {
    TIMER_A0->CCR[1] = REFRESH_DELAY_DELTA;  // set CCR1 count
}

inline void timer_stop_fast() {
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;  // TACCR1 interrupt enabled
}

inline void timer_resume_fast() {
    TIMER_A0->CCTL[1] = 0;  // TACCR1 interrupt enabled
}
