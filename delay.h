/*
 * delay.h
 * 
 * Danica Fujiwara & Spencer Shaw
 * 
 * CPE 329-17/18 Spring 2019
 */

#ifndef __DELAY_H__
#define __DELAY_H__

#include "msp.h"

#define LOCK_CS_KEY 0
#define CLK_PER_LOOP 4

#define FREQ_1_5_MHZ 0
#define FREQ_3_MHZ 1
#define FREQ_6_MHZ 2
#define FREQ_12_MHZ 3
#define FREQ_24_MHZ 4
#define FREQ_48_MHZ 5

#define NOP asm(" NOP");

#define ONE_S_MS (1000)

// Initialize DCO and other clock values
void init_dco(void);

// Set DCO to provide frequency
void set_dco(unsigned int freq);

// Delay for 1ms at the given frequency
inline delay_one_ms(const unsigned int freq);

// Delay for the given number of ms at the given frequency
inline void delay_ms(unsigned int msec, unsigned int freq);

inline void delay_ms_auto(unsigned int msec);

// Fine tuned 1 us delay only to be used at 48MHz
inline void delay_one_us_at_48();

// Delay for the given number of us 
inline void delay_us(unsigned int usec, unsigned int freq);

#endif
