/*
 * my_msp.h
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */

#ifndef __MY_MSP_H__
#define __MY_MSP_H__

// Port 1
#define P1_0 BIT0  // P1.0
#define P1_1 BIT1  // P1.1
#define P1_2 BIT2  // P1.2
#define P1_3 BIT3  // P1.3
#define P1_4 BIT4  // P1.4
#define P1_5 BIT5  // P1.5
#define P1_6 BIT6  // P1.6
#define P1_7 BIT7  // P1.7

// Port 2
#define P2_0 BIT0  // P2.0
#define P2_1 BIT1  // P2.1
#define P2_2 BIT2  // P2.2
#define P2_3 BIT3  // P2.3
#define P2_4 BIT4  // P2.4
#define P2_5 BIT5  // P2.5
#define P2_6 BIT6  // P2.6
#define P2_7 BIT7  // P2.7

// Port 3
#define P3_0 BIT0  // P3.0
#define P3_1 BIT1  // P3.1
#define P3_2 BIT2  // P3.2
#define P3_3 BIT3  // P3.3
#define P3_4 BIT4  // P3.4
#define P3_5 BIT5  // P3.5
#define P3_6 BIT6  // P3.6
#define P3_7 BIT7  // P3.7

// Port 4
#define P4_0 BIT0  // P4.0
#define P4_1 BIT1  // P4.1
#define P4_2 BIT2  // P4.2
#define P4_3 BIT3  // P4.3
#define P4_4 BIT4  // P4.4
#define P4_5 BIT5  // P4.5
#define P4_6 BIT6  // P4.6
#define P4_7 BIT7  // P4.7

// Port 5
#define P5_0 BIT0  // P5.0
#define P5_1 BIT1  // P5.1
#define P5_2 BIT2  // P5.2
#define P5_3 BIT3  // P5.3
#define P5_4 BIT4  // P5.4
#define P5_5 BIT5  // P5.5
#define P5_6 BIT6  // P5.6
#define P5_7 BIT7  // P5.7

// Port 6
#define P6_0 BIT0  // P6.0
#define P6_1 BIT1  // P6.1
#define P6_2 BIT2  // P6.2
#define P6_3 BIT3  // P6.3
#define P6_4 BIT4  // P6.4
#define P6_5 BIT5  // P6.5
#define P6_6 BIT6  // P6.6
#define P6_7 BIT7  // P6.7

// Macro for disabling watchdog
#define DISABLE_WATCHDOG WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

// Initialize all used periphrials
inline void init(const unsigned int freq);
inline void init_all_pins();

#endif
