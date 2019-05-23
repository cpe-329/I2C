// /*
//  * main.c
//  *
//  * Danica Fujiwara & Spencer Shaw
//  *
//  * CPE 329-17/18 Spring 2019
//  *
//  *  MCLK = = DCO ~24MHz
//  *  SMCLK = MCLK / 8
//  *  TIMER_A0 utilize SMCLK 3MHz
//  *  TIMER_A0 CCR0 and CCR1 to create 2 timing events
//  */

#ifndef __TIMERS_H__
#define __TIMERS_H__

#include <stdint.h>
#include "delay.h"
#include "led.h"
#include "msp.h"
#include "my_msp.h"

void timer_init(void);

inline void timer_stop_main();
inline void timer_restart();
inline void increment_refresh_delay();
inline void reset_refresh_delay();

inline void timer_stop_fast();
inline void timer_resume_fast();
#endif
