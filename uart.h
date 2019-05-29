/*
 * uart.h
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */
#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#include "msp.h"
#include "my_msp.h"

#define UART_PINS (P1_2 | P1_3)

#define ESCAPE_VAL 0xD  //'CR'

volatile unsigned int has_new;
volatile char new_char;

inline void uart_init();

inline void uart_write(unsigned char c);
void uart_write_str(unsigned char c[], unsigned int size);
void uart_write_int(unsigned int acc);
void uart_write_nl();

unsigned int uart_get_int();

void uart_write_volts(unsigned int val_mv);

#endif
