/*
 * i2c.h
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "my_msp.h"

#define I2C_SCL_PIN P6_5
#define I2C_SDA_PIN P6_4
#define I2C_PINS (I2C_SCL_PIN | I2C_SDA_PIN)

#define I2C_SLACE_ADDR (0b1010000)

#define I2C_TX_DATA_MAX_SIZE (3)

volatile int finish_writing;
void i2c_init();
void i2c_write(uint8_t addr, uint8_t data);
unsigned int i2c_read(uint8_t addr);

#endif /* I2C_H_ */
