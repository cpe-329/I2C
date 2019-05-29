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

void i2c_write(uint8_t addr, uint8_t data);
unsigned int i2c_read(uint8_t addr);


#endif /* I2C_H_ */
