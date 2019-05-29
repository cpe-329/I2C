/*
 * i2c.h
 *
 *  Created on: May 23, 2019
 *      Author: sfshaw
 */

#ifndef I2C_H_
#define I2C_H_

void i2c_write(uint8_t addr, uint8_t data);
uint8_t i2c_read(uint8_t addr);


#endif /* I2C_H_ */
