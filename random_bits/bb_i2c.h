/*
 * bb_i2c.c - Berry Generation II
 *
 *	Created on: Jan 13, 2016
 *		Author: Kristian Sims
 *
 *	This is a software bit-banged i2c for the berry devices
 */

#ifndef BB_I2C_H_
#define BB_I2C_H_

#include <stdint.h>

// Public function prototypes
uint8_t bb_i2c_init(void);
void bb_i2c_write(unsigned address, uint8_t* data, int bytes);
uint8_t bb_i2c_read(unsigned address, uint8_t* buffer, int bytes);
uint8_t bb_i2c_reg_read(unsigned address, uint8_t reg, uint8_t* buffer, int bytes);

#endif /* BB_I2C_H_ */
