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

// Public function prototypes
uint8 i2c_init(void);
void i2c_write(uint16 address, uint8* data, int16 bytes);
uint8 i2c_read(uint16 address, uint8* buffer, int16 bytes);
uint8 i2c_reg_read(uint16 address, uint8 reg, uint8* buffer, int16 bytes);

//#define I2C_FSCL	100							// ~100kHz
//#define I2C_FSCL	200							// ~200kHz
#define I2C_FSCL	4800							// ~400kHz

void wait(uint16 time);

#endif /* BB_I2C_H_ */
