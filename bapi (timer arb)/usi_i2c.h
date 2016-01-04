/*
 * usi_i2c.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Broderick
 */

#ifndef USI_I2C_H_
#define USI_I2C_H_

#include <stdint.h>

//Defines
//#define IOBUFFER_SIZE 30
//#define SLV_ADDR 0x55<<1
#define ADDR_REG 0
#define SDA_PIN BIT7
#define SCL_PIN BIT6

typedef enum {
	I2C_IDLE = 0,
	I2C_START = 2,
	I2C_ADDR = 4,
	I2C_RESET = 6,
	I2C_RX = 8,
	I2C_HANDLERX = 10
} i2c_state_t;

//Prototypes
void init_usi();

//Extern functions
extern inline uint8_t get_register(uint8_t reg);
extern inline void set_register(uint8_t reg, uint8_t value);

#endif /* USI_I2C_H_ */
