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
#define SDA_PIN BIT7
#define SCL_PIN BIT6

//Prototypes
void usi_init();

//Extern functions
extern void set_register(uint8_t value);
extern uint8_t get_register();

//Extern variables

#endif /* USI_I2C_H_ */
