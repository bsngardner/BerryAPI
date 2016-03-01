/*
 * usi_i2c.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Broderick
 */

#ifndef BERRY_USI_I2C_H_
#define BERRY_USI_I2C_H_

#include <stdint.h>

//Defines
#define SDA_PIN BIT7
#define SCL_PIN BIT6

//Prototypes
void usi_init();
__interrupt void USI_TXRX(void);

//Extern functions
extern void set_register(uint8_t value);
extern uint8_t get_register();

//Extern variables

#endif /* BERRY_USI_I2C_H_ */
