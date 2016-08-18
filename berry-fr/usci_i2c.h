/*
 * eusci_i2c.h
 *
 *  Created on: Jun 20, 2016
 *      Author: Broderick
 */

#ifndef EUSCI_I2C_H_
#define EUSCI_I2C_H_

#include <stdint.h>

void usci_init();

extern volatile int16_t current_register;

void set_register(uint8_t);
uint8_t get_register();

#endif /* EUSCI_I2C_H_ */
