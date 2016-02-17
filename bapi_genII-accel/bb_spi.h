/*
 * bb_spi.h
 *
 *	Bit-banged SPI
 *
 *  Created on: Feb 8, 2014
 *      Author: proper
 */

#ifndef BB_SPI_H_
#define BB_SPI_H_

#include <stdint.h>


int spi_init(void);
uint8_t spi_transfer(uint8_t byte);
void spi_write(uint8_t);
uint8_t spi_read(void);

#endif /* BB_SPI_H_ */
