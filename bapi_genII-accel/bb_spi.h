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


int bb_spi_init(void);
uint8_t bb_spi_transfer(uint8_t byte);
void bb_spi_write(uint8_t);
uint8_t bb_spi_read(void);
void bb_spi_assert_csel(void);
void bb_spi_release_csel(void);

#endif /* BB_SPI_H_ */
