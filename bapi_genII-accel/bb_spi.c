/*
 * bb_spi.c
 *
 *	Bit-banged SPI
 *
 *  Created on: Feb 8, 2014
 *      Author: proper
 */

#include <msp430.h>
#include <stdint.h>
#include "pins.h"

#define BB_SPI_DELAY	__delay_cycles(1)
#define BB_SPI_CPOL		1
#define BB_SPI_CPHA		1

int bb_spi_init(void)
{
	P1DIR |= BB_SPI_CSEL | BB_SPI_MOSI | BB_SPI_SCLK;
	P1DIR &= ~BB_SPI_MISO;
#if BB_SPI_CPOL==1
	P1OUT |= BB_SPI_CSEL | BB_SPI_MOSI | BB_SPI_SCLK;
#elif BB_SPI_CPOL==0
	P1OUT |= BB_SPI_CSEL | BB_SPI_MOSI;
	P1OUT &= ~BB_SPI_SCLK;
#endif
	BB_SPI_SCLK_HIGH;
	BB_SPI_SCLK_LOW;
	return 0;
} // end spi_init


#if BB_SPI_CPOL==0 && BB_SPI_CPHA==0
unsigned char bb_spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) BB_SPI_MOSI_HIGH;		// Set MOSI
		else BB_SPI_MOSI_LOW;
		byte <<= 1;
		BB_SPI_DELAY;							// Beat
		BB_SPI_SCLK_HIGH;						// Clock high
		if (BB_SPI_MISO_READ) byte |= 1;		// Read MISO
		BB_SPI_DELAY;							// Beat
		BB_SPI_SCLK_LOW;						// Clock low
	} while (--counter);

	return byte;
}

#elif BB_SPI_CPOL==1 && BB_SPI_CPHA==0
unsigned char bb_spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) BB_SPI_MOSI_HIGH;		// Set MOSI
		else BB_SPI_MOSI_LOW;
		byte <<= 1;
		BB_SPI_DELAY;							// Beat
		BB_SPI_SCLK_LOW;						// Clock low
		if (BB_SPI_MISO_READ) byte |= 1;		// Read MISO
		BB_SPI_DELAY;							// Beat
		BB_SPI_SCLK_HIGH;						// Clock high
	} while (--counter);

	return byte;
}

#elif BB_SPI_CPOL==0 && BB_SPI_CPHA==1
unsigned char bb_spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) BB_SPI_MOSI_HIGH;		// Set MOSI
		else BB_SPI_MOSI_LOW;
		byte <<= 1;
		BB_SPI_SCLK_HIGH;						// Clock high
		BB_SPI_DELAY;							// Beat
		if (BB_SPI_MISO_READ) byte |= 1;		// Read MISO
		BB_SPI_SCLK_LOW;						// Clock low
		BB_SPI_DELAY;							// Beat
	} while (--counter);

	return byte;
}
#elif BB_SPI_CPOL==1 && BB_SPI_CPHA==1
unsigned char bb_spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) BB_SPI_MOSI_HIGH;		// Set MOSI
		else BB_SPI_MOSI_LOW;
		byte <<= 1;
		BB_SPI_SCLK_HIGH;						// Clock high
		BB_SPI_DELAY;							// Beat
		if (BB_SPI_MISO_READ) byte |= 1;		// Read MISO
		BB_SPI_SCLK_LOW;						// Clock low
		BB_SPI_DELAY;							// Beat
	} while (--counter);

	return byte;
}
#endif

unsigned char bb_spi_read(void)
{
	return bb_spi_transfer(0xFF);
}

void bb_spi_write(unsigned char byte)
{
	bb_spi_transfer(byte);
}

inline void bb_spi_assert_csel() {
	BB_SPI_CSEL_LOW;
}

inline void bb_spi_release_csel() {
	BB_SPI_CSEL_HIGH;
}
