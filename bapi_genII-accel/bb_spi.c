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

#define BB_SPI_DELAY	__delay_cycles(spi_delay)
#define BB_SPI_CPOL		1
#define BB_SPI_CPHA		1

const uint16 spi_delay = 1;	// Hard to tell with cycle counts...

int spi_init(void)
{
	P1DIR |= BB_SPI_CSEL | BB_SPI_MOSI | BB_SPI_SCLK;
	P1DIR &= ~BB_SPI_MISO;
#if BB_SPI_CPOL==1
	P1OUT |= BB_SPI_CSEL | BB_SPI_MOSI | BB_SPI_SLCK;
#elif BB_SPI_CPOL==0
	P1OUT |= BB_SPI_CSEL | BB_SPI_MOSI;
	P1OUT &= ~BB_SPI_SCLK;
#endif
	SCLK_HIGH;
	SCLK_LOW;
	return 0;
} // end spi_init


#if BB_SPI_CPOL==0 && BB_SPI_CPHA==0
unsigned char spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) MOSI_HIGH;		// Set MOSI
		else MOSI_LOW;
		byte <<= 1;
		BB_SPI_DELAY;					// Beat
		SCLK_HIGH;						// Clock high
		if (MISO_READ) byte |= 1;		// Read MISO
		BB_SPI_DELAY;					// Beat
		SCLK_LOW;						// Clock low
	} while (--counter);

	return byte;
}

#elif BB_SPI_CPOL==1 && BB_SPI_CPHA==0
unsigned char spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) MOSI_HIGH;		// Set MOSI
		else MOSI_LOW;
		byte <<= 1;
		BB_SPI_DELAY;					// Beat
		SCLK_LOW;						// Clock low
		if (MISO_READ) byte |= 1;		// Read MISO
		BB_SPI_DELAY;					// Beat
		SCLK_HIGH;						// Clock high
	} while (--counter);

	return byte;
}

#elif BB_SPI_CPOL==0 && BB_SPI_CPHA==1
unsigned char spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) MOSI_HIGH;		// Set MOSI
		else MOSI_LOW;
		byte <<= 1;
		SCLK_HIGH;						// Clock high
		BB_SPI_DELAY;					// Beat
		if (MISO_READ) byte |= 1;		// Read MISO
		SCLK_LOW;						// Clock low
		BB_SPI_DELAY;					// Beat
	} while (--counter);

	return byte;
}
#elif BB_SPI_CPOL==1 && BB_SPI_CPHA==1
unsigned char spi_transfer(unsigned char byte)
{
	int counter = 8;
	do {
		if (byte & 0x80) MOSI_HIGH;		// Set MOSI
		else MOSI_LOW;
		byte <<= 1;
		SCLK_HIGH;						// Clock high
		BB_SPI_DELAY;					// Beat
		if (MISO_READ) byte |= 1;		// Read MISO
		SCLK_LOW;						// Clock low
		BB_SPI_DELAY;					// Beat
	} while (--counter);

	return byte;
}
#endif

unsigned char spi_read(void)
{
	return spi_transfer(0xFF);
}

void spi_send(unsigned char byte)
{
	spi_transfer(byte);
	return;
}
