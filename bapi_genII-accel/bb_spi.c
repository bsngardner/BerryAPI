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

#define SPI_DELAY		{ volatile int delay = spi_delay; while (--delay); }
#define SPI_MODE	0

uint16 spi_delay;

int spi_init(void)
{
	P1DIR |= CS_SD | MOSI | SCLK;
	P1DIR &= ~MISO;
	P1OUT |= CS_SD | MOSI;
	P1OUT &= ~SCLK;
	spi_delay = 1;
	return 0;
} // end spi_init


#if SPI_MODE==0
unsigned char spi_transfer(unsigned char byte)
{
	int counter;
	for (counter = 8; counter; --counter)
	{
		if (byte & 0x80) MOSI_HIGH;
		else MOSI_LOW;
		byte <<= 1;
		if (MISO_READ) byte |= 1;
		//SPI_DELAY;
		SCLK_HIGH;					// even edge (latch)
		//SPI_DELAY;
		SCLK_LOW;					// odd edge
	}
	return byte;
}
#elif SPI_MODE==1
unsigned char spi_transfer(unsigned char byte)
{
	int counter;
	for (counter = 8; counter; --counter)
	{
		SPI_DELAY;
		SCLK_HIGH;					// even edge (latch)
		SPI_DELAY;
		SCLK_LOW;					// odd edge
		if (byte & 0x80) MOSI_HIGH;
		else MOSI_LOW;
		byte <<= 1;
		if (MISO_READ) byte |= 1;
	}
	return byte;
}
#elif SPI_MODE==2
unsigned char spi_transfer(unsigned char byte)
{
	int counter;
	for (counter = 8; counter; --counter)
	{
		if (byte & 0x80) MOSI_HIGH;
		else MOSI_LOW;
		byte <<= 1;
		if (MISO_READ) byte |= 1;
		SPI_DELAY;
		SCLK_LOW;					// even edge (latch)
		SPI_DELAY;
		SCLK_HIGH;					// odd edge
	}
	return byte;
}
#elif SPI_MODE==3
unsigned char spi_transfer(unsigned char byte)
{
	int counter;
	for (counter = 8; counter; --counter)
	{
		SPI_DELAY;
		SCLK_LOW;					// even edge (latch)
		SPI_DELAY;
		SCLK_HIGH;					// odd edge
		if (byte & 0x80) MOSI_HIGH;
		else MOSI_LOW;
		byte <<= 1;
		if (MISO_READ) byte |= 1;
	}
	return byte;
}
#endif

#if 0
unsigned char spi_receive(void)
{
	volatile uint16 delay;
	unsigned char rx_data = 0x00;
	unsigned char shift = 0x80;

	MOSI_HIGH;							// set MOSI high
	do
	{
		SCLK_HIGH;						// SCLK high
		if (MISO_READ)					// read MISO while SCLK high
		{
			rx_data |= shift;
		}
		//for (i=SPI_DELAY; i; i--);		// delay
		delay = i2c_delay;
		while (delay--);				// delay
		SCLK_LOW;						// SCLK low
		//for (i=SPI_DELAY; i; i--);		// delay
		delay = i2c_delay;
		while (delay--);				// delay
	} while (shift >>= 1);
	return rx_data;
} // end spi_receive


void spi_send(unsigned char byte)
{
	volatile uint16 delay;
	unsigned char shift = 0x80;

	SCLK_LOW;							// SCLK low
	while (shift)
	{
		if (byte & shift)
		{
			MOSI_HIGH;					// set MOSI high
		}
		else
		{
			MOSI_LOW;					// set MOSI low
		}
		//for (i=SPI_DELAY; i; i--);		// delay
		delay = i2c_delay;
		while (delay--);				// delay
		SCLK_HIGH;						// SCLK high (latch input data)
		//for (i=SPI_DELAY; i; i--);		// delay
		delay = i2c_delay;
		while (delay--);				// delay
		SCLK_LOW;						// SCLK low
		shift >>= 1;					// adjust mask
	}
	MOSI_HIGH;							// leave MOSI high
	return;
} // end spi_send
#endif
