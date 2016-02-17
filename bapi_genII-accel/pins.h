/*
 * pins.h
 *
 *  Created on: Jan 13, 2016
 *      Author: Kristian Sims
 */

#ifndef PINS_H_
#define PINS_H_

// Port 1
#define BB_SPI_SCLK		BIT0		// Clock
#define BB_SPI_MOSI		BIT2		// Master out slave in
#define BB_SPI_MISO		BIT4		// Master in slave out
#define BB_SPI_CSEL		BIT1		// Chip select

#define BB_SCLK_HIGH	(P1OUT |= BB_SPI_SCLK)
#define BB_SCLK_LOW		(P1OUT &= ~BB_SPI_SCLK)
#define BB_MOSI_HIGH	(P1OUT |= BB_SPI_MOSI)
#define BB_MOSI_LOW		(P1OUT &= ~BB_SPI_MOSI)
#define BB_MISO_READ	(P1IN & BB_SPI_MISO)
#define BB_CSEL_HIGH	(P1OUT |= BB_SPI_CSEL)
#define BB_CSEL_LOW		(P1OUT &= ~BB_SPI_CSEL)

#endif /* PINS_H_ */
