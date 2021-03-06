/*
 * spi.c
 *
 *  Created on: Sep 16, 2016
 *      Author: Kristian Sims
 */

#include <msp430.h>

#define SCS 0x0010
#define UCMPUSPI	(UCCKPL|UCMSB|UCMST|UCMODE_0|UCSYNC|UCSSEL__SMCLK|UCSTEM)
// CPHA=0, CPOL=1, MSB first, master, 3-wire SPI, SYNC, SMCLK, master

static volatile char* spi_data;
static volatile unsigned spi_count;

int spi_init() {

	UCA0CTLW0 |= UCSWRST;				// Reset device
	UCA0CTLW0 = UCMPUSPI | UCSWRST;		// Set registers for MPU9250
	UCB0BRW = 23;						// Divide clock by 24

	PASEL0 &= ~0x0330;					// Set pins to SPI mode
	PASEL1 |= 0x0320;					// Word access for great justice

	PASEL1 &= ~SCS;						// Set up CS pin
	PAOUT |= SCS;
	PADIR |= SCS;

	UCA0CTLW0 &= ~UCSWRST;				// Start eUSCI_A0

	return 0;
}

int spi_transfer(char* data, unsigned count) {

	spi_count = count;					// Pass count to global
	spi_data = data;					// Pass buffer pointer to global

	PAOUT &= ~SCS;						// Chip select
	UCA0IE |= UCTXIE|UCRXIE;			// Enable interrupts

	do
		LPM0;
	while (spi_count > 0);

	PAOUT |= SCS;						// De-select

	return 0;
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR() {
	switch (__even_in_range(UCA0IV, 4)) {
	case 0:								// No interrupt
		break;
	case 2:								// RX interrupt
		*spi_data++ = UCA0RXBUF; 		// Copy data into buffer
		if (!--spi_count)
			UCA0IE = 0;					// Disable interrupts to end transfer
		break;
	case 4:								// TX interrupt
		UCA0TXBUF = *(spi_data);	// Send next byte
		break;
	}
	__bic_SR_register_on_exit(LPM0_bits);
}
