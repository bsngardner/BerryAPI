/*
 * bapiLib.c
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#include "msp430.h"
#include <stdlib.h>
#include <stdint.h>

#include "berry.h"
#include "usi_i2c.h"

//Prototypes
int bapi_init(CLOCK_SPEED clock);
void seed_rand();
void msp430_init(CLOCK_SPEED clock);
void gpio_port_init();
inline void check_timeout();

//Constants for clock speed init
static const struct {
	volatile unsigned char* calbc1;
	volatile unsigned char* caldco;
} dco_cal[] = { { &CALBC1_1MHZ, &CALDCO_1MHZ }, { &CALBC1_8MHZ, &CALDCO_8MHZ },
		{ &CALBC1_12MHZ, &CALDCO_12MHZ }, { &CALBC1_16MHZ, &CALDCO_16MHZ } };

volatile uint8_t* PxOUT[3] = { 0, &P1OUT, &P2OUT };
volatile uint8_t* PxDIR[3] = { 0, &P1DIR, &P2DIR };
volatile uint8_t* PxIN[3] = { 0, &P1IN, &P2IN };

//Global variables
volatile uint8_t regs[TABLE_SIZE] = { 0 };
register_table_t reg_table = { regs, TABLE_SIZE };
volatile uint16_t sys_event = 0;
volatile uint16_t tick_max = 0;
volatile uint16_t tick_count = 1;

void main() {
	bapi_init(CLOCK);
	reg_table.table[0] = device_init();
	tick_count = 1;
	// Enable global interrupts after all initialization is finished.
	__enable_interrupt();

	// Wait for an interrupt
	while (1) {
		// disable interrupts before check sys_event
		__disable_interrupt();

		if (!sys_event) {
			// no events pending, enable interrupts and goto sleep (LPM0)
			__bis_SR_register(LPM0_bits | GIE);
			continue;
		} else {
			// at least 1 event is pending, enable interrupts before servicing
			__enable_interrupt();
			tick();
			sys_event = 0;
		}
	}
}

inline int bapi_init(CLOCK_SPEED clock) {
	seed_rand();
	msp430_init(clock);
	gpio_port_init();
	usi_init();
	return 0;
}

inline void msp430_init(CLOCK_SPEED clock) {
	WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog
	if (*dco_cal[clock].calbc1 == 0xFF)   // If calibration constants erased
			{
		while (1)
			;                          // do not load, trap CPU!!
	}

	DCOCTL = 0;                      // Select lowest DCOx and MODx settings
	BCSCTL1 = *dco_cal[clock].calbc1;                          // Set DCO
	DCOCTL = *dco_cal[clock].caldco;

	// configure Watchdog
	WDTCTL = WDT_CTL;					// Set Watchdog interval
	IE1 |= WDTIE;					// Enable WDT interrupt
}

inline void gpio_port_init() {
	P1OUT = P2OUT = 0;
	P2SEL = P2SEL2 = P1SEL = P1SEL2 = 0;
	P1DIR = P2DIR = 0xFF;

	*PxDIR[LED0_PORT] |= LED0_PIN;
	*PxOUT[LED0_PORT] |= LED0_PIN;

}

void seed_rand() {
	int16_t seed;
	int16_t random[16];
	int i = 16;
	while (i-- > 0) {
		seed ^= random[i];
	}
	srand(seed);
	__no_operation();
}

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {
	check_timeout();
	if (tick_max && !(--tick_count)) {
		tick_count = tick_max;
		sys_event |= 0x01;
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}
	return;
} // end WDT_ISR

