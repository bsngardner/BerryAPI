/*
 * bapiLib.c
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#include "msp430.h"
#include "bapi.h"
#include "bapi.h"
#include <stdlib.h>
#include "usi_i2c.h"

//Prototypes
void seed_rand();
void msp430_init(CLOCK_SPEED clock);
void port_init();

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

int bapi_init(CLOCK_SPEED clock, uint8_t device_type) {
	seed_rand();
	msp430_init(clock);
	port_init();
	init_usi();
	reg_table.table[0] = device_type;
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

inline void port_init() {
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
