//******************************************************************************
//
//******************************************************************************
/*
 *
 *	Register table:
 *	[0]:	Device type
 * 	[1]:	Status
 * 	[2]:	Command
 * 	[3..]:	Data
 *
 *
 * switch Berry:
 *	[0]:	0x06
 *	[1]:	status
 *
 *
 */

#include <msp430.h>
#include "bapi.h"

#include <stdint.h>
#include "string.h"

//Defines
#define SW 2
#define SWPORT P1IN
#define SW0  BIT0

//Global variables

//Function prototypes
void port_init();

//defines
#define TYPE 6

#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))
//Register table mapped to PxOUT

void port_init() {
	P1OUT |= SW0;
	P1DIR &= ~SW0;
	P1REN |= SW0;
}
//
////Main
//int main(void) {
//	bapi_init(_16MHZ, TYPE);
//	port_init();
//	__enable_interrupt();
//
//	while (1) {
//		LPM0;                              // CPU off, await USI interrupt
//		__no_operation();                  // Used for IAR
//
//	}
//}

void set_register(uint8_t value) {
	switch (reg_table.current) {
	case 2:
		reg_table.table[2] = value;
		break;
	default:
		break;
	}

	return;
}

uint8_t get_register() {

	switch (reg_table.current) {
	case 2:

		return reg_table.table[2];
	default:
		return reg_table.table[0];

	}
}
#define test(byte,bit) (byte & bit)
//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {

	check_timeout();

	if (!test(SWPORT, SW0)) {
		reg_table.table[SW] = 1;
		P2OUT |= BIT6;
	} else {
		reg_table.table[SW] = 0;
		P2OUT &= ~BIT6;
	}
	return;
} // end WDT_ISR
