/*
 * main_template.c
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#include "msp430.h"
#include "bapi.h"
//#include "pins.h"

//Function Prototypes
void set_register(uint8_t value);
uint8_t get_register();

//Global variable externs
extern register_table_t reg_table;

#define TYPE 02 //LED

void main() {
	bapi_init(_16MHZ, TYPE);

}

void set_register(uint8_t value) {
	switch (reg_table.current) {
	case 0:

		break;
	case 1:

		break;
		//....
	case 15:

		break;

	}

	return;
}

uint8_t get_register() {

	switch (reg_table.current) {
	case 0:

		return reg_table.table[0];
	case 1:

		return reg_table.table[1];
		//....
	case 15:

		return reg_table.table[15];

	}
	return 0;
}

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {
	check_timeout();


	return;
} // end WDT_ISR
