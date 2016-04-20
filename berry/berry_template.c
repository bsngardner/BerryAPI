/*
 * main_template.c
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#include "msp430.h"

#include "berry.h"


//Global externs
extern register_table_t reg_table;

#define DEV_TYPE 0x00

uint8_t device_init(){

	return DEV_TYPE;
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
