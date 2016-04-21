/*
 * main_template.c
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#include "msp430.h"

#include "berry.h"

#define DEV_TYPE 0x00

uint8_t device_init(){

	return DEV_TYPE;
}

void set_register(uint8_t value) {
	switch (current_register) {
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

	switch (current_register) {
	case 0:

		return registers[0];
	case 1:

		return registers[1];
		//....
	case 15:

		return registers[15];

	}
	return 0;
}
