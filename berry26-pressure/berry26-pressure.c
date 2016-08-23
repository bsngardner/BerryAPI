/*
 * pressure.c
 *
 *  Created on: Aug 19, 2016
 *      Author: Marshall Garey
 *
 * 	Resistor is sampled 16 times every 10 ms, the values are averaged
 * 	and the result is stored in registers 2 and 3 in the register table
 *
 * 	Registers:
 * 	[0] : 26 (device type)
 * 	[1] : (status)
 *  [2] : adc result scaled to 1 byte
 *  [3] : adc result lower byte
 *  [4] : adc result upper byte
 *  [5] : pressure level in range [0,3]
 *
 *  Pressure Levels:
 *    0: not pressed
 *    1: adc value in range [...,...)
 *    2: adc value in range [...,...)
 *    3: adc value in range [...,...]
 *
 * 	Resources used:
 * 		Timer A
 * 		ADC10
 * 	Code Size:
 * 	RAM usage:
 *
 */

#include <msp430.h>
#include "berry.h"
#include "adc.h"

#define DEV_TYPE 26

#define R_READ0 2
#define R_READ1 3
#define R_READB 4
#define R_LEVEL 5

uint8_t device_init()
{
	// todo: initialize pins
	adc_init();

	return DEV_TYPE;
}

void tick()
{

}

void set_register(uint8_t value)
{
	switch (current_register)
	{
	default:
		break;
	}
}

uint8_t get_register()
{
	switch (current_register)
	{
	case R_READ0:
		current_register = R_READ1;
		return registers[R_READ0];
	case R_READ1:
		current_register = R_READB;
		return registers[R_READ1];
	case R_READB:
		return registers[R_READB];
	case R_LEVEL:
		return registers[R_LEVEL];
	default:
		break;
	}
	return 0;
}

