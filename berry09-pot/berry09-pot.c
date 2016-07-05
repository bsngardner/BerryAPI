/*
 * Firmware for Potentiometer device, berry09-pot revision E
 * 		Broderick Gardner
 * 		6/15/2016
 *
 * 	Potentiometer is sampled 16 times every 10 ms, the values are averaged
 * 	and the result is stored in registers 2 and 3 in the register table
 *
 *  Registers:
 *  [0] : 9 (device type)
 *  [1] : (status)
 *  [2] : adc result lower byte
 *  [3] : adc result upper byte
 *  [4] : adc result scaled to 1 byte
 *
 *  Resources used:
 *  	Timer A
 *  	ADC10
 *  Code Size:
 *  	2,500/4064 bytes
 *  RAM usage:
 *  	193/256 bytes
 *
 * main.c
 */

#include <msp430.h> 

#include "berry.h"
#include "adc.h"

#define DEV_TYPE 0x09

#define R_READ0 2
#define R_READ1 3
#define R_READB 4

uint8_t device_init()
{
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

uint8_t get_register()
{

	switch (current_register)
	{
	//On reading each byte of the potentiometer value,
	//	switch current register to the other byte
	//	This allows reading 2 bytes in quick succession
	case R_READ0:
		current_register = R_READ1;
		return registers[R_READ0];
	case R_READ1:
		current_register = R_READ0;
		return registers[R_READ1];
	case R_READB:
		return registers[R_READB];
	default:
		break;
	}
	return 0;
}
