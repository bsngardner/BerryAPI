/*
 * Firmware for Potentiometer device, berry09-pot revision E
 * 		Broderick Gardner
 * 		6/15/2016
 *
 * 	Potentiometer is sampled 16 times every 10 ms, the values are averaged
 * 	and the result is stored in registers 2 and 3 in the register table
 *
 *  Registers:
 *  [0] : 27 (device type)
 *  [1] : (status)
 *  [2] : adc result scaled to 1 byte
 *  [3] : adc result lower byte
 *  [4] : adc result upper byte
 *
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

#define DEV_TYPE 24
//Registers
//	2 - Config (0: on/off, 1: pwm)
//	3 - on/off
//	4 - PWM

const int led0_port = 1;
const int led0_pin = BIT0;

#define R_READ0 2
#define R_READ1 3
#define R_READB 4

void timer_init();

uint8_t device_init()
{
	P1DIR |= (BIT1 | BIT2);
	P1OUT |= (BIT1 | BIT2);
	P1SEL0 |= BIT3;
	P1SEL1 |= BIT3;
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
	case 2:
		break;
	case 3:
		break;
	case 4:
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
