/*
 * pressure.c
 *
 *  Created on: Aug 25, 2016
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
 *  [5] : pressure level (in range [0,4])
 *
 *  Bend Levels (10-bit adc value converted to an 8-bit value):
 *  Straight ~ 80
 *  Very bent ~ 160
 *    0: adc value in range [0,100)
 *    1: adc value in range [100,120)
 *    2: adc value in range [120,140)
 *    3: adc value in range [140,255)
 *
 *  Interrupts:
 *    0x01: on pressed - threshold Level 1
 *    0x02: on pressed - threshold Level 2
 *    0x04: on pressed - threshold Level 3
 *    0x08: on pressed - threshold Level 4
 *    0x10: on level changed
 *    0x20: on released
 *
 * 	Resources used:
 * 		Timer A
 * 		ADC10
 * 	Code Size: 3548
 * 	RAM usage: 290
 *
 */

#include "berry30-flex.h"

#define DEV_TYPE 30

const int led0_port = 0;
const int led0_pin = BIT1;

#define VCC_PIN (BIT5)
#define GND_PIN (BIT2 | BIT3)
#define PRESSURE_PIN BIT4

uint8_t device_init()
{
	// initialize pins
	P1DIR |= VCC_PIN | GND_PIN;
	P1OUT |= VCC_PIN; // vref
	P1OUT &= ~GND_PIN;
	P1SEL0 |= PRESSURE_PIN; // use for the adc
	P1SEL1 |= PRESSURE_PIN; // use for the adc

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
	case R_READ8:
		current_register = R_LEVEL;
		return registers[R_READ8];
	case R_LEVEL:
		return registers[R_LEVEL];
	case R_READ10_LO:
		current_register = R_READ10_HI;
		return registers[R_READ10_LO];
	case R_READ10_HI:
		return registers[R_READ10_HI];
	default:
		break;
	}
	return 0;
}

