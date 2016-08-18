/*
 * Firware for servo controlling device, berry03-servo revision E?
 * 		Broderick Gardner
 * 		7/5/2016
 *
 *
 */

#include <msp430.h> 
#include "berry.h"
#include "timer.h"
#include "berry03-servo.h"

volatile int change_flag = 0;

uint8_t device_init()
{
	timer_init();
	tick_speed = WDT_HZ / 2;
	P1OUT |= BIT1;
	P1DIR |= BIT1;
	return DEV_TYPE;
}

void tick()
{
	int val = registers[R_OUT0_B];
	registers[R_OUT0_B] = (val + 64) & 0x0ff;
	change_flag = 1;
}

void set_register(uint8_t value)
{
	switch (current_register)
	{
	case R_CONFIG:
		registers[R_CONFIG] = value;
		break;
	case R_OUT0_B: //reg 2, output to servo 0, 1 byte
		registers[R_OUT0_B] = value;
		change_flag = 1;
		break;
	case R_OUT0_0: //reg 3, output to servo 0, lower byte of 12 bits
		current_register = R_OUT0_1;
		registers[R_OUT0_0] = value;
		break;
	case R_OUT0_1: //reg 4, output to servo 0, upper 4 bits of 12 bits
		current_register = R_OUT0_0;
		registers[R_OUT0_1] = value & 0x07;
		change_flag = 1;
		break;
	case R_OUT1_B: //reg 5, output to servo 1, 1 byte
		registers[R_OUT1_B] = value;
		change_flag = 1;
		break;
	case R_OUT1_0:	//reg 6, output to servo 1, lower byte of 12 bits
		registers[R_OUT1_0] = value;
		break;
	case R_OUT1_1:
		registers[R_OUT1_1] = value & 0x07;
		change_flag = 1;
		break;
	default:
		break;
	}
	return;
}

uint8_t get_register()
{
	switch (current_register)
	{
	case R_CONFIG:
		return registers[R_CONFIG];
	case R_OUT0_B:
		return registers[R_OUT0_B];
	case R_OUT0_0:
		current_register = R_OUT0_1;
		return registers[R_OUT0_0];
	case R_OUT0_1:
		current_register = R_OUT0_0;
		return registers[R_OUT0_1];
	case R_OUT1_B:
		return registers[R_OUT1_B];
	case R_OUT1_0:
		current_register = R_OUT1_1;
		return registers[R_OUT1_0];
	case R_OUT1_1:
		current_register = R_OUT1_0;
		return registers[R_OUT1_1];
	default:
		break;
	}
	return 0;
}
