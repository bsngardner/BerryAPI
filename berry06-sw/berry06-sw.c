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
#include "berry.h"

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
#define DEV_TYPE 6

//Register table mapped to PxOUT

void port_init()
{
	P1OUT |= SW0;
	P1DIR &= ~SW0;
	P1REN |= SW0;
	P1IES |= SW0;
	P1IE |= SW0;
}

uint8_t device_init()
{
	port_init();
	tick_speed = WDT_HZ / 2;
	return DEV_TYPE;
}

void tick()
{
	P2OUT ^= LED0_PIN;
}

void set_register(uint8_t value)
{
	switch (current_register)
	{
	case 2:
		//reg_table.table[2] = value;
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
	case 2:
		return registers[2];
	default:
		return registers[0];
	}
}

#define test(byte,bit) (byte & bit)

#pragma vector = PORT1_VECTOR
__interrupt void p1_isr(void)
{
	P1IFG &= ~SW0;
	if (test(P1IN, SW0))
	{
		P1IES |= SW0;
		registers[2] = 0;
	}
	else
	{
		P1IES &= ~SW0;
		registers[2] = 1;
	}
}

