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
enum { NONE = -1, PRESSED = 0, RELEASED = 1 };
static volatile int sw_state = NONE;
static volatile int debounce_cnt = 0;

//Function prototypes
void port_init();

//defines
#define DEV_TYPE 6

// interrupts
#define SWITCH_PRESSED 	0x1
#define SWITCH_RELEASED 0x2

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
	return DEV_TYPE;
}

void tick()
{
	// Wait until finished debouncing before asserting interrupt
	if (debounce_cnt && (--debounce_cnt))
		return;

	switch (sw_state)
	{
	case PRESSED:
		// Only assert interrupt bit if interrupt was enabled
		if (registers[-4] & SWITCH_PRESSED)
		{
			registers[-5] |= SWITCH_PRESSED;
			P1OUT &= ~BIT5; // low asserted interrupt
		}
		break;
	case RELEASED:
		if (registers[-4] & SWITCH_RELEASED)
		{
			registers[-5] |= SWITCH_RELEASED;
			P1OUT &= ~BIT5;
		}
		break;
	default:
		break;
	}
	tick_speed = 0;
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
	tick_speed = 1;
	debounce_cnt = 2;
	if (test(P1IN, SW0))
	{
		sw_state = RELEASED;
		P1IES |= SW0;
		registers[2] = 0;
	}
	else
	{
		sw_state = PRESSED;
		P1IES &= ~SW0;
		registers[2] = 1;
	}
}

