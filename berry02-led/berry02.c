/*
 *
 *
 */

#include "berry.h"
#include <msp430.h> 

#include <stdint.h>
#include <stdlib.h>
#include "string.h"

static const int log_table[] =
{ 16384, 13653, 12056, 10923, 10044, 9325, 8718, 8192, 7728, 7313, 6937, 6595,
		6279, 5987, 5716, 5461, 5223, 4997, 4784, 4582, 4390, 4207, 4032, 3864,
		3703, 3549, 3400, 3257, 3118, 2985, 2856, 2731, 2609, 2492, 2378, 2267,
		2159, 2054, 1951, 1852, 1754, 1659, 1567, 1476, 1388, 1301, 1216, 1133,
		1052, 973, 894, 818, 743, 669, 597, 526, 456, 388, 320, 254, 189, 125,
		62, 0 };

//Defines
#define DEV_TYPE 0x02	//LED type

#define LED1 2
#define LED2 3
#define LED3 4
#define LED4 5
#define READ 6

#define L1 BIT0
#define L2 BIT1
#define L3 BIT2
#define L4 BIT3

//Macros
#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))

volatile char step;
volatile int speed;

//Function prototypes

//IO port init function
inline void port_init()
{
	P1DIR |= (L1 | L2 | L3 | L4);
	P1OUT |= ( L2 | L3);
}

uint8_t device_init()
{
	port_init();
	tick_speed = WDT_HZ/2;
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
	case 0:	//Type register
		//BAD! Should not set type register, read only
		break;
	case 1:	//Status register

		break;
	case LED1:	//LED 1
		registers[LED1] = value;
		COND_BIT(value, P1OUT, L1);
		current_register = LED2;
		break;
	case LED2:	//LED 2
		registers[LED2] = value;
		COND_BIT(value, P1OUT, L2);
		current_register = LED3;
		break;
	case LED3:	//LED 3
		registers[LED3] = value;
		COND_BIT(value, P1OUT, L3);
		current_register = LED4;
		break;
	case LED4:
		registers[LED4] = value;
		COND_BIT(value, P1OUT, L4);
		current_register = LED1;
		break;
	case 15:
		current_register = LED1;
		break;
	default:
		//registers.current++;
		break;
	}
}

uint8_t get_register()
{
	switch (current_register)
	{
	case 0:	//Type register
		return registers[0];
	case 1:	//Status register
		return registers[1];
	case LED1:	//LED 1
		current_register++;
		return registers[LED1];
	case LED2:	//LED 2
		current_register++;
		return registers[LED2];
	case LED3:	//LED 3
		current_register++;
		return registers[LED3];
	case LED4:
		current_register = LED1;
		return registers[LED4];
	case READ:
		registers[READ] = rand() & (255);
		return registers[READ];
	case 15:
		current_register = 1;
		break;
	default:
		//current_register++;
		break;
	}
	return 0;
}
