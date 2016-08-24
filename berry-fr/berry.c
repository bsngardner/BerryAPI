/*
 * bapiLib.c
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#include "msp430.h"
#include <stdlib.h>
#include <stdint.h>

#include "berry.h"
#include "pins.h"
#include "usci_i2c.h"

//local macros
#define COND_BIT(bool,byte,mask) ((byte) ^= ((-bool) ^ (byte)) & (mask))

//Prototypes
int bapi_init();
void seed_rand();
void msp430_init(CLOCK_SPEED clock);
void gpio_port_init();

volatile unsigned char * const PxOUT[3] =
{ &PJOUT_L, &P1OUT, &P2OUT };
volatile unsigned char * const PxIN[3] =
{ &PJIN_L, &P1IN, &P2IN };
volatile unsigned char * const PxDIR[3] =
{ &PJDIR_L, &P1DIR, &P2DIR };

//Status LED
extern const int led0_port;
extern const int led0_pin;

//Global variables
volatile uint8_t all_registers[TABLE_SIZE * 2] =
{ 0 };
volatile uint8_t* const registers = all_registers + (TABLE_SIZE);
volatile int16_t current_register = 0;

volatile uint16_t sys_event = 0;
volatile uint16_t tick_speed = 0;
volatile uint16_t tick_count = 1;

// Copies of persistent variables in RAM
#pragma PERSISTENT(proj_key)
volatile uint16_t proj_key = 0;
#pragma PERSISTENT(slave_addr)
volatile uint16_t slave_addr = 0;

//Local function prototypes

void main()
{
	bapi_init();
	registers[TYPE] = device_init();
	tick_count = 1;

// Enable global interrupts after all initialization is finished.
	__enable_interrupt();

// Wait for an interrupt
	while (1)
	{
		// disable interrupts before check sys_event
		__disable_interrupt();

		if (!sys_event)
		{
			// no events pending, enable interrupts and goto sleep (LPM3)
			__bis_SR_register(LPM0_bits | GIE);
			continue;
		}

		// at least 1 event is pending, enable interrupts before servicing
		__enable_interrupt();

		// User-defined tick function
		if (sys_event & TICK_EVENT)
		{
			sys_event &= ~TICK_EVENT;
			tick();
		}
		else
		{
			sys_event = 0; //TODO replace with proper error handling
		}
	}
}

void sys_set_register(uint8_t value)
{
	switch (current_register)
	{
	case TYPE:
		//Dont change my type!!!
		return;
	case STATUS:
		registers[STATUS] = value;
		//conditionally set or clear status led according to value
		COND_BIT(value, *PxOUT[led0_port], led0_pin);
		return;
	case INT_ENABLE:
		registers[INT_ENABLE] = value;
		return;
	case INTERRUPT:
		//Read only!
		return;
	default:
		break;
	}
	return;
}

uint8_t sys_get_register()
{
	uint8_t ret;
	switch (current_register)
	{
	case TYPE:
		return registers[TYPE];
	case STATUS:
		return registers[STATUS];
	case INT_ENABLE:
		return registers[INT_ENABLE];
	case INTERRUPT:
		//Release interrupt line and clear interrupts when read
		RELEASE_INTR;
		ret = registers[INTERRUPT];
		registers[INTERRUPT] = 0;
		return ret;
	default:
		break;
	}
	return 0;
}

int bapi_init()
{
	seed_rand();
	msp430_init(CLOCK);
	gpio_port_init();
	usci_init();
	return 0;
}

// Initialize the msp430 clock and crystal
int setClock()
{
	CSCTL0 = CSKEY; 				// Enable Clock access
	CSCTL1 &= ~(DCORSEL | 0x0006); 	// Clear clock control bits
	CSCTL3 = 0;						// Clear all dividers
	CSCTL1 |= DCORSEL | 0x0006; 	// Set clock to 24 MHz

	CSCTL0_H = 0x00; // Disable Clock access

	return 0;
}

void msp430_init(CLOCK_SPEED clock)
{
	WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog
	setClock();
// configure Watchdog
	WDTCTL = WDT_CTL;					// Set Watchdog interval
	SFRIE1 |= WDTIE;					// Enable WDT interrupt
}

void gpio_port_init()
{
	P1OUT = P2OUT = 0;
	P2SEL0 = P2SEL1 = P1SEL0 = P1SEL1 = 0;

	*PxDIR[led0_port] |= led0_pin;
	*PxOUT[led0_port] |= led0_pin;
}

void seed_rand()
{
	int16_t seed;
	int16_t random[16];
	int i = 16;
	while (i-- > 0)
	{
		seed ^= random[i];
	}
	srand(seed);
}

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	//check_timeout();
	if (tick_speed && !(--tick_count))
	{
		tick_count = tick_speed;
		sys_event |= TICK_EVENT;
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}
	return;
} // end WDT_ISR
