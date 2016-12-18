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
#define GUID_LO_ADDR 0x01A0A
#define GUID_HI_ADDR 0x01A11
#define GUID_LEN 8

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
volatile uint16_t slave_addr = 0;

#pragma DATA_SECTION(guid, ".infoA")
volatile uint64_t guid;

//Local function prototypes
void get_guid();

void main()
{
	get_guid();
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

void get_guid()
{
	uint8_t *addr = (uint8_t*) GUID_LO_ADDR;
	guid = (
			((uint64_t)*(addr + 0) << 56) |
			((uint64_t)*(addr + 1) << 48) |
			((uint64_t)*(addr + 2) << 40) |
			((uint64_t)*(addr + 3) << 32) |
			((uint64_t)*(addr + 4) << 24) |
			((uint64_t)*(addr + 5) << 16) |
			((uint64_t)*(addr + 6) << 8) |
			((uint64_t)*(addr + 7))
			);
	registers[GUID0] = (uint8_t) (guid & 0xff);
	registers[GUID1] = (uint8_t) ((guid >> 8) & 0xff);
	registers[GUID2] = (uint8_t) ((guid >> 16) & 0xff);
	registers[GUID3] = (uint8_t) ((guid >> 24) & 0xff);
	registers[GUID4] = (uint8_t) ((guid >> 32) & 0xff);
	registers[GUID5] = (uint8_t) ((guid >> 40) & 0xff);
	registers[GUID6] = (uint8_t) ((guid >> 48) & 0xff);
	registers[GUID7] = (uint8_t) ((guid >> 56) & 0xff);
}

void sys_set_register(uint8_t value)
{
	switch (current_register)
	{
	case STATUS:
		registers[STATUS] = value;
		//conditionally set or clear status led according to value
		COND_BIT(value, *PxOUT[led0_port], led0_pin);
		return;
	case INT_ENABLE:
		registers[INT_ENABLE] = value;
		return;
	default:
		//All other system registers are read-only
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
	case GUID0:
		current_register = GUID1;
		return registers[GUID0];
	case GUID1:
		current_register = GUID2;
		return registers[GUID1];
	case GUID2:
		current_register = GUID3;
		return registers[GUID2];
	case GUID3:
		current_register = GUID4;
		return registers[GUID3];
	case GUID4:
		current_register = GUID5;
		return registers[GUID4];
	case GUID5:
		current_register = GUID6;
		return registers[GUID5];
	case GUID6:
		current_register = GUID7;
		return registers[GUID6];
	case GUID7:
		current_register = GUID0;
		return registers[GUID7];
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

	ADC10CTL0 &= ADC10ENC;
	ADC10CTL0 = ADC10SHT_0 | ADC10MSC | ADC10ON;
	ADC10CTL1 = ADC10SHP | ADC10DIV_0 | ADC10SSEL_2 | ADC10CONSEQ_2;
	ADC10CTL2 = ADC10PDIV_2 | ADC10RES;
	ADC10MCTL0 = ADC10SREF_0 | ADC10INCH_10;

	ADC10IE = 0;

	ADC10CTL0 |= ADC10ENC | ADC10SC;

	uint16_t count = 16;
	while (count --> 0)
	{
		while (!(ADC10IFG & ADC10IFG0))
			;
		int read = ADC10MEM0;
		ADC10IFG &= ~ADC10IFG0;
		seed = (seed << 1) | (read & 0x01);
	}
	srand(seed);

	ADC10CTL0 &= ~ADC10ENC;
	ADC10CTL0 = 0;
	ADC10CTL1 = 0;
	ADC10CTL2 = ADC10RES;
	ADC10MCTL0 = 0;

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
