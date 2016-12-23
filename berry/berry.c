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
#include "usi_i2c.h"

//local macros
#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))

//System registers
#define GUID0 -1
#define GUID1 -2
#define GUID2 -3
#define GUID3 -4
#define GUID4 -5
#define GUID5 -6
#define GUID6 -7
#define GUID7 -8

//Prototypes
int bapi_init();
void seed_rand();
void msp430_init(CLOCK_SPEED clock);
void gpio_port_init();
inline void check_timeout();

//Constants for clock speed init
static const struct
{
	volatile unsigned char* calbc1;
	volatile unsigned char* caldco;
} dco_cal[] =
{
{ &CALBC1_1MHZ, &CALDCO_1MHZ },
{ &CALBC1_8MHZ, &CALDCO_8MHZ },
{ &CALBC1_12MHZ, &CALDCO_12MHZ },
{ &CALBC1_16MHZ, &CALDCO_16MHZ } };

// Set Flash Timing Generator (needs to be between 257 and 476 kHz)
/* Divide Flash clock by 1 to 64 using FN0 to FN5 according to: */
/*  32*FN5 + 16*FN4 + 8*FN3 + 4*FN2 + 2*FN1 + FN0 + 1 */
static const uint16_t FCTL2_CLK_DIV[] =
{
FWKEY + FSSEL0 + FN1, // DCO 1 MHz, set to MCLK/3
		FWKEY + FSSEL0 + FN4 + FN2 + FN1 + FN0, // DCO 8 MHz, set to MCLK/24
		FWKEY + FSSEL0 + FN5 + FN1 + FN0, // DCO 12 MHz, set to MCLK/36
		FWKEY + FSSEL0 + FN5 + FN3 + FN2 + FN1 + FN0 // DCO 16 MHz, set to MCLK/48
};

//Const port arrays
volatile uint8_t* const PxOUT[3] =
{ 0, &P1OUT, &P2OUT };
volatile uint8_t* const PxDIR[3] =
{ 0, &P1DIR, &P2DIR };
volatile uint8_t* const PxIN[3] =
{ 0, &P1IN, &P2IN };

//Global variables
volatile uint8_t all_registers[TABLE_SIZE * 2] =
{ 0 };
volatile uint8_t* const registers = all_registers + (TABLE_SIZE);
volatile int8_t current_register = 0;

volatile uint16_t sys_event = 0;
volatile uint16_t tick_speed = 0;
volatile uint16_t tick_count = 1;

//Persistent memory - DO NOT USE SEGMENT A - IT IS FOR CALIBRATION DATA
typedef union
{
	struct
	{
		uint64_t guid;
	};
	struct
	{
		uint8_t guid0;
		uint8_t guid1;
		uint8_t guid2;
		uint8_t guid3;
		uint8_t guid4;
		uint8_t guid5;
		uint8_t guid6;
		uint8_t guid7;
	};
} flash_var_t; // 8 bytes

#pragma DATA_SECTION(flash_guid, ".infoB");
static flash_var_t flash_guid;

//Local function prototypes
static void flash_write_byte(uint8_t *address, uint8_t byte);
//static void flash_write_word(uint16_t *ptr, uint16_t word);
static void flash_delete_segment(uint16_t segment);
static void flash_update_event();

void main()
{
	bapi_init();
	registers[TYPE] = device_init();
	registers[GUID0] = flash_guid.guid0;
	registers[GUID1] = flash_guid.guid1;
	registers[GUID2] = flash_guid.guid2;
	registers[GUID3] = flash_guid.guid3;
	registers[GUID4] = flash_guid.guid4;
	registers[GUID5] = flash_guid.guid5;
	registers[GUID6] = flash_guid.guid6;
	registers[GUID7] = flash_guid.guid7;
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
		// Update persistent memory
		else if (sys_event & FLASH_UPDATE_EVENT)
		{
			sys_event &= ~FLASH_UPDATE_EVENT;
			flash_update_event();
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
		COND_BIT(value, *PxOUT[LED0_PORT], LED0_PIN);
		return;
	case GUID0:
		registers[GUID0] = value;
		current_register = GUID1;
		return;
	case GUID1:
		registers[GUID1] = value;
		current_register = GUID2;
		return;
	case GUID2:
		registers[GUID2] = value;
		current_register = GUID3;
		return;
	case GUID3:
		registers[GUID3] = value;
		current_register = GUID4;
		return;
	case GUID4:
		registers[GUID4] = value;
		current_register = GUID5;
		return;
	case GUID5:
		registers[GUID5] = value;
		current_register = GUID6;
		return;
	case GUID6:
		registers[GUID6] = value;
		current_register = GUID7;
		return;
	case GUID7:
		registers[GUID7] = value;
		current_register = GUID0;
		sys_event |= FLASH_UPDATE_EVENT; // MAKE SURE TO UPDATE THE GUID IN FLASH
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
	usi_init();
	return 0;
}

void msp430_init(CLOCK_SPEED clock)
{
	WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog
	if (*dco_cal[clock].calbc1 == 0xFF)   // If calibration constants erased
	{
		while (1)
			;        // do not load, trap CPU!!
	}
	DCOCTL = 0;                      // Select lowest DCOx and MODx settings
	BCSCTL1 = *dco_cal[clock].calbc1;                          // Set DCO
	DCOCTL = *dco_cal[clock].caldco;
	BCSCTL3 = LFXT1S_2; //Select VLO

	// Set Flash Timing Generator (needs to be between 257 and 476 kHz)
	FCTL2 = FCTL2_CLK_DIV[clock];

	// configure Watchdog
	WDTCTL = WDT_CTL;					// Set Watchdog interval
	IE1 |= WDTIE;					// Enable WDT interrupt
}

void gpio_port_init()
{
	P2SEL = P2SEL2 = P1SEL = P1SEL2 = 0;
	// interrupt pin is input (high impedance) until it needs to be asserted
	P1DIR = 0xFF & ~BINT_PIN;
	P2DIR = 0xFF;
	P1OUT =	P2OUT = 0;

	*PxDIR[LED0_PORT] |= LED0_PIN;
	*PxOUT[LED0_PORT] |= LED0_PIN;
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

static void flash_delete_segment(uint16_t segment)
{ // Argument is an address in desired segment
	uint8_t *flash_ptr;

	flash_ptr = (uint8_t*) segment; // Initialize pointer
	FCTL3 = FWKEY;                  // Clear lock-bit
	FCTL1 = (FWKEY | ERASE);        // Set erase-bit
	*flash_ptr = 0;                 // Dummy-write to delete segment - CPU hold
	while ((BUSY & FCTL3))
		;
	FCTL3 = (FWKEY | LOCK);         // Set lock-bit
}

static void flash_write_byte(uint8_t *address, uint8_t byte)
{
	uint8_t *flash_ptr;

	flash_ptr = address; // Initialize pointer
	FCTL3 = FWKEY;// Clear lock-bit
	FCTL1 = (FWKEY | WRT);// Set write-bit
	*flash_ptr = byte;// Write byte - CPU hold
	while ((BUSY & FCTL3))
	;
	FCTL1 = FWKEY;// Clear write-bit
	FCTL3 = (FWKEY | LOCK);// Set lock-bit
}

#if 0
static void flash_write_word(uint16_t* ptr, uint16_t word)
{
	FCTL3 = FWKEY;                  // Clear lock-bit
	FCTL1 = (FWKEY | WRT);          // Set write-bit
	*ptr = word;              // Write byte - CPU hold
	while ((BUSY & FCTL3))
		;
	FCTL1 = FWKEY;                  // Clear write-bit
	FCTL3 = (FWKEY | LOCK);         // Set lock-bit
}
#endif

static void flash_update_event()
{
	static uint16_t flash_segment_addr = (uint16_t)&flash_guid;
	short sr = __get_SR_register();
	__disable_interrupt();

	// erase the segment, then you can write to flash
    flash_delete_segment(flash_segment_addr);
    flash_write_byte(&flash_guid.guid0, registers[GUID0]);
    flash_write_byte(&flash_guid.guid1, registers[GUID1]);
    flash_write_byte(&flash_guid.guid2, registers[GUID2]);
    flash_write_byte(&flash_guid.guid3, registers[GUID3]);
    flash_write_byte(&flash_guid.guid4, registers[GUID4]);
    flash_write_byte(&flash_guid.guid5, registers[GUID5]);
    flash_write_byte(&flash_guid.guid6, registers[GUID6]);
    flash_write_byte(&flash_guid.guid7, registers[GUID7]);

	__bis_SR_register(sr & GIE);
}

void delayed_copy_to_flash(volatile uint16_t *local_data, uint16_t new_data,
		uint16_t event)
{
// update the local copy
	*local_data = new_data;

// queue event so data will be copied to flash
	sys_event |= event;
}

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	check_timeout();
	if (tick_speed && !(--tick_count))
	{
		tick_count = tick_speed;
		sys_event |= TICK_EVENT;
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}
	return;
} // end WDT_ISR
