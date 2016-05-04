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
		FWKEY + FSSEL0+ FN1, // DCO 1 MHz, set to MCLK/3
		FWKEY + FSSEL0+ FN4 + FN2 + FN1 + FN0, // DCO 8 MHz, set to MCLK/24
		FWKEY + FSSEL0+ FN5 + FN1 + FN0, // DCO 12 MHz, set to MCLK/36
		FWKEY + FSSEL0+ FN5 + FN3 + FN2 + FN1 + FN0 // DCO 16 MHz, set to MCLK/48
};

//Const port arrays
volatile uint8_t* const PxOUT[3] =
{ 0, &P1OUT, &P2OUT };
volatile uint8_t* const PxDIR[3] =
{ 0, &P1DIR, &P2DIR };
volatile uint8_t* const PxIN[3] =
{ 0, &P1IN, &P2IN };

//Global variables
volatile uint8_t registers[TABLE_SIZE] =
{ 0 };
volatile uint16_t current_register = 0;

volatile uint16_t sys_event = 0;
volatile uint16_t tick_speed = 0;
volatile uint16_t tick_count = 1;

//Persistent memory - DO NOT USE SEGMENT A - IT IS FOR CALIBRATION DATA
#pragma DATA_SECTION(persistent_vars, ".infoB");
struct
{
	uint8_t flash_proj_hash;
	uint8_t flash_slave_addr;
} persistent_vars;

// Copies of persistent variables in RAM
extern volatile uint8_t proj_hash;
extern volatile uint8_t slave_addr;

//Local function prototypes
static void flash_write_byte(uint16_t address, uint8_t byte);
static void flash_delete_segment(uint16_t segment);
static void flash_update_event();
static void project_mem_init();

void main()
{
	bapi_init();
	registers[TYPE_REG] = device_init();
	tick_count = 1;

	// Read persistent variables into local copies
	project_mem_init();

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
	P1OUT = P2OUT = 0;
	P2SEL = P2SEL2 = P1SEL = P1SEL2 = 0;
	P1DIR = P2DIR = 0xFF;

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

static void project_mem_init()
{
	// Copy project hash and berry address into local memory
	proj_hash = persistent_vars.flash_proj_hash;
	slave_addr = persistent_vars.flash_slave_addr;

	// If slave address is 0xff, it was just programmed and should be reset
	if (slave_addr == 0xff)
	{
		// Immediately clear slave addr and project key
		slave_addr = proj_hash = 0;
		flash_update_event();
	}
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

static void flash_write_byte(uint16_t address, uint8_t byte)
{
	uint8_t *flash_ptr;

	flash_ptr = (uint8_t*) address; // Initialize pointer
	FCTL3 = FWKEY;                  // Clear lock-bit
	FCTL1 = (FWKEY | WRT);          // Set write-bit
	*flash_ptr = byte;              // Write byte - CPU hold
	while ((BUSY & FCTL3))
		;
	FCTL1 = FWKEY;                  // Clear write-bit
	FCTL3 = (FWKEY | LOCK);         // Set lock-bit
}

static void flash_update_event()
{
	short sr = __get_SR_register();
	__disable_interrupt();
	// erase the segment, then you can write to flash
	flash_delete_segment((uint16_t) &persistent_vars.flash_proj_hash);
	flash_write_byte((uint16_t) &persistent_vars.flash_proj_hash, proj_hash);
	flash_write_byte((uint16_t) &persistent_vars.flash_slave_addr, slave_addr);
	__bis_SR_register(sr & GIE);
}

void delayed_copy_to_flash(volatile uint8_t *local_data, uint8_t byte,
		uint16_t event)
{
	// update the local copy
	*local_data = byte;

	// queue event so data will be copied to flash
	sys_event |= event;
}

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
//	check_timeout();
	if (tick_speed && !(--tick_count))
	{
		tick_count = tick_speed;
		sys_event |= TICK_EVENT;
		__bic_SR_register_on_exit(LPM0_bits); // wake up on exit
	}
	return;
} // end WDT_ISR
