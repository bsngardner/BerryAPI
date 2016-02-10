//	bb_i2c.c - Berry Generation II
//******************************************************************************
//******************************************************************************
//
//	Author:			Paul Roper, Brigham Young University
//	Revised:		Kristian Sims 1/15/2016
//
//******************************************************************************

#include <msp430.h>
#include <setjmp.h>
#include "bb_i2c.h"
#include "pins.h"

//******************************************************************************
inline void bb_i2c_clocklow(void);
inline void bb_i2c_clockhigh(void);
inline void bb_i2c_out_bit(uint8_t bit);
void bb_i2c_start_address(unsigned address, uint8_t rwFlag);
void bb_i2c_out_stop(void);
int bb_i2c_out8bits(uint8_t c);

//******************************************************************************
//	LCD Global variables (NOT ZERO'D!!!)
//
jmp_buf bb_i2c_context;				// error context
unsigned bb_i2c_delay;

//******************************************************************************
// Pin macros
//
#if 0
#define BB_I2C_CLOCK_LOW	{P1OUT &= ~BB_I2C_SCL_PIN; P1DIR |= BB_I2C_SCL_PIN;}
#define BB_I2C_CLOCK_HIGH	{P1DIR &= ~BB_I2C_SCL_PIN; P1OUT |= BB_I2C_SCL_PIN;}
#define BB_I2C_READ_CLOCK	(P1IN & BB_I2C_SCL_PIN)

#define BB_I2C_DATA_LOW		{P1OUT &= ~BB_I2C_SDA_PIN; P1DIR |= BB_I2C_SDA_PIN;}
#define BB_I2C_DATA_HIGH	{P1DIR &= ~BB_I2C_SDA_PIN; P1OUT |= BB_I2C_SDA_PIN;}
#define BB_I2C_READ_DATA	(P1IN & BB_I2C_SDA_PIN)
#else
#define BB_I2C_CLOCK_LOW	P1DIR |= BB_I2C_SCL_PIN;
#define BB_I2C_CLOCK_HIGH	P1DIR &= ~BB_I2C_SCL_PIN;
#define BB_I2C_READ_CLOCK	(P1IN & BB_I2C_SCL_PIN)

#define BB_I2C_DATA_LOW		P1DIR |= BB_I2C_SDA_PIN;
#define BB_I2C_DATA_HIGH	P1DIR &= ~BB_I2C_SDA_PIN;
#define BB_I2C_READ_DATA	(P1IN & BB_I2C_SDA_PIN)

#endif

//******************************************************************************
// Soft delay -- doesn't support changing clock speed
//
// Delay should be 1/4 of a period. 1 ms / rate (kHz) = period (ms)
// 1 ms / 400 = .0025 = 2.5 us / 4 = 0.625 us

// 1 MHz	I2C_DELAY = 1
// 8 MHz	I2C_DELAY = 5
// 12 MHz	I2C_DELAY = 7
// 16 MHz	I2C_DELAY = 10

#define BB_I2C_DELAY	25

//******************************************************************************
//	Initialize pins
//
uint8_t bb_i2c_init()
{
	unsigned i;

	// Set pins to GPIO mode
	P1SEL &= ~(BB_I2C_SCL_PIN | BB_I2C_SDA_PIN);
	P1SEL2 &= ~(BB_I2C_SCL_PIN | BB_I2C_SDA_PIN);

	// Set pullups and open-drain mode (pins high)
	P1DIR &= ~(BB_I2C_SCL_PIN | BB_I2C_SDA_PIN);
//	P1OUT |= BB_I2C_SCL_PIN | BB_I2C_SDA_PIN;
//	P1REN |= BB_I2C_SCL_PIN | BB_I2C_SDA_PIN;
	P1OUT &= ~(BB_I2C_SCL_PIN | BB_I2C_SDA_PIN);

	// output 9 clocks with SDA high
	for (i = 9; i > 0; i--)
	{
		bb_i2c_clocklow();				// clock SCL
		bb_i2c_clockhigh();
	}

	// send stop condition
	bb_i2c_out_stop();
	return 0;
}

//******************************************************************************
// Timed clock low
//
inline void bb_i2c_clocklow()
{
	BB_I2C_CLOCK_LOW;
	__delay_cycles(BB_I2C_DELAY);

	return;
}

//******************************************************************************
// Timed clock high
//
inline void bb_i2c_clockhigh()
{
	BB_I2C_CLOCK_HIGH;
	__delay_cycles(2*BB_I2C_DELAY);

	// wait for clock to rise (pullups/stretch)
	while (!BB_I2C_READ_CLOCK);

	return;
}

//******************************************************************************
// Output one bit
//
//	.         .__delay__.
//	|         |
//	|__delay__|
//	 ^
//	 ^--> 0/1 SDA
//
inline void bb_i2c_out_bit(uint8_t bit)
{
	bb_i2c_clocklow();		// drop clock

	if (bit)
		BB_I2C_DATA_HIGH	// set SDA high
	else
		BB_I2C_DATA_LOW	// or SDA low

	__delay_cycles(BB_I2C_DELAY);
	bb_i2c_clockhigh();		// raise clock

	return;
}

//******************************************************************************
// Output eight bits
//
//	.         .__delay__.      .          .__delay__.
//	|         |            x8  |          |^
//	|__delay__|                |__delay __|^
//	 ^                          ^          ^--> check for ack
//	 ^                          ^
//	 ^--> 0/1 SDA               ^--> 0/1 SDA
//
//	exit high impedance
//
int bb_i2c_out8bits(uint8_t c)
{
	uint8_t shift = 0x80;

	// output 8 bits during SDA low
	while (shift)
	{
		bb_i2c_out_bit(c & shift);
		shift >>= 1;
	}

	// look for slave ack, if not low, then error
	BB_I2C_CLOCK_LOW;
	BB_I2C_DATA_HIGH;
	__delay_cycles(2*BB_I2C_DELAY);
	bb_i2c_clockhigh();
	if (BB_I2C_READ_DATA)
	{
		// try again
		__delay_cycles(BB_I2C_DELAY);
		if (BB_I2C_READ_DATA)
			return 1;
	}
	BB_I2C_CLOCK_LOW; // Is this needed to end the ACK?

	return 0;
}

//******************************************************************************
//	exit w/high impedance SDA
//
void bb_i2c_start_address(unsigned address, uint8_t rwFlag)
{
	int error;

	// output start
	BB_I2C_DATA_HIGH;
	BB_I2C_CLOCK_HIGH;				// w/SCL & SDA high

	BB_I2C_DATA_LOW;				// output start (SDA high to low while SCL high)
	__delay_cycles(BB_I2C_DELAY);

	// output (address * 2 + read/write bit)
	if (error = bb_i2c_out8bits((address << 1) + rwFlag))
	{
		bb_i2c_out_stop();					// output stop 1st
		longjmp(bb_i2c_context, error);	//return error;
	}
	return;
}

//******************************************************************************
// Output stop condition
//
void bb_i2c_out_stop()
{
	bb_i2c_clocklow();					// put clock low
	BB_I2C_DATA_LOW;					// make sure SDA is low
	bb_i2c_clockhigh();				// clock high
	BB_I2C_DATA_HIGH;					// stop = low to high
	__delay_cycles(BB_I2C_DELAY);

	return;
}

//******************************************************************************
// Write bytes over i2c
//
void bb_i2c_write(unsigned address, uint8_t* data, int bytes)
{
	int error;
	bb_i2c_start_address(address, 0);	// output write address
	while (bytes--)					// write 8 bits
		if (error = bb_i2c_out8bits(*data++))
			longjmp(bb_i2c_context, error);	//return error
	bb_i2c_out_stop();					// output stop

	return;							// return success
}

//******************************************************************************
//	read bytes into buffer using i2c
//
//	IN:		address	i2c address
//			buffer	pointer to input buffer
//			bytes	# of bytes to read
//	OUT:	last byte read
//
uint8_t bb_i2c_read(unsigned address, uint8_t* buffer, int bytes)
{
	unsigned i, data;
	bb_i2c_start_address(address, 1);	// output read address

	while (bytes--)					// read 8 bits
	{
		for (i = 8; i > 0; i--)
		{
			bb_i2c_clocklow();
			BB_I2C_DATA_HIGH;
			bb_i2c_clockhigh();
			data <<= 1;
			if (BB_I2C_READ_DATA)
				data++;
		}

		// save data
		*buffer++ = data;

		// output ack or nack
		bb_i2c_clocklow();
		if (bytes)
			BB_I2C_DATA_LOW			// ack (0)
		else
			BB_I2C_DATA_HIGH		// nack (1)
		bb_i2c_clockhigh();
	}

	bb_i2c_out_stop();

	return data;
}

//******************************************************************************
//	read bytes from a register address into buffer using i2c
//
uint8_t i2c_reg_read(unsigned address, uint8_t reg, uint8_t* buffer, int bytes)
{
	int error;

	bb_i2c_start_address(address, 0);
	if (error = bb_i2c_out8bits(reg)) longjmp(bb_i2c_context, error);	//return error

	return bb_i2c_read(address, buffer, bytes);
}
