//	bb_i2c.c - Berry Generation II
//******************************************************************************
//******************************************************************************
//
//	Author:			Paul Roper, Brigham Young University
//	Revised:		Kristian Sims 1/15/2016
//
//******************************************************************************

#include "bb_i2c.h"
#include <msp430.h>
#include <setjmp.h>
#include "pins.h"

//******************************************************************************
void bb_i2c_clocklow(void);
void bb_i2c_clockhigh(void);
void bb_i2c_out_bit(uint8 bit);
void bb_i2c_start_address(uint16 address, uint8 rwFlag);
void bb_i2c_out_stop(void);
int bb_i2c_out8bits(uint8 c);

//******************************************************************************
//	LCD Global variables (NOT ZERO'D!!!)
//
jmp_buf bb_i2c_context;				// error context
uint16 bb_i2c_delay;

//******************************************************************************
// Pin macros
//
#define
#define BB_I2C_CLOCK_LOW	P1OUT &= ~BB_I2C_SCL_PIN; P1DIR |= BB_I2C_SCL_PIN;
#define BB_I2C_CLOCK_HIGH	P1DIR &= ~BB_I2C_SCL_PIN; P1OUT |= BB_I2C_SCL_PIN;
#define BB_I2C_READ_CLOCK	(P1IN & BB_I2C_SCL_PIN)

#define BB_I2C_DATA_LOW		P1OUT &= ~BB_I2C_SDA_PIN; P1DIR |= BB_I2C_SDA_PIN;
#define BB_I2C_DATA_HIGH	P1DIR &= ~BB_I2C_SDA_PIN; P1OUT |= BB_I2C_SDA_PIN;
#define BB_I2C_READ_DATA	(P1IN & BB_I2C_SDA_PIN)

//******************************************************************************
// Soft delay -- doesn't support changing clock speed
//
// Delay should be half a period. 1 ms / rate (kHz) = period (ms)
// 1 ms / 400 = .0025 = 2.5 us / 2 = 1.25 us

// 1 MHz	I2C_DELAY = 1 (instructions take time too)
// 8 MHz	I2C_DELAY = 10
// 12 MHz	I2C_DELAY = 15
// 16 MHz	I2C_DELAY = 20

#define BB_I2C_DELAY	20

//******************************************************************************
//	Initialize pins
//
uint8 i2c_init()
{
	uint16 i;

	// Set pins to GPIO mode
	P1SEL &= ~(BB_I2C_SCL_PIN | BB_I2C_SDA_PIN);
	P1SEL2 &= ~(BB_I2C_SCL_PIN | BB_I2C_SDA_PIN);

	// Set pullups and open-drain mode (pins high)
	P1DIR &= ~(BB_I2C_SCL_PIN | BB_I2C_SDA_PIN);
	P1OUT |= BB_I2C_SCL_PIN | BB_I2C_SDA_PIN;
	P1REN |= BB_I2C_SCL_PIN | BB_I2C_SDA_PIN;

	// output 9 clocks with SDA high
	for (i = 9; i > 0; i--)
	{
		i2c_clocklow();				// clock SCL
		i2c_clockhigh();
	}

	// send stop condition
	i2c_out_stop();
	return 0;
}

//******************************************************************************
// Timed clock low
//
inline void i2c_clocklow()
{
	I2C_CLOCK_LOW;
	__delay_cycles(BB_I2C_DELAY);

	return;
}

//******************************************************************************
// Timed clock high
//
inline void i2c_clockhigh()
{
	I2C_CLOCK_HIGH;
	__delay_cycles(BB_I2C_DELAY)

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
inline void i2c_out_bit(uint8 bit)
{
	i2c_clocklow();		// drop clock
	if (bit)
		I2C_DATA_HIGH;	// set SDA high
	else
		I2C_DATA_LOW;	// or SDA low
	i2c_clockhigh();	// raise clock

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
int i2c_out8bits(uint8 c)
{
	uint8 shift = 0x80;

	// output 8 bits during SDA low
	while (shift)
	{
		i2c_out_bit(c & shift);
		shift >>= 1;
	}

	// look for slave ack, if not low, then error
	I2C_CLOCK_LOW;
	I2C_DATA_HIGH;
	i2c_clockhigh();
	if (BB_I2C_READ_DATA)
	{
		// try again
		__delay_cycles(BB_I2C_DELAY);
		if (P1IN & SDA)
			return SYS_ERR_I2C_ACK;
	}
	I2C_CLOCK_LOW; // Is this needed to end the ACK?

	return 0;
}

//******************************************************************************
//	exit w/high impedance SDA
//
void i2c_start_address(uint16 address, uint8 rwFlag)
{
	int error;

	// output start
	I2C_DATA_HIGH;
	I2C_CLOCK_HIGH;				// w/SCL & SDA high

	I2C_DATA_LOW;				// output start (SDA high to low while SCL high)
	__delay_cycles(BB_I2C_DELAY);

	// output (address * 2 + read/write bit)
	if (error = i2c_out8bits((address << 1) + rwFlag))
	{
		i2c_out_stop();					// output stop 1st
		longjmp(i2c_context, error);	//return error;
	}
	return;
} // end i2c_out_address

//******************************************************************************
// Output stop condition
//
void i2c_out_stop()
{
	i2c_clocklow();					// put clock low
	I2C_DATA_LOW;					// make sure SDA is low
	i2c_clockhigh();				// clock high
	I2C_DATA_HIGH;					// stop = low to high
	__delay_cycles(BB_I2C_DELAY);

	return;
} // end i2c_out_stop

//******************************************************************************
// Write bytes over i2c
//
void bb_i2c_write(uint16 address, uint8* data, int16 bytes)
{
	int error;
	i2c_start_address(address, 0);	// output write address
	while (bytes--)					// write 8 bits
		if (error = i2c_out8bits(*data++))
			longjmp(i2c_context, error);	//return error
	i2c_out_stop();					// output stop

	return;							// return success
} // end i2c_write

//******************************************************************************
//	read bytes into buffer using i2c
//
//	IN:		address	i2c address
//			buffer	pointer to input buffer
//			bytes	# of bytes to read
//	OUT:	last byte read
//
uint8 bb_i2c_read(uint16 address, uint8* buffer, int16 bytes)
{
	uint16 i, data;
	i2c_start_address(address, 1);	// output read address

	while (bytes--)					// read 8 bits
	{
		for (i = 8; i > 0; i--)
		{
			i2c_clocklow();			// I2C_CLOCK_LOW;
			I2C_DATA_HIGH;			// high impedance
			i2c_clockhigh();		// I2C_CLOCK_HIGH;
			data <<= 1;				// assume 0
			if (P1IN & SDA)
				data++;
		}

		// save data
		*buffer++ = data;

		// output ack or nack
		i2c_clocklow();				// I2C_CLOCK_LOW;
		if (bytes)
			I2C_DATA_LOW;	// ack (0)
		else
			I2C_DATA_HIGH;			// nack (1)
		i2c_clockhigh();			// I2C_CLOCK_HIGH;
	}

	i2c_out_stop();					// output stop

	return data;
} // end i2c_read

//******************************************************************************
//	read bytes from a register address into buffer using i2c
//
uint8 i2c_reg_read(uint16 address, uint8 reg, uint8* buffer, int16 bytes)
{
	int error;

	i2c_start_address(address, 0);
	if (error = i2c_out8bits(reg)) longjmp(i2c_context, error);	//return error

	return bb_i2c_read(address, buffer, bytes);
}
