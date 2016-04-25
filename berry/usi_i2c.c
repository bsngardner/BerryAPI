/*
 * usi_i2c.c
 *
 *  Created on: Nov 5, 2015
 *      Author: Broderick
 */

#include "msp430.h"
#include "usi_i2c.h"
#include <stdlib.h>
#include "berry.h"

//enum
typedef enum
{
	I2C_IDLE = 0,
	I2C_START = 2,
	I2C_ADDR = 4,
	I2C_RX = 6,
	I2C_HANDLE_RX = 8,
	I2C_HANDLE_GLOBAL = 10,
	I2C_READ_STOP = 12,
	I2C_TX = 14,
	I2C_RX_NACK = 16,
	I2C_HANDLE_NACK = 18,
	I2C_RESET = 20
} i2c_state_t;

//Defines
#define GEN_CALL 0x00
#define NEW_ADDR 0x00
#define RESET_ALL 0x01
#define VERIFY_PROJECT_HASH 0x02

//Macros
#define SDA_OUT USICTL0 |= USIOE
#define SDA_IN USICTL0 &= ~USIOE
#define SDA_READ (P1IN & SDA_PIN)
#define send_ack {SDA_OUT; USISRL = 0x00; USICNT |= 0x01;}// Bit counter = 1, send Ack bit
#define send_nack {SDA_IN; USISRL = 0xFF; USICNT |= 0x01;}// Bit counter = 1, send NAck bit

//Static variables
volatile static i2c_state_t i2cState = I2C_IDLE;  // State variable
static uint8_t global_addr = 0;
volatile static uint8_t temp_proj_hash = 0;

// Persistent memory
#pragma DATA_SECTION(proj_hash, ".infoA");
volatile static uint8_t proj_hash = 0;
#pragma DATA_SECTION(slave_addr, ".infoB");
volatile static uint8_t slave_addr = 0;

inline void usi_init()
{

	P1SEL |= SDA_PIN | SCL_PIN;
	P1SEL2 &= ~(SDA_PIN | SCL_PIN);

	P1DIR &= ~SDA_PIN;
	P1OUT &= ~SDA_PIN;

	USICTL0 = USIPE6 + USIPE7 + USISWRST;    // Port & USI mode setup
	USICTL1 = USII2C + USIIE + USISTTIE;     // Enable I2C mode & USI interrupts
	USICKCTL = USICKPL;                  // Setup clock polarity
	USICNT |= USIIFGCC;                  // Disable automatic clear control
	USICTL0 &= ~USISWRST;                // Enable USI
	USICTL1 &= ~USIIFG;                  // Clear pending flag

}

inline void set_io_pin()
{
	P1SEL &= ~SDA_PIN;
	P1SEL2 &= ~SDA_PIN;

	SDA_IN;
	P1DIR &= ~SDA_PIN;
	return;
}
inline reset_i2c_pin()
{
	SDA_IN;
	P1SEL |= SDA_PIN;
	P1SEL2 &= ~SDA_PIN;
	return;
}

static volatile uint8_t i2c_sleep;
inline uint16_t arbitration()
{

	uint16_t rand_wait = rand() & 0x00FF;
	//uint16_t rand_wait = 100;
	set_io_pin();

	while (rand_wait-- > 0)
	{
		if (!SDA_READ)
		{
			reset_i2c_pin();
			return 0;
		}
	}

	P1DIR |= (P1IN & SDA_PIN);
	reset_i2c_pin();
	return (P1DIR & SDA_PIN) ? 1 : 0;
}

static int timeout_cnt = 0;

inline void check_timeout()
{
	if (timeout_cnt && !(--timeout_cnt))
	{
		i2cState = I2C_IDLE;
	}
}

//******************************************************************************
// USI interrupt service routine
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USI_VECTOR
__interrupt void USI_TXRX(void)
{
#elif defined(__GNUC__)
	void __attribute__ ((interrupt(USI_VECTOR))) USI_TXRX (void)
	{
#else
#error Compiler not supported!
#endif

	static uint8_t byte_count = 0;
	static uint8_t cmd;
	uint8_t addr;

	timeout_cnt = USI_TIMEOUT;

	if (USICTL1 & USISTTIFG)             // Start entry?
	{
		i2cState = I2C_START;                     // Enter 1st state on start
	}

	switch (__even_in_range(i2cState, 22))
	{
	case I2C_IDLE: // 0 Idle, should not get here
		break;

	case I2C_START: // 2 RX Address
		USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, RX address
		USICTL1 &= ~USISTTIFG; // Clear start flag
		i2cState = I2C_ADDR; // Go to next state: check address
		break;

	case I2C_ADDR: // 4 Process Address and send (N)Ack
		addr = USISRL >> 1;
		if (addr == GEN_CALL || addr == slave_addr)
		{
			byte_count = 0;
			global_addr = !USISRL;
			if (USISRL & BIT0)
			{
				i2cState = I2C_TX;
			}
			else
			{
				i2cState = I2C_RX;         // Go to next state: RX data
			}
			send_ack
			;
		}
		else
		{
			USICNT |= USISCLREL;
			i2cState = I2C_RESET;		// Go to next state: prep for next Start
			send_nack
			;
		}
		break;
	case I2C_READ_STOP:
		if (USICTL1 & USISTP)
		{	//Stop bit, reset
			SDA_IN; // SDA = input
			i2cState = I2C_IDLE;		// Reset state machine
			break;
		}
		/* no break */

	case I2C_RX:
		// 8 Receive data byte
		SDA_IN;		// SDA = input
		USICNT |= 0x08;		// Bit counter = 8, RX data
		if (global_addr)
			i2cState = I2C_HANDLE_GLOBAL;// Go to next state: Test data and (N)Ack
		else
			i2cState = I2C_HANDLE_RX;
		break;
	case I2C_HANDLE_RX:
		// 10 Check Data & TX (N)Ack
		byte_count++;
		if (byte_count == 1)
		{
			current_register = USISRL;
		}
		else
		{
			if (current_register == 0)
				registers[0] = USISRL;
			else if (current_register == 1)
				registers[1] = USISRL;
			else
				set_register(USISRL);
		}
		i2cState = I2C_READ_STOP;
		USICTL1 &= ~USISTP;
		send_ack
		;
		break;
	case I2C_HANDLE_GLOBAL:
		byte_count++;
		if (byte_count == 1)
		{
			cmd = USISRL;
		}
		switch (cmd)
		{
		case NEW_ADDR:
			if (slave_addr)
			{
				send_nack
				;
				i2cState = I2C_RESET;
				break;
			}
			else
			{
				if (byte_count == 1)
				{
					send_ack
					;
					i2cState = I2C_RX;
					break;
				}
				else if (byte_count == 2)
				{
					if (arbitration())
					{
						slave_addr = USISRL;
						send_ack
						;
					}
					else
					{
						send_nack
						;
					}
					i2cState = I2C_RESET;
					break;
				}
			}
			break;
		case RESET_ALL:
			slave_addr = 0;
			send_ack
			;
			i2cState = I2C_RESET;
			break;

		case VERIFY_PROJECT_HASH:
			// Get the next byte (project hash)
			if (byte_count == 1)
			{
				send_ack
				;
				i2cState = I2C_RX;
			}
			// Got the project hash; check if it matches our own project hash
			else if (byte_count == 2)
			{
				temp_proj_hash = USISRL;
				if (proj_hash != temp_proj_hash)
				{
					// Hashes don't match, clear slave addr and update hash
					slave_addr = 0;
					proj_hash = temp_proj_hash;
					send_nack
					;
				}
				else
				{
					// Hashes match, ack
					send_ack
					;
				}
				// reset state machine
				i2cState = I2C_RESET;
			}
			else
			{
				i2cState = I2C_RESET;
			}
			break;

		default:
			break;
		}
		break;
	case I2C_HANDLE_NACK:	//Moved here to enable fall through to tx code
		if (USISRL & BIT0)
		{	//NACK
			SDA_IN; // SDA = input
			i2cState = I2C_IDLE;		// Reset state machine
			break;
		}
		//ACK: fall through to transmit another byte
		/* no break */
	case I2C_TX:
		SDA_OUT;
		if (current_register == 0)
			USISRL = registers[0];
		else if (current_register == 1)
			USISRL = registers[1];
		else
			USISRL = get_register();
		USICNT |= 0x08;
		i2cState = I2C_RX_NACK;
		break;
	case I2C_RX_NACK:
		SDA_IN;
		USICNT |= 0x01;
		i2cState = I2C_HANDLE_NACK;
		break;
	case I2C_RESET:
		SDA_IN; // SDA = input
		i2cState = I2C_IDLE;		// Reset state machine
		break;
	default:
		break;
	}

	USICTL1 &= ~USIIFG;                  // Clear pending flags
}

