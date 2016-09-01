#include <msp430.h>
#include "usci_i2c.h"
#include "pins.h"
#include <stdlib.h>
/*
 * eusci_i2c.c
 *
 *  Created on: Jun 20, 2016
 *      Author: Broderick
 */

//I2C slave mode
#define UCB_CTLW0 (UCMODE_3 | UCSYNC)
#define SDA_PIN BSDA
#define SCL_PIN BSCL
#define SDA_READ (P1IN & SDA_PIN)

extern volatile uint16_t proj_key;
extern volatile uint16_t slave_addr;

void sys_set_register(uint8_t value);
uint8_t sys_get_register();

void usci_init()
{
	UCB0CTLW0 = UCSWRST; //Module in reset
	UCB0CTLW0 |= UCB_CTLW0; //configure

	P1DIR &= ~(SDA_PIN | SCL_PIN);
	P1OUT &= ~(SDA_PIN | SCL_PIN);
	P1SEL0 &= ~(SDA_PIN | SCL_PIN);
	P1SEL1 |= (SDA_PIN | SCL_PIN);

	UCB0I2COA0 = UCGCEN | UCOAEN | slave_addr; //Start with slave address
	UCB0CTL1 &= ~UCSWRST;
	UCB0IE |= UCTXIE | UCRXIE | UCSTPIE;

}

inline void set_sda()
{
	P1DIR &= ~SDA_PIN;
	P1SEL1 &= ~SDA_PIN;
	return;
}

inline reset_sda()
{
	P1SEL1 |= SDA_PIN;
	return;
}

uint8_t arbitration()
{
	uint16_t rand_wait = rand();
	set_sda();

	while (rand_wait-- > 0)
	{
		if (!SDA_READ)
		{
			reset_sda();
			return 0;
		}
	}

	P1DIR |= (P1IN & SDA_PIN);
	__delay_cycles(100);
	reset_sda();
	return (P1DIR & SDA_PIN) ? 1 : 0;
}

#define NEW_ADDR 0x00
#define RESET_ALL 0x01
#define VERIFY_PROJECT_KEY 0x02
#define UPDATE_PROJECT_KEY 0x03

#define SEND_NACK UCB0CTLW0 |= UCTXNACK
#define CLOCK_HELD (UCB0STATW & UCSCLLOW)

volatile enum
{
	IDLE, RX_1, RX, GENCALL, TX, NACK
} usci_state = IDLE;

#define UCBIV_NONE 0x00
#define UCBIV_AL 0x02
#define UCBIV_NACK 0x04
#define UCBIV_STT 0x06
#define UCBIV_STP 0x08
#define UCBIV_RX3 0x0A
#define UCBIV_TX3 0x0C
#define UCBIV_RX2 0x0E
#define UCBIV_TX2 0x10
#define UCBIV_RX1 0x12
#define UCBIV_TX1 0x14
#define UCBIV_RX 0x16
#define UCBIV_TX 0x18
#define UCBIV_BCNT 0x1A
#define UCBIV_CLTO 0x1C
#define UCBIV_BIT9 0x1E

#pragma vector = USCI_B0_VECTOR
__interrupt void usci_b0_isr(void)
{
	static volatile uint16_t cmd = 0;
	static volatile uint16_t addr_change = 0;
	static volatile uint16_t byte_count = 0;
	static volatile uint16_t rxData;

	switch (__even_in_range(UCB0IV, 0x1E))
	{
	case UCBIV_STP:
		usci_state = IDLE;
		if (addr_change)
		{
			addr_change = 0;
			UCB0CTLW0 |= UCSWRST; //Module in reset
			UCB0I2COA0 = UCGCEN | UCOAEN | slave_addr; //Load new slave address
			UCB0CTLW0 &= ~UCSWRST; //Module in reset
			UCB0IE |= UCTXIE | UCRXIE | UCSTPIE;
		}
		break;
	case UCBIV_RX:

		if (usci_state == IDLE)
			usci_state = RX_1;
		else
			usci_state = RX;

		//If general call
		if (UCB0STATW & UCGC)
		{
			//If first byte of message
			if (usci_state == RX_1)
			{
				byte_count = 1;
				cmd = UCB0RXBUF;
				switch (cmd)
				{
				case NEW_ADDR:
					break;
				case RESET_ALL:
					slave_addr = 0;
					addr_change = 1;
					proj_key = 0;
					break;
				case VERIFY_PROJECT_KEY:
					break;
				case UPDATE_PROJECT_KEY:
					break;
				default:
					break;
				}
			}
			//If second+ byte of message
			else
			{
				byte_count++;
				switch (cmd)
				{
				//New address response, including arbitration
				case NEW_ADDR:
					if (!addr_change && slave_addr)
					{
						SEND_NACK;
						rxData = UCB0RXBUF;
						break;
					}

					if (byte_count > 2)
					{
						rxData = UCB0RXBUF;
						break; //If this is byte number 3 or more, ignore the byte
					}

					while (!CLOCK_HELD)
						;

					if (arbitration())
					{
						addr_change = 1;
						slave_addr = UCB0RXBUF;
					}
					else
					{
						SEND_NACK;
						rxData = UCB0RXBUF;
					}
					break;

					//Verify project key command response
				case VERIFY_PROJECT_KEY:
					switch (byte_count)
					{
					case 2:
						rxData = UCB0RXBUF;
						break;
					case 3:
						rxData = rxData | (UCB0RXBUF << 8);
						if (rxData != proj_key)
						{
							slave_addr = 0;
							addr_change = 1;
							proj_key = rxData;
						}
						break;
					default:
						rxData = UCB0RXBUF;
						break;
					}
					break;

					//Direct update of the project key
				case UPDATE_PROJECT_KEY:
					switch (byte_count)
					{
					case 2:
						rxData = UCB0RXBUF;
						break;
					case 3:
						proj_key = rxData | (UCB0RXBUF << 8);
						break;
					default:
						rxData = UCB0RXBUF;
						break;
					}
					break;

				default:
					rxData = UCB0RXBUF;
					break;
				}
			}
		}
		else
		{
			if (usci_state == RX_1)
				current_register = (int16_t) ((int8_t) UCB0RXBUF);
			else if (current_register < 2)
				sys_set_register(UCB0RXBUF);
			else
				set_register(UCB0RXBUF);
		}

		break;

	case UCBIV_TX:
		usci_state = TX;
		if (current_register < 2)
			UCB0TXBUF = sys_get_register();
		else
			UCB0TXBUF = get_register();
		break;
	case UCBIV_BIT9:

		break;
	default:
		break;
	}
}
