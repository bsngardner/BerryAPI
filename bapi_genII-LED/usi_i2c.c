/*
 * usi_i2c.c
 *
 *  Created on: Nov 5, 2015
 *      Author: Broderick
 */

#include "usi_i2c.h"
#include "msp430.h"
#include <stdlib.h>

//Macros
#define SDA_OUT USICTL0 |= USIOE
#define SDA_IN USICTL0 &= ~USIOE
#define SDA_READ (P1IN & SDA_PIN)

//Global variables

//Local prototypes
inline void temp_init();
inline uint16_t sample_temp();

inline void init_usi() {

//P1OUT = 0xC0;                        // P1.6 & P1.7 Pullups
//P1REN |= 0xC0;                       // P1.6 & P1.7 Pullups

	P1SEL |= SDA_PIN | SCL_PIN;
	P1SEL2 &= ~(SDA_PIN | SCL_PIN);

	P1DIR &= ~SDA_PIN;
	P1OUT &= ~SDA_PIN;
	P1IES = SDA_PIN;
	P1IE = SDA_PIN;
	P1IFG = 0;

	USICTL0 = USIPE6 + USIPE7 + USISWRST;    // Port & USI mode setup
	USICTL1 = USII2C + USIIE + USISTTIE;     // Enable I2C mode & USI interrupts
	USICKCTL = USICKPL;                  // Setup clock polarity
	USICNT |= USIIFGCC;                  // Disable automatic clear control
	USICTL0 &= ~USISWRST;                // Enable USI
	USICTL1 &= ~USIIFG;                  // Clear pending flag
//
//	rxBuffer = IObuffer_create(IOBUFFER_SIZE);
//	rxBuffer->bytes_ready = callback;
	TA0CTL |= TASSEL_2 | ID_0 | MC_1;
	TA0CCTL0 = 0;
	TA0CCTL0 |= CCIE;
}
//
//inline void temp_init() {
//	ADC10CTL1 = ADC10DIV_0 + INCH_10 + SHS_0 + CONSEQ_2;  // TA trig., rpt, A10
//	ADC10CTL0 = SREF_1 + ADC10SHT_0 + REF2_5V + REFON + REFBURST + ADC10ON;
//	ADC10CTL0 |= ENC | ADC10SC;
//}
//inline uint16_t sample_temp() {
//	ADC10CTL0 |= ADC10SC;
//	__delay_cycles(100);
//	return ADC10MEM;
//}

inline void set_io_pin() {
	P1SEL &= ~SDA_PIN;
	P1SEL2 &= ~SDA_PIN;

	SDA_IN;
	P1DIR &= ~SDA_PIN;
	return;
}
inline reset_i2c_pin() {
	SDA_IN;
	P1SEL |= SDA_PIN;
	P1SEL2 &= ~SDA_PIN;
	return;
}

#define DEBUGMODE
#ifdef DEBUGMODE
#define dbg_light	P1OUT ^= BIT4
#define dbg_light_set	P1OUT |= BIT4
#else
#define dbg_light
#define dbg_light_set
#endif

static volatile uint8_t i2c_sleep;
inline uint16_t arbitration() {

	uint16_t rand_wait = rand() & 0x00FF;
	//uint16_t rand_wait = 100;
	set_io_pin();

	while (rand_wait-- > 0) {
		if (!SDA_READ) {
			reset_i2c_pin();
			return 0;
		}
	}

	P1DIR |= (P1IN & SDA_PIN);
	reset_i2c_pin();
	return (P1DIR & SDA_PIN) ? 1 : 0;
}

#define GEN_CALL 0x00
#define NEW_ADDR 0x00
#define RESET_ALL 0x01
#define send_ack SDA_OUT; USISRL = 0x00; USICNT |= 0x01// Bit counter = 1, send Ack bit
#define send_nack SDA_IN; USISRL = 0xFF; USICNT |= 0x01// Bit counter = 1, send NAck bit
//Static variables
volatile static i2c_state_t i2cState = I2C_IDLE;  // State variable
volatile static uint8_t slave_addr = 0;
static char global_addr = 0;

//******************************************************************************
// USI interrupt service routine
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USI_VECTOR
__interrupt void USI_TXRX(void) {
#elif defined(__GNUC__)
	void __attribute__ ((interrupt(USI_VECTOR))) USI_TXRX (void) {
#else
#error Compiler not supported!
#endif

	static uint8_t byte_count = 0;
	static uint8_t cmd;
	uint8_t addr;

	if (USICTL1 & USISTTIFG) {  // Start entry?
		i2cState = I2C_START;                     // Enter 1st state on start
	}

	switch (__even_in_range(i2cState, 22)) {
	case I2C_IDLE: // 0 Idle, should not get here
		break;

	case I2C_START: // 2 RX Address
		USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, RX address
		USICTL1 &= ~USISTTIFG; // Clear start flag
		i2cState = I2C_ADDR; // Go to next state: check address
		break;

	case I2C_ADDR: // 4 Process Address and send (N)Ack
		addr = USISRL >> 1;
		if (addr == GEN_CALL || addr == slave_addr) {
			byte_count = 0;
			global_addr = !USISRL;
			if (USISRL & BIT0) {
				i2cState = I2C_TX;
			} else {
				i2cState = I2C_RX;         // Go to next state: RX data
			}
			send_ack
			;
		} else {
			USICNT |= USISCLREL;
			i2cState = I2C_RESET;		// Go to next state: prep for next Start
			send_nack
			;
		}
		break;
	case I2C_READ_STOP:
		if (USICTL1 & USISTP) {	//Stop bit, reset
			SDA_IN; // SDA = input
			i2cState = I2C_IDLE;		// Reset state machine
			break;
		}
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
		if (byte_count == 1) {
			registers.current = USISRL;
		} else {
			set_current_register(USISRL);
		}
		i2cState = I2C_READ_STOP;
		USICTL1 &= ~USISTP;
		send_ack
		;
		break;
	case I2C_HANDLE_GLOBAL:
		byte_count++;
		if (byte_count == 1) {
			cmd = USISRL;
		}
		switch (cmd) {
		case NEW_ADDR:
			if (slave_addr) {
				send_nack
				;
				i2cState = I2C_RESET;
				break;
			} else {
				if (byte_count == 1) {
					send_ack
					;
					i2cState = I2C_RX;
					break;
				} else if (byte_count == 2) {
					if (arbitration()) {
						slave_addr = USISRL;
						send_ack
						;
					} else {
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
		default:
			break;
		}
		break;
	case I2C_HANDLE_NACK:	//Moved here to enable fall through to tx code
		if (USISRL & BIT0) {	//NACK
			SDA_IN; // SDA = input
			i2cState = I2C_IDLE;		// Reset state machine
			break;
		}
		//ACK: fall through to transmit another byte
	case I2C_TX:
		SDA_OUT;
		USISRL = get_current_register();
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

