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

typedef enum {
	NEW_ADDR = 0x00, PING = 0x02, CMD = 0x04, RESET = 0x06, START = 0x0C
} cmd_header_t;

//Static variables
volatile static i2c_state_t i2cState = I2C_IDLE;              // State variable
volatile static cmd_header_t i2cHeader = START;

//Global variables

//Local prototypes
inline void temp_init();
inline uint16_t sample_temp();

inline void init_usi() {

//P1OUT = 0xC0;                        // P1.6 & P1.7 Pullups
//P1REN |= 0xC0;                       // P1.6 & P1.7 Pullups
	P1DIR = 0xFF;                        // Unused pins as outputs
	P2OUT = 0;
	P2DIR = 0xFF;

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

inline void temp_init() {
	ADC10CTL1 = ADC10DIV_0 + INCH_10 + SHS_0 + CONSEQ_2;  // TA trig., rpt, A10
	ADC10CTL0 = SREF_1 + ADC10SHT_0 + REF2_5V + REFON + REFBURST + ADC10ON;
	ADC10CTL0 |= ENC | ADC10SC;
}
inline uint16_t sample_temp() {
	ADC10CTL0 |= ADC10SC;
	__delay_cycles(100);
	return ADC10MEM;
}

inline void set_io_pin() {
	SDA_IN;
	P1SEL &= ~SDA_PIN;
	P1SEL2 &= ~SDA_PIN;

	return;
}
inline reset_i2c_pin() {
	P1SEL |= SDA_PIN;
	P1SEL2 &= ~SDA_PIN;
	SDA_IN;
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
static volatile uint8_t addr_accept;
static volatile uint8_t i2c_sleep;
inline void arbitration() {

	int i;
	TA0CCR0 = 0;
	TA0CTL |= TACLR;

	uint16_t rand_wait = rand();
	i2c_sleep = 1;
	set_io_pin();
	USICTL1 &= ~(USISTTIE | USIIE);
	__enable_interrupt();
	TA0CCR0 = rand_wait;

	while (i2c_sleep)
		LPM0;
	__disable_interrupt();

	USICTL1 |= (USISTTIE | USIIE);
	reset_i2c_pin();

	return;
//
//	P1DIR |= (P1IN & SDA_PIN);
//	reset_i2c_pin();
//	return (P1DIR & SDA_PIN) ? 1 : 0;
}

inline void send_ack() {
	SDA_OUT;
	USISRL = 0x00;         // Send Ack
	USICNT |= 0x01;          // Bit counter = 1, send (N)Ack bit
}

inline void send_nack() {
	SDA_OUT;
	USISRL = 0xFF;         // Send NAck
	USICNT |= 0x01;          // Bit counter = 1, send (N)Ack bit
}

inline void rx_data(uint8_t data) {
	static uint8_t innerState = 0;
	static uint8_t reg, value;
	static uint8_t byte_count = 0;

	if (i2cHeader == START)
		i2cHeader = (cmd_header_t) (data << 1);

	switch (__even_in_range(i2cHeader, 12)) {
	case START:
		__no_operation();	//Should not get here
		break;
	case NEW_ADDR:	//0 Address arbitration
		switch (innerState) {
		case 0:
			if (get_register(ADDR_REG)) {
				send_nack();
				i2cState = I2C_RESET;
			} else {
				send_ack();
				i2cState = I2C_RX;
				innerState = 1;
			}
			break;
		case 1:
			arbitration();
			if (addr_accept) {
				set_register(ADDR_REG, data);
				send_ack();
			} else {
				send_nack();
			}
			i2cState = I2C_RESET;
			innerState = 0;

		}
		break;
	case PING:	//	0x01 sent
		send_ack();
		i2cState = I2C_RESET;
		break;
	case CMD:	//	0x02 send
		switch (innerState) {
		case 0:
			innerState = 1;
			byte_count = 2;
			send_ack();
			i2cState = I2C_RX;
			break;
		case 1:
			byte_count--;
			if (byte_count == 1) {
				reg = data;
				i2cState = I2C_RX;
			} else if (byte_count == 0) {
				value = data;
				set_register(reg, value);
				i2cState = I2C_RESET;
				innerState = 0;
			}
			send_ack();
		}
		break;
	case RESET:	//	0x03 sent
		set_register(ADDR_REG, 0x00);
		send_ack();
		i2cState = I2C_RESET;
		break;
	default:

		break;
	}

}
#pragma vector = PORT1_VECTOR
__interrupt void p1_isr(void) {
	P1IFG &= ~SDA_PIN;
	TA0CCR0 = 0;
	addr_accept = 0;
	__bic_SR_register_on_exit(LPM0_bits);
	i2c_sleep = 0;
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timera_isr(void) {
	if (P1IN & SDA_PIN) {
		P1DIR |= SDA_PIN;
		addr_accept = 1;
	} else
		addr_accept = 0;
	TA0CCR0 = 0;
	P1IFG &= ~SDA_PIN;
	__bic_SR_register_on_exit(LPM0_bits);
	i2c_sleep = 0;
}

#define GEN_CALL 0x00
//******************************************************************************
// USI interrupt service routine
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USI_VECTOR
__interrupt void USI_TXRX(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USI_VECTOR))) USI_TXRX (void)
#else
#error Compiler not supported!
#endif
{
	static char SLV_Addr;

	if (USICTL1 & USISTTIFG)             // Start entry?
	{
		i2cState = I2C_START;                     // Enter 1st state on start
	}

	switch (__even_in_range(i2cState, 10)) {
	case I2C_IDLE: // 0 Idle, should not get here
		break;

	case I2C_START: // 2 RX Address
		USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, RX address
		USICTL1 &= ~USISTTIFG; // Clear start flag
		i2cState = I2C_ADDR; // Go to next state: check address
		break;

	case I2C_ADDR: // 4 Process Address and send (N)Ack
		SLV_Addr = get_register(ADDR_REG) << 1;
		if (USISRL & 0x01) // If read...
			SLV_Addr++; // Save R/W bit

		SDA_OUT; // SDA = output
		if (USISRL == SLV_Addr || (USISRL & ~BIT0) == GEN_CALL) // Address match?
		{
			send_ack();         // Send Ack
			i2cState = I2C_RX;         // Go to next state: RX data
		} else {
			send_nack();			// Send NAck
			i2cState = I2C_RESET;		// Go to next state: prep for next Start
		}
		break;

	case I2C_RESET: // 6 Prep for Start condition 6
		SDA_IN; // SDA = input
		//Deprecated: setting SLV_Addr here
		i2cState = I2C_IDLE;		// Reset state machine
		i2cHeader = START;
		break;

	case I2C_RX:		// 8 Receive data byte
		SDA_IN;		// SDA = input
		USICNT |= 0x08;		// Bit counter = 8, RX data
		i2cState = I2C_HANDLERX;	// Go to next state: Test data and (N)Ack
		break;

	case I2C_HANDLERX:		// 10 Check Data & TX (N)Ack
		rx_data(USISRL);
		break;
	}

	USICTL1 &= ~USIIFG;                  // Clear pending flags
}

