/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430G2xx2 Demo - I2C Slave Receiver, single byte
//
//  Description: I2C Slave communicates with I2C Master using
//  the USI. Master data should increment from 0x00 with each transmitted byte
//  which is verified by the slave.
//  LED off for address or data Ack; LED on for address or data NAck.d by the slave.
//  ACLK = n/a, MCLK = SMCLK = Calibrated 1MHz
//
//  ***THIS IS THE SLAVE CODE***
//
//                  Slave                      Master
//                                      (MSP430G2xx2_usi_07.c)
//             MSP430G2xx2          MSP430G2xx2
//             -----------------          -----------------
//         /|\|              XIN|-    /|\|              XIN|-
//          | |                 |      | |                 |
//          --|RST          XOUT|-     --|RST          XOUT|-
//            |                 |        |                 |
//      LED <-|P1.0             |        |                 |
//            |                 |        |             P1.0|-> LED
//            |         SDA/P1.7|<-------|P1.7/SDA         |
//            |         SCL/P1.6|<-------|P1.6/SCL         |
//
//  Note: internal pull-ups are used in this example for SDA & SCL
//
//  D. Dang
//  Texas Instruments Inc.
//  December 2010
//  Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
/*
 *
 *	Register table:
 *	[0]:	Device type
 * 	[1]:	Status
 * 	[2]:	Command
 * 	[3..]:	Data
 *
 *
 * LED Berry:
 *	[0]:	0x02
 *	[1]:
 *
 *
 */

#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include "usi_i2c.h"

//Global defines
#define P2LEDS (BIT6|BIT7)

//Typedef for clock speed adjustment
typedef enum {
	_1MHZ, _8MHZ, _12MHZ, _16MHZ
} CLOCK_SPEED;

//8 bytes
static const struct {
	volatile unsigned char* calbc1;
	volatile unsigned char* caldco;
} dco_cal[] = { { &CALBC1_1MHZ, &CALDCO_1MHZ }, { &CALBC1_8MHZ, &CALDCO_8MHZ },
		{ &CALBC1_12MHZ, &CALDCO_12MHZ }, { &CALBC1_16MHZ, &CALDCO_16MHZ } };

//Defines
#define DEFAULT_REG 2
#define REG_TABLE_SIZE 16

#define SW0_REG 2

#define SW0  0
#define LED0 0
#define LED1 1
#define LED2 2

//Macros
#define set(num)	(*out_pins[num].PxOUT |= out_pins[num].bit)
#define clear(num)	(*out_pins[num].PxOUT &= ~out_pins[num].bit)
#define test(num)	(*in_pins[num].PxIN & in_pins[num].bit)

//Global variables
volatile uint8_t table[REG_TABLE_SIZE];
volatile register_table_t registers;

//Variable externs

//Function prototypes
void port_init();
void msp430_init(CLOCK_SPEED clock);
void seed_rand();

#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))
//Register table mapped to PxOUT
const struct {
	volatile unsigned char * PxOUT;
	uint8_t bit;
} out_pins[] = { { &P1OUT, BIT4 }, { &P2OUT, BIT7 }, { &P1OUT, BIT5 }, { 0, 0 },
		{ 0, 0 } };

const struct {
	volatile unsigned char * PxIN;
	uint8_t bit;
} in_pins[] = { { &P2IN, BIT6 }, { 0, 0 } };

//Main
int main(void) {
	seed_rand();
	msp430_init(_16MHZ);
	port_init();
	init_usi(0);
//
	memset((void*) table, 0x00, sizeof(table));
	registers.table = table;
	registers.table[0] = 0x06;
	registers.size = REG_TABLE_SIZE;
	registers.current = DEFAULT_REG;

//
//	static int i = 0;
//	static const struct {
//		uint8_t P1;
//		uint8_t P2;
//	} states[] = { { BIT5, 0 }, { 0, BIT6 }, { 0, BIT7 } };
//
//	while (1) {
//		P1OUT &= ~BIT5;
//		P2OUT &= ~P2LEDS;
//		P1OUT |= states[i].P1;
//		P2OUT |= states[i].P2;
//		if (++i > 2)
//			i = 0;
//		__delay_cycles(500000);
//	}

	__enable_interrupt();

	while (1) {
		LPM0;                              // CPU off, await USI interrupt
		__no_operation();                  // Used for IAR

	}
}
void seed_rand() {
	int16_t seed;
	int16_t random[16];
	int i = 16;
	while (i-- > 0) {
		seed ^= random[i];
	}
	srand(seed);
	__no_operation();
}

void set_current_register(uint8_t value) {
	switch (registers.current) {
	case 0:	//Type register
		//BAD! Should not set type register, read only
		break;
	case 1:	//Status register

		break;
	case 2:	//LED 1
		registers.table[2] = value;
		COND_BIT(value, *out_pins[2].PxOUT, out_pins[2].bit);
		registers.current++;
		break;
	case 3:	//LED 2
		registers.table[3] = value;
		COND_BIT(value, *out_pins[3].PxOUT, out_pins[3].bit);
		registers.current++;
		break;
	case 4:	//LED 3
		registers.table[4] = value;
		COND_BIT(value, *out_pins[4].PxOUT, out_pins[4].bit);
		registers.current++;
		break;
	case 15:
		registers.current = 1;
		break;
	default:
		//registers.current++;
		break;
	}
}

uint8_t get_current_register() {
	switch (registers.current) {
	case 0:	//Type register
		return registers.table[0];
	case 1:	//Status register
		return registers.table[1];
	case 2:	//LED 1
		registers.current++;
		return registers.table[2];
	case 3:	//LED 2
		registers.current++;
		return registers.table[2];
	case 4:	//LED 3
		registers.current++;
		return registers.table[2];
	case 15:
		registers.current = 1;
		break;
	default:
		//registers.current++;
		break;
	}
	return 0;
}
inline uint8_t get_register(uint8_t reg) {
	return registers.table[reg];
}

inline void set_register(uint8_t reg, uint8_t value) {
	registers.table[reg] = value;
}

inline void port_init() {

	P1SEL &= ~(BIT4 | BIT5);
	P1SEL2 &= ~(BIT4 | BIT5);
	P1DIR |= (BIT4 | BIT5);

	P2SEL &= ~(BIT7 | BIT6);
	P2SEL2 &= ~(BIT7 | BIT6);
	P2DIR |= BIT7;
	P2DIR &= ~BIT6;

	P1OUT = BIT4;
	P2OUT = BIT6;
	P2REN = BIT6;

}

#define WDT_HZ
#define WDT_CTL WDT_MDLY_8

inline void msp430_init(CLOCK_SPEED clock) {
	WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog
	if (*dco_cal[clock].calbc1 == 0xFF)   // If calibration constants erased
			{
		while (1)
			;                          // do not load, trap CPU!!
	}

	DCOCTL = 0;                      // Select lowest DCOx and MODx settings
	BCSCTL1 = *dco_cal[clock].calbc1;                          // Set DCO
	DCOCTL = *dco_cal[clock].caldco;

	// configure Watchdog
	WDTCTL = WDT_CTL;					// Set Watchdog interval
	IE1 |= WDTIE;					// Enable WDT interrupt
}

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {
//
//	if (WDT_debounce_cnt) {
//		if (--WDT_debounce_cnt) {
//			if (P1IN & SW1)
//				WDT_debounce_cnt = 0;
//		} else
//			__bic_SR_register_on_exit(LPM3_bits);
//		// Change sys_mode? TODO
//	}
	if (!test(SW0)) {
		set(LED1);
		set(LED2);
		set_register(SW0_REG, 1);
	} else {
		clear(LED1);
		clear(LED2);
		set_register(SW0_REG, 0);
	}
	return;
} // end WDT_ISR
