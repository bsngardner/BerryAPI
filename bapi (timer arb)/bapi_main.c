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
#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include "usi_i2c.h"

//Global defines
#define P1LEDS (BIT4|BIT5)
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

//Global variables
volatile uint8_t register_table[16];

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
} out_table[] =
		{ { 0, 0 }, { &P1OUT, BIT5 }, { &P2OUT, BIT6 }, { &P2OUT, BIT7 } };

//Main
int main(void) {
	seed_rand();
	msp430_init(_16MHZ);
	port_init();
	init_usi(0);
//
	memset((void*) register_table, 0x00, sizeof(register_table));
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

inline void table_changed(uint8_t reg) {
	if (out_table[reg].PxOUT) {
		COND_BIT(register_table[reg], *out_table[reg].PxOUT,
				out_table[reg].bit);
	}
}

inline uint8_t get_register(uint8_t reg) {
	return register_table[reg];
}

inline void set_register(uint8_t reg, uint8_t value) {
	table_changed(reg);
	register_table[reg] = value;
}

inline void port_init() {

	P1SEL &= ~P1LEDS;
	P1SEL2 &= ~P1LEDS;
	P1DIR |= P1LEDS;
	P2SEL &= ~P2LEDS;
	P2SEL2 &= ~P2LEDS;
	P2DIR |= P2LEDS;

	P1OUT = BIT4;
	P2OUT = 0;
}

inline void msp430_init(CLOCK_SPEED clock) {
	WDTCTL = WDTPW + WDTHOLD;            // Stop watchdog
	if (*dco_cal[clock].calbc1 == 0xFF)       // If calibration constants erased
			{
		while (1)
			;                          // do not load, trap CPU!!
	}

	DCOCTL = 0;                          // Select lowest DCOx and MODx settings
	BCSCTL1 = *dco_cal[clock].calbc1;                          // Set DCO
	DCOCTL = *dco_cal[clock].caldco;
}
