/*
 * timer.c
 *
 *  Created on: Jul 5, 2016
 *      Author: Broderick
 */

#include <msp430.h>
#include "berry.h"
#include "timer.h"
#include "berry03-servo.h"

#define TA_CTL (TASSEL_2 | ID_3 | MC_1 | TACLR)

#define TA_CLOCK 2000000
#define PERIOD 40000 //counts - 20ms
#define PW_MID 3000
#define PW_MIN (PW_MID-1024)
#define PW_MAX (PW_MID+1024)

extern volatile int change_flag;

void timer_init()
{
	TACTL = TA_CTL;
	TACCTL1 = CCIE;
	TACCR1 = PW_MID;
	TACCTL0 = CCIE;
	TACCR0 = PERIOD;
}

volatile uint16_t pw_servo0;
volatile uint16_t pw_servo1;

#define NONE 0x00
#define CC1_IV 0x02
#define CC2_IV 0x04
#define TA_IV 0x0A

#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER_A1_ISR(void)
{
	switch (__even_in_range(TAIV, 0x0A))
	{
	case NONE:
		break;
	case CC1_IV:
		P1OUT &= ~BIT1;
		break;
	case CC2_IV:

		break;
	case TA_IV:

		break;
	default:
		break;
	}

}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER_A0_ISR(void)
{
	if (change_flag)
	{
		if (registers[R_CONFIG])
		{
			pw_servo0 = (registers[R_OUT0_1] << 8) | registers[R_OUT0_0];
		}
		else
		{
			pw_servo0 = registers[R_OUT0_B] << 3;
		}
		TACCR1 = PW_MIN + pw_servo0;
		P1OUT |= BIT1;
		change_flag = 0;
	}
}
