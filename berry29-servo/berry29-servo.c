/*
 * Firmware for vibrator berry, berry27-vibrerry
 * 		Broderick Gardner
 * 		6/15/2016
 */

#include <msp430.h> 

#include "berry.h"

#define DEV_TYPE 29
//Registers
//	2 - Config (0: on/off, 1: pwm)
//	3 - on/off
//	4 - PWM

const int led0_port = J;
const int led0_pin = BIT1;

#define MOTOR_PIN BIT1
#define MOTOR_PORT P1OUT

#define R_READ0 2
#define R_READ1 3
#define R_READB 4

#define SET_PWM		TA0CCTL1 |= OUTMOD_7
#define SET_ONOFF	TA0CCTL1 &= ~OUTMOD_7
#define SET_ON		TA0CCTL1 |= OUT
#define SET_OFF		TA0CCTL1 &= ~OUT

void timer_init();

uint8_t device_init()
{
	P1OUT &= ~MOTOR_PIN;
	P1DIR |= MOTOR_PIN;
	P1SEL1 &= ~MOTOR_PIN;
	P1SEL0 |= MOTOR_PIN;

	timer_init();
	SET_OFF;

	registers[2] = 0;	//default config to on/off
	registers[3] = 0;	//start off
	return DEV_TYPE;
}

void tick()
{

}

void set_register(uint8_t value)
{
	switch (current_register)
	{
	case 2:
		registers[2] = value;
		if (value)
		{
			SET_ONOFF;
		}
		else
		{
			SET_PWM;
		}
		break;
	case 3:
		registers[3] = value;
		if (value)
			SET_ON;
		else
			SET_OFF;
		break;
	case 4:
		registers[4] = value;
		TA1CCR1 = value;
		break;
	}

	return;
}

uint8_t get_register()
{
	switch (current_register)
	{
	//On reading each byte of the potentiometer value,
	//	switch current register to the other byte
	//	This allows reading 2 bytes in quick succession
	case R_READ0:
		current_register = R_READ1;
		return registers[R_READ0];
	case R_READ1:
		current_register = R_READ0;
		return registers[R_READ1];
	case R_READB:
		return registers[R_READB];
	default:
		break;
	}
	return 0;
}

void timer_init()
{
	TA0CCTL0 = 0; // TA1CCR0 interrupt enabled
	TA0CCTL1 = OUTMOD_0; // TA1CCR0 interrupt enabled
	TA0CTL = TACLR | TASSEL_1 | MC_1; // ACLK, continuous mode
	TA0CCR0 = 0xff;
	TA0CCR1 = 0xff >> 1;
}
