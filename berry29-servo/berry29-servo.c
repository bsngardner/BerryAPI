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

void timer_init();

uint8_t device_init()
{
	P1OUT &= ~MOTOR_PIN;
	P1DIR |= MOTOR_PIN;
	P1SEL1 &= ~MOTOR_PIN;
	P1SEL0 |= MOTOR_PIN;

	timer_init();

	registers[2] = 0;	//default config to on/off
	registers[3] = 0;	//start off

	current_register = 2;
	set_register(150);

	return DEV_TYPE;
}

void tick()
{

}

//Map 8 bit register to (1428,7572), which is about .5ms to 2.5ms
//	true .5ms to 2.5ms is (1500,7500) given a 3MHz clock and 16bit timer
#define OFFSET 2856 //1500 (true .5ms) minus 72 (to round scale factor to 6144)

void set_register(uint8_t value)
{
	uint16_t duty;
	switch (current_register)
	{
	case 2:
		registers[2] = value;
		duty = (value << 6) + (value << 5);
		duty = (duty + 1) >> 1;
		duty += OFFSET;
		TA0CCR2 = duty;
		break;
	case 3:
		registers[3] = value;
		break;
	case 4:
		registers[4] = value;
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
	TA0CCTL2 = OUTMOD_7;
	TA0CCR0 = 1;
	TA0CCR2 = 0;
	TA0CTL = TACLR | TASSEL_2 | ID_2 | MC_2; // SMCLK, continuous mode
}
