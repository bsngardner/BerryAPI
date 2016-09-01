/*
 * Firmware for Potentiometer device, berry09-pot revision E
 * 		Broderick Gardner
 * 		6/15/2016
 *
 * 	Potentiometer is sampled 16 times every 10 ms, the values are averaged
 * 	and the result is stored in registers 2 and 3 in the register table
 *
 *  Registers:
 *  [0] : 9 (device type)
 *  [1] : (status)
 *  [2] : adc result lower byte
 *  [3] : adc result upper byte
 *  [4] : adc result scaled to 1 byte
 *
 *  Resources used:
 *  	Timer A
 *  	ADC10
 *  Code Size:
 *  	2,500/4064 bytes
 *  RAM usage:
 *  	193/256 bytes
 *
 * main.c
 */

#include <msp430.h> 

#include "berry.h"

#define DEV_TYPE 28
//Registers
//	2 - Config (0: on/off, 1: pwm)
//	3 - on/off
//	4 - PWM

const int led0_port = J;
const int led0_pin = BIT1;

#define PCH BIT1
#define NCH BIT0

#define LOW (PCH | NCH)
#define MID (PCH)
#define HIGH (0)
#define RESET P1OUT &= ~LOW

#define R_READ0 2
#define R_READ1 3
#define R_READB 4

//8MHz clock
const uint16_t notes[12] =
{ 61156, //C3
		57724, //C#/Db3
		54484, //D3
		51426, //D#/Eb3
		48540, //E3
		45815, //F3
		43244, //F#/Gb3
		40817, //G3
		38526, //G#/Ab3
		36364, //A3
		34323, //A#/Bb3
		32396  //B3
		};

volatile uint16_t level = 4;

void timer_init();

uint8_t device_init()
{
	P1OUT &= ~(NCH | PCH);
	P1DIR |= PCH | NCH;
	timer_init();

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
		if (!value)
		{
			TA0CCR0 = TA0CCR1 = TA1CCR0 = TA1CCR1 = 0;
		}
		else
		{
			uint16_t note = notes[value >> 4];
			uint16_t octave = value & 0xff;
			if (octave > 1) //Round result of octave shift
				note += (1 << (octave - 1));
			uint16_t period = (note) >> (octave);
			TA0CCR0 = TA1CCR0 = period;
			TA0CCR1 = TA1CCR1 = period >> 1;
		}
		TA0CTL |= TACLR;
		TA1CTL |= TACLR;
		P1OUT &= ~(NCH | PCH);
		level = 4;
		break;
	case 3:

		break;
	case 4:

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

#define LOW (PCH | NCH)
#define MID (PCH)
#define HIGH (0)
#define RESET P1OUT &= ~LOW

void timer_init()
{
	TA0CTL = (TASSEL_2 | MC_1 | ID_3 | TACLR); //ACLK source, up mode
	TA0CCR0 = 0;
	TA0CCTL0 = CCIE; //get an interrupt once per period
	TA0CCTL1 = CCIE; //reset/set mode

	TA0CCR0 = 0;
	TA0CCR1 = 0;

	TA1CTL = (TASSEL_2 | MC_1 | ID_3 | TACLR);
	TA1CCR0 = 0;
	TA1CCTL0 = CCIE;
	TA1CCTL1 = CCIE;

	TA1CCR0 = 0;
	TA1CCR1 = 0;

}

inline void new_level()
{
	uint8_t delta;
	switch (__even_in_range(level, 4))
	{
	case 0: //Low
		delta = (P1OUT & LOW) ^ LOW;
		break;
	case 2: //Mid
		delta = (P1OUT & LOW) ^ MID;
		break;
	case 4: //High
		delta = (P1OUT & LOW) ^ HIGH;
		break;
	default:
		level = 4;
		TA0CTL |= TACLR;
		TA1CTL |= TACLR;
		P1OUT &= ~(NCH | PCH);
		break;
	}
	P1OUT ^= delta;
}

//CCR0
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A1_CCR0_isr(void)
{
	//t0c0++;
	level += 2;
	new_level();
}

//TAIFG
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer0_A1_isr(void)
{
	switch (__even_in_range(TA0IV, 0x0e))
	{
	case 0x02: //CCR1
		//t0c1++;
		level -= 2;
		new_level();
		break;
	default:
		break;
	}
}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_isr(void)
{
	//t1c0++;
	level += 2;
	new_level();
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer1_A1_isr(void)
{
	switch (__even_in_range(TA1IV, 0x0e))
	{
	case 0x02:
		//t1c1++;
		level -= 2;
		new_level();
		break;
	default:
		break;
	}
}

