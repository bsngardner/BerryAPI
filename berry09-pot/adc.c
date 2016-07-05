/*
 * adc.c
 *
 *  Created on: Jun 14, 2016
 *      Author: Broderick
 */

#include <msp430.h>
#include <stdint.h>
#include "adc.h"

//	/*Vcc and GND*/				SREF_0		*	|\
//	/*8 adc clock hold time*/	ADC10SHT_1	*	|\
//	/*Output reference?*/		REFOUT			|\
//	/*buffer ref during */		REFBURST	*	|\
//	/*multiple samples*/		MSC				|\
//	/*enable 2.5V ref*/			REF2_5V			|\
//	/*Turn on ref gen*/			REFON			|\
//	/*Turn ADC on*/				ADC10ON		*	|\
//	/*ADC interrupt enable*/	ADC10IE		*	|\
//	/*Interrupt flag*/			ADC10IFG		|\
//	/*Enable*/					ENC			*	|\
//	/*Start conversion*/		ADC10SC			|

#define LOG_SAMPLE_SIZE 3
#define SAMPLE_SIZE (1<<LOG_SAMPLE_SIZE)
volatile uint16_t adc_block[SAMPLE_SIZE];

//#define BUF_SIZE 16
#ifdef BUF_SIZE
volatile uint16_t adc_buffer[BUF_SIZE] =
{	0};
volatile uint16_t adc_index = 0;
#endif

//SMCLK source, divide by 8,
#define TA_CTL (TASSEL_2 | ID_3 | MC_1 | TACLR)

//Reset/Set
#define TA_CCTL1 (OUTMOD_7)

#define SAMPLE_PERIOD 20000
#define ASSERT_TIME 200

//init the timer A module
void timera_init()
{
	TACCR0 = 0;
	TACTL = TA_CTL;
	TACCTL0 = 0;
	TACCTL1 = TA_CCTL1;
	TACCR1 = ASSERT_TIME;
	TACCR0 = SAMPLE_PERIOD;
}

#define ADCIN_PIN (1 << ADC_INPUT)
#define ADC_INCH (ADC_INPUT*0x1000u)

//Vcc and Vss, 8 adc clks sample time, 50ksps reference buffer,
//	ref buf on only during sample, multiple samples, adc on,
//	enable interrupt
#define ADC_CTL0 (SREF_0 | ADC10SHT_1 | ADC10SR | REFBURST | MSC | ADC10ON | ADC10IE)

//Choose input channel, TimerA.OUT1 as trigger, divide clock by 6,
//	choose MCLK as input (could change to SMCLK or ADC10OSC), repeat single channel
#define ADC_CTL1 (ADC_INCH | SHS_1 | ADC10DIV_5 | ADC10SSEL_2 | CONSEQ_2)

//Non continuous mode, one block mode
#define ADC_DTC0 (0)

//init the 10 bit adc
void adc_init()
{
	ADC10CTL0 &= ~ENC; //Disable ADC
	ADC10CTL0 = ADC_CTL0; //configure
	ADC10CTL1 = ADC_CTL1; //configure
	ADC10AE0 = ADCIN_PIN; //Enable analog read on pin
	ADC10DTC0 = ADC_DTC0; //two block, continuous transfer
	ADC10DTC1 = SAMPLE_SIZE; //block size
	//DTC will read data starting here:
	ADC10SA = (uint16_t) adc_block; //block start address

	ADC10CTL0 |= ENC; //Enable ADC

	timera_init();
}

#pragma vector = ADC10_VECTOR
__interrupt void adc_isr(void)
{
	ADC10CTL0 &= ~ENC; //Disable ADC
	int adc_sum = 0;
	int i;
	//Add samples
	for (i = 0; i < SAMPLE_SIZE; ++i)
	{
		adc_sum += adc_block[i];
	}
	union
	{
		uint16_t word;
		struct
		{
			uint8_t lsb;
			uint8_t msb;
		};
	} adc_avg;

	//Find average sample value
	adc_avg.word = (adc_sum + (LOG_SAMPLE_SIZE >> 1)) >> LOG_SAMPLE_SIZE;
	//store separate bytes in registers
	registers[2] = adc_avg.lsb;
	registers[3] = adc_avg.msb;
	registers[4] = (uint8_t) ((adc_avg.word + 0x02) >> 2);

#ifdef BUF_SIZE
	//Buffering them is actually unnecessary
	if (++adc_index >= BUF_SIZE)
	adc_index = 0;
	adc_buffer[adc_index] = adc_avg.word;
#endif

	ADC10SA = (uint16_t) adc_block; //block start address
	ADC10CTL0 |= ENC; //Disable ADC
} // end adc isr

//Timer ISRs are not used at this point
#pragma vector = TIMER0_A0_VECTOR
__interrupt void timer_a0_isr(void)
{

} // end CCR0 timer isr

#pragma vector = TIMER0_A1_VECTOR
__interrupt void timer_a1_isr(void)
{
	switch (__even_in_range(TAIV, 0x0e))
	{
	case TA0IV_NONE:
		break;
	case TA0IV_TACCR1:

		break;
	case TA0IV_TACCR2:

		break;
	case TA0IV_TAIFG:

		break;
	default:
		break;
	}
} // end timer general interrupt isr
