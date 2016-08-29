/*
 * adc.c
 *
 * Authors: Broderick, Marshall
 */
#include <msp430.h>
#include "berry26-res.h"

//SMCLK source, divide by 8,
#define TA_CTL (TASSEL__SMCLK | ID_3 | MC_1 | TACLR)

//Reset/Set
#define TA_CCTL1 (OUTMOD_7)

#define SAMPLE_PERIOD 4800//2400
#define ASSERT_TIME 1

//init the timer A module
void timera_init()
{
	TA0CCR0 = 0;
	TA0CTL = TA_CTL;
	TA0CCTL0 = 0;
	TA0CCTL1 = TA_CCTL1;
	TA0CCR1 = ASSERT_TIME;
	TA0CCR0 = SAMPLE_PERIOD;
}

volatile uint16_t dest[16] =
{ 0 };
volatile uint8_t block_count = 12;

//Vcc and Vss, 8 adc clks sample time, 50ksps reference buffer,
//	ref buf on only during sample, multiple samples, adc on,
//	enable interrupt
#define ADC_CTL0 (ADC10SHT_0 | ADC10ON)

//Choose input channel, TimerA.OUT1 as trigger, divide clock by 6,
//	choose MCLK as input (could change to SMCLK or ADC10OSC), repeat single channel
#define ADC_CTL1 (ADC10SHS_1 | ADC10SHP | ADC10DIV_0 | ADC10SSEL_2 | ADC10CONSEQ_2)

#define ADC_CTL2 (ADC10PDIV_2 | ADC10RES | ADC10SR)

//Non continuous mode, one block mode
#define ADC_DTC0 (0)

volatile uint16_t value_window[16] =
{ 0 };
volatile uint16_t sum = 0;
volatile uint16_t index = 0;
volatile uint8_t prev_level = 0;

static inline uint8_t get_level(uint8_t adc_8bit);
#define adc_10_to_8(adc10) ((adc10 + BIT1) >> 2)

//init the 10 bit adc
void adc_init()
{
	ADC10CTL0 &= ~ADC10ENC; //Disable ADC

	ADC10CTL0 = ADC_CTL0; //configure
	ADC10CTL1 = ADC_CTL1; //configure
	ADC10CTL2 = ADC_CTL2; //configure
	ADC10MCTL0 = (ADC10SREF_0 | ADC10INCH_4);

	timera_init();

	ADC10CTL0 |= ADC10ENC; //Enable ADC

	while (!(ADC10IFG & ADC10IFG0))
		;
	ADC10IFG &= ~ADC10IFG0;
	uint16_t adc_read = ADC10MEM0;

	prev_level = get_level(adc_10_to_8(adc_read));
	int i;
	for (i = 0; i < 16; i++)
	{
		value_window[i] = adc_read;
		sum += adc_read;
	}

	ADC10IE |= ADC10IE0;
}

static inline uint8_t get_level(uint8_t adc_8bit)
{
	if (adc_8bit > LEVEL1)
		return 0;
	else if ((adc_8bit <= LEVEL1) && (adc_8bit > LEVEL2))
		return 1;
	else if ((adc_8bit <= LEVEL2) && (adc_8bit > LEVEL3))
		return 2;
	else if ((adc_8bit <= LEVEL3) && (adc_8bit > LEVEL4))
		return 3;
	else
		return 4;
}

#pragma vector = ADC10_VECTOR
__interrupt void adc_isr(void)
{
	uint16_t adc_read;
	uint8_t adc_8bit;
	uint8_t int_en;
	uint8_t level;
	switch (__even_in_range(ADC10IV, 0x0C))
	{
	case 0:
		break;
	case 0x0C:
		//subtract old value from sum, read new into array and add to sum
		//  divide sum by 16, rounding, to find average.
		adc_read = ADC10MEM0;
		sum -= value_window[index];
		value_window[index] = adc_read;
		sum += adc_read;
		if ((++index) > 15)
			index = 0;
		adc_read = (sum + BIT3) >> 4;
		adc_8bit = adc_10_to_8(adc_read);

		//get level:
		level = get_level(adc_8bit);

		int_en = registers[INT_ENABLE];
		if (int_en)
		{
			// Exceeded threshold:
			if (level && (level > prev_level))
			{
				if (level == 1 && (prev_level < 1))
					registers[INTERRUPT] |= (int_en & ON_PRESS_1);
				else if (level == 2 && (prev_level < 2))
					registers[INTERRUPT] |= (int_en & ON_PRESS_2);
				else if (level == 3 && (prev_level < 3))
					registers[INTERRUPT] |= (int_en & ON_PRESS_3);
				else if (level == 4 && (prev_level < 4))
					registers[INTERRUPT] |= (int_en & ON_PRESS_4);
			}
			// Released:
			if (prev_level && !level)
				registers[INTERRUPT] |= (int_en & ON_RELEASE);

			// Level changed:
			if (level != prev_level)
				registers[INTERRUPT] |= (int_en & ON_CHANGE);

			// Assert interrupt if any of these were true.
			if (registers[INTERRUPT])
				ASSERT_INTR;
		}
		prev_level = level;
		registers[R_READ8] = adc_8bit;
		registers[R_LEVEL] = level;
		registers[R_READ10_LO] = adc_read & 0xff;
		registers[R_READ10_HI] = adc_read >> 8;
		break;
	default:
		break;
	}
}
