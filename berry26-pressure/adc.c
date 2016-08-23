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
#define TA_CTL (TASSEL__SMCLK | ID_3 | MC_1 | TACLR)

//Reset/Set
#define TA_CCTL1 (OUTMOD_7)

#define SAMPLE_PERIOD 150
#define ASSERT_TIME 1

//init the timer A module
static void timera_init()
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

static void dma_init()
{
	DMACTL0 |= DMA0TSEL__ADC10IFG;
	DMA0SZ = 4;
	__data16_write_addr((unsigned int) &DMA0SA, (unsigned long) &ADC10MEM0);
	__data16_write_addr((unsigned int) &DMA0DA, (unsigned long) dest);
	DMA0CTL =
			(DMADT_4 | DMADSTINCR_3 | DMASRCINCR_0 | DMAEN | DMAIE | DMALEVEL);
}

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

//init the 10 bit adc
void adc_init()
{
	ADC10CTL0 &= ~ADC10ENC; //Disable ADC

	//dma_init();

	ADC10CTL0 = ADC_CTL0; //configure
	ADC10CTL1 = ADC_CTL1; //configure
	ADC10CTL2 = ADC_CTL2; //configure
	ADC10MCTL0 = (ADC10SREF_0 | ADC10INCH_3);
	ADC10IE |= ADC10IE0;

	timera_init();

	ADC10CTL0 |= ADC10ENC; //Enable ADC

}

enum
{
	NONE = 0x00,
	DMA0 = 0x02,
	DMA1 = 0x04,
	DMA2 = 0x06,
	DMA3 = 0x08,
	DMA4 = 0x0A,
	DMA5 = 0x0C,
	DMA6 = 0x0E,
	DMA7 = 0x20,
};

#pragma vector = DMA_VECTOR
__interrupt void dma_isr(void)
{
	uint16_t accum;
	switch (__even_in_range(DMAIV, 0x10))
	{
	case DMA0:
		DMA0CTL &= ~DMAEN;
		if ((block_count += 4) > 12)
			block_count = 0;
		__data16_write_addr((unsigned int) &DMA0DA,
				(unsigned long) dest + block_count);
		accum = dest[0];
		accum += dest[1];
		accum += dest[2];
		accum += dest[3];
		accum += dest[4];
		accum += dest[5];
		accum += dest[6];
		accum += dest[7];
		accum += dest[8];
		accum += dest[9];
		accum += dest[10];
		accum += dest[11];
		accum += dest[12];
		accum += dest[13];
		accum += dest[14];
		accum += dest[15];
		accum += BIT3;
		accum >>= 4;
		registers[2] = (accum + BIT1) >> 2;
		registers[3] = accum & 0xff;
		registers[4] = accum >> 8;
		DMA0CTL |= DMAEN;
		break;
	default:
		break;
	}
}

#pragma vector = ADC10_VECTOR
__interrupt void adc_isr(void)
{
	uint16_t adc_read;
	switch (__even_in_range(ADC10IV, 0x0C))
	{
	case 0:
		break;
	case 0x0C:
		adc_read = ADC10MEM0;
		registers[2] = (adc_read + BIT1) >> 2;
		registers[3] = adc_read & 0xff;
		registers[4] = adc_read >> 8;
		break;
	default:
		break;
	}
}
