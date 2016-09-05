/*
 * Firmware for Slider
 * 		Broderick Gardner
 * 		6/15/2016
 *
 * 	Potentiometer is sampled 16 times every 10 ms, the values are averaged,
 * 	and the result is stored in registers 2, 3, and 4 in the register table
 *
 *  Registers:
 *  [0] : 27 (device type)
 *  [1] : (status)
 *  [2] : adc result scaled to 1 byte
 *  [3] : adc result lower byte
 *  [4] : adc result upper byte
 *  [5] : interrupt threshold (default = DEFAULT_THRESHOLD)
 *  [6] : interrupt delay (default = DEFAULT_DELAY)
 *
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
#include "pins.h"

#define DEV_TYPE 25

const int led0_port = 1;
const int led0_pin = BIT0;

#define R_READ0 			2
#define R_READ1 			3
#define R_READB 			4
#define R_INTR_THRESHOLD 	5
#define R_INTR_DELAY 		6

#define DEFAULT_THRESHOLD 	8
#define DEFAULT_DELAY 		10

void adc_init();

uint8_t device_init()
{
	P1DIR |= (BIT1 | BIT2);
	P1OUT |= (BIT1 | BIT2);
	P1SEL0 |= BIT3;
	P1SEL1 |= BIT3;
	adc_init();
	registers[R_INTR_THRESHOLD] = DEFAULT_THRESHOLD;
	registers[R_INTR_DELAY] = DEFAULT_DELAY;

	return DEV_TYPE;
}

void tick()
{
	tick_speed = 0;
}

void set_register(uint8_t value)
{
	switch (current_register)
	{
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	// Interrupt threshold and delay must be nonzero
	case R_INTR_THRESHOLD:
		registers[R_INTR_THRESHOLD] = value ? value : DEFAULT_THRESHOLD;
		break;
	case R_INTR_DELAY:
		registers[R_INTR_DELAY] = value ? value : DEFAULT_DELAY;
		break;
	default:
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
	case R_INTR_THRESHOLD:
		return registers[R_INTR_THRESHOLD];
	case R_INTR_DELAY:
		return registers[R_INTR_DELAY];
	default:
		break;
	}
	return 0;
}
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

#define SAMPLE_PERIOD 2400
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

void dma_init()
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

volatile uint16_t value_window[16] =
{ 0 };
volatile uint16_t sum = 0;
volatile uint16_t index = 0;
volatile int16_t adc_prev;

//init the 10 bit adc
void adc_init()
{
	ADC10CTL0 &= ~ADC10ENC; //Disable ADC

	//dma_init();

	ADC10CTL0 = ADC_CTL0; //configure
	ADC10CTL1 = ADC_CTL1; //configure
	ADC10CTL2 = ADC_CTL2; //configure
	ADC10MCTL0 = (ADC10SREF_0 | ADC10INCH_3);

	timera_init();

	ADC10CTL0 |= ADC10ENC; //Enable ADC

	while (!(ADC10IFG & ADC10IFG0))
		;
	ADC10IFG &= ~ADC10IFG0;
	uint16_t adc_read = ADC10MEM0;
	adc_prev = adc_read;
	int i;
	for (i = 0; i < 16; i++)
	{
		value_window[i] = adc_read;
		sum += adc_read;
	}

	ADC10IE |= ADC10IE0;

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
	int16_t diff;
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

		if (!tick_speed && registers[INT_ENABLE])
		{
			diff = adc_prev - (int16_t) adc_read;
			diff = (diff < 0) ? -(unsigned) diff : diff;
			if (diff > registers[R_INTR_THRESHOLD])
			{
				tick_speed = registers[R_INTR_DELAY];
				tick_count = registers[R_INTR_DELAY];
				registers[INTERRUPT] = 1;
				P2DIR |= BINT;
				adc_prev = adc_read;
			}
		}
		registers[2] = (adc_read + BIT1) >> 2;
		registers[3] = adc_read & 0xff;
		registers[4] = adc_read >> 8;
		break;
	default:
		break;
	}
}
