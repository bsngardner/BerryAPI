#include <msp430.h> 
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
//#include "usi_i2c.h"
/*
 * main.c
 */

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

//Macros
#define set(num)	(*out_pins[num].PxOUT |= out_pins[num].bit)
#define clear(num)	(*out_pins[num].PxOUT &= ~out_pins[num].bit)
#define toggle(num) (*out_pins[num].PxOUT ^= out_pins[num].bit)
#define test(num)	(*in_pins[num].PxIN & in_pins[num].bit)

//Global variables
volatile uint8_t table[REG_TABLE_SIZE];
//volatile register_table_t registers;

//Variable externs

//Function prototypes
void port_init();
void msp430_init(CLOCK_SPEED clock);
void timera_init();
void seed_rand();

#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))
//Register table mapped to PxOUT
const struct {
	volatile unsigned char * PxOUT;
	uint8_t bit;
} out_pins[] = { { &P2OUT, BIT6 }, { 0, 0 } };

const struct {
	volatile unsigned char * PxIN;
	uint8_t bit;
} in_pins[] = { { &P1IN, BIT0 }, { 0, 0 } };

inline void port_init() {

	P2SEL = P2SEL2 = P1SEL = P1SEL2 = 0;
	P2SEL |= out_pins[LED0].bit;

	P2DIR |= out_pins[LED0].bit;
	set(LED0);

	P1REN = in_pins[SW0].bit;
	P1OUT |= in_pins[SW0].bit;

}

const int log_table[] = { 16384, 13653, 12056, 10923, 10044, 9325, 8718, 8192,
		7728, 7313, 6937, 6595, 6279, 5987, 5716, 5461, 5223, 4997, 4784, 4582,
		4390, 4207, 4032, 3864, 3703, 3549, 3400, 3257, 3118, 2985, 2856, 2731,
		2609, 2492, 2378, 2267, 2159, 2054, 1951, 1852, 1754, 1659, 1567, 1476,
		1388, 1301, 1216, 1133, 1052, 973, 894, 818, 743, 669, 597, 526, 456,
		388, 320, 254, 189, 125, 62, 0 };

#define FIRST_STEP 0
#define DIM_STEP 250
#define SPEED 1
char step;
int speed;
#define PERIOD 16384

#define TA_CTL (TASSEL_2 | ID_2 | MC_1 | TACLR)
#define TA_CCTL0 (CCIE | CCIFG)
#define TA_CCTL1 (OUT | OUTMOD_7)
inline void timera_init() {
	TACTL = TA_CTL;
	CCR0 = CCR1 = 0;
	CCTL0 = TA_CCTL0;
	CCTL1 = TA_CCTL1;

	step = FIRST_STEP;
	speed = SPEED;
	CCR0 = PERIOD;
}

int main(void) {
	seed_rand();
	msp430_init(_16MHZ);
	port_init();
	timera_init();

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

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER_A0_ISR(void) {

	if (step < DIM_STEP) {

		if (!(--speed)) {
			char index = step >> 2;
			char fraction = step & 0x03;
			speed = SPEED;
			step++;

			switch (fraction) {
			case 0x00:
				CCR1 = log_table[index];
				break;
			case 0x01:
				CCR1 = 3 * (log_table[index] >> 2)
						+ (log_table[index + 1] >> 2);
				break;
			case 0x02:
				CCR1 = (log_table[index] >> 1) + (log_table[index + 1] >> 1);
				break;
			case 0x03:
				CCR1 = (log_table[index] >> 2)
						+ 3 * (log_table[index + 1] >> 2);
				break;
			}

		}
	}

}
#define NONE 0x00
#define CC1_IV 0x02
#define CC2_IV 0x04
#define TA_IV 0x0A
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER_A1_ISR(void) {
	switch (__even_in_range(TAIV, 0x0A)) {
	case NONE:
		break;
	case CC1_IV:

		break;
	case CC2_IV:

		break;
	default:
		break;
	}
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

	if (test(SW0)) {
		//set_register(SW0_REG, 1);
	} else {
		step = FIRST_STEP;
		//set_register(SW0_REG, 0);
	}
	return;
} // end WDT_ISR
