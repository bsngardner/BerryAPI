/*
 *
 *
 */

#include <msp430.h> 
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include "berry.h"

static const int log_table[] = { 16384, 13653, 12056, 10923, 10044, 9325, 8718,
		8192, 7728, 7313, 6937, 6595, 6279, 5987, 5716, 5461, 5223, 4997, 4784,
		4582, 4390, 4207, 4032, 3864, 3703, 3549, 3400, 3257, 3118, 2985, 2856,
		2731, 2609, 2492, 2378, 2267, 2159, 2054, 1951, 1852, 1754, 1659, 1567,
		1476, 1388, 1301, 1216, 1133, 1052, 973, 894, 818, 743, 669, 597, 526,
		456, 388, 320, 254, 189, 125, 62, 0 };

//Defines
#define TYPE 0x05	//LED type

//Macros
#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))
#define LG BIT0
#define LR BIT1
#define LB BIT2

#define R_LR 2
#define R_LG 3
#define R_LB 4

//Global variables
volatile uint8_t table[REG_TABLE_SIZE];
volatile register_table_t registers;

volatile char step;
volatile int speed;
//Variable externs

//Function prototypes
void port_init();

//IO port init function
inline void port_init() {
	P1DIR |= (LR | LG | LB);
}
uint8_t device_init(){
	port_init();
	return DEV_TYPE;
}

void set_register(uint8_t value) {
	switch (registers.current) {
	case REG_LED_RED:	//Red LED
		registers.table[REG_LED_RED] = value;
		registers.current = REG_LED_GREEN;
		break;
	case REG_LED_GREEN:	//LED 2
		registers.table[REG_LED_GREEN] = value;
		registers.current = REG_LED_BLUE;
		break;
	case REG_LED_BLUE:	//LED 3
		registers.table[REG_LED_BLUE] = value;
		registers.current = REG_LED_RED;
		break;
	default:
		//registers.current++;
		break;
	}
}

uint8_t get_register() {
	switch (registers.current) {
	case R_LR:	//LED 1
		registers.current++;
		return registers.table[R_LR];
	case R_LG:	//LED 2
		registers.current++;
		return registers.table[R_LG];
	case R_LB:	//LED 3
		registers.current++;
		return registers.table[R_LB];
	default:
		//registers.current++;
		break;
	}
	return 0;
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

uint8_t states[] = { 0, 0x01, 0x02, 0x04, 0x03, 0x06, 0x05, 0x07 };
#define LEDPORT P1OUT
#define LEDS 0x07

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {
	check_timeout();

	return;
} // end WDT_ISR
