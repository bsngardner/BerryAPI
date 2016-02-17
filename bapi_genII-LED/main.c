/*
 *
 *
 */

#include <msp430.h> 
#include "bapi.h"

#include <stdint.h>
#include <stdlib.h>
#include "string.h"

static const int log_table[] = { 16384, 13653, 12056, 10923, 10044, 9325, 8718,
		8192, 7728, 7313, 6937, 6595, 6279, 5987, 5716, 5461, 5223, 4997, 4784,
		4582, 4390, 4207, 4032, 3864, 3703, 3549, 3400, 3257, 3118, 2985, 2856,
		2731, 2609, 2492, 2378, 2267, 2159, 2054, 1951, 1852, 1754, 1659, 1567,
		1476, 1388, 1301, 1216, 1133, 1052, 973, 894, 818, 743, 669, 597, 526,
		456, 388, 320, 254, 189, 125, 62, 0 };

//Defines
#define DEFAULT_REG 2
#define REG_TABLE_SIZE 16
#define TYPE 0x02	//LED type

#define LED0 0
#define STATUS 1
#define LED1 2
#define LED2 3
#define LED3 4
#define LED4 5
#define READ 6

#define L0 BIT6
#define L1 BIT0
#define L2 BIT1
#define L3 BIT2
#define L4 BIT3

//Macros
#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))

volatile char step;
volatile int speed;
//Variable externs

//Function prototypes

//IO port init function
inline void port_init() {
	P1DIR |= (L1 | L2 | L3 | L4);
	P1OUT |= ( L2 | L3);

	P2OUT |= L0;
}

//**************************MAIN*******************************************//
int main(void) {
	bapi_init(_16MHZ, TYPE);
	port_init();

	__enable_interrupt();
	while (1) {

		LPM0;                              // CPU off, await USI interrupt
		__no_operation();                  // Used for IAR
	}

}

void set_register(uint8_t value) {
	switch (reg_table.current) {
	case 0:	//Type register
		//BAD! Should not set type register, read only
		break;
	case 1:	//Status register

		break;
	case LED1:	//LED 1
		reg_table.table[LED1] = value;
		COND_BIT(value, P1OUT, L1);
		reg_table.current++;
		break;
	case LED2:	//LED 2
		reg_table.table[LED2] = value;
		COND_BIT(value, P1OUT, L2);
		reg_table.current++;
		break;
	case LED3:	//LED 3
		reg_table.table[LED3] = value;
		COND_BIT(value, P1OUT, L3);
		reg_table.current++;
		break;
	case LED4:
		reg_table.table[LED4] = value;
		COND_BIT(value, P1OUT, L4);
		reg_table.current++;
		break;
	case 15:
		reg_table.current = LED1;
		break;
	default:
		//registers.current++;
		break;
	}
}

uint8_t get_register() {
	switch (reg_table.current) {
	case 0:	//Type register
		return reg_table.table[0];
	case 1:	//Status register
		return reg_table.table[1];
	case LED1:	//LED 1
		reg_table.current++;
		return reg_table.table[LED1];
	case LED2:	//LED 2
		reg_table.current++;
		return reg_table.table[LED2];
	case LED3:	//LED 3
		reg_table.current++;
		return reg_table.table[LED3];
	case LED4:
		reg_table.current = LED1;
		return reg_table.table[LED4];
	case READ:
		reg_table.table[READ] = rand() & (255);
		return reg_table.table[READ];
	case 15:
		reg_table.current = 1;
		break;
	default:
		//reg_table.current++;
		break;
	}
	return 0;
}

//------------------------------------------------------------------------------
//	Watchdog Timer ISR
//
#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void) {
	check_timeout();
	return;
} // end WDT_ISR
