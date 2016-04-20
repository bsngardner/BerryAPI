/*
 *
 *
 */

#include <msp430.h> 
#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include "berry.h"

static const uint16_t log_table[] = { 65535, 65422, 65308, 65195, 65082, 64968,
		64849, 64714, 64563, 64394, 64207, 64000, 63773, 63525, 63254, 62960,
		62641, 62298, 61928, 61531, 61106, 60652, 60168, 59653, 59106, 58526,
		57912, 57263, 56579, 55858, 55099, 54301, 53464, 52587, 51667, 50705,
		49700, 48650, 47555, 46414, 45225, 43988, 42702, 41365, 39977, 38538,
		37045, 35498, 33896, 32238, 30523, 28750, 26919, 25028, 23075, 21062,
		18985, 16845, 14640, 12369, 10032, 7628, 5155, 2613, 0 };

//Defines
#define DEV_TYPE 0x05	//LED type

//Macros
#define COND_BIT(bool,byte,mask) (byte ^= ((-bool) ^ (byte)) & (mask))
#define GRN_L BIT0
#define RED_L BIT1
#define BLU_L BIT2
#define RGB_L (RED_L | GRN_L | BLU_L)

#define RED_R 2
#define GRN_R 3
#define BLU_R 4

#define COLLISION_DELTA 1

//Global variables
volatile char step;
volatile int speed;

typedef struct color_pair color_pair_t;
struct color_pair {
	uint16_t color;
	uint16_t value;
	volatile color_pair_t *next;
};

typedef struct RGB_STRUCT {
	uint16_t changed;
	volatile color_pair_t *first;
} RGB_t;

volatile color_pair_t color_buf[3];
volatile RGB_t rgb = { 1, 0 };
volatile color_pair_t* current;

//Variable externs

//Function prototypes

//IO port init function
void port_init() {
	P1DIR |= RGB_L;
	P1OUT &= ~RGB_L;
}

#define TA_CTL (TASSEL_2 | TACLR | TAIE)
void timera_init() {
	current = 0;
	TACTL = TA_CTL;
	TACCTL0 = 0;
	TACCR0 = 0;
	TACTL |= MC_2;
}

uint8_t device_init() {
	port_init();
	rgb.changed = 1; //Important that rgb.changed be initialized to 1
	rgb.first = 0;
	timera_init();

	tick_max = WDT_HZ / 2;
	reg_table.table[RED_R] = 64;
	reg_table.table[GRN_R] = 128;
	reg_table.table[BLU_R] = 160;
	return DEV_TYPE;
}

void tick() {
	P2OUT ^= BIT6;
}

void set_register(uint8_t value) {
	switch (reg_table.current) {
	case RED_R:	//Red LED
		reg_table.table[RED_R] = value;
		reg_table.current = GRN_R;
		break;
	case GRN_R:	//LED 2
		reg_table.table[GRN_R] = value;
		reg_table.current = BLU_R;
		break;
	case BLU_R:	//LED 3
		reg_table.table[BLU_R] = value;
		reg_table.current = RED_R;
		break;
	default:
		//registers.current++;
		break;
	}
}

uint8_t get_register() {
	switch (reg_table.current) {
	case RED_R:	//LED 1
		reg_table.current++;
		return reg_table.table[RED_R];
	case GRN_R:	//LED 2
		reg_table.current++;
		return reg_table.table[GRN_R];
	case BLU_R:	//LED 3
		reg_table.current++;
		return reg_table.table[BLU_R];
	default:
		//registers.current++;
		break;
	}
	return 0;
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER_A0_ISR(void) {
	P1OUT |= current->color;
	current = current->next;
	if (current)
		TACCR0 = current->value;
}
#define NONE 0x00
#define CC1_IV 0x02
#define CC2_IV 0x04
#define TA_IV 0x0A

#define SWAP(x,y) {temp = x; x = y; y = temp;}
static volatile color_pair_t *first, *second, *third, *temp;
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER_A1_ISR(void) {
	switch (__even_in_range(TAIV, 0x0A)) {
	case NONE:
		break;
	case CC1_IV:

		break;
	case CC2_IV:

		break;
	case TA_IV:
		P1OUT &= ~RGB_L;

		if (rgb.changed) {
			rgb.changed = 0;
			//Load values from register table
			first = color_buf;
			second = color_buf + 1;
			third = color_buf + 2;
			first->color = RED_L;
			first->value = reg_table.table[RED_R];
			second->color = GRN_L;
			second->value = reg_table.table[GRN_R];
			third->color = BLU_L;
			third->value = reg_table.table[BLU_R];

			//Load gamma corrected values into struct
			uint16_t index, fraction;
			uint16_t part;
			int i;
			for (i = 3; i-- > 0;) {
				index = color_buf[i].value >> 2;
				fraction = color_buf[i].value & 0x03;
				switch (fraction) {
				case 0x00:
					color_buf[i].value = log_table[index];
					break;
				case 0x01:
					part = log_table[index] >> 1;
					color_buf[i].value = part
							+ (part + log_table[index + 1] >> 1) >> 1;
					break;
				case 0x02:
					color_buf[i].value = (log_table[index] >> 1)
							+ (log_table[index + 1] >> 1);
					break;
				case 0x03:
					part = log_table[index + 1] >> 1;
					color_buf[i].value = part + (part + log_table[index] >> 1)
							>> 1;
					break;
				default:
					break;
				}
			}

			//Sort values into descending order
			if (first->value > third->value)
				SWAP(first, third);
			if (first->value > second->value)
				SWAP(first, second);
			if (second->value > third->value)
				SWAP(second, third);

			first->next = second;
			second->next = third;
			third->next = 0;

			if (third->value == 65535) {
				second->next = 0;
			} else if (third->value - second->value < COLLISION_DELTA) {
				second->color |= third->color;
				second->next = 0;
			}
			if (second->value == 65535) {
				first->next = second->next;
			} else if (second->value - first->value < COLLISION_DELTA) {
				first->color |= second->color;
				first->value = second->value;
				first->next = second->next;
			}
			if (first->value == 65535) {
				if (first->next)
					first = first->next;
				else
					first = 0;
			}
			rgb.first = first;
			TACTL |= TACLR;
		}

		current = rgb.first;
		if (current) {
			TACCTL0 = CCIE;
			TACCR0 = current->value;
		}

		break;
	default:
		break;
	}
}
