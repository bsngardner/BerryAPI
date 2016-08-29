/*
 * adc.h
 *
 *  Created on: Jun 14, 2016
 *      Author: Marshall
 */

#include <stdint.h>
#include "berry.h"
#include "pins.h"

// Thresholds:
#define LEVEL1 220
#define LEVEL2 160
#define LEVEL3 90
#define LEVEL4 30

// Interrupts:
#define ON_PRESS_1 	0x01
#define ON_PRESS_2 	0x02
#define ON_PRESS_3 	0x04
#define ON_PRESS_4 	0x08
#define ON_CHANGE	0x10
#define ON_RELEASE 	0x20

// Registers
#define R_READ8 	2
#define R_LEVEL 	3
#define R_READ10_LO 4
#define R_READ10_HI 5


void adc_init();
