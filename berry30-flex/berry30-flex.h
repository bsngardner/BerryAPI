/*
 * berry30-flex.h
 *
 *  Created on: Aug 25, 2016
 *      Author: Marshall
 */

#ifndef BERRY30_FLEX_H_
#define BERRY30_FLEX_H_

#include <msp430.h>
#include <stdint.h>
#include "berry.h"
#include "pins.h"

// Thresholds:
#define LEVEL1 100
#define LEVEL2 120
#define LEVEL3 140

// Interrupts:
#define ON_BEND_1 	0x01
#define ON_BEND_2 	0x02
#define ON_BEND_3 	0x04
#define ON_CHANGE	0x10
#define ON_STRAIGHT	0x20

// Registers
#define R_READ8 	2
#define R_LEVEL 	3
#define R_READ10_LO 4
#define R_READ10_HI 5

void adc_init();

#endif /* BERRY30_FLEX_H_ */
