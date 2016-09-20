/*
 * Firmware for accelerometer device, berry08-accel
 * 		Broderick Gardner
 * 		6/15/2016
 */

#include <msp430.h> 
#include "berry08-accel.h"
#include "berry.h"
#include "pins.h"
#include "spi.h"
#include "MPU9250.h"
#include "motion.h"

#define DEV_TYPE	0x08

#define DEFAULT_INTERRUPT_COOLDOWN	10
#define DEFAULT_MOVE_THRESHOLD		400
#define DEFAULT_SPIKE_THRESHOLD		10000
#define DEFAULT_ORIENT_THRESHOLD	400
#define DEFAULT_ORIENT_DIV			5

const int led0_port = 1;
const int led0_pin = BIT0;

volatile uint8_t interrupt_cooldown = 0;

uint8_t device_init() {
	tick_speed = 1;
	spi_init();

	// Initial values for thresholds
	move_threshold = DEFAULT_MOVE_THRESHOLD;
	spike_threshold = DEFAULT_SPIKE_THRESHOLD;
	orient_threshold = DEFAULT_ORIENT_THRESHOLD;

	registers[INTR_COOLDOWN] = DEFAULT_INTERRUPT_COOLDOWN;
	registers[MOVE_THRESHOLD_L] = DEFAULT_MOVE_THRESHOLD & 0xff;
	registers[MOVE_THRESHOLD_H] = (DEFAULT_MOVE_THRESHOLD >> 8) & 0xff;
	registers[SPIKE_THRESHOLD_L] = DEFAULT_SPIKE_THRESHOLD & 0xff;
	registers[SPIKE_THRESHOLD_H] = (DEFAULT_SPIKE_THRESHOLD >> 8) & 0xff;
	registers[ORIENT_DIV] = DEFAULT_ORIENT_DIV;

	return DEV_TYPE;
}

void tick() {

	// Poll accelerometer and (once init is done) analyze the measurements
	if (!MPU9250_tick())
		calculate_motion();

	// Check cooldown and signal interrupts
	if (interrupt_cooldown)
		--interrupt_cooldown;
	else if (registers[INT_ENABLE] & registers[INTERRUPT] & ALL_INTERRUPTS)
		interrupt_cooldown = registers[INTR_COOLDOWN], ASSERT_INTR;
}

void set_register(uint8_t value) {
	switch (current_register) {
	case MOVE_THRESHOLD_L:
		registers[MOVE_THRESHOLD_L] = value;
		move_threshold = 0;
		current_register++;
		break;
	case MOVE_THRESHOLD_H:
		registers[MOVE_THRESHOLD_H] = value;
		current_register++;
		move_threshold = value << 8 | registers[MOVE_THRESHOLD_L];
		break;
	case SPIKE_THRESHOLD_L:
		registers[SPIKE_THRESHOLD_L] = value;
		spike_threshold = 0;
		current_register++;
		break;
	case SPIKE_THRESHOLD_H:
		registers[SPIKE_THRESHOLD_H] = value;
		current_register++;
		spike_threshold = value << 8 | registers[SPIKE_THRESHOLD_L];
		break;
	case ORIENT_THRESHOLD_L:
		registers[ORIENT_THRESHOLD_L] = value;
		orient_threshold = 0;
		current_register++;
		break;
	case ORIENT_THRESHOLD_H:
		registers[ORIENT_THRESHOLD_H] = value;
		orient_threshold = value << 8 | registers[ORIENT_THRESHOLD_L];
		break;
	case INTR_COOLDOWN:
		registers[INTR_COOLDOWN] = value;
		break;
	case ORIENT_DIV:
		if (value < 16) {
			registers[ORIENT_DIV] = value;
		} else {
			registers[ORIENT_DIV] = 1;
			value>>=1;
			while (value>>1)
				registers[ORIENT_DIV] <<= 1;
		}
		break;
	default:
		break;
	}

	return;
}

uint8_t get_register() {

	uint8_t ret = 0;
	if (current_register <= MAX_REG) {
		ret = registers[current_register];
		if (++current_register > MAX_REG)
			current_register = ACCEL_X_L;
	}

	return ret;

}
