/*
 * motion.c
 *
 *  Created on: Sep 17, 2016
 *      Author: Kristian Sims
 */

#include "motion.h"
#include "berry08-accel.h"
#include <msp430.h>
#include "berry.h"

#define abs(x)	((x) > 0 ? (x) : (-x))
#define ORIENT_DIV	2 // Look at real gyro data and figure out this value

extern volatile uint8_t interrupt_cooldown;
volatile int accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
volatile unsigned move_threshold, spike_threshold, orient_threshold;
// TODO: Should these be signed? Comparison with signed and unsigned... :(

void calculate_motion() {

	// Accel and gyro values will be set by now, so we can use those
	// In each case, if the threshold is zero, it is disabled and we ignore it

	/* The move event is meant to tell us if the berry is moving at all.
	 * Ideally, move_threshold will be set such that regular sensor noise
	 * (such as might exist after the filter) won't cause any false positives,
	 * but picking up or moving/touching the berry will. This is achieved by
	 * storing the accelerometer values and taking a delta in each dimension
	 * and comparing them to the threshold. Also, the gyroscope values could
	 * be compared in absolute value to the threshold, since they are going to
	 * be centered around zero when the berry is at rest.
	 *
	 * Note: Whether this is calculated or not is determined by whether or not
	 * move_threshold is nonzero, so the interrupt bit will be set even if the
	 * interrupt is not enabled. We may want to change this in the future, but
	 * it mirrors how the MSP430 does interrupts, so I like to think that it's
	 * a little more intuitive.
	 */
	if (move_threshold) {
		static int move_anchor_x, move_anchor_y, move_anchor_z;
		if (
				abs(move_anchor_x-accel_x) > move_threshold ||
				abs(move_anchor_y-accel_y) > move_threshold ||
				abs(move_anchor_z-accel_z) > move_threshold ||
				abs(gyro_x) > move_threshold ||
				abs(gyro_y) > move_threshold ||
				abs(gyro_z) > move_threshold) {

			// Signal interrupt
			registers[INTERRUPT] |= MOVE_INTERRUPT;

			// Reset anchors
			move_anchor_x = accel_x;
			move_anchor_y = accel_y;
			move_anchor_z = accel_z;
		}
	}

	/* The spike event should signal when the absolute value of any
	 * accelerometer axis exceeds spike_threshold. That's about all there is to
	 * it. It's intended to register big hits like being dropped or punched.
	 */
	if (spike_threshold) {
		if (
				abs(accel_x) > spike_threshold ||
				abs(accel_y) > spike_threshold ||
				abs(accel_z) > spike_threshold) {

			// Signal interrupt
			registers[INTERRUPT] |= SPIKE_INTERRUPT;
		}
	}


	/* The orient event should ignore lateral motion but register rotations.
	 * I'm not sure if this will work, and it probably won't be used for the
	 * paper, so if it's not successful, don't pay too much attention to it.
	 */
	if (orient_threshold) {
		static int orient_x_accum, orient_y_accum, orient_z_accum;

		orient_x_accum += gyro_x >> ORIENT_DIV;
		orient_y_accum += gyro_y >> ORIENT_DIV;
		orient_z_accum += gyro_z >> ORIENT_DIV;

		if (
				abs(orient_x_accum) > orient_threshold ||
				abs(orient_x_accum) > orient_threshold ||
				abs(orient_x_accum) > orient_threshold) {

			// Signal interrupt
			registers[INTERRUPT] |= SPIKE_INTERRUPT;

			// Clear the accumulators
			orient_x_accum = 0;
			orient_y_accum = 0;
			orient_z_accum = 0;
		}
	}

	return;
}


