/*
 * motion.h
 *
 *  Created on: Sep 17, 2016
 *      Author: Kristian Sims
 */

#ifndef MOTION_H_
#define MOTION_H_

void calculate_motion();

extern volatile int accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
extern volatile int move_threshold, spike_threshold, orient_threshold;




#endif /* MOTION_H_ */
