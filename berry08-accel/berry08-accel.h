/*
 * berry08-accel.h
 *
 *  Created on: Sep 17, 2016
 *      Author: Kristian Sims
 */

#ifndef BERRY08_ACCEL_H_
#define BERRY08_ACCEL_H_

enum reg {
	MOVE_THRESHOLD_L = 2,
	MOVE_THRESHOLD_H,
	SPIKE_THRESHOLD_L,
	SPIKE_THRESHOLD_H,
	ORIENT_THRESHOLD_L,
	ORIENT_THRESHOLD_H,
	ACCEL_X_L,
	ACCEL_X_H,
	ACCEL_Y_L,
	ACCEL_Y_H,
	ACCEL_Z_L,
	ACCEL_Z_H,
	GYRO_X_L,
	GYRO_X_H,
	GYRO_Y_L,
	GYRO_Y_H,
	GYRO_Z_L,
	GYRO_Z_H,
	MAG_X_L,
	MAG_X_H,
	MAG_Y_L,
	MAG_Y_H,
	MAG_Z_L,
	MAG_Z_H,
	TEMP_L,
	TEMP_H
};



#endif /* BERRY08_ACCEL_H_ */
