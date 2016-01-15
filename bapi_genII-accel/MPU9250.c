/*
 * MPU9250.c
 *
 *  Created on: Sep 5, 2014
 *      Author: Kristian Sims
 *      Built on the SDK by invensense (inv_mpu.c)
 */

#include <msp430.h>
#include <setjmp.h>
#include "MPU9250.h"
#include "bb_i2c.h"

extern jmp_buf bb_i2c_context;				// error context

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:
 *  Gyro FSR: +/- 2000DPS
 *  Accel FSR +/- 2G
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 *  FIFO: Disabled.
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
int MPU9250_init()
{
	unsigned char data[2];
	int error;

	if (error = setjmp(bb_i2c_context))
		return error;

	/* TODO: Sleep for 0.1 s.
	 *
	 * This was previously 1 s, but that was probably
	 * more than needed. This delay is needed to allow the MPU9250 time to
	 * bring up its serial comm unit. This was one of the first functions
	 * called on the BDL, and without a delay here, I think it didn't start up
	 * right. */
	error = 1600;
	while (error--)
		__delay_cycles(1000);

	// Reset device
	data[0] = 0x06B; // pwr_mgmt_1
	data[1] = BIT_RESET;
	i2c_write(MPU9250_ADR, data, 2);

	// TODO: Sleep for 0.1 s
	error = 1600;
	while (error--)
		__delay_cycles(1000);

	// Wake up chip
	// Same register: data[0] = 0x06B;
	data[1] = 0x00;
	bb_i2c_write(MPU9250_ADR, data, 2);

	// Set gyro full-scale range
	data[0] = 0x1B; // gyro_cfg
	data[1] = (INV_FSR_2000DPS << 3) | 0x03; // the 0x03 is to enable the LPF
	bb_i2c_write(MPU9250_ADR, data, 2);

	// Set accelerometer full-scale range
	data[0] = 0x1C; // accel_cfg
	data[1] = INV_FSR_16G << 3;
	bb_i2c_write(MPU9250_ADR, data, 2);

	// Set digital low-pass filter for gyro
	data[0] = 0x1A; // lpf
	data[1] = INV_FILTER_42HZ;
	i2c_write(MPU9250_ADR, data, 2);

	// Set sample rate to 1000 Hz
	data[0] = 0x19; // rate_div
	data[1] = 1000 / 1000 - 1;
	bb_i2c_write(MPU9250_ADR, data, 2);
	// by default, the SDK would now overwrite the low-pass filter to be 20 Hz,
	// but let's skip this for now...

	// Set up interrupt pin
	data[0] = 0x37; // int pin
	data[1] = 0xC0; // open-drain, active low
	i2c_write(MPU9250_ADR, data, 2);

	// Disable data ready interrupt.
	data[0] = 0x38; // int_enable
	data[1] = 0; //BIT_DATA_RDY_EN;
	bb_i2c_write(MPU9250_ADR, data, 2);

	// There was the set_bypass here... but that was for offboard sensors, I think

	// Configure sensors... enable gyro and don't disable accel
	data[0] = 0x6B; // pwr_mgmt_1
	data[1] = INV_CLK_PLL;
	bb_i2c_write(MPU9250_ADR, data, 2);

	data[0] = 0x6C; // pwr_mgmt_2
	data[1] = 0x00;
	bb_i2c_write(MPU9250_ADR, data, 2);

	return 0;
}

int MPU9250_sleep()
{
	unsigned char data[2];
	int error;

	if (error = setjmp(i2c_context))
		return error;

	//WDT_sleep(WDT_1SEC_CNT); // Sleep for 1 s

	// Reset device
	data[0] = 0x06B; // pwr_mgmt_1
	data[1] = BIT_SLEEP;
	bb_i2c_write(MPU9250_ADR, data, 2);

	return 0;
}

void MPU9250_accel_event()
{
	int error;
	XYZ XYZdata;
	char pbuffer[32]; // WAY too big

	XYZdata.xyz.XX = 0; // is this still necessary?
	XYZdata.xyz.YY = 0;
	XYZdata.xyz.ZZ = 0;

	if (error = setjmp(i2c_context))
	{
		my_sprintf(pbuffer, "i2c error %d", error);
	}
	else
	{
		if (!(sys_event_enable & MPU9250A))
			return;
		MPU9250_read(0x3B /* raw_accel*/, XYZdata.axis_data, 6);
		XYZdata.xyz.XX = _SWAP_BYTES(XYZdata.xyz.XX);
		XYZdata.xyz.YY = _SWAP_BYTES(XYZdata.xyz.YY);
		XYZdata.xyz.ZZ = _SWAP_BYTES(XYZdata.xyz.ZZ);
		my_sprintf(pbuffer, "%d,%d,%d", XYZdata.xyz.XX, XYZdata.xyz.YY,
				XYZdata.xyz.ZZ);
	}
	record_event('A', pbuffer);

	return;
}

void MPU9250_gyro_event()
{
	int error;
	XYZ XYZdata;
	char pbuffer[32];

	XYZdata.xyz.XX = 0;
	XYZdata.xyz.YY = 0;
	XYZdata.xyz.ZZ = 0;

	if (error = setjmp(i2c_context))
	{
		my_sprintf(pbuffer, "i2c error %d", error);
	}
	else
	{
		if (!(sys_event_enable & MPU9250G))
			return;
		MPU9250_read(0x43 /*raw_gyro*/, XYZdata.axis_data, 6);
		XYZdata.xyz.XX = _SWAP_BYTES(XYZdata.xyz.XX);
		XYZdata.xyz.YY = _SWAP_BYTES(XYZdata.xyz.YY);
		XYZdata.xyz.ZZ = _SWAP_BYTES(XYZdata.xyz.ZZ);
		my_sprintf(pbuffer, "%d,%d,%d", XYZdata.xyz.XX, XYZdata.xyz.YY,
				XYZdata.xyz.ZZ);
	}
	record_event('G', pbuffer);

	return;
}

//******************************************************************************
//	read multiple bytes from MPU9250 into buffer
//
//		regaddr = mpu9250 register
//		    buf = pointer to buffer
//		  count = # of bytes to read
//
//		returns last byte read
//
uint8 MPU9250_read(uint8 regaddr, uint8 *buf, uint8 count)
{
	bb_i2c_write(MPU9250_ADR, &regaddr, 1);
	return bb_i2c_read(MPU9250_ADR, buf, count);
}

