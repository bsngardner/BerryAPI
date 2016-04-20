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
#include "bb_spi.h"

extern jmp_buf bb_i2c_context;				// error context

uint8_t MPU9250_read(uint8_t regaddr, uint8_t *buf, uint8_t count);

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
	int count;
	/* TODO: Sleep for 0.1 s.
	 *
	 * This was previously 1 s, but that was probably
	 * more than needed. This delay is needed to allow the MPU9250 time to
	 * bring up its serial comm unit. This was one of the first functions
	 * called on the BDL, and without a delay here, I think it didn't start up
	 * right. */
	count = 16000;
	do
		__delay_cycles(1000);
	while (--count);

	// Reset device
	MPU9250_write(0x6B /*pwr_mgmt_1*/, BIT_RESET);

	// TODO: Sleep for 0.1 s
	count = 1600;
	do
		__delay_cycles(1000);
	while (--count);

	// Wake up chip
	MPU9250_write(0x6B /*pwr_mgmt_1*/, 0);

	// Set gyro full-scale range
	MPU9250_write(0x1B /*gyro_cfg*/, (INV_FSR_2000DPS << 3) | 0x03);
	// the 0x03 is to enable the LPF

	// Set accelerometer full-scale range
	MPU9250_write(0x1C /*accel_cfg*/, INV_FSR_16G << 3);

	// Set digital low-pass filter for gyro
	MPU9250_write(0x1A /*lpf*/, INV_FILTER_42HZ);

	// Set sample rate to 1000 Hz
	MPU9250_write(0x19 /*rate_div*/, 1000 / 1000 - 1);
	// by default, the SDK would now overwrite the low-pass filter to be 20 Hz,
	// but let's skip this for now...

	// Set up interrupt pin
	MPU9250_write(0x37 /*int pin*/, 0xC0); // open-drain, active low

	// Disable data ready interrupt.
	MPU9250_write(0x38 /*int_enable*/, 0); //BIT_DATA_RDY_EN;

	// There was the set_bypass here... but that was for offboard sensors

	// Configure sensors... enable gyro and don't disable accel
	MPU9250_write(0x6B /*pwr_mgmt_1*/, INV_CLK_PLL);

	MPU9250_write(0x6C /*pwr_mgmt_2*/, 0);

	return 0;
}


int MPU9250_sleep()
{
	// Reset device
	MPU9250_write(0x06B /*pwr_mgmt_1*/, BIT_SLEEP);
	return 0;
}


void MPU9250_get_raw_values()
{
	XYZ XYZdata;

	XYZdata.xyz.XX = 0; // is this still necessary?
	XYZdata.xyz.YY = 0;
	XYZdata.xyz.ZZ = 0;

	MPU9250_read(0x3B /* raw_accel*/, XYZdata.axis_data, 6);
	XYZdata.xyz.XX = _SWAP_BYTES(XYZdata.xyz.XX);
	XYZdata.xyz.YY = _SWAP_BYTES(XYZdata.xyz.YY);
	XYZdata.xyz.ZZ = _SWAP_BYTES(XYZdata.xyz.ZZ);

	MPU9250_read(0x43 /*raw_gyro*/, XYZdata.axis_data, 6);
	XYZdata.xyz.XX = _SWAP_BYTES(XYZdata.xyz.XX);
	XYZdata.xyz.YY = _SWAP_BYTES(XYZdata.xyz.YY);
	XYZdata.xyz.ZZ = _SWAP_BYTES(XYZdata.xyz.ZZ);

	return;
}


//******************************************************************************
//	write one byte to MPU9250 from buffer
//
//		regaddr = mpu9250 register (must be < 128)
//		   data = value to write
//
//		returns last byte read
//
void MPU9250_write(uint8_t regaddr, uint8_t data)
{
	bb_spi_assert_csel();
	bb_spi_write(regaddr);
	bb_spi_write(data);
	bb_spi_release_csel();
	return;
}


//******************************************************************************
//	read multiple bytes from MPU9250 into buffer
//
//		regaddr = mpu9250 register
//		    buf = pointer to buffer, or 0 to get value from return
//		  count = # of bytes to read
//
//		returns last byte read
//
uint8_t MPU9250_read(uint8_t regaddr, uint8_t *buf, uint8_t count)
{
	uint8_t val;

	// Assert chip select
	bb_spi_assert_csel();

	// Write register address
	bb_spi_write(0x80+regaddr);		// First bit is to read

	// If buf is null, just return one byte
	if (!buf) {
		val = bb_spi_read();
		bb_spi_release_csel();
		return val;
	}

	// Otherwise, copy the bytes into buf
	do
		*buf++ = bb_spi_read();
	while (--count > 0);

	// Release chip select
	bb_spi_release_csel();

	// Return the last value read
	return *(buf-1);
}

