/*
 * MPU9250.c
 *
 *  Created on: Sep 5, 2014
 *      Author: Kristian Sims
 *      Built on the SDK by invensense (inv_mpu.c)
 */

#include <msp430.h>
#include "MPU9250.h"
#include "spi.h"

void MPU9250_reset() {
	char data[2];

	// Reset device
	data[0] = 0x06B; 					// pwr_mgmt_1
	data[1] = BIT_RESET;
	spi_transfer(data, 2);

	return;
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
int MPU9250_init()
{
	char data[2];

	// Wake up chip
	data[0] = 0x06B;
	data[1] = 0x00;
	spi_transfer(data, 2);

	// Disable i2c
	data[0] = 0x6A;
	data[1] = 0x10;
	spi_transfer(data, 2);

	// Set gyro full-scale range
	data[0] = 0x1B; // gyro_cfg
	data[1] = (INV_FSR_2000DPS << 3) | 0x03; // the 0x03 is to enable the LPF
	spi_transfer(data, 2);

	// Set accelerometer full-scale range
	data[0] = 0x1C; 					// accel_cfg
	data[1] = INV_FSR_16G << 3;
	spi_transfer(data, 2);

	// Set digital low-pass filter for gyro
	data[0] = 0x1A; 					// lpf
	data[1] = INV_FILTER_42HZ;
	spi_transfer(data, 2);

	// Set sample rate to 1000 Hz
	data[0] = 0x19; 					// rate_div
	data[1] = 1000 / 1000 - 1;
	spi_transfer(data, 2);
	// by default, the SDK would now overwrite the low-pass filter to be 20 Hz,
	// but let's skip this for now...

	// Set up interrupt pin
	data[0] = 0x37; // int pin
	data[1] = 0xC0; // open-drain, active low
	spi_transfer(data, 2);

	// Disable data ready interrupt.
	data[0] = 0x38; // int_enable
	data[1] = 0; //BIT_DATA_RDY_EN;
	spi_transfer(data, 2);

	// There was the set_bypass here... but that was for offboard sensors, I think

	// Configure sensors... enable gyro and don't disable accel
	data[0] = 0x6B; // pwr_mgmt_1
	data[1] = INV_CLK_PLL;
	spi_transfer(data, 2);

	data[0] = 0x6C; // pwr_mgmt_2
	data[1] = 0x00;
	spi_transfer(data, 2);

	// Check the WHOAMI register...
	data[0] = 0x80 + 0x75; // read
	data[1] = 0x00;
	spi_transfer(data, 2);

	return data[1] == 0x71 ? 0 : -1;
}

int MPU9250_sleep()
{
	char data[2];

	//WDT_sleep(WDT_1SEC_CNT); // Sleep for 1 s

	// Reset device
	data[0] = 0x06B; // pwr_mgmt_1
	data[1] = BIT_SLEEP;
	spi_transfer(data, 2);

	return 0;
}

int MPU9250_get_raw() {
	// Make a union to word align the data
	union {
		char bytes[14];
		int words[7];
	} raw_data;
	unsigned i;

	// Read gyroscope
	raw_data.bytes[7] = 0x80 + 0x43;
	spi_transfer(raw_data.bytes+7, 7);

	// Read accelerometer
	raw_data.bytes[1] = 0x80 + 0x3B;
	spi_transfer(raw_data.bytes+1, 7);

	// Fix endianness
	for (i=7; i>0; --i)
		raw_data.words[i]=(int)__swap_bytes((unsigned short)raw_data.words[i]);

	return 0;
}

int MPU9250_tick() {
	static unsigned timer = 500;		// ~5 ms?

	if (timer) {
		switch (timer--) {
		case 300:						// ~100 ms passed
			MPU9250_reset();
			break;
		case 100:						// ~200 ms passed
			MPU9250_init();
			break;
		default:
			break;
		}
	} else {
		MPU9250_get_raw();
	}
}
