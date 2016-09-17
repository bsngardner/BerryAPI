/*
 * Firmware for accelerometer device, berry08-accel
 * 		Broderick Gardner
 * 		6/15/2016
 */

#include <msp430.h> 

#include "berry.h"
#include "spi.h"
#include "MPU9250.h"

#define DEV_TYPE 0x08

const int led0_port = 1;
const int led0_pin = BIT0;

#define R_READ0 2
#define R_READ1 3
#define R_READB 4

volatile uint16_t level = 4;

void timer_init();

uint8_t device_init()
{
	spi_init();

	return DEV_TYPE;
}

void tick()
{
	MPU9250_tick();
}

void set_register(uint8_t value)
{
	switch (current_register)
	{
	case 2:
		registers[2] = value;

		break;
	case 3:

		break;
	case 4:

		break;
	}

	return;
}

uint8_t get_register()
{
	switch (current_register)
	{
	//On reading each byte of the potentiometer value,
	//	switch current register to the other byte
	//	This allows reading 2 bytes in quick succession
	case R_READ0:
		current_register = R_READ1;
		return registers[R_READ0];
	case R_READ1:
		current_register = R_READ0;
		return registers[R_READ1];
	case R_READB:
		return registers[R_READB];
	default:
		break;
	}
	return 0;
}
