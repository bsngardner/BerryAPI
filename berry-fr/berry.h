/*
 * bapi.h
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#ifndef BERRY_H_
#define BERRY_H_
#include <stdint.h>

//Typedef for clock speed adjustment
typedef enum
{
	_1MHZ, _8MHZ, _12MHZ, _16MHZ
} CLOCK_SPEED;

//Status LED
#define J 0

//Universally defined registers
#define STATUS 1
#define TYPE 0

// p1.5 is vine interrupt line - low asserted
#define ASSERT_INTR 	(P2DIR |= BINT) // make it an output
#define RELEASE_INTR	(P2DIR &= ~BINT) // high impedance input

//Watchdog defines
#define WDT_HZ 200
#define WDT_CTL WDT_ADLY_1_9
#define USI_TIMEOUT (WDT_HZ/2)

//Important defines
#define TABLE_SIZE 32
#define CLOCK _16MHZ

//Events
#define TICK_EVENT 0x01
#define FLASH_UPDATE_EVENT 0x02

//System registers
#define SLAVE_ADDR -1
#define PROJ_KEY0 -2
#define PROJ_KEY1 -3
#define INT_ENABLE -4
#define INTERRUPT -5

//available functions
void main();

//Internal functions
void sys_set_register(uint8_t value);
uint8_t sys_get_register();

//Function Prototypes -- must be declared by user
uint8_t device_init();
void tick();
void set_register(uint8_t value);
uint8_t get_register();

//Variable externs
extern volatile uint8_t* const registers;
extern volatile int16_t current_register;
extern volatile uint16_t tick_speed;

#endif /* BERRY_H_ */