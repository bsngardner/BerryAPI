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
#define GUID0 -1
#define GUID1 -2
#define GUID2 -3
#define GUID3 -4
#define GUID4 -5
#define GUID5 -6
#define GUID6 -7
#define GUID7 -8
#define INT_ENABLE -9
#define INTERRUPT -10

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
extern volatile uint16_t tick_count;

#endif /* BERRY_H_ */
