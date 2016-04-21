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
typedef enum {
	_1MHZ, _8MHZ, _12MHZ, _16MHZ
} CLOCK_SPEED;

//Status LED
#define LED0_PORT 2
#define LED0_PIN BIT6

//Universally defined registers
#define TYPE_REG 0
#define STATUS_REG 1

//Watchdog defines
#define WDT_HZ 200
#define WDT_CTL WDT_ADLY_1_9
#define USI_TIMEOUT (WDT_HZ/2)

//Important defines
#define TABLE_SIZE 32
#define CLOCK _16MHZ

//available functions
void main();

//Function Prototypes -- must be declared by user
uint8_t device_init();
void tick();
void set_register(uint8_t value);
uint8_t get_register();

//Variable externs
extern volatile uint8_t registers[TABLE_SIZE];
extern volatile uint16_t current_register;
extern volatile uint16_t tick_speed;

#endif /* BERRY_H_ */
