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

//Register table struct
typedef struct {
	volatile uint8_t * table;
	volatile uint8_t size;
	volatile uint8_t current;
} register_table_t;

//Status LED
#define LED0_PORT 2
#define LED0_PIN BIT6

//Watchdog defines
#define WDT_HZ 2000
#define WDT_CTL WDT_MDLY_8
#define USI_TIMEOUT (WDT_HZ/2)

//Important defines
#define TABLE_SIZE 16
#define CLOCK	_16MHZ

//available functions
int bapi_init(CLOCK_SPEED clock);
inline void check_timeout();
extern void main();

//Function Prototypes -- must be declared by user
uint8_t device_init();
void set_register(uint8_t value);
uint8_t get_register();

#endif /* BERRY_H_ */
