/*
 * bapi.h
 *
 *  Created on: Feb 2, 2016
 *      Author: Broderick
 */

#ifndef BAPI_H_
#define BAPI_H_
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

//Global externs
extern register_table_t reg_table;

//Status LED
#define LED0_PORT 2
#define LED0_PIN BIT6

//Register table defines
#define TABLE_SIZE 16

//Watchdog defines
#define WDT_HZ 2000
#define WDT_CTL WDT_MDLY_8
#define USI_TIMEOUT (WDT_HZ/2)

//available functions
int bapi_init(CLOCK_SPEED clock, uint8_t device_type);
inline void check_timeout();

#endif /* BAPI_H_ */
