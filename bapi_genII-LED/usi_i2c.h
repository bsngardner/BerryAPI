/*
 * usi_i2c.h
 *
 *  Created on: Nov 5, 2015
 *      Author: Broderick
 */

#ifndef USI_I2C_H_
#define USI_I2C_H_

#include <stdint.h>

//Defines
//#define IOBUFFER_SIZE 30
//#define SLV_ADDR 0x55<<1
#define ADDR_REG 0
#define SDA_PIN BIT7
#define SCL_PIN BIT6

typedef enum {
	I2C_IDLE = 0,
	I2C_START = 2,
	I2C_ADDR = 4,
	I2C_RX = 6,
	I2C_HANDLE_RX = 8,
	I2C_HANDLE_GLOBAL = 10,
	I2C_READ_STOP = 12,
	I2C_TX = 14,
	I2C_RX_NACK = 16,
	I2C_HANDLE_NACK = 18,
	I2C_RESET = 20
} i2c_state_t;

//Prototypes
void init_usi();

//Register table struct
typedef struct {
	volatile uint8_t * table;
	volatile uint8_t size;
	volatile uint8_t current;
} register_table_t;

//Extern functions
extern inline uint8_t get_register(uint8_t reg);
extern inline void set_register(uint8_t reg, uint8_t value);
extern void set_current_register(uint8_t value);
extern uint8_t get_current_register();

//Extern variables
extern volatile register_table_t registers;

#endif /* USI_I2C_H_ */
