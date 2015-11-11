/*
 * I2C.h
 *
 * Created: 25.8.2015 14:22:11
 *  Author: Markus
 */ 


#ifndef I2C_H_
#define I2C_H_

#define F_CPU 8000000
#include <avr/io.h>
#include <util/delay.h>

/* function prototypes */

void i2c_init();
uint8_t i2c_read_sda();
uint8_t i2c_read_scl();
void i2c_set_sda_low();
void i2c_set_scl_low();


void i2c_start_cond();
void i2c_stop_cond();
void i2c_write_bit(uint8_t bit);
uint8_t i2c_read_bit();
uint8_t i2c_transmit(uint8_t byte, uint8_t start, uint8_t stop);
uint8_t i2c_receive(uint8_t nack, uint8_t stop);


#endif /* I2C_H_ */