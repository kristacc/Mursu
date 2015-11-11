/*
 * I2C.c
 *
 * Created: 25.8.2015 14:24:04
 *  Author: Markus
 */ 

#include "I2C.h"

/* function implementations */
/* I2C */
#define PIN_SCL PORTA1
#define PIN_SDA PORTA0

#define I2C_DELAY (100)

void i2c_init(){
	// SET A0 and A1 to input-mode
	DDRA &= ~( _BV(PIN_SCL) | _BV(PIN_SDA) );
	
	
}

uint8_t i2c_read_sda(){
	DDRA &= ~(_BV(PIN_SDA));
	return PINA & _BV(PIN_SDA);


}

uint8_t i2c_read_scl(){
	DDRA &= ~(_BV(PIN_SCL));
	return PINA & _BV(PIN_SCL);

}

void i2c_set_sda_low(){
	DDRA |= _BV(PIN_SDA);
	PORTA &= ~( _BV(PIN_SDA) );
	
}

void i2c_set_scl_low(){
	DDRA |= _BV(PIN_SCL);
	PORTA &= ~( _BV(PIN_SCL) );

}


void i2c_start_cond(){
	i2c_read_scl();
	_delay_us(I2C_DELAY);
	i2c_read_sda();
	_delay_us(I2C_DELAY);
	i2c_set_sda_low();
	_delay_us(I2C_DELAY);
	i2c_set_scl_low();
	_delay_us(I2C_DELAY);
}

void i2c_stop_cond(){
	i2c_set_sda_low();
	_delay_us(I2C_DELAY);
	// clock stretching
	while( i2c_read_scl() == 0){}
	_delay_us(I2C_DELAY);
}

void i2c_write_bit(uint8_t bit){
	if( bit ){
		i2c_read_sda();
		} else {
		i2c_set_sda_low();
	}

	_delay_us(I2C_DELAY);
	// clock stretching
	while( i2c_read_scl() == 0){}
	
	_delay_us(I2C_DELAY);
	i2c_set_scl_low();
	_delay_us(I2C_DELAY);
}

uint8_t i2c_read_bit(){
	uint8_t bit;
	
	i2c_read_sda();
	_delay_us(I2C_DELAY);
	// clock stretching
	while( i2c_read_scl() == 0){}
	
	bit = i2c_read_sda();
	_delay_us(I2C_DELAY);
	i2c_set_scl_low();
	_delay_us(I2C_DELAY);
	
	return bit > 0;
}

uint8_t i2c_transmit(uint8_t byte, uint8_t start, uint8_t stop){
	uint8_t bit, nack;
	
	if( start ){
		i2c_start_cond();
	}
	
	for( bit = 0 ; bit < 8 ; ++bit ){
		i2c_write_bit( byte & 0x80 );
		byte <<= 1;
	}
	
	nack = i2c_read_bit();
	if( stop ){
		i2c_stop_cond();
	}
	
	return nack;
}

uint8_t i2c_receive(uint8_t nack, uint8_t stop){
	uint8_t byte = 0;
	uint8_t bit;
	
	for( bit = 0 ; bit < 8 ; ++bit){
		byte = (byte << 1) | i2c_read_bit();
	}
	
	i2c_write_bit(nack);
	if( stop ){
		i2c_stop_cond();
	}
	
	return byte;
}

