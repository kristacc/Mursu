/*
 * MS5607.c
 *
 * Created: 25.8.2015 15:55:29
 *  Author: Markus
 */ 

#include "MS5607.h"

void ms5607_init( ms5607_conf_t *conf){
	uint8_t address = 0xEE;
	uint16_t value = 0;
					
	for( uint8_t i = 0; i < 6 ; ++i){
		i2c_transmit(address, 1, 0 );
		i2c_transmit(0xA2 + 2*i, 0, 1);
						
		i2c_transmit(address | 0x01, 1, 0 );
		value = i2c_receive(0,0);
		value <<= 8;
		value |= i2c_receive(1,1);	
		conf->coeffs[i] = value;
		}
}

uint32_t ms5607_read_d1(){
	uint8_t address = 0xEE;
	uint32_t value = 0;
		
	i2c_transmit(address, 1, 0);
	i2c_transmit(0x42, 0, 1);


	i2c_transmit(address, 1, 0);
	i2c_transmit(0x00, 0, 1);

	_delay_ms(10);

	i2c_transmit(address | 0x01, 1, 0);

	value = i2c_receive(0, 0);
	value <<= 8;
	value |= i2c_receive(0, 0);
	value <<= 8;
	value |= i2c_receive(1, 1);
	return value;	
}
uint32_t ms5607_read_d2(){
	uint8_t address = 0xEE;
	uint32_t value = 0;
	
	
	i2c_transmit(address, 1, 0);
	i2c_transmit(0x52, 0, 1);
					
					
	i2c_transmit(address, 1, 0);
	i2c_transmit(0x00, 0, 1);
					
	_delay_ms(10);
					
	i2c_transmit(address | 0x01, 1, 0);
				
	value = i2c_receive(0, 0);
	value <<= 8;
	value |= i2c_receive(0, 0);
	value <<= 8;
	value |= i2c_receive(1, 1);	
	
	return value;
}

int32_t ms5607_get_temperature( ms5607_conf_t *conf ){
	//uint32_t d2 = ms5607_read_d2();
	//int32_t dt = (int32_t)((int64_t)d2 - (int64_t)((int32_t)conf->coeffs[4]*256));
	//return 2000 + (dt*conf->coeffs[5]) / ((int32_t)1 << 23);
	int64_t d2 = ms5607_read_d2();
	int64_t dt = d2 - ((int64_t)(conf->coeffs[4]))*((int64_t)256);
	
	return (int64_t)2000L + (dt*((int64_t)(conf->coeffs[5]))) / ((int64_t)8388608L);
}

int32_t ms5607_get_pressure( ms5607_conf_t *conf ){
	int64_t d1 = ms5607_read_d1();
	int64_t d2 = ms5607_read_d2();
	int64_t dt = d2 - ((int64_t)(conf->coeffs[4]))*((int64_t)256);
	
	int64_t off = (int64_t)(conf->coeffs[1])*((int64_t)131072L) + ((int64_t)(conf->coeffs[3])*dt) / (int64_t)64L;
	int64_t sens  = (int64_t)(conf->coeffs[0])*((int64_t)65536L) + ((int64_t)(conf->coeffs[2])*dt) / (int64_t)127L;
	return ((d1 * sens)/((int64_t)2097152L) - off) / ((int64_t)32768L);
}
