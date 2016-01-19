/*
 * MS5607.h
 *
 * Created: 25.8.2015 15:51:26
 *  Author: Markus
 */ 


#ifndef MS5607_H_
#define MS5607_H_

#include "I2C.h"

typedef struct {
	uint16_t coeffs[6];
	} ms5607_conf_t;

void ms5607_init( ms5607_conf_t *conf);
uint32_t ms5607_read_d1();
uint32_t ms5607_read_d2();

int32_t ms5607_get_temperature( ms5607_conf_t *conf );
int32_t ms5607_get_pressure( ms5607_conf_t *conf );


#endif /* MS5607_H_ */