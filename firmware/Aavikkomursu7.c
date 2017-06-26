/*
 * Aavikkomursu7.c
 *
 * Created: 5.7.2015 22:04
 * Author: Markus, Tuikku & Krista
 *
 */ 

#define F_CPU 8000000
#include <avr/interrupt.h>
#include <util/delay.h>

#include <avr/eeprom.h>

#include "Modbus.h"
#include "I2C.h"

#include "MS5607.h"

/* Defines and constants */

/* Constants*/

const uint8_t hextable[16] = "0123456789abcdef";


/* global variables */

volatile uint8_t g_uart_new_message = 1;
volatile uint8_t g_uart_buffer[64] = {0};
volatile uint8_t g_uart_counter = 0;

volatile uint8_t modbus_address = 250;
volatile uint16_t device_serial_number = 0;

uint16_t modbus_start_address = 1000;
uint16_t modbus_stop_address = 1010;

uint16_t measurement_table[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
uint8_t ADDRESS_TEMP = 0;

int16_t measurement_offsets[8] = {0,0,0,0};

// function prototypes used in interrupt handlers
void timer0_reset();


/* Interrupt service request handlers */

/* timer 0 compare A */
ISR(TIMER0_COMPA_vect){
	g_uart_new_message = 1;
}

/* UART receives a byte */
ISR(USART0_RX_vect){
	if( g_uart_new_message == 1){
		g_uart_counter = 0;
		g_uart_new_message = 0;
	}
	
	g_uart_buffer[ g_uart_counter ] = UDR0;
	
	++g_uart_counter;
	
	// don't let buffer overflow
	if( g_uart_counter > 63){ g_uart_counter = 63; }
	timer0_reset();
}


/* function prototypes */
void timer0_init();
void timer1_init();
void timer1_reset();

void rs485_init();
void rs485_mode_send();
void rs485_mode_receive();

void uart_init();
void uart_transmit(uint8_t byte);
uint8_t uart_bytes_received();
void uart_clear();

void configure();

uint16_t compute_crc( uint8_t byte, uint16_t crc);

void modbus_error_response( uint8_t func, uint8_t error );
void modbus_read_register( uint16_t start, uint16_t stop );
void modbus_handle_message();

void adc_init();
uint16_t adc_read(uint8_t channel);

/* function implementations */

void rs485_init(){
	// init RS-485
	DDRA |= _BV(PORTA6); // RW pin as output
	PORTA &= ~(_BV(PORTA6)); // read enable
}

void rs485_mode_send(){
	// disable UART receiving
	UCSR0B &= ~(_BV(RXEN0) | _BV(RXCIE0));
	
	PORTA |= _BV(PORTA6); // write enable
	_delay_us(10);
}

void rs485_mode_receive(){
	// wait for previous transmit to finish
	while( !( (UCSR0A) & _BV(UDRE0) ) ){
		_delay_us(1);
	}
	
	// wait at least 11bits/38400baud (286us ~ 300us)
	// to be sure that all bits are written to the bus
	_delay_us(300);
	
	PORTA &= ~(_BV(PORTA6)); // read enable
	_delay_us(10);
	
	// enable UART receiving
	UCSR0B |= _BV(RXEN0) | _BV(RXCIE0);
	
	uart_clear(); // clear buffer
}


void timer0_init(){
	// Clear on compare
	TCCR0A = _BV(WGM00);
	
	// prescaler to clock/64: 8MHz clock -> 125kHz
	TCCR0B = _BV(CS01) | _BV(CS00);
	
	// compare A interrupt when 1.5*modbus character time is reached (~450us)
	OCR0A = 60;
	// enable compare A interrupt
	TIMSK0 = _BV(OCIE0A);
}

void timer1_init(){
	TCCR1A = 0;
	
	// prescaler to clock/1024: 8MHz clock -> 7812.5Hz
	TCCR1B = _BV(CS11) | _BV(CS01) | _BV(CS00);
}

void timer0_reset(){
	TCNT0 = 0;
}

void timer1_reset(){
	TCNT1 = 0;
}


void uart_init(){
	
	// initializes UART0 hardware

	// 38.4 kbaud (datasheet pg 179 )
	UBRR0H = 0;
	UBRR0L = 12;

	// enable transmitter, receiver and receiver interrupt
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
	
	// data format: 8 bit, 1 stop bit, 	no parity (datasheet pg 183 - 184)
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	
	// remap UART pins to secondary configuration
	REMAP = _BV(U0MAP);
	
}

void uart_transmit( uint8_t byte ){
	// wait for previous transmit to finish
	while( !( (UCSR0A) & _BV(UDRE0) ) ){}
	
	// transmit
	UDR0 = byte;
	
}

uint8_t uart_bytes_received(){
	return g_uart_counter;
}

void uart_clear(){
	g_uart_counter = 0;
}


void configure(){
	device_serial_number = eeprom_read_word( (uint16_t *)0 );
	modbus_address = eeprom_read_byte( (uint8_t *)2 );
	
	if( (modbus_address > 250) || (modbus_address < 2)){
		modbus_address = 250;
	}
	//modbus_address = 250;
	
	uint16_t offset0 = eeprom_read_word((uint16_t *)4);
	uint16_t offset1 = eeprom_read_word((uint16_t *)5);
	uint16_t offset2 = eeprom_read_word((uint16_t *)6);
	uint16_t offset3 = eeprom_read_word((uint16_t *)7);
	
	if( offset0 < 0xFFFF){ measurement_offsets[0] = (int16_t)offset0 - (int16_t)10000; }
	if( offset1 < 0xFFFF){ measurement_offsets[1] = (int16_t)offset1 - (int16_t)10000; }
	if( offset2 < 0xFFFF){ measurement_offsets[2] = (int16_t)offset2 - (int16_t)10000; }
	if( offset3 < 0xFFFF){ measurement_offsets[3] = (int16_t)offset3 - (int16_t)10000; }
	
	
}

uint16_t compute_crc( uint8_t byte, uint16_t crc){
	uint8_t i;
	crc ^= byte;
	for( i = 0; i < 8 ; ++i  ){
		if( (crc & 0x0001) != 0 ){
			crc >>= 1;
			crc ^= 0xA001;
		}
		else {
			crc >>= 1;
		}
	}
	
	return crc;
}

void modbus_error_response( uint8_t func, uint8_t error ){
	uint16_t msg_crc = 0xFFFF; // modbus crc initial value
	
	rs485_mode_send();
	
	// message format: | address | function code with highest bit on | error code | crc |
	
	// transmit message and update crc
	
	// send address
	uart_transmit( modbus_address );
	msg_crc = compute_crc( modbus_address, msg_crc ); // update crc
	
	// send function code with highest bit on (to mark that this is an error response)
	uart_transmit( func | 0x80 );
	msg_crc = compute_crc( func | 0x80, msg_crc ); // update crc
	
	// send error code
	uart_transmit( error );
	msg_crc = compute_crc( error, msg_crc ); // update crc
	
	
	// transmit crc
	uart_transmit( msg_crc & 0xFF );
	uart_transmit( (msg_crc >> 8) & 0xFF );
	
	// done
	rs485_mode_receive();
}

void modbus_read_device_serial_number(){
	uint16_t msg_crc = 0xFFFF; // modbus crc initial value
	
	// message format: | address | function code | byte count | register contents | crc |
	
	
	rs485_mode_send();
	
	// send address
	uart_transmit( modbus_address );
	msg_crc = compute_crc( modbus_address, msg_crc );

	// send function code
	uart_transmit( MODBUS_READ_HOLDING_REGISTER );
	msg_crc = compute_crc( MODBUS_READ_HOLDING_REGISTER, msg_crc );
	
	// send byte count ( amount of register * 2 bytes (16bit words) )
	uart_transmit( (1) * 2 );
	msg_crc = compute_crc( (1) * 2, msg_crc );

	uart_transmit( (device_serial_number >> 8) & 0xFF );
	msg_crc = compute_crc( (device_serial_number >> 8) & 0xFF, msg_crc );
		
	uart_transmit( device_serial_number & 0xFF );
	msg_crc = compute_crc( device_serial_number & 0xFF, msg_crc );

	// transmit crc
	uart_transmit( msg_crc & 0xFF );
	uart_transmit( (msg_crc >> 8) & 0xFF );

	// done
	rs485_mode_receive();


}



void modbus_read_register( uint16_t start, uint16_t stop ){
	uint16_t msg_crc = 0xFFFF; // modbus crc initial value
	uint16_t addr;
	
	// message format: | address | function code | byte count | register contents | crc |
	
	
	rs485_mode_send();
	
	// send address
	uart_transmit( modbus_address );
	msg_crc = compute_crc( modbus_address, msg_crc );
	
	// send function code
	uart_transmit( MODBUS_READ_HOLDING_REGISTER );
	msg_crc = compute_crc( MODBUS_READ_HOLDING_REGISTER, msg_crc );
	
	// send byte count ( amount of register * 2 bytes (16bit words) )
	uart_transmit( (stop - start) * 2 );
	msg_crc = compute_crc( (stop - start) * 2, msg_crc );
	
	
	//read_temperature();
	
	// send registers
	for( addr = start; addr < stop; ++addr ){
		// get register content
		// placeholder stuff here
		
		uint8_t byte;
		
		
		
		uint16_t register_content = measurement_table[addr - modbus_start_address]; //0xABBA;
		
		// splitting register value in high and low byte
		// and send them byte by bye
		
		byte = (register_content >> 8);
		uart_transmit( byte );
		msg_crc = compute_crc( byte, msg_crc );
		
		byte = (register_content & 0xFF);
		uart_transmit( byte );
		msg_crc = compute_crc( byte, msg_crc );
	}

	// transmit crc
	uart_transmit( msg_crc & 0xFF );
	uart_transmit( (msg_crc >> 8) & 0xFF );

	// done
	rs485_mode_receive();
}

void modbus_write_register(uint16_t address, uint16_t data){
	
	uint16_t msg_crc = 0xFFFF; // modbus crc initial value
	uint8_t byte;
	// message format: | address | function code | byte count | register contents | crc |
	
	
	rs485_mode_send();
	
	// send address
	uart_transmit( modbus_address );
	msg_crc = compute_crc( modbus_address, msg_crc );
	
	// send function code
	uart_transmit( MODBUS_WRITE_SINGLE_REGISTER );
	msg_crc = compute_crc( MODBUS_WRITE_SINGLE_REGISTER, msg_crc );
	
	// send register address
	byte = (address >> 8);
	uart_transmit( byte );
	msg_crc = compute_crc( byte, msg_crc );
	
	byte = (address & 0xFF);
	uart_transmit( byte );
	msg_crc = compute_crc( byte, msg_crc );
	
	// send data
	
	byte = (data >> 8);
	uart_transmit( byte );
	msg_crc = compute_crc( byte, msg_crc );
	
	byte = (data & 0xFF);
	uart_transmit( byte );
	msg_crc = compute_crc( byte, msg_crc );

	// transmit crc
	uart_transmit( msg_crc & 0xFF );
	uart_transmit( (msg_crc >> 8) & 0xFF );

	// done
	rs485_mode_receive();
	
	// Change slave address
	if(address == MODBUS_CHANGE_SLAVE_ADDRESS){
		
		modbus_address = data;
		eeprom_write_byte((uint8_t *)2, modbus_address);
	}
	
	if( address == (MODBUS_SET_OFFSET_ADDRESS + 0)){
		measurement_offsets[0] = (int16_t)data  - (int16_t)10000;
		eeprom_write_word((uint16_t *)4, data);
	}
	
	if( address == (MODBUS_SET_OFFSET_ADDRESS + 1)){
		measurement_offsets[1] = (int16_t)data  - (int16_t)10000;
		eeprom_write_word((uint16_t *)5, data);
	}
	
	if( address == (MODBUS_SET_OFFSET_ADDRESS + 2)){
		measurement_offsets[2] = (int16_t)data  - (int16_t)10000;
		eeprom_write_word((uint16_t *)6, data);
	}
	
	if( address == (MODBUS_SET_OFFSET_ADDRESS + 3)){
		measurement_offsets[3] = (int16_t)data  - (int16_t)10000;
		eeprom_write_word((uint16_t *)6, data);
	}
	
	if( address == MODBUS_CHANGE_SERIAL_NUMBER){
		device_serial_number = data;
		eeprom_write_word((uint16_t *)0, data);
	}
}




void modbus_handle_message(){
	// check address
	if( g_uart_buffer[MODBUS_ADDRESS_IDX] != modbus_address ){ return ; }
	
	// check function code (we support only "READ HOLDING REGISTER" )
	// TODO: add support for diagnostics too
	
	if( (g_uart_buffer[MODBUS_FUNC_CODE_IDX] != MODBUS_READ_HOLDING_REGISTER) && (g_uart_buffer[MODBUS_FUNC_CODE_IDX] != MODBUS_WRITE_SINGLE_REGISTER) ){
		modbus_error_response( g_uart_buffer[MODBUS_FUNC_CODE_IDX], MODBUS_ILLEGAL_FUNCTION );
		return;
	}
	
	if(g_uart_buffer[MODBUS_FUNC_CODE_IDX] == MODBUS_READ_HOLDING_REGISTER) {
		if( g_uart_counter != 8 ){ return; }
	
		// local scope for handling message
		{
			uint16_t starting_address, quantity;
		
			// byte order: high, low
			starting_address =	(g_uart_buffer[2] << 8) | g_uart_buffer[3];
			quantity =			(g_uart_buffer[4] << 8) | g_uart_buffer[5];
			
			// special cases
			if( (starting_address == MODBUS_DEVICE_SERIAL_ADDRESS) && (quantity == 1) ){
				modbus_read_device_serial_number();
				return;
			}
		
			// test that start and stop addresses (starting_address + quantity) are within right range
			if( starting_address < modbus_start_address ){
				modbus_error_response( MODBUS_READ_HOLDING_REGISTER, MODBUS_ILLEGAL_DATA_ADDRESS );
				return;
			}
		
			if( starting_address + quantity > modbus_stop_address ){
				modbus_error_response( MODBUS_READ_HOLDING_REGISTER, MODBUS_ILLEGAL_DATA_ADDRESS );
				return;
			}
		
			// everything seems to be ok -> delegate query forward
			modbus_read_register( starting_address, starting_address + quantity );
		}
	}
	
	if(g_uart_buffer[MODBUS_FUNC_CODE_IDX] == MODBUS_WRITE_SINGLE_REGISTER) {
			
			uint16_t register_address, data;
			
			// byte order: high, low
			register_address =	(g_uart_buffer[2] << 8) | g_uart_buffer[3];
			data =			    (g_uart_buffer[4] << 8) | g_uart_buffer[5];
			modbus_write_register(register_address,data);
		}

}





int main(void)
{
	ms5607_conf_t painemittari;
	
	// read configuration from EEPROM
	configure();

	// initialize subsystems
	timer0_init();
	timer1_init();
	rs485_init();
	uart_init();

	i2c_init();

	// enable interrupts
	sei();
	
	_delay_ms(10);
	// two times, just to be sure
	ms5607_init(&painemittari);
	_delay_ms(100);
	ms5607_init(&painemittari);

    while(1)
    {
		
		// wait for message to arrive (and test if it is long enough
		if( (g_uart_new_message == 1) && (g_uart_counter > 2) ){
			modbus_handle_message();
			
			// message handled, clear uart buffer
			uart_clear();
		} 
		else {
			// we have a message, but it is too short
			if( g_uart_new_message == 1){
				// message was too short, clear buffer
				uart_clear();
			}
		// continue to wait for the message to arrive
		}
		
		
		// every ~0.5s
		if( TCNT1 > 4000 ) {
			int32_t t;
			uint32_t value;
			
			t = ms5607_get_temperature(&painemittari);
			t = ms5607_get_temperature(&painemittari);
			value = t;
			
			value += measurement_offsets[0];
			
			measurement_table[0] = (value >> 16) & 0xFFFF;
			measurement_table[1] = (value ) & 0xFFFF;
			
			
			t = ms5607_get_pressure(&painemittari);
			value = t;
			
			value += measurement_offsets[1];
			
			measurement_table[2] = (value >> 16) & 0xFFFF;
			measurement_table[3] = (value ) & 0xFFFF;
			
			
			// humidity
			{
				uint8_t byte0, byte1, checksum;
				uint16_t humidity;
				
				i2c_transmit(0x80, 1, 0);
				i2c_transmit(0xE5, 0, 1);
				
				i2c_transmit(0x81, 1, 0);
				byte0 = i2c_receive(0,0);
				byte1 = i2c_receive(0,0);
				checksum = i2c_receive(1,1);
				checksum = checksum;
				
				humidity = (byte0 << 8) | byte1;
				
				
				humidity >>= 2;
				
				{
					int32_t rh = humidity << 2;
					rh = -((int32_t)600) + (((int32_t)125*(int32_t)100)*rh)/((int32_t)65536);
					humidity = (int16_t)rh;
				}
				
				humidity += measurement_offsets[2];
				
				measurement_table[4] = humidity;
			}
			
			// temperature 2
			{
				uint8_t byte0, byte1, checksum;
				uint16_t temperature;
				
				i2c_transmit(0x80, 1, 0);
				i2c_transmit(0xE3, 0, 1);
				
				i2c_transmit(0x81, 1, 0);
				byte0 = i2c_receive(0,0);
				byte1 = i2c_receive(0,0);
				checksum = i2c_receive(1,1);
				checksum = checksum;
				
				temperature = (byte0 << 8) | byte1;
				
				
				temperature >>= 2;
				
				{
					int32_t temp = temperature << 2;
					temp = -(int32_t)4685 + (((int32_t)17572)*temp)/((int32_t)65536);
					temperature = (int16_t)temp;
				}
				
				temperature += measurement_offsets[3];
				
				measurement_table[5] = temperature;
			}

			// start waiting for next round
			timer1_reset();
		}
		
		
		// I need no reason
		_delay_us(100);
    }
}
