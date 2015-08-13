/*
 * MursuBus.c
         __  _
        /oo\/ \
       /(_X_) /
      /  V-V  ~--.__    /\
      /              ~--~ /
      (__/\_______________/

 */ 



// defines clock rate, must be before every include because they depend on it
#define F_CPU 8000000


// includes all necessary libraries (pretty much the same stuff as in any processor code has):

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>


// Constants:

#define MODBUS_ADDRESS_IDX		(0)
#define MODBUS_FUNC_CODE_IDX	(1)

#define MODBUS_READ_HOLDING_REGISTER	(0x03)
#define MODBUS_WRITE_SINGLE_REGISTER	(0x06)

#define MODBUS_ILLEGAL_FUNCTION			(0x01)
#define MODBUS_ILLEGAL_DATA_ADDRESS		(0x02)
#define MODBUS_ILLEGAL_DATA_VALUE		(0x03)
#define MODBUS_DEVICE_FAILURE			(0x04)

#define MODBUS_CHANGE_SLAVE_ADDRESS		(0xCAFE)


// Global variables:

volatile uint8_t g_uart_new_message = 1;
volatile uint8_t g_uart_buffer[64] = {0};
volatile uint8_t g_uart_counter = 0;

volatile uint8_t modbus_address = 250;
volatile uint16_t device_serial_number = 0;

uint16_t modbus_start_address = 1000;
uint16_t modbus_stop_address = 1010;

uint16_t measurement_table[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
uint8_t ADDRESS_TEMP = 0;


// Function prototype used in interrupt handlers:
void timer_reset();


// Interrupt service request handlers:
// these functions should be introduced first because they don't have too much to do with the actual program, they just run in background
// no need for function prototypes as they are already defined in device specific start up code.

/* Interrupts stop the process in processor if something happens and
   forces processor to execute certain code parts. Then back to normal processing/same place it was before....
   This behaviour is hard wired to processor (==it is a physical property and in physical logic / logic ports).
*/

// Timer 0 compare A:
/* This is an interrupt that fires when timer 0 reaches compare A threshold.
   This is not a real function but a macro, so the result is calculated elsewhere.
   Global variable, that has to exists because it is defined in startup library
*/
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
	timer_reset();
}


/* function prototypes */

// confs timer 0 to work as intended
// (fires interrupt when Modbus message cap is reached meaning that by Modbus standard there can/will be a new message after that) 
void timer_init();

// confs needed pins to work with RS485-chip
void rs485_init();

// puts RS-485 chip into send mode
void rs485_mode_send();

// puts RS-485 chip and processor into receive mode (both are listening the Modbus-bus...)
void rs485_mode_receive();

// confs processors UART to work with the bus
void uart_init();

// processor sends one byte out
void uart_transmit(uint8_t byte);

// how many bytes is received, needed because in Modbus standard one can send messages with different lenghts.
// So, making sure that we don't read pass the message in buffer (==no old data)  
uint8_t uart_bytes_received();

// clears UART's received buffer, in reality it just changes the point where we write stuff, coming back to the start point point of the buffer.
void uart_clear();

// reads serial number and Modbus address from eepron for the device/Mursu
// in beginning always the same numbers but they can be overwritten with Python script and then those numbers will remain in memory
void configure();

// computes the checksum which is used in Modbus messages (error detection)
uint16_t compute_crc( uint8_t byte, uint16_t crc);

// in Modbus standard device must send out error response when something goes wrong (there is a fault in master's message or an error happening during handling the message)
void modbus_error_response( uint8_t func, uint8_t error );

// read stuff from register (stuff gets there by some other code...)
void modbus_read_register( uint16_t start, uint16_t stop );

// handles Modbus write register function, meaning master wants to write something into the register
void modbus_write_register( uint16_t address, uint16_t data );

// parses and handless received messages from master
void modbus_handle_message();

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


void timer_init(){
	// Clear on compare
	TCCR0A = _BV(WGM00);
	
	// prescaler to clock/64: 8MHz clock -> 125kHz
	TCCR0B = _BV(CS01) | _BV(CS00);
	
	// compare A interrupt when 1.5*modbus character time is reached (~450us)
	OCR0A = 60;
	// enable compare A interrupt
	TIMSK0 = _BV(OCIE0A);
}

void timer_reset(){
	TCNT0 = 0;
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
	
}


void modbus_handle_message(){
	// check address
	if( g_uart_buffer[MODBUS_ADDRESS_IDX] != modbus_address ){ return ; }
	
	// check function code (we support only "READ HOLDING REGISTER" and "WRITE SINGLE REGISTER" )
	// TODO: add support for diagnostics too
	
	
	
	
	
	if( (g_uart_buffer[MODBUS_FUNC_CODE_IDX] != MODBUS_READ_HOLDING_REGISTER) && (g_uart_buffer[MODBUS_FUNC_CODE_IDX] != MODBUS_WRITE_SINGLE_REGISTER)){
		modbus_error_response( g_uart_buffer[MODBUS_FUNC_CODE_IDX], MODBUS_ILLEGAL_FUNCTION );
		return;
	}
	
	
	if(g_uart_buffer[MODBUS_FUNC_CODE_IDX] == MODBUS_READ_HOLDING_REGISTER) {
		// check that message is long enough
		// device address + function code + 2*starting addres  + 2*quantity of registers  + 2*crc = 8
		if( g_uart_counter != 8 ){ return; }
	
		// local scope for handling message
		{
			uint16_t starting_address, quantity;
		
			// byte order: high, low
			starting_address =	(g_uart_buffer[2] << 8) | g_uart_buffer[3];
			quantity =			(g_uart_buffer[4] << 8) | g_uart_buffer[5];
		
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

/*
int read_temperature(void) {
	
	 uint8_t checksum;
	 uint8_t address = (1 << 7);
	 uint16_t temperature;
	 
	 //Kirjoitetaan ensin, etta halutaan mitata lampotila
	 i2c_transmit(address, 1, 0);
	 
	 // Lahetetaan mittauskomento 1110 0011 (0xE3)
	 i2c_transmit(0xE3, 0, 1);
	 
	 //Lahetetaan lukukomento
	 i2c_transmit((address | 1), 1, 0);
	 
	 temperature = i2c_receive(0,0);
	 temperature <<= 8;
	 temperature |= i2c_receive(0,0);
	 checksum = i2c_receive(1,1);
	
	measurement_table[ADDRESS_TEMP] = temperature;
		
	return 0;
}
*/


int main(void)
{
	
	// read configuration from EEPROM
	configure();

	// initialize subsystems
	timer_init();
	rs485_init();
	uart_init();

	// enable interrupts
	sei();


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
		
		// I need no reason
		_delay_us(100);
    }
}