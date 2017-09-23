/*

 * Aavikkomursu7.c
 *
 * Created: 5.7.2015 22:04
 * Author: Markus, Tuikku & Krista
 *
 */ 

/*
         __  _
        /oo\/ \
       /(_X_) /
      /  V-V  ~--.__    /\
      /              ~--~ /
      (__/\_______________/

   bit == 1 or 0
   byte == 8bits
   
 */ 



// defines clock rate, must be before every include because they depend on it
#define F_CPU 8000000


// includes all necessary libraries (pretty much the same stuff as in any processor code has):


#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "Modbus.h"
#include "I2C.h"
#include "MS5607.h"


/* Defines and constants */

/* global variables */

volatile uint8_t g_uart_new_message = 1;

// longest modbus message can be 64bytes long
// messages could be longer but we try to save device's memory and keep messages short and sweet
volatile uint8_t g_uart_buffer[64] = {0};
volatile uint8_t g_uart_counter = 0;

// address to 250 as it is high number, so there should not be mix ups easily: people tend to give small address numbers if they change them -> new boards should not interfere with existing ones.
volatile uint8_t modbus_address = 250;
volatile uint16_t device_serial_number = 0;

// we choose memory slot that is easy to remember and does not come across accidentally because if it is for example 0, you just have forgotten to initialise your variable and use that slot
uint16_t modbus_start_address = 1000;
uint16_t modbus_stop_address = 1010;

// place to save all data that is measured. If one gets this specific sequence back -> no data has ever been saved into this slot (or you have got reallyreally good chances to win lottery :) So, for predebugging just in case)
uint16_t measurement_table[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};

// sensors offsets if need to "calibrate"
int16_t measurement_offsets[8] = {0,0,0,0};

// function prototypes used in interrupt handlers
void timer0_reset();


/* Interrupt service request handlers */
// these functions should be introduced first because they don't have too much to do with the actual program, they just run in background
// no need for function prototypes as they are already defined in device specific start up code.

/* Interrupts stop the process in processor if something happens and
   forces processor to execute certain code parts. Then back to normal processing/same place it was before....
   This behaviour is hard wired to processor (==it is a physical property and in physical logic / logic ports).
*/

/* timer 0 compare A */
/* This is an interrupt that fires when timer 0 reaches compare A threshold.
   This is not a real function but a macro, so the result is calculated elsewhere.
   Global variable, that has to exists because it is defined in startup library
*/
ISR(TIMER0_COMPA_vect){
	// before this interrupt value is 0 unless this fires twice and uart has received no bytes.
	// when 1, it means that there has been enought time without received bytes -> new byte belongs to new message (modbus rule)
	g_uart_new_message = 1;
}

/* UART receives a byte    ISR == interrupt service request

 */
// USART0_RX_vect -name from data sheet:
// USART0_RX_vect usart0-device receives a byte (usart means that it could also work in syncronous mode,
// meaning explicit clock signal that tells when bit changes, so not sniffing when there is a change)

ISR(USART0_RX_vect){
	if( g_uart_new_message == 1){
		// counter: how many bytes has been received to zero as a new message
		g_uart_counter = 0;
		// now we are already in new message, so it doesn't begin anymore... 
		g_uart_new_message = 0;
	}
	
	// save received byte to buffer, to correct place
	// UDR0 == memory address where the new byte is located (from chip manual...)
	g_uart_buffer[ g_uart_counter ] = UDR0;
	
	// add one to the place count....
	++g_uart_counter;
	
	// don't let buffer overflow
	// when buffer full (we only have 64 bytes because we had to limit memory usage, so just a picked number -sort of)
	// if overflow happens, stop increasing byte counter -> last byte will be corrupted and overwritten
	// tough luck
	if( g_uart_counter > 63){ g_uart_counter = 63; }

	// zero timer0, so it won't fire in middle of message
	timer0_reset();
}


/* function prototypes */

// confs timer 0 to work as intended
// (fires interrupt when Modbus message cap is reached meaning that by Modbus standard there can/will be a new message after that) 
void timer0_init();
void timer1_init();
void timer1_reset();

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

	// More careful explanation:
	//  PORTA6 == port A, in pin no 6, certain pin
	//  port A is just definition and comes from chips inner structure
	//  _BV is a macro, and means BITVALUE -> this takes the ID of the bit
	//  and returns binary number of the pin. Macro is ready-made by Atmel
	
	//  DDRA == data direction register A
	//  sets wether the port A's pins are outputs or inputs.
	//  1 means output, 0 input
	//  Here we choose pin 6 as output because we want to control the RW pin of RS485 chip
	//  |=  means 'bit OR' meaning that we only touch/select DDRA bits that are 1 in _BV()
	//  
	//  DDRA |= _BV(PORTA6);  is same as  DDRA = DDRA | _BV(PORTA6);
	//  But later is not as effective/fast because compiler produces different output
	//  Info can be found from manual
	//  
	//   ~()  is 'bit NOT' meaning that it changes 0 ->1 and 1-> 0
	//   &=  'bit AND'  meaning that it will zero PORTA bits that are 0 in ~(_BV(PORTA6))
	//   PORTA &= ~(_BV(PORTA6))  ==  PORTA = PORTA & ~(_BV(PORTA6))  (again compiler stuff)

}

void rs485_mode_send(){
	// disable UART receiving
	// First we take electricity off from UART-receiver circuit (inside processor), ~(_BV(RXEN0) 
	// And also we we disable UART receive interrupt, _BV(RXCIE0)
	// Because then anything we write to the bus won't be received back immediately:
	// we are sending and do not want to receive that message :P
	
	// UCSR0B == UART control and status register B (p. 182, datasheet ATtiny841, using UART 0)
	// _BV means bit value
	// RXEN0 == receiver enable (p. 182), it is a CONSTANT (or preprocessor label) containing bit number that is used for
	// switching bit (to 0) that controls a pin -pin that when put to 0 turns UART receiver off.
	
	// RXCIE0 == receive (RX) complete interrupt enable
	// RX == receive in electrical engineering jargon....
	// TX == transmit in electrical engineering jargon....
	
	// UCSR0B &= means UCSR0B = UCSR0B & ...., meaning '+=', meaning 'a = a + ...'
	// A | B means: bitwise OR, so bit will be 1 if it is 1 in A or B
	// ~(_BV(RXEN0) | _BV(RXCIE0)) is  bit-NOT( (bit-value-of-A OR bit-value-of-B)
	// So, we get something like this:
	
	/*
	_BV(RXCIE0) = 10000000
	_BV(RXEN0)  = 00001000

	_BV(RXCIE0) | _BV(RXEN0) = 10001000

	~( _BV(RXCIE0) | _BV(RXEN0) ) = 01110111	
	*/
	
	// this bit number is sent to register that controls UART-device in chip.
	// -> electricity off from UART receiver and we start ignoring received interrupts
	
	UCSR0B &= ~(_BV(RXEN0) | _BV(RXCIE0));
	

	// Look explanation from previous function, we put RS485 chip into transmit mode, meaning we can WRITE TO BUS.
	PORTA |= _BV(PORTA6); // write enable
	// insert delay, so things settle and nothing weird happens if things are not done yet in real world.
	// 10 microseconds is bit too much but in this application time is not so critical.....
	// Normally, you should measure what the minimum delay has to be.
	_delay_us(10);
}

void rs485_mode_receive(){
	// wait for previous transmit to finish
	// when changing from transmit mode to receive mode, we want to be sure all things has been really transmitted.
	// So, let's wait until data register is saying it is empty:
	// UCSR0A is UART control and status register A,
	// whose bit UDR0 we want to be 1 if we proceed. Otherwise wait.
	// UDRE0 tells that USART has sent all data == USART data register empty (p. 181 ATtiny841)
	while( !( (UCSR0A) & _BV(UDRE0) ) ){
		_delay_us(1);
	}
	

	// wait at least 11bits/38400baud (286us ~ 300us)
	// to be sure that all bits are written to the bus
	_delay_us(300);
	
	// RS485 chip into receive mode
	// we put PORTA, and it's PA6-pin into 0
	// (PORTA6 is the pin we are talking about, but we have to take invert from it, to get 1->0.
	// Then put that bit number to PORTA to create the actual change in pin. 
	PORTA &= ~(_BV(PORTA6)); // read enable
	_delay_us(10);
	
	// enable UART receiving
	// |= means 'bit OR', meaning we take RXEN0 and RXCIE0, and their 1 bit's and add them to control register :)  
	UCSR0B |= _BV(RXEN0) | _BV(RXCIE0);
	
	uart_clear(); // clear buffer
}


void timer0_init(){
	// Clear on compare

	// TCCR0A is timer/counter control register A, (p. 85)
	// WGM01 is clear timer on compare (=CTC....), (p. 87)
	// Timer has two limits: compare A and compare B
	// TCCR0A = _BV(WGM01) makes timer to reset when it counts to compare A limit
	// compare A limit is set later in this function
	// So, here we put timer into state where it can now reset itself, but NO RESETING YET

	TCCR0A = _BV(WGM00);
	
	// prescaler to clock/64: 8MHz clock -> 125kHz
	// Processor's clock frequency is divided by 64 because p.88........
	// TCCR0B is timer counter control register B, controls counting rate of the timer
	// meaning with this time can be slowed down or speeded up, normal speed would be just the clock frequency of processor 
	// This '_BV(CS01) | _BV(CS00)' combination means dividing with 64 (p.88)
	// Time is slowed down so we can measure long enough time periods required in modbus
	// If time is run normally, 255 clock ticks is a lot smaller time slot than what needs to be measured.
	TCCR0B = _BV(CS01) | _BV(CS00);
	
	// compare A interrupt when 1.5*modbus character time is reached (~450us)
	// 1,5*modbus character period = ~450us
	// 8MHz clock divided by 64 is 125kHz,
	// meaning 8us per our new clock tick
	// 450us = 56,25 clock ticks
	// So, our compare limit A is 60 (rounded up clock ticks)
	// This is the time between modbus messages by standard
	OCR0A = 60;

	// enable compare A interrupt
		// TIMSK0 is timer/counter interrupt mask register
	// OCIE0A is timer/counter0 output compare match A interrupt enable
	// with this we get interrupt when clock ticks to compare A limit
	// With this we allow the system to do interrupt when  compare limit A is reached
	// THIS DOES NOT DO ANYTHIMG, it only sets up the possibility to make interrupts in this way.
	TIMSK0 = _BV(OCIE0A);
}


void timer1_init(){
	// See timer0_init above, this just has different clock rate
	// needed because we define how often temperatures are read
	TCCR1A = 0;
	
	// prescaler to clock/1024: 8MHz clock -> 7812.5Hz
	TCCR1B = _BV(CS11) | _BV(CS01) | _BV(CS00);
}

void timer0_reset(){
	// TCNT0 is Timer counter 0 register
	// we use it to see how many clicks has been ticked.... And here we zero it.
	TCNT0 = 0;
}

void timer1_reset(){
	TCNT1 = 0;
}


void uart_init(){
	
	// initializes UART0 hardware
	// Setting up:

	// 38.4 kbaud (datasheet pg 179 )
	UBRR0H = 0;
	UBRR0L = 12;

	// enable transmitter, receiver and receiver interrupt
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
	
	// data format: 8 bit, 1 stop bit, 	no parity (datasheet pg 183 - 184)
	// UCSR0C USART status control register C, used for dataformat configuration
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	
	// remap UART pins to secondary configuration
	// maps which pins in processor are receive and which are for transmit
	// processor has two options, so setting the other option here.
	// choice depends on physical circuit board and it's design
	// Choice is normally made just how it is easies to wire processor.
	REMAP = _BV(U0MAP);
	
}

void uart_transmit( uint8_t byte ){
	// wait for previous transmit to finish
	while( !( (UCSR0A) & _BV(UDRE0) ) ){}

	// transmit
	// UDR0 is UART data register, used for holding data that will be sent to bus.
	// Transmit to bus starts automatically when this register has anything in it (see manual, p. 180).
	// shift register is a physical device that transmits byte bit by bit to output which is transmit pin on processor
	// from transmit pin bits go to bus.....
	UDR0 = byte;
	
}

// when we receive bytes with processor, we increment g_uart counter, so we know how many bytes we have received in current message
// this  is important because then we can know if whole message is received (in modbus we get the amount of bytes in message) 
uint8_t uart_bytes_received(){
	return g_uart_counter;
}


// start receiving messages from beginning by clearing  byte counter
void uart_clear(){
	g_uart_counter = 0;
}

// get processor configuration from eeprom memory
// So, this is just doing initial setup.

void configure(){
	device_serial_number = eeprom_read_word( (uint16_t *)0 );
	modbus_address = eeprom_read_byte( (uint8_t *)2 );
	
	// if Mursu's address is invalid, put it to 250.
	// otherwise we use the set address in Mursu's processor.
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


// computes cyclical redundancy check for modbus message
// see modbus specification for math....
// So, this is just checking that the message is ok and contains no errors.
// == error detection
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
	
	// returns current value of crc checksum.
	//Later this is compared to the checksum receiver has calculated from the message by themselves. 
	return crc;
}

// sends error message depending on the error.
// messages are specified in modbus specification, see document from modbus.org
void modbus_error_response( uint8_t func, uint8_t error ){
	uint16_t msg_crc = 0xFFFF; // modbus crc initial value
	
	// send mode on:
	rs485_mode_send();
	
	// message format: | address | function code with highest bit on | error code | crc |
	
	// transmit message and update crc
	
	// send address
	uart_transmit( modbus_address );
	msg_crc = compute_crc( modbus_address, msg_crc ); // update crc for sent address
	
	// send function code with highest bit on (to mark that this is an error response)
	uart_transmit( func | 0x80 );
	msg_crc = compute_crc( func | 0x80, msg_crc ); // update crc for sent address AND function code
	
	// send error code
	uart_transmit( error );
	msg_crc = compute_crc( error, msg_crc ); // update crc for sent address AND function code AND error code
	
	// transmit crc in two bytes because it won't fit into one byte :D
	// first sending right most bits: (0xFF is 1111 1111, so taking the first 8 bits)
	uart_transmit( msg_crc & 0xFF );
	// moving bits, and taking the rest of them for sending
	uart_transmit( (msg_crc >> 8) & 0xFF );
	
	// done
	// so, we go into receive mode again
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


// handling modbus read register-message
// when our device get's read_register-command/message from master
// that is in this case a computer/Rasberry pi that is in same bus as Mursu.
// master sends this message when it is sending commands that it wants the slaves to read and execute 
void modbus_read_register( uint16_t start, uint16_t stop ){
	uint16_t msg_crc = 0xFFFF; // modbus crc initial value

	uint16_t addr; // address of current register to be read
	
	// message format: | address | function code | byte count | register contents | crc |
	
	// go into send mode for we want to answer to the read register message	
	rs485_mode_send();
	

	// send Mursu's address to master (so master will know who responds)
	uart_transmit( modbus_address );
	msg_crc = compute_crc( modbus_address, msg_crc );   // crc for sent data...
	

	// send function code, we tell to master that we response to read register message
	uart_transmit( MODBUS_READ_HOLDING_REGISTER );
	msg_crc = compute_crc( MODBUS_READ_HOLDING_REGISTER, msg_crc );
	
	// send byte count ( amount of register * 2 bytes (16bit words) ), so master will know how big message it is getting
	uart_transmit( (stop - start) * 2 );
	msg_crc = compute_crc( (stop - start) * 2, msg_crc );
	
	
	// send register content:
	// start from starting address (place in register array)
	// end with stop address (place in register array)
	// register can hold more stuff as read register message may say that only certain information is needed.
	// register holds holds example sensor values, serial number etc.
	//  Like we may have several temperature sensors in Mursu, and master only wants to read one of them
	// even Mursu may have all data in register (this is defined by our code and how we want this to work)
	for( addr = start; addr < stop; ++addr ){

		// get register content
		
		uint8_t byte;
		
		

		// read data from measurement table
		uint16_t register_content = measurement_table[addr - modbus_start_address]; //0xABBA;
		
		// splitting register value in high and low byte (all stuff goes into two bytes because by modbus spec all number are 16 bits)
		// and send them byte by byte
		
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


	// done, getting into receive mode again
	rs485_mode_receive();
}


// device has got write register command,
// this function first replies to the gotten  write register command ("I have received the write register command!")
// After that it handles the content, like "change address", "change sensor value offset as different sensors had different base level"
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
	
	// Set sensor offsets == if temperature sensor is always giving 2 degree too much, can be "calibrated out"
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
	
	// if one wants to change the serial number:
	if( address == MODBUS_CHANGE_SERIAL_NUMBER){
		device_serial_number = data;
		eeprom_write_word((uint16_t *)0, data);
	}
}



// when we have got a message, then what we do??? :P
void modbus_handle_message(){

	// check address of the message, meaning to who the message is for, skip message if it is not meant for us
	if( g_uart_buffer[MODBUS_ADDRESS_IDX] != modbus_address ){ return ; }
	
	// check function code (we support only "READ HOLDING REGISTER"  and "WRITE SINGLE REGISTER" )
	// TODO: add support for diagnostics too
	


	// Check that the function meantioned in message is something we support in oour Mursu
	// if not, send error message
	if( (g_uart_buffer[MODBUS_FUNC_CODE_IDX] != MODBUS_READ_HOLDING_REGISTER) && (g_uart_buffer[MODBUS_FUNC_CODE_IDX] != MODBUS_WRITE_SINGLE_REGISTER) ){


		modbus_error_response( g_uart_buffer[MODBUS_FUNC_CODE_IDX], MODBUS_ILLEGAL_FUNCTION );
		return;
	}
	
	// if we got read_register-command:
	if(g_uart_buffer[MODBUS_FUNC_CODE_IDX] == MODBUS_READ_HOLDING_REGISTER) {

		// check that message is long enough
		// device address + function code + 2*starting addres  + 2*quantity of registers  + 2*crc = 8
		if( g_uart_counter != 8 ){ return; }
	
		// local scope for handling message
		// for keeping code cleaner, these variables have same names as other parts of code have,
		// and we only want to deal with these in this specific place and not to have anything to do with them in future
		{
			uint16_t starting_address, quantity;
		
			// byte order: high, low
			// starting address is read from buffer, where is both starting address and quantity
			// both have two bytes, so moving the first byte (left most) 8bits to the left
			// this is how we get space for grabbing the other byte
			// so, we get xxxx yyyy with using OR to combine them, where xxxx is first byte in buffer
			starting_address =	(g_uart_buffer[2] << 8) | g_uart_buffer[3];
			quantity =			(g_uart_buffer[4] << 8) | g_uart_buffer[5];
			
		// tests for errors that stop actions:
			
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
	
	// write register-command:
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
		

		
		// measure every 0.5 seconds
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
