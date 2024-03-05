/* 
* Rotary_Encoder.h
*
* Created: 7/23/2019 12:51:23 AM
* Author: neutron
*/

//#include <avr/io.h>
//#if defined (__AVR_ATmega328p__)

//we can try using anti-clock and clock wise action call back functions so that it will be called automatically, once we have an, but may be that will overload
//the ISR time, 

#ifndef __ROTARY_ENCODER_H__
#define __ROTARY_ENCODER_H__

#include "project_config.h"
#include "Arduino.h"
#include "fifo.h"


#define ENCODER_DATA_BUFFER 4	//tested and found that it works fine with even 1, but kept 4 so that because of some delays by other sub-routines
								//if polling is not possible at real-time then we can preserve a few clicks

#define PIN_CHANGE_INTERRUPT_COUNT_ON_ATMEGA328P 3 //PCINT0, PCINT1 and PCINT2

#define BUFFER_EMPTY	0
#define CLOCK			2
#define ANTICLOCK		3

#define PC_INT_0 0
#define PC_INT_1 1
#define PC_INT_2 2

#define PC_INT_PORTB 0
#define PC_INT_PORTC 1
#define PC_INT_PORTD 2

struct encProps{
	uint8_t pcint_num;
	uint8_t _clk_pin,_dat_pin;
	
	fifo_byte *buffer;	//declared here as pointer, because it will otherwise be declared as an object and it will try to instantiate an object, below in "encProps _properties;"
						//in C/CPP you cannot instantiate an object or initialize a variable in a header file. The instantiation is done is in the constructor definition.
	uint8_t state;	
	};

volatile void isrDetectEncRotationCode(encProps * var);

class Rotary_Encoder
{
//variables
public:
uint16_t encoder_val, max_value, min_value, step_size;
uint8_t steps_divisor, increase_step_count, decrease_step_count;

protected:

private:
encProps _properties;


//functions
public:
	Rotary_Encoder();
	Rotary_Encoder(uint8_t  pc_int_port, uint8_t clk, uint8_t dat);	//example: PORTD, clk = 4 and data = 2, these pin numbers are not Arduino pin numbers they are PORT specific.
	~Rotary_Encoder();
	void config(uint8_t  pc_int_port, uint8_t clk, uint8_t dat); //since we can't initialize in global scope outside main, this can be used in that case
	uint8_t encoderDir();
	
	void disable();
	void enable();
	
	void increase();
	void decrease();
	void configure_attributes(uint16_t encoder_init_val=0, uint16_t max_value=100, uint16_t min_value=0, uint16_t step_size=1, uint8_t steps_divisor=1);

protected:
private:
	

}; //Rotary_Encoder



#endif //__ROTARY_ENCODER_H__

//#endif //__AVR_ATmega328p__