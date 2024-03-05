/* 
* Rotary_Encoder.h
*
* Created: 7/23/2019 12:51:23 AM
* Author: neutron
*/

//#include <avr/io.h>
//#if defined (__AVR_ATmega328p__)

#ifndef __ROTARY_ENCODER_H__
#define __ROTARY_ENCODER_H__

#include "project_config.h"
#include "Arduino.h"
#include "fifo.h"

#include "GPIO.h"

#define PIN_CHANGE_INTERRUPT_COUNT_ON_ATMEGA328P 3 //PCINT0, PCINT1 and PCINT2

#define CLOCK		2
#define ANTICLOCK	3

#define PC_INT_0 0
#define PC_INT_1 1
#define PC_INT_2 2

#define PC_INT_PORTB 0
#define PC_INT_PORTC 1
#define PC_INT_PORTD 2

struct encProps{
	volatile uint8_t * port_pin_read_reg_ptr;
	uint8_t _clk_pin,_dat_pin;
	fifo_byte *buffer; //declared here as pointer, because it will otherwise be declared as an object and it will try to instantiate an object, below in "encProps _properties;"
	volatile uint8_t state;	//in C/CPP you can instantiate an object or initialize a variable in a header file. The instantiation is done is in the constructor definition.
	};



volatile void isrDetectEncRotationCode(encProps * var);

class Rotary_Encoder
{
//variables
public:
encProps _properties;

protected:

private:

//volatile uint8_t * PCMSK_ADDRESS;
/*
volatile uint8_t * port_pin_read_reg_ptr;
uint8_t _clk_pin,_dat_pin;
fifo_byte buffer;
volatile uint8_t state;*/



//functions
public:
	Rotary_Encoder();
	Rotary_Encoder(uint8_t  pc_int_port, uint8_t clk, uint8_t dat);	//example: PORTD, clk = 4 and data = 2, these pin numbers are not Arduino pin numbers they are PORT specific.
	~Rotary_Encoder();
	void config(uint8_t  pc_int_port, uint8_t clk, uint8_t dat); //since we can't initialize in global scope outside main, this can be used in that case
protected:
private:
	uint8_t encoderDir();

}; //Rotary_Encoder



#endif //__ROTARY_ENCODER_H__

//#endif //__AVR_ATmega328p__