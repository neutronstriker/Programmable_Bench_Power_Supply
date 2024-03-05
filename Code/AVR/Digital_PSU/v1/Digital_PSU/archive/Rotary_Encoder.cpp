/* 
* Rotary_Encoder.cpp
*
* Created: 7/23/2019 12:51:23 AM
* Author: neutron
*/


#include "Rotary_Encoder.h"

encProps *encPropsPerPCINT[PIN_CHANGE_INTERRUPT_COUNT_ON_ATMEGA328P];

uint8_t Rotary_Encoder::encoderDir()
{
	if (!this->_properties.buffer->isEmpty())
		return this->_properties.buffer->deque();
	else return 0;
}

// default constructor
 Rotary_Encoder::Rotary_Encoder(uint8_t pc_int_port, uint8_t clk, uint8_t dat)
{
	this->config(pc_int_port,  clk,  dat);
}

 Rotary_Encoder::Rotary_Encoder()
{

}

// default destructor
Rotary_Encoder::~Rotary_Encoder()
{
	//*PCMSK_ADDRESS &= ~((1<<_clk_pin) | (1<<_dat_pin)); //if you want the interrupts to be disabled when the object goes out of scope then
														//we will need this and then we will have to make a change to the constructor to save the
														//PCMSK_ADDRESS otherwise we can save a few bytes of RAM
									
} //~Rotary_Encoder


 void Rotary_Encoder::config(uint8_t pc_int_port, uint8_t clk, uint8_t dat)
{
	//initialize the vars
	this->_properties._clk_pin = clk;
	this->_properties._dat_pin = dat;
	this->_properties.state=0;
	this->_properties.buffer = new fifo_byte(16); //we can't initialize objects directly when using in another class, so we have to initialize by reference
	
	switch(pc_int_port){
		case PC_INT_PORTB:
		//Port direction and control setup
		DDRB &= ~((1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin)); //set pin direction input
		PORTB |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin); //set input pull-up
		this->_properties.port_pin_read_reg_ptr = &PINB;		//save the PORT read register address
		
		//PC interrupt setup
		//PCMSK_ADDRESS = &PCMSK0; //read comment below in destructor to understand if you need to uncomment this
		PCMSK0 |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin);	//set PCI on digital Pins, an change on either pins of encoder triggers IRQ.
		PCICR |= (1<<PCIE0);					//Enable the PCI0 IRQ
		encPropsPerPCINT[PC_INT_0] = &this->_properties;
		break;
		case PC_INT_PORTC:
		//Port direction and control setup
		DDRC &= ~((1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin)); //set pin direction input
		PORTC |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin); //set input pull-up
		this->_properties.port_pin_read_reg_ptr = &PINC;		//save the PORT read register address
		
		//PC interrupt setup
		//PCMSK_ADDRESS = &PCMSK0; //read comment below in destructor to understand if you need to uncomment this
		PCMSK1 |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin);	//set PCI on digital Pins, an change on either pins of encoder triggers IRQ.
		PCICR |= (1<<PCIE1);					//Enable the PCI1 IRQ
		encPropsPerPCINT[PC_INT_1] = &this->_properties;
		break;
		case PC_INT_PORTD:
		//Port direction and control setup
		DDRD &= ~((1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin)); //set pin direction input
		PORTD |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin); //set input pull-up
		this->_properties.port_pin_read_reg_ptr = &PINC;		//save the PORT read register address
		
		//PC interrupt setup
		//PCMSK_ADDRESS = &PCMSK0; //read comment below in destructor to understand if you need to uncomment this
		PCMSK2 |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin);	//set PCI on digital Pins, an change on either pins of encoder triggers IRQ.
		PCICR |= (1<<PCIE2);					//Enable the PCI2 IRQ
		encPropsPerPCINT[PC_INT_2] = &this->_properties;
		break;
		default:	return;
	}
	sei(); //set Global interrupt flag
}

volatile void isrDetectEncRotationCode(encProps * var)
{
	volatile uint8_t data = 0, val=0;
	
	//check and update this if direction detected doesn't match the CLOCK and ANTICLOCK labels
	val = *var->port_pin_read_reg_ptr; //the register should be read in single cycle otherwise we will loose the state
	
	if (val & (1<<var->_clk_pin))
	data |= (1<<0);
	if (val & (1<<var->_dat_pin))
	data |= (1<<1);
		
/*
	#ifdef PRINT_DEBUG_MSG
	Serial.print(F_STR("ENTER ISR CODE\r\n"));
	#endif	*/ 
	
	
/*
			
	#ifdef PRINT_DEBUG_MSG
	Serial.print(F_STR("RAW_VAL="));
	Serial.println(val,HEX);
	Serial.print(F_STR("DATA="));
	Serial.println(data,HEX);
	Serial.print(F_STR("STATE="));
	Serial.println(var->state);
	
		Serial.print(F_STR("CLK_PIN="));
		Serial.println(var->_clk_pin);
		Serial.print(F_STR("DAT_PIN="));
		Serial.println(var->_dat_pin);
	#endif*/
	
	if(var->state != data)
	{
		if(data == 0)
		{
			if(var->state == 1)
			{
				var->buffer->enque(ANTICLOCK);
			}
		}
		else if (data == 1)
		{
			if (var->state == 0)
			{
				var->buffer->enque(CLOCK);
			}
			
		}
		else if (data == 3)
		{
			if (var->state == 2)
			{
				var->buffer->enque(ANTICLOCK);
			}
			
		}
		else if (data == 2)
		{
			if (var->state == 3)
			{
				var->buffer->enque(CLOCK);
			}
			
		}
		
		//tried adding low pass filter to reduce glitch
		var->state = data;		//but even that couldn't help much in reducing glitch
		
/*
		#ifdef PRINT_DEBUG_MSG
		Serial.print(F_STR("DATA="));
		Serial.println(data);
		#endif*/
		
	}
}

SIGNAL(PCINT0_vect) {
	if (encPropsPerPCINT[PC_INT_0])	//if the struct pointer is not initialized it will be void by default
		isrDetectEncRotationCode(encPropsPerPCINT[PC_INT_0]);
		
/*
		#ifdef PRINT_DEBUG_MSG
		Serial.print(F_STR("PCINT0\r\n"));
		#endif*/
}

SIGNAL(PCINT1_vect) {
	if (encPropsPerPCINT[PC_INT_1])
		isrDetectEncRotationCode(encPropsPerPCINT[PC_INT_1]);
		
/*
		#ifdef PRINT_DEBUG_MSG
		Serial.print(F_STR("PCINT1\r\n"));
		#endif*/
}

SIGNAL(PCINT2_vect) {
	volatile uint8_t data = PINB;
	if (encPropsPerPCINT[PC_INT_2])
		isrDetectEncRotationCode(encPropsPerPCINT[PC_INT_2]);
		
			#ifdef PRINT_DEBUG_MSG
			Serial.print(F_STR("RAW_VAL_IN_ISR="));
			Serial.println(data,HEX);
			#endif
		
/*
		#ifdef PRINT_DEBUG_MSG
		Serial.print(F_STR("PCINT2\r\n"));
		#endif*/
}