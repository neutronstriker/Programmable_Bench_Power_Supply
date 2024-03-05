/* 
* Rotary_Encoder.cpp
*
* Created: 7/23/2019 12:51:23 AM
* Author: neutron
*/


#include "Rotary_Encoder.h"

encProps* encPropsPerPCINT[PIN_CHANGE_INTERRUPT_COUNT_ON_ATMEGA328P];

volatile uint8_t PCINT_PORT_VAL[PIN_CHANGE_INTERRUPT_COUNT_ON_ATMEGA328P];

uint8_t Rotary_Encoder::encoderDir()
{
	if (!this->_properties.buffer->isEmpty())
		return this->_properties.buffer->deque();
	else return 0;
}

void Rotary_Encoder::disable()
{
	PCICR &= ~(1<<this->_properties.pcint_num); //this will disable the interrupt, effectively disable the encoder
}


void Rotary_Encoder::enable()
{
	PCICR |= (1<<this->_properties.pcint_num);	//this will enable the interrupt
}


void Rotary_Encoder::increase()
{
	if(++(this->increase_step_count) == this->steps_divisor)
	{
		if(this->encoder_val <= this->max_value-step_size)
		{
			this->encoder_val+=this->step_size;
		}
		this->increase_step_count = 0;
	}
}

void Rotary_Encoder::decrease()
{
	if(++(this->decrease_step_count) == this->steps_divisor)
	{
		if (this->encoder_val >= this->min_value+this->step_size)
		{
			this->encoder_val-=this->step_size;
		}
		this->decrease_step_count = 0;
	}
}

void Rotary_Encoder::configure_attributes(uint16_t encoder_init_val/*=0*/, uint16_t max_value/*=100*/, uint16_t min_value/*=0*/, uint16_t step_size/*=1*/, uint8_t steps_divisor/*=1*/)
{
	this->encoder_val = encoder_init_val;
	this->max_value = max_value;
	this->min_value = min_value;
	this->step_size = step_size;
	this->steps_divisor = steps_divisor;
	
	this->increase_step_count = 0;
	this->decrease_step_count = 0;
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
	this->_properties.buffer = new fifo_byte(ENCODER_DATA_BUFFER); //we can't initialize objects directly when using in another class, so we have to initialize by reference
	
	switch(pc_int_port){
		
		case PC_INT_PORTB:
		//Port direction and control setup
		DDRB &= ~((1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin)); //set pin direction input
		PORTB |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin); //set input pull-up
		this->_properties.pcint_num = PC_INT_0;		//save the PORT read register address
		
		//PC interrupt setup
		//PCMSK_ADDRESS = &PCMSK0; //read comment below in destructor to understand if you need to uncomment this
		PCMSK0 |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin);	//set PCI on digital Pins, an change on either pins of encoder triggers IRQ.
		PCICR |= (1<<PCIE0);					//Enable the PCI0 IRQ
		encPropsPerPCINT[PC_INT_0] = &(this->_properties);
		break;
		
		case PC_INT_PORTC:
		//Port direction and control setup
		DDRC &= ~((1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin)); //set pin direction input
		PORTC |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin); //set input pull-up
		this->_properties.pcint_num = PC_INT_1;		//save the PORT read register address
		
		//PC interrupt setup
		//PCMSK_ADDRESS = &PCMSK0; //read comment below in destructor to understand if you need to uncomment this
		PCMSK1 |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin);	//set PCI on digital Pins, an change on either pins of encoder triggers IRQ.
		PCICR |= (1<<PCIE1);					//Enable the PCI1 IRQ
		encPropsPerPCINT[PC_INT_1] = &(this->_properties);
		break;
		
		case PC_INT_PORTD:
		//Port direction and control setup
		DDRD &= ~((1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin)); //set pin direction input
		PORTD |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin); //set input pull-up
		this->_properties.pcint_num = PC_INT_2;		//save the PORT read register address
		
		//PC interrupt setup
		//PCMSK_ADDRESS = &PCMSK0; //read comment below in destructor to understand if you need to uncomment this
		PCMSK2 |= (1<<this->_properties._clk_pin) | (1<<this->_properties._dat_pin);	//set PCI on digital Pins, an change on either pins of encoder triggers IRQ.
		PCICR |= (1<<PCIE2);					//Enable the PCI2 IRQ
		encPropsPerPCINT[PC_INT_2] = &(this->_properties);
		break;
		
		default:
			Serial.println(F_STR("Rotary_Encoder()::config->Unknown PCINT_PORT Argument"));	
			return;
	}
	
	sei(); //set Global interrupt flag
}

volatile void isrDetectEncRotationCode(encProps * var)
{
	volatile uint8_t data = 0; //val=0;
	
	//val = PCINT_PORT_VAL[var->pcint_num]; 
	
	if (PCINT_PORT_VAL[var->pcint_num] & (1<<var->_clk_pin))
	data |= (1<<0);
	if (PCINT_PORT_VAL[var->pcint_num] & (1<<var->_dat_pin))
	data |= (1<<1);

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

		var->state = data;
		
	}
}

ISR(PCINT0_vect) {
	PCINT_PORT_VAL[PC_INT_0] = PINB;
	if (encPropsPerPCINT[PC_INT_0])
		isrDetectEncRotationCode(encPropsPerPCINT[PC_INT_0]);
}

ISR(PCINT1_vect) {
	PCINT_PORT_VAL[PC_INT_1] = PINC;
	if (encPropsPerPCINT[PC_INT_1])
		isrDetectEncRotationCode(encPropsPerPCINT[PC_INT_1]);
}

ISR(PCINT2_vect) {
	PCINT_PORT_VAL[PC_INT_2] = PIND;
	if (encPropsPerPCINT[PC_INT_2])
		isrDetectEncRotationCode(encPropsPerPCINT[PC_INT_2]);
}