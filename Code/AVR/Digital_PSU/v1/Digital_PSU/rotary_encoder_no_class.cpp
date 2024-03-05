/*
 * rotary_encoder_no_class.cpp
 *
 * Created: 7/25/2019 1:03:11 AM
 *  Author: neutron
 */ 


#include "project_config.h"
#include "Arduino.h"
#include "rotary_encoder_no_class.h"

uint16_t encoder_val;

volatile uint8_t state,data,val;

fifo_byte buffer(16); //I tested by reducing this upto 1 as well and it worked well, maybe unless we have some issues with polling latency, it won't be necessary
						//even that can be probably omitted if we keep a callback function which will take care of the clockwise and anticlockwise actions and 
						//that will polled in the main(){...while(1){..<polling code here>.}} and all other stuff in loop()

uint16_t knob_click_value;		//default click value is 1 per knob click in any direction.

void init_encoder(){
/*
	knob_clk.setInput();	//set knob pins as input
	knob_dat.setInput();
	knob_btn.setInputPullUp();		//set Knob Button as Input Pullup.

	PCMSK0 = (1<<PCINT1) | (1<<PCINT0);	//set PCI on digital Pin 8 & 9, an change on either pins of encoder triggers IRQ.
	PCICR = (1<<PCIE0);					//Enable the PCI0 IRQ
*/

	//buffer(16);
	encoder_val=ENCODER_INIT_VALUE;
	state=0;
	knob_click_value=ENCODER_STEP_SIZE;
	
	//set input pull-up
	ENCODER_PORT_DIR_REG &= ~((1<<ENCODER_DATA_PIN) | (1<<ENCODER_CLK_PIN));
	ENCODER_PORT_WRITE_REG |= ((1<<ENCODER_DATA_PIN) | (1<<ENCODER_CLK_PIN));
	
	ENCODER_PIN_CHANGE_INT_MASK_REG |= ((1<<ENCODER_DATA_PIN) | (1<<ENCODER_CLK_PIN));
	PCICR |= (1<<ENCODER_PIN_CHANGE_INT_NUMBER); 
	
	
}

ISR(ENCODER_ISR_VECTOR)
{
	val = ENCODER_PORT_READ_REG;
	cli();	
	//using a demanding and memory intensive statement like Serial.print in ISR is dangerous as it can lead to buffer overrun
	//Serial.println(F_STR("TRIG"));
	
	data = 0;
		
	if(val & (1<<ENCODER_CLK_PIN))
		data |= (1<<0);
	if(val & (1<<ENCODER_DATA_PIN))
		data |= (1<<1);
	
	if(state != data)
	{
		if(data == 0)
		{
			if(state == 1)
			{
				buffer.enque(ANTICLOCK);
			}
		}
		else if (data == 1)
		{
			if (state == 0)
			{
				buffer.enque(CLOCK);
			}
			
		}
		else if (data == 3)
		{
			if (state == 2)
			{
				buffer.enque(ANTICLOCK);
			}
			
		}
		else if (data == 2)
		{
			if (state == 3)
			{
				buffer.enque(CLOCK);
			}
			
		}
		
		state = data;
		
	}
	
	_delay_us(ISR_STABILIZATION_DELAY_us);
	_delay_ms(ISR_STABILIZATION_DELAY_ms);
	
	sei();
	
}

void increase()
{
	if(encoder_val <= ENCODER_COUNT_MAX-knob_click_value)
	{
		encoder_val+=knob_click_value;
	}
	
}

void decrease()
{
	if (encoder_val >= ENCODER_COUNT_MIN+knob_click_value)
	{
		encoder_val-=knob_click_value;
	}
	
}

