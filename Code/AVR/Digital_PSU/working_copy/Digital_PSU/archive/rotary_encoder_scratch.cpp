/*
 * rotary_encoder.cpp
 *
 * Created: 7/22/2019 9:43:29 PM
 *  Author: neutron
 */ 


uint32_t btn_sw_interval=0;
#define BUTTON_DEBOUNCE_DELAY	200 //200 Milli Seconds
#define DISPLAY_UPDATE_PERIOD	50	//30 Milli Seconds
#define MAX_SUPPORTED_CURRENT 2000		//in mA

GPIO blueLed(4);
GPIO redLed(13);
GPIO knob_clk(9);
GPIO knob_dat(7);
GPIO knob_btn(3);//knob_btn(4);

uint8_t state=0,data;

fifo_byte buffer(16);

#define CLOCK		2
#define ANTICLOCK	3


uint16_t iValueSetting=0;
void increase();
void decrease();

uint8_t int1_count=0;
uint16_t knob_click_value=1;		//default click value is 1 per knob click in any direction.

void init_encoder(){
		knob_clk.setInput();	//set knob pins as input
		knob_dat.setInput();
		knob_btn.setInputPullUp();		//set Knob Button as Input Pullup.

		PCMSK0 = (1<<PCINT1) | (1<<PCINT0);	//set PCI on digital Pin 8 & 9, an change on either pins of encoder triggers IRQ.
		PCICR = (1<<PCIE0);					//Enable the PCI0 IRQ
		
		
}


void poll_this_in_loop(){
	
		/////////////////////////////////// KNOB MULTIPLIER CODE //////////////////////////////////////////
		if(!knob_btn.getState() && millis()-btn_sw_interval>BUTTON_DEBOUNCE_DELAY)
		{
			//pot_channel = !pot_channel;		//by switch Press Toggle Between POT Channels.
			int1_count++;
			
			if (int1_count == 4)
			{
				int1_count = 0;
			}
			
			knob_click_value = 1;
			
			for (uint8_t i=0;i<int1_count;i++)
			{
				knob_click_value *= 10;
			}

			btn_sw_interval = millis();
		}
		
		
		


		
		//////////////////////////////////// KNOB DIRECTION DECODER ////////////////////////////////////////////
		
		uint8_t test;
		if(!buffer.isEmpty())
		{
			test = buffer.deque();
			
			if(test == ANTICLOCK)
			{
				
				//pot_decrease(pot_channel);
				//anti();
				decrease();
			}
			else if (test == CLOCK)
			{
				//pot_increase(pot_channel);
				//clockwise();
				increase();
			}
			
		}
		
}

ISR(PCINT0_vect)
{
	
	data = PINB & 0x03;
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
		
		//try adding low pass filter to reduce glitch--done
		state = data;		//added a low pass filter but even that couldn't help much.
		
	}
	
	//I understood that when Inside this ISR the delay() doesn't work properly but it works as opposed what I thought previously
	//that it will get stuck but surprising I saw that it was working but maybe it wasn't working as it was supposed but in a
	//an irregular way, for example the 100mS specified delay might would have taken even longer than 100.
	
	
	//I found it you can read the micros() def which is used by delay() now in that there is a condition which checks if TIFR flag
	//is cleared or up for the current run when TCNT value is less than 255 but since we are inside the ISR the timer_ISR wont
	//execute as a result the flag wont be cleared as a result the variable 'm' is always increment on each call to micros()
	//as a result the delay() ends up sooner that it should be, that is why when I used 100millisecond delay using _delay_ms()
	//it doesn't work that well.
	
	//May be I have not fully understood that yet, it seems that the 'm' value is always reloaded from timer_0_overflow_count
	//which will not increment if timer0ISR doesn't run. So the only way I think to find out the cause of early exit of delay
	//instead of getting stuck is by doing a debug. (For debug we have to write a minimal code, use Timer2 ISR and use delay with
	//in that). We cannot use any communication polling stuff like I2C or SPI, absolutely minimal code.
	
	//another thing to note is that since now program complexity has slightly grown which eventually slows down thing even without
	//any delay here it works fairly well and stable.
	
	//_delay_ms(5);
	//delay(100);				//now the delay works here. but doesn't improve much still can try the low pass, but you can disable this delay and slowly move the knob to see the bounce effect.
	//you can also experiment by choosing only one pin as INT source instead of both.--done--didn't help.
	//100ms is the most optimal delay, below or above this can make the system unstable.
	
	//even though when moving it very fast it slips, and sometimes shows a few anti-clocks, in real-life implementation
	//we will not count how many anti-clocks and clocks it turned, but if it turned really fast in one direction
	//and go a few glitches of the opposite direction, take it this way like while increasing from 0 to 100 it will
	//increase from 0 to 97 instead because 100 fast turns  + a few glitches in opposite direction
	//but we won't be able to notice it. So it won't matter at all at that speed.
}

void increase()
{
	if(iValueSetting < MAX_SUPPORTED_CURRENT-knob_click_value)
	{
		iValueSetting+=knob_click_value;
	}
	
}

void decrease()
{
	if (iValueSetting > 0+knob_click_value)
	{
		iValueSetting-=knob_click_value;
	}
	
}


