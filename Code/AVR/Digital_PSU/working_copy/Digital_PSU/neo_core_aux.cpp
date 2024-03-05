/*
* neo_core_aux.cpp
*
* Description:	This lib contains helper functions most frequently used in some of my projects and is part of neo_core_project, these will mostly small functions or wrappers which are not designated as
*				libraries in their own right.
*
* Created: 8/12/2019 8:58:51 PM
* Author: neutron
*/


#include "neo_core_aux.h"

/*
// default constructor
neo_core_aux::neo_core_aux()
{
} //neo_core_aux

// default destructor
neo_core_aux::~neo_core_aux()
{
} //~neo_core_aux
*/

uint8_t	last_rst_was_by_wdt=0;		//this flag tells you that if last rst was by WDT, this is controlled in watchDog_off()

void reset_millis()
{
	
	extern volatile unsigned long timer0_overflow_count;
	extern volatile unsigned long timer0_millis;
	timer0_overflow_count=0;
	timer0_millis=0;
	
}

uint16_t minutes()
{
	return (millis()/1000)/60; //check this that division operation can seriously decrease performance
	//because there is no hardware division algorithm
	//try to take minutes and seconds as global values and run only seconds that will update
	//the minutes value also once in every 60 seconds instead of once on every call.
}

uint8_t seconds()
{
	return (millis()/1000)%60; //check this that modulo operation can seriously decrease performance
}                               //find an optimized way try a mix of >> and subtraction instead


void watchDog_off()
{
	/* make sure this function is called before global interrupt (Sei) is enabled
	or first disable it and then call it. */
	cli();
	asm volatile("wdr"::);				//reset watchdog
	if(MCUSR & (1<<WDRF))				//check write this to flag before clearing WDRF flag, so we can check the state by using this.
		last_rst_was_by_wdt = 1;
	MCUSR &= ~(1<<WDRF);				//clear WDRF flag, not sure why we need to do this.
	WDTCSR |= (1<<WDCE) | (1<<WDE);		//following WDTCSR change sequence
	WDTCSR = (1<<WDIF);					//clear the Interrupt flag if it exists because of previous WDT reset,
										//WDE cleared, i.e. WDT is disabled, WDIE is also disabled i.e. No interrupt on WDT expire, 
										//clock timeout period selected to 16ms (minimum Period).
										
	sei();
}

void resetSysUsingWDT()
{
	cli();								//clear global interrupt first.
	WDTCSR |= (1<<WDCE) | (1<<WDE);		//following WDTCSR change sequence
	WDTCSR = (1<<WDIF) | (1<<WDE);		//clear the Interrupt flag if it exists because of previous WDT reset,
	//WDE set, i.e. WDT is disabled, WDIE is also disabled i.e. No interrupt on WDT expire,
	//clock timeout period selected to 16ms (minimum Period).
	_delay_ms(50);						//I am not using delay() arduino function because we have disabled interrupt delay will
	//not work properly.
}


void i2c_internal_pullup(bool state)
{
	GPIO sda(A4);
	GPIO scl(A5);
	
	if(state == true)
	{
		sda.setInputPullUp();
		scl.setInputPullUp();
	}
	else
	{
		sda.setInputPullDown();
		sda.setInputPullDown();
	}
}