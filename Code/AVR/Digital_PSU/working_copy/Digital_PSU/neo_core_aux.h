/* 
* neo_core_aux.h
*
* Description:	This lib contains helper functions most frequently used in some of my projects and is part of neo_core_project, these will mostly small functions or wrappers which are not designated as
*				libraries in their own right.
*
* Created: 8/12/2019 8:58:51 PM
* Author: neutron
*/


#ifndef __NEO_CORE_AUX_H__
#define __NEO_CORE_AUX_H__

#include "Arduino.h"
#include "GPIO.h"
#include <util/delay.h>

/*
class neo_core_aux
{
//variables
public:
protected:
private:

//functions
public:
	neo_core_aux();
	~neo_core_aux();
protected:
private:
	neo_core_aux( const neo_core_aux &c );
	neo_core_aux& operator=( const neo_core_aux &c );

}; //neo_core_aux
*/

//COMMS
void i2c_internal_pullup(bool state);


//WDT and Reset control
void resetSysUsingWDT();
void watchDog_off();


//time and delay
void reset_millis();
uint8_t seconds();
uint16_t minutes();

#endif //__NEO_CORE_AUX_H__
