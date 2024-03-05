/* 
* PowerModule.h
*
* Created: 8/10/2019 3:52:27 PM
* Author: neutron
*/


#ifndef __POWERMODULE_H__
#define __POWERMODULE_H__
#include "project_config.h"
#include "Arduino.h"
#include "i2c.h"

/*
class PowerModule
{
//variables
public:
protected:
private:

//functions
public:
	PowerModule();
	~PowerModule();
protected:
private:
	PowerModule( const PowerModule &c );
	PowerModule& operator=( const PowerModule &c );

};*/ //PowerModule

uint8_t setVoltage(uint16_t voltage_mV);
#define  VOLTAGE_TRIMMER_DPOT_TPL0102_100_I2C_ADDR	0x50
#define	 VOLTAGE_TRIMMER_DAC_MCP4725_I2C_ADDR		0x60
#define  VOLTAGE_SET_RANGE_MIN_mV					1300
#define  VOLTAGE_SET_RANGE_MAX_mV					18000
#define	 VOLTAGE_SET_POINT_STEP_SIZE_mV				100
#define  VOLTAGE_SET_POINT_INIT_VALUE_mV			3300

uint8_t setCurrentLimit(uint16_t current_mA);
#define CURRENT_LIMIT_DPOT_TPL0102_100_I2C_ADDR		0x51
#define CURRENT_LIMIT_RANGE_MIN_mA					10
#define CURRENT_LIMIT_RANGE_MAX_mA					5000	//theoretical is 12.5A or 12500mA but for testing I have now set it to 5A, since we will need to rework the circuit to achieve the full range.
#define CURRENT_LIMIT_STEP_SIZE_mA					10		//the Input Offset is around 5mA to 30mA range for INA250A3 so we are limited by that in here.
#define CURRENT_LIMIT_SET_POINT_INIT_VALUE_mA		1000



#endif //__POWERMODULE_H__
