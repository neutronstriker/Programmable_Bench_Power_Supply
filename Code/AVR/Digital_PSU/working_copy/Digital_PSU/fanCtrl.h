/* 
* fanCtrl.h
*
* Created: 8/30/2019 12:00:16 AM
* Author: neutron
*/


#ifndef __FANCTRL_H__
#define __FANCTRL_H__

#include "project_config.h"
#include "Arduino.h"

/*

class fanCtrl
{
//variables
public:
protected:
private:

//functions
public:
	fanCtrl();
	~fanCtrl();
protected:
private:
	fanCtrl( const fanCtrl &c );
	fanCtrl& operator=( const fanCtrl &c );

}; //fanCtrl*/



#define FAN_PWM_GPIO_PIN		10

#define FAN_SPEED_MAX			150
#define FAN_SPEED_MIN			1
#define FAN_SPEED_OFF			0

#define FAN_ON_OFF_TEMP_HYST_C		5
#define FAN_ON_THRESHOLD_TEMP_C		40	//we are using two different thresholds to have hysteresis control
#define FAN_OFF_THRESHOLD_TEMP_C	(FAN_ON_THRESHOLD_TEMP_C - FAN_ON_OFF_TEMP_HYST_C)
#define TEMPERATURE_MAX_LIMIT_C		80

#define OVER_TEMPERATURE_LIMIT_C	TEMPERATURE_MAX_LIMIT_C		//40 was used for testing
#define OVER_TEMPERATURE_HYST_C		FAN_ON_OFF_TEMP_HYST_C		//5 was used for testing

#define FAN_CTRL_TEMP_SENSITIVITY_C	2

#define THERMISTOR_ADC_CHANNEL				A6
#define ADC_REF								DEFAULT
#define ADC_LSB_mV							5000.0f/1024UL
#define THERMISTOR_INPUT_VOLTAGE_mV			7430.0f

#define NTC_DIVIDER_BOTTOM_RESISTOR_OHMS	2200UL	//divider bottom resistor value
#define NTC_THERMISTOR_R0_OHMS				10000UL	//NTC temperature at 25C
#define NTC_THERMISTOR_T0_KELVIN			298UL		//273.15+25 base temperature for which the NTC resistance is mentioned in data-sheet.
#define NTC_THERMISTOR_BETA					3950.0f	//A common BETA number for 103 written black disc NTC thermistors, since I don't have data-sheet


extern uint8_t fanspeed_val, temperature, fanspeed_last_val, fanOverride, overTempFlag;
extern float temperaturePrecise;

float ntcThermistorTemp();
uint8_t fanSpeed(uint8_t speedVal);
void fanControlSetup();
void temperatureControlLoop();
uint8_t setPwmFrequency(int pin, int divisor);

#endif //__FANCTRL_H__
