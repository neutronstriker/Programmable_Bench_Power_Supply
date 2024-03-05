/* 
* fanCtrl.cpp
*
* Created: 8/30/2019 12:00:15 AM
* Author: neutron
*/


#include "fanCtrl.h"

/*
// default constructor
fanCtrl::fanCtrl()
{
} //fanCtrl

// default destructor
fanCtrl::~fanCtrl()
{
} //~fanCtrl
*/
uint8_t overTempFlag = 0;		//overT flag will be used by various other process
uint8_t fanOverride=0;
uint8_t fanspeed_val=0, temperature=0, fanspeed_last_val=0,temperature_last_val=0;
float temperaturePrecise = 0;

float ntcThermistorTemp(){
	analogReference(DEFAULT);
	
	uint16_t ntcDividerVoltageNow_mV = ADC_LSB_mV*analogRead(THERMISTOR_ADC_CHANNEL);
	
	#ifdef PRINT_DEBUG_MSG
	Serial.print(F_STR("Divider Voltage (mV): ")); Serial.println(ntcDividerVoltageNow_mV);
	#endif
	
	uint16_t ntcResistanceNow_ohms = ((THERMISTOR_INPUT_VOLTAGE_mV*NTC_DIVIDER_BOTTOM_RESISTOR_OHMS)/ntcDividerVoltageNow_mV)-NTC_DIVIDER_BOTTOM_RESISTOR_OHMS;
	
	#ifdef PRINT_DEBUG_MSG
	Serial.print(F_STR("NTC Resistance (ohms): ")); Serial.println(ntcResistanceNow_ohms);
	#endif
	//float ntcTemperature_Celcius = (1.0f / ((log(ntcResistanceNow_ohms)/NTC_THERMISTOR_BETA) - (log(NTC_THERMISTOR_R0_OHMS)/NTC_THERMISTOR_BETA) + (1/NTC_THERMISTOR_T0_KELVIN))) - 273;		//subtracting 273 to get value in degree celcius
	
	float ntcTemperature_Celcius = (1000.0f/((log(ntcResistanceNow_ohms)/ 3.95f)  + 1.024f ))-273; //using simplified formula by resolving constants, as math overflow was occuring in atmega328p
	
	#ifdef PRINT_DEBUG_MSG
	Serial.print(F_STR("Temperature (C): ")); Serial.println(ntcTemperature_Celcius);
	#endif
	
	return ntcTemperature_Celcius;
}



uint8_t fanSpeed(uint8_t speedVal){
	if (speedVal > FAN_SPEED_MAX)//|| speedVal < FAN_SPEED_MIN) // unsigned argument, not required to check "<0" condition
	{
		#ifdef PRINT_DEBUG_MSG
		Serial.println(F_STR("Fan Speed requested out of range!"));
		#endif
		return 0;
	}
	if (speedVal == 0){
		analogWrite(FAN_PWM_GPIO_PIN,255);		//drive to 1, completely turn-off fan
	}
	else{
		analogWrite(FAN_PWM_GPIO_PIN,(FAN_SPEED_MAX-speedVal));	//at PWM-0 fan speed is max, and PWM-150 it is lowest so doing a simple inversion.	
	}
	return 1;
}

void fanControlSetup(){
	digitalWrite(FAN_PWM_GPIO_PIN,LOW);
	pinMode(FAN_PWM_GPIO_PIN,OUTPUT);
	setPwmFrequency(FAN_PWM_GPIO_PIN,1);
	fanSpeed(FAN_SPEED_MAX);
	fanspeed_last_val = FAN_SPEED_MAX;	//this is required so that on bootup fanControlLoop() can compare both values and work properly.
}

void temperatureControlLoop(){
	temperaturePrecise = ntcThermistorTemp(); //get the temperature //if needed we can use the temperaturePrecise Var where necessary.
	temperature = temperaturePrecise;		//and use the unsigned int version where required
	
	/*
	uint16_t x = 0;
	for (uint8_t i = 0 ; i<10;i++)
	{
		x=x+ntcThermistorTemp();	
	}
	temperature = x/10;
	*/
	
	if (fanOverride==0)	//see if override is enabled
	{
		int temp_delta = temperature-temperature_last_val;
		if (temp_delta < 0) {	//get mod of delta
			temp_delta = temp_delta*-1;
		}
		//fan speed linear control with hysteresis of 5C so that we don't have fan running into hiccup mode i.e. continuously switching between ON and OFF states
		if (temperature >= FAN_ON_THRESHOLD_TEMP_C && temp_delta >= FAN_CTRL_TEMP_SENSITIVITY_C) //another hysteresis so that fan speed switching noise can be reduced
		{
			fanspeed_val = map(temperature, FAN_ON_THRESHOLD_TEMP_C,TEMPERATURE_MAX_LIMIT_C,FAN_SPEED_MIN,FAN_SPEED_MAX); //map linearly between temperature and FAN minimum speed
			//to max, do not use 0 as minimum speed, otherwise there won't be any benefit of using hysteresis control
			temperature_last_val = temperature;
		}
		else if (temperature <= FAN_OFF_THRESHOLD_TEMP_C)
		{
			fanspeed_val = FAN_SPEED_OFF;
			temperature_last_val = temperature;
		}
	
		
		
	}
	
	//now in Serial_cmd_repl fanspeed setting function "fanspeed_val" is updated externally and it is taken care in next loop.
	//if "fana" is issued, automatically new value will start loading, we can override fan-control by using "fano"
	if (fanspeed_val!=fanspeed_last_val)
	{
		fanSpeed(fanspeed_val);
		fanspeed_last_val = fanspeed_val;
	}

}


uint8_t setPwmFrequency(int pin, int divisor)
{
	byte mode;
	if(pin == 5 || pin == 6 || pin == 9 || pin == 10)
	{
		switch(divisor)
		{
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 64: mode = 0x03; break;
			case 256: mode = 0x04; break;
			case 1024: mode = 0x05; break;
			default: return 0;
		}
		if(pin == 5 || pin == 6)
		{
			TCCR0B = (TCCR0B & 0b11111000) | mode;
		} else
		{
			TCCR1B = (TCCR1B & 0b11111000) | mode;
		}
	}
	else if(pin == 3 || pin == 11)
	{
		switch(divisor)
		{
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 32: mode = 0x03; break;
			case 64: mode = 0x04; break;
			case 128: mode = 0x05; break;
			case 256: mode = 0x06; break;
			case 1024: mode = 0x07; break;
			default: return 0;
		}
		TCCR2B = (TCCR2B & 0b11111000) | mode;
	}
	else
	{
		//not necessary but still implementing pin check
		return 0;
	}
	
	
	return 1;
}