//16MHZ Crystal Freq defined in Compiler Global Defines
#include "project_config.h"

#include "Arduino.h"
#include "i2c.h"
#include "GPIO.h"
#include <util/delay.h>
#include <avr/wdt.h>

//#include "Adafruit_SSD1306.h"
//#define OLED_ADD 0x3C
//Adafruit_SSD1306 lcd;

#include "INA260/INA260.h"
INA260 ina;


#define F_STR(x) (__FlashStringHelper*)PSTR(x) //to be used only for Arduino print class based functions

uint8_t seconds();
uint16_t minutes();
void loop();
void reset_millis();
void initPlatform();
void watchDog_off();
void resetSysUsingWDT();
void i2c_internal_pullup(bool state);

//do not changes this
uint8_t case_sensitivity_status=0;
//do not changes this
uint8_t backspace_det=0;



void initPlatform()
{
	init();	//Initialize Arduino Core
	
	Serial.begin(57600);//increases a lot or Ram and Rom usage disable after diagnostics and recompile.
	
	Serial.print(F_STR("DEVICE_INITIALISED\r\n"));
	
	i2c_init(I2C_BUS_CLK);	//it is called with 400Khz by adafruit display lib so not necessary any more, but before display is initialized many other I2C devices are
	//also initialized so we call it here. It is fine though to be set twice not a big deal
	
	i2c_internal_pullup(true);	//If this is not enabled BUS hangs most of the times. So this has enabled. Maybe pull-up value is too high
	//so heavy bus capacitance of three devices is not being handled by 10k resistors alone, I should have used 4.7k or less value
	//same change should be done in PPS also.
	//this is not required since pull-ups are present in LCD board, however
	//if we add more devices to bus we should add external pull-ups.	Find updated answers to this in Readme log of electronic load.

//	lcd.begin(SSD1306_SWITCHCAPVCC, OLED_ADD);
	
//	lcd.dim(true);//dim the display brightness
		
}


int main(void)
{
	wdt_reset();		//reset Watchdog timer
	wdt_disable();
	//wdt_enable(WDTO_4S);	//enable Watchdog timer with a Value of 4 seconds
	
	initPlatform();		//Initialize all platform devices like UART and Display and other peripherals.

	ina.begin(0x44);
	ina.configure();
	while(1)
	{
		loop();
		if (serialEventRun)
		{
			serialEventRun();	//Polls the Serial port for new data and updates Buffer accordingly, if serialEvent() function is used
		}
		
	}
	

	return 0;
}

void loop()
{
	
	Serial.print("INA Current:"); Serial.print(ina.readShuntCurrent()); Serial.print(" INA Bus Voltage:"); Serial.print(ina.readBusVoltage()); Serial.print(" INA Bus Power:"); Serial.print(ina.readBusPower());
	Serial.println();
	delay(1000);

}

ISR(INT0_vect)
{

}


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
