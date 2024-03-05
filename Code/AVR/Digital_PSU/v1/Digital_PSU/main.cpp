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

#include "ADS1115/Adafruit_ADS1015.h"
Adafruit_ADS1115 ads;


#include "Adafruit_PCD8544/Adafruit_PCD8544.h"
Adafruit_PCD8544 lcd(9,10,8);
GPIO lcd_bl(7);

#ifndef F_STR
	#define F_STR(x) (__FlashStringHelper*)PSTR(x) //to be used only for Arduino print class based functions
#endif


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



uint8_t setVoltage(uint16_t voltage_mV);
#define  MAX_SUPPORTED_VOLTAGE_RANGE_mV	18000
#define  MIN_SUPPORTED_VOLTAGE_RANGE_mV	1400
#define  VOLTAGE_TRIMMER_DPOT_TPL0102_100_I2C_ADDR	0x50


uint8_t setCurrentLimit(uint16_t current_mA);
#define CURRENT_LIMIT_DPOT_TPL0102_100_I2C_ADDR 0x51
#define MIN_SUPPORTED_CURRENT_LIMIT_RANGE_mA	10
#define MAX_SUPPORTED_CURRENT_LIMIT_RANGE_mA	5000	//theoretical is 12.5A or 12500mA but for testing I have now set it to 5A, since we will need to rework the circuit to achieve the full range.
#define CURRENT_LIMIT_STEP_SIZE_mA				10		//the Input Offset is around 5mA to 30mA range for INA250A3 so we are limited by that in here.


void tcaSetP0_mode_output()
{
	uint8_t data = 0;
	i2c_write(0x20,0x06,&data,1);
}
void tcaWriteP0(uint8_t data)
{
	i2c_write(0x20,0x02,&data,1);
}

uint32_t disp_update_timestamp=0;
#define DISPLAY_UPDATE_PERIOD_ms 100

void displayUpdate(float Vset, float I_lim_set){
		if(millis()-disp_update_timestamp > DISPLAY_UPDATE_PERIOD_ms){
			//lcd_bl.High();
			lcd.clearDisplay();
			lcd.setRotation(2);
			lcd.setTextWrap(true);
		
			//fix this block to accommodate Voltage values in 5 digit voltages now it can only take 4 digits
			lcd.setTextSize(1);
			lcd.setTextColor(WHITE,BLACK);
			lcd.print(F_STR("Ilim:")),lcd.print(I_lim_set,3),lcd.println("A"); //current limit set
			lcd.print(F_STR("Vset:")),lcd.print(Vset,3),lcd.println("V"); //voltage set value
			lcd.setTextColor(BLACK);
			lcd.setTextSize(2);
			lcd.print(ina.readBusVoltage(),2),lcd.println("V");
			lcd.print(ina.readShuntCurrent(),3),lcd.println("A");
			lcd.display();
			
			disp_update_timestamp = millis();
		}
}


//////////////////////////////////ENCODER TEST SCRATCH CODE/////////////////////////////////////
#include "rotary_encoder_no_class.h"
extern uint16_t encoder_val;
extern fifo_byte buffer;

void displayUpdateKnobTest(uint16_t val){
		if(millis()-disp_update_timestamp > DISPLAY_UPDATE_PERIOD_ms){
			lcd.clearDisplay();
			lcd.setRotation(2);
			lcd.setTextWrap(true);
		
			lcd.print(F_STR("Val:")); lcd.println(val);
			lcd.display();
				
			disp_update_timestamp = millis();			//remember delays can cause problems for knob. so we use millis to do this.
		}

}
//////////////////////////////////ENCODER TEST SCRATCH CODE END/////////////////////////////////////

int main(void)
{
	wdt_reset();		//reset Watchdog timer
	wdt_disable();
	//wdt_enable(WDTO_4S);	//enable Watchdog timer with a Value of 4 seconds
	
	initPlatform();		//Initialize all platform devices like UART and Display and other peripherals.

	ina.begin(0x44);
	ina.configure();
	
	//float multiplier =  0.1875F;
	//ads.setGain(GAIN_TWOTHIRDS);
	//ads.begin();
	
	//setting TPL0102-100 ACR register to Volatile only mode, so that we do not wear its eeprom.
	uint8_t val = 0xC0;
	i2c_write(VOLTAGE_TRIMMER_DPOT_TPL0102_100_I2C_ADDR,0x10,&val,1);
	
	//setVoltage(15500);
	
	//tcaSetP0_mode_output();
	
	#define VOLTAGE_SET_mV	2500
	#define CURRENT_LIMIT_SET_mA	5000
	setVoltage(VOLTAGE_SET_mV);
	setCurrentLimit(CURRENT_LIMIT_SET_mA);//5A
	
	delay(200); //delay(1000);
	lcd.begin();
	displayUpdate((float)VOLTAGE_SET_mV/1000,(float)CURRENT_LIMIT_SET_mA/1000);	
	
	init_encoder();
	
	while(1)
	{
		
		//////////////////////////////////// KNOB DIRECTION DECODER ////////////////////////////////////////////
		
		uint8_t test;
		static uint8_t clkwise=0,anticlkwise=0;
		uint8_t div=2; //the only viable approach to tackle the double increment per click has been this, I have tried the _delay_us/ms_ in the ISR already.
		if(!buffer.isEmpty())
		{
			test = buffer.deque();
			
			if(test == ANTICLOCK)
			{
				Serial.println(F_STR("ANTI"));
				//pot_decrease(pot_channel);
				//anti();
				
				clkwise++;	//the knob sometimes provides two transitions per click, so have added a divisor to tackle that, although it is a dirty trick.
				if (clkwise==div)
				{
					decrease();
					clkwise=0;
				}
				
			}
			else if (test == CLOCK)
			{
				Serial.println(F_STR("CLK"));
				//pot_increase(pot_channel);
				//clockwise();
				anticlkwise++;
				if (anticlkwise==div)
				{
					increase();
					anticlkwise=0;
				}
				
				
			}
			
		}
		
		//displayUpdateKnobTest(encoder_val);
		//delay(500);
		
		//////////////////////////////////// KNOB DIRECTION DECODER ////////////////////////////////////////////		
		
		//for(uint8_t i=0;i<255;i++)
		//{
			//tcaWriteP0(i);
			//delay(500);
			//Serial.print("TCA P0 Val: "); Serial.print(i); Serial.print(", Voltage: "); Serial.println(ads.readADC_Differential_0_1()*multiplier,3);
		//}
		
		//loop();
		
		//use a code to check if there is change in value and only they call setVoltage(), same should apply for setCurrentLimit()
		setVoltage(encoder_val);
		displayUpdate((float)encoder_val/1000,(float)CURRENT_LIMIT_SET_mA/1000);
		

		if (serialEventRun)
		{
			serialEventRun();	//Polls the Serial port for new data and updates Buffer accordingly, if serialEvent() function is used
		}
		
	}
	

	return 0;
}

void loop()
{
	


		  // The ADC input range (or gain) can be changed via the following
		  // functions, but be careful never to exceed VDD +0.3V max, or to
		  // exceed the upper and lower limits if you adjust the input range!
		  // Setting these values incorrectly may destroy your ADC!
		  //                                  Gain          Range   ADS1115
		  //                                 --------     --------  --------
		  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  0.1875mV (default)
		  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  0.125mV
		  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  0.0625mV
		  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  0.03125mV
		  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  0.015625mV
		  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  0.0078125mV
	  
	  /* Be sure to update this value based on the IC and the gain settings! */
	  //float multiplier = 0.125F; 

	 

	Serial.print("Setting Voltage,");Serial.print("INA VBUS Reading");Serial.println("INA Shunt Current");
	//Serial.print("INA Current:"); Serial.print(ina.readShuntCurrent()); Serial.print(" INA Bus Voltage:"); Serial.print(ina.readBusVoltage()); Serial.print(" INA Bus Power:"); Serial.print(ina.readBusPower());
	//Serial.println();
	for(uint16_t voltage_test_mV = MIN_SUPPORTED_VOLTAGE_RANGE_mV; voltage_test_mV < MAX_SUPPORTED_VOLTAGE_RANGE_mV; voltage_test_mV+=100 )
	{
		//Serial.print("Setting Voltage: "); Serial.print(voltage_test_mV); Serial.println("mV");
		
		setVoltage(voltage_test_mV);
		
		delay(2000);  //2s delay is for stabilization as I have a constant current load, not a resistive load
		
		//Serial.print("INA Voltage Read Back: "); Serial.print(ina.readBusVoltage(),3); Serial.println("V");	

		//Serial.print("ADS1115 Voltage Reading: "); Serial.print(ads.readADC_Differential_0_1() * multiplier); Serial.println("mV");
		
		Serial.print(voltage_test_mV);Serial.print(","); Serial.print(ina.readBusVoltage(),3); Serial.print(","); Serial.println(ina.readShuntCurrent(),3);
		
		//Nokia LCD output test
		displayUpdate((float)voltage_test_mV/1000,5000.0/1000);
		
	}
	

}

uint8_t setVoltage(uint16_t voltage_mV)
{
	uint8_t pot[2];
	
	if (!(voltage_mV <= MAX_SUPPORTED_VOLTAGE_RANGE_mV && voltage_mV >= MIN_SUPPORTED_VOLTAGE_RANGE_mV))
	{
		Serial.println(F_STR("Error! Voltage Requested Out of Range"));
		return 0;
	}
		
		//uint16_t vTrimRequired = voltage_mV;
		float vTrimRequired = 1335.0f-((float)(17UL*voltage_mV)/250UL);
		//uint16_t vTrimRequired = 1335-((17UL*voltage_mV)/250UL);

		//pot[0] = ((float)(128*voltage_mV))/625; //first pot is used for scaling voltage to whatever value we can get; Vwl = (Vh-Vl)*(D/256) => Vwl = 1.25*(D/256) => D = (1024*Vwl)/5
		/*
		pot[0] = (256UL*voltage_mV)/625UL;
		float voltageOutPot0Wiper_mV = ((float)(625UL*pot[0]))/128UL;
		pot[1] = (256UL*voltage_mV)/voltageOutPot0Wiper_mV; //pot1 is used for adjusting the fraction error from pot 0
		*/
		
		
		uint16_t pot0Val = (vTrimRequired*256UL)/625UL;
		if (pot0Val >= 256)
		{
			pot0Val=255;
		}
		if (pot0Val == 0)
		{
			pot0Val = 1;
		}
		pot[0] = pot0Val;
		float voltageOutPot0Wiper_mV = ((float)(625UL*pot[0]))/128UL;
		pot[1] = (vTrimRequired*256UL)/voltageOutPot0Wiper_mV;
		
		//pot[1] = (256UL*voltage_mV)/((625UL*pot[0])/128UL);
		//pot[1] = 255;

		//pot[0] = (65025UL*voltage_mV)/(1250UL*127UL);	
		//pot[1] = (65025UL*voltage_mV)/(1250UL*pot[0]);
	//0.00068V is the step size required to increment or decrement voltage by 0.01V in the DCDC
	
	//this implementation of DAC is not truly 16bit as a result we loose 16bit effect after the MSB DAC has a value more than 1
	//we could actually implement a Voltage summing circuit that could how ever do that.
	
	//1. Now what is possible is that we scale the input voltage using POTA to get required voltage assuming that POTB is at its center
	//2. Then we adjust POTB to correct the fractional error by tuning it to whatever possible extent.
	
	//now there are multiple problems once the POTA taper is around 255, we are completely dependent on POTB to provide the scaling, so it essentially becomes 8bit
	//i.e. whenever the POTA moves higher and higher towards its H pin we are losing the resolution that we can achieve.
	
	//apart from that the DCDC buck 150W is a low freq 160KHz around BUCK and is not synchronous as well, so its response time is very high and when input voltage is high
	//the lower output voltage settings do not work very well. As it does not have the PWM resolution to maintain such low voltages, with high input voltage.
	
	//And since it is not synchronous it takes time to respond for output voltage change since it is unable to discharge the output cap and get lower voltages.
	
	//we can however make a 16bit DAC using two different 8bit DAC, I have made an explanation in the non-inverting summer example in everycircuit.com
	//we will need a non-inverting summing circuit such that the LSB DAC's output will scaled by full-scale value i.e. divided by 256 in this case and summed with the direct
	//output of MSB DAC. To make a proper summing arrangement we will need to buffer the LSB DAC output and SUMMING amplifier input resistance and GAIN calculated properly.
	//generally if the input resistance is same for all sum inputs then it will straight way average of all inputs multiplied by gain. If number of inputs is same as the gain
	//then it will become a simple sum. Or we can triple DAC method where two DAC's will set the lower and higher voltage inputs of the a third DAC and this will actually be used for
	//trimming.
	
	//Now a minimum load resistor drawing at least 10mA has to be put across the output as close as possible to the power module for the output to be stable
	
	//also the problem was that the measurement was shared between Load and Vbus_sense as a result the IR drop effect was also encountered in it.
	//Apart from that the Ground wire was also shared between the load and was very long cable, it should be a short wire going straight between this Vtrim header and 
	//the gnd pin close to the Gnd but away from the Power ground path, generally mostly DCDC converters will have a Analog ground plane for that.
	
	//tested a 8-bit implementation of R-2R DAC, need to build and check 16-bit results. 8-bit DAC results in R-2R_DAC_test.xlsx
	
	#ifdef PRINT_DEBUG_MSG
		//Serial.print("Voltage set is :");
		//Serial.println(current);
	
		Serial.print("VtrimRequired is :");
		Serial.println(vTrimRequired);
	
		Serial.print("Pot 0 :");
		Serial.println(pot[0]);
	
		Serial.print("Pot 1 :");
		Serial.println(pot[1]);
	
		uint16_t achievedVoltage = (1250UL*pot[0]*pot[1])/65536UL;
	
		Serial.print("Achieved Vtrim_voltage :");
		Serial.println(achievedVoltage);
		
		Serial.print("Achieved DCDC_Voltage :");
		uint16_t achievedDCDCVoltage = (333750UL-(250UL*achievedVoltage))/17UL;
		Serial.println(achievedDCDCVoltage);
		
		
	#endif
	
	i2c_write(VOLTAGE_TRIMMER_DPOT_TPL0102_100_I2C_ADDR,0,pot,2);		

	return 1;
}


uint8_t setCurrentLimit(uint16_t current_mA){
		uint8_t pot[2];
	
	if (!(current_mA <= MAX_SUPPORTED_CURRENT_LIMIT_RANGE_mA && current_mA >= MIN_SUPPORTED_CURRENT_LIMIT_RANGE_mA))
	{
		Serial.println(F_STR("Error! Current Limit Requested Out of Range"));
		return 0;
	}
		
		float iLimDACVout_mV = (float)current_mA/5UL;//INA250A3 800mV/A and then used a by 4 voltage divider before comparator so it is finally 200mV/A or 2mV per 10mA.
		//add a protection to not let the above value go to 0 after calculation
	
		uint16_t pot0Val = (iLimDACVout_mV*128UL)/625UL;
		if (pot0Val >= 256)
		{
			pot0Val=255;
		}
		if (pot0Val == 0)
		{
			pot0Val = 1;
		}
		pot[0] = pot0Val;
		float voltageOutPot0Wiper_mV = ((float)(625UL*pot[0]))/64UL;
		pot[1] = (iLimDACVout_mV*256UL)/voltageOutPot0Wiper_mV;
			
	#ifdef PRINT_DEBUG_MSG
		Serial.print("Current Limit set is :");
		Serial.println(current_mA);
	
		Serial.print("iLimDACVout_mV is :");
		Serial.println(iLimDACVout_mV);
	
		Serial.print("Pot 0 :");
		Serial.println(pot[0]);
	
		Serial.print("Pot 1 :");
		Serial.println(pot[1]);
	
		float achievedVoltage = (float)(2500UL*pot[0]*pot[1])/65536UL;
	
		Serial.print("Achieved iLim_DAC Voltage :");
		Serial.println(achievedVoltage);
		
		Serial.print("Achieved Current Limit :");
		float achievedCurrentLimit = (achievedVoltage*5UL);
		Serial.println(achievedCurrentLimit);
		
		
	#endif
	
	i2c_write(CURRENT_LIMIT_DPOT_TPL0102_100_I2C_ADDR,0,pot,2);		

	return 1;
}


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
