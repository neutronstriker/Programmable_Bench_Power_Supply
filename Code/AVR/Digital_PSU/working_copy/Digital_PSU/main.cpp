//16MHZ Crystal Freq defined in Compiler Global Defines
#include "project_config.h"

#include "Arduino.h"
#include "neo_core_aux.h"
#include "i2c.h"
#include "GPIO.h"
#include <util/delay.h>
#include <avr/wdt.h>

//#include "Adafruit_SSD1306.h"
//#define OLED_ADD 0x3C
//Adafruit_SSD1306 lcd;

#include "INA260/INA260.h"
INA260 ina;
float busPower=0, shuntCurrent=0, busVoltage=0;
#define INA260_ADDRESS_ON_DCDC_CONTROL_BOARD	0x44

#include "ADS1115/Adafruit_ADS1015.h"
Adafruit_ADS1115 ads;


#include "Adafruit_PCD8544/Adafruit_PCD8544.h"
//Adafruit_PCD8544 lcd(9,10,8); //HW_SPI
//Adafruit_PCD8544 lcd(13, 11, 9, 10, 8); //SW_SPI
Adafruit_PCD8544 lcd(13, 11, 9, 8); //SW_SPI with CS tied to GND
//GPIO lcd_cs(10);//required for testing PCD8544 LCD Software SPI with CS tied to GND
GPIO lcd_bl(7);

#include "Serial_CMD_REPL.h"	//now serial Interpreter is also modular, however you may have to customize the readSerialCmd logic as per your project requirements
//extern void readSerialCmd();

#include "PowerModule.h"

#include "fanCtrl.h"

#ifndef F_STR
	#define F_STR(x) (__FlashStringHelper*)PSTR(x) //to be used only for Arduino print class based functions
#endif


void loop();
void initPlatform();


/*
//do not changes this
uint8_t case_sensitivity_status=0;
//do not changes this
uint8_t backspace_det=0;
*/

uint32_t disp_update_timestamp=0;
#define DISPLAY_UPDATE_PERIOD_ms 100

//////////////////////////////Encoder class test////////////////////////////////////////////////////////
#include "Rotary_Encoder.h"
/*
#define ENCODER_COUNT_MAX	18000
#define ENCODER_COUNT_MIN	1400
#define	ENCODER_INIT_VALUE	3300
#define	ENCODER_STEP_SIZE	100

uint16_t encoder_val=ENCODER_INIT_VALUE;
uint16_t knob_click_value=ENCODER_STEP_SIZE;

void volts_increase()
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
	
}*/

//////////////////////////////Encoder class test////////////////////////////////////////////////////////

void displayUpdateVi(float Vset, float I_lim_set){	//update Voltage and Current on the display
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
			lcd.print(busVoltage,2),lcd.println("V");
			lcd.print(shuntCurrent,3),lcd.println("A");
			lcd.display();
			
			disp_update_timestamp = millis();
		}
}


void displayUpdateOt(uint8_t temperature, uint8_t tempLimit){	//Show overTemperature warning and update display
	if(millis()-disp_update_timestamp > DISPLAY_UPDATE_PERIOD_ms){
		//lcd_bl.High();
		lcd.clearDisplay();
		lcd.setRotation(2);
		lcd.setTextWrap(true);
		
		//fix this block to accommodate Voltage values in 5 digit voltages now it can only take 4 digits
		lcd.setTextColor(BLACK);
		lcd.setTextSize(1);
		lcd.println();
		lcd.println(F_STR("  OVER-TEMP"));
		lcd.setTextSize(2);
		lcd.print(F_STR("  ")), lcd.print(temperature),lcd.println("C");
		lcd.setTextSize(1);
		lcd.print(F_STR(" LIMIT: ")),lcd.print(tempLimit),lcd.println("C");
		lcd.display();
		
		disp_update_timestamp = millis();
	}
}

/*
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
*/


/////////////////////////////////GPIO_OUTPUTS-START//////////////////////////////////////////////////////////
GPIO output_enable_relay(A3);
//GPIO fan(10);
/////////////////////////////////GPIO_OUTPUTS-END//////////////////////////////////////////////////////////

/////////////////////////////////GPIO_INPUTS-START//////////////////////////////////////////////////////////
#define SWITCH_DEBOUNCE_DELAY_ms	200
GPIO output_enable_sw(A2);
uint32_t output_enable_sw_debounce_timestamp=0;
GPIO voltage_knob_sw(3);
uint32_t voltage_knob_sw_debounce_timestamp=0;
GPIO current_knob_sw(6);
uint32_t current_knob_sw_debounce_timestamp=0;

uint16_t current_limit_prev_state_val=0, voltage_set_prev_state_val=0;
Rotary_Encoder voltage_knob(PC_INT_PORTD,PORTD4,PORTD5);
Rotary_Encoder current_knob(PC_INT_PORTC,PORTC0,PORTC1);
/////////////////////////////////GPIO_INPUTS-END//////////////////////////////////////////////////////////
uint8_t overTempHysteresisVal_C = 0;
#define OVER_CURRENT_LIMIT_A	8	//8A is the physical limitation of the DCDC buck module used in the Digital-PSU
uint8_t overCurrentFlag = 0;

void system_configuration_init(){
	//fan.High();
	//fan.Low(); //default FAN-ON at full speed when GPIO low
	//lcd_cs.Low(); //required for testing PCD8544 LCD Software SPI with CS tied to GND
	
	fanControlSetup(); //init Fan with full speed, if in worst case fan control doesn't work properly, this might help save the system.
	ina.begin(INA260_ADDRESS_ON_DCDC_CONTROL_BOARD);
	ina.configure(INA260_AVERAGES_16);
	ina.calibrate(0.004,0); //set calibration offset for INA260, there might be some part to part tolerances which could cause some errors and this is there to offset such errors
	/*
	//setting short-circuit protection, using INA260 for now until we have the control board with comparator+hysteresis for INA250 out comparison and relay toggle using open_drain ANDed circuitry with relay controlling GPIO
	//ina.setAlertLatch(true);		//enabled so that we don't miss any OVER-Current events because of the Interrupt detection minimum cycle count requirement, but later I selected level trigger, so there is very less chance of missing the interrupt anyway
	//if we enable the latch then, the level is always present as a result the interrupt continuously keeps re-entering which is not required, because we are anyway not clearing the "overCurrentFlag"
	ina.enableOverCurrentLimitAlert();
	ina.setCurrentLimit(OVER_CURRENT_LIMIT_A);
	EICRA &= ~((1<<ISC01) | (1<<ISC00));		//Set interrupt 
	EIMSK |= (1<<INT0);
	*/
	
	//float multiplier =  0.1875F;
	//ads.setGain(GAIN_TWOTHIRDS);
	//ads.begin();
	
	//setting TPL0102-100 ACR register to Volatile only mode, so that we do not wear out its EEPROM.
	uint8_t val = 0xC0;
	i2c_write(VOLTAGE_TRIMMER_DPOT_TPL0102_100_I2C_ADDR,0x10,&val,1);
	i2c_write(CURRENT_LIMIT_DPOT_TPL0102_100_I2C_ADDR,0x10,&val,1);
	
	//setVoltage(15500);
	
	//tcaSetP0_mode_output();
	
	//#define VOLTAGE_SET_mV	2500
	//#define CURRENT_LIMIT_SET_mA	5000
	setVoltage(VOLTAGE_SET_POINT_INIT_VALUE_mV);
	setCurrentLimit(CURRENT_LIMIT_SET_POINT_INIT_VALUE_mA);
	
	delay(200); //delay(1000);
	lcd.begin();
	//displayUpdate((float)VOLTAGE_SET_mV/1000,(float)CURRENT_LIMIT_SET_mA/1000);	
	
		
	voltage_knob.configure_attributes(VOLTAGE_SET_POINT_INIT_VALUE_mV,VOLTAGE_SET_RANGE_MAX_mV,VOLTAGE_SET_RANGE_MIN_mV,VOLTAGE_SET_POINT_STEP_SIZE_mV,2);
	current_knob.configure_attributes(CURRENT_LIMIT_SET_POINT_INIT_VALUE_mA,CURRENT_LIMIT_RANGE_MAX_mA,CURRENT_LIMIT_RANGE_MIN_mA,CURRENT_LIMIT_STEP_SIZE_mA,1); //it seems that the specific encoder part used for voltage knob is actually malfunctioning and providing these extra clicks
															//so for current control knob part the divisor can be 1.

	output_enable_sw.setInputPullUp();
	current_knob_sw.setInputPullUp(); //no pull-up resistor present for this pin on the module.
/*
	//turn lcd back light on															
	lcd_bl.High();
	//turn output relay on
	relay.High();
*/
	
	//now we are using switches to toggle LCD Back-light and relay
	lcd_bl.High();
	output_enable_relay.Low();
}

void main_system_loop(){
		
		busVoltage = ina.readBusVoltage();
		shuntCurrent = ina.readShuntCurrent();
		busPower = ina.readBusPower();
		
		temperatureControlLoop(); //poll temperature and control fan speed, also updates the temperature and FanSpeed global variables used by Serial_Cmd_repl
		
		if (!overCurrentFlag){
		
			if(temperature >= (OVER_TEMPERATURE_LIMIT_C-overTempHysteresisVal_C)){
				overTempHysteresisVal_C = OVER_TEMPERATURE_HYST_C;	//set hysteresis to prevent temperature measurement noise impact on the overTemp condition 
			
				overTempFlag = 1;			//set overT flag
				if (output_enable_relay.getState()){
					output_enable_relay.Low();	//turn off the output relay
					delay(10); //using 10ms delay in order to make sure that the relay is disconnected before we drop the voltage low otherwise we could cause heavy out-rush current from bulk capacitors if present on load
				}
				if (VOLTAGE_SET_RANGE_MIN_mV != voltage_set_prev_state_val){
					setVoltage(VOLTAGE_SET_RANGE_MIN_mV);	//set voltage to lowest possible value, will help cooling faster
					voltage_set_prev_state_val = VOLTAGE_SET_RANGE_MIN_mV;	//set the var so that when temperature is within safe limits again, the actual set value saved in
					// voltage_knob.encoder_val var is restored back to the power module
					//but this particular decision can be changed later, maybe a more safe option should be to keep the voltage
					//at lowest point because even if output is off the voltage set impacts temperature
				}

	/*
				if(!overTemp){
					output_enable_relay.Low();	//turn off the output relay
					delay(10); //using 10ms delay in order to make sure that the relay is disconnected before we drop the voltage low otherwise we could cause heavy out-rush current from bulk capacitors if present on load
					setVoltage(VOLTAGE_SET_RANGE_MIN_mV);	//set voltage to lowest possible value, will help cooling faster
					voltage_set_prev_state_val = VOLTAGE_SET_RANGE_MIN_mV;	//set the var so that when temperature is within safe limits again, the actual set value saved in
					// voltage_knob.encoder_val var is restored back to the power module
					//but this particular decision can be changed later, maybe a more safe option should be to keep the voltage
					//at lowest point because even if output is off the voltage set impacts temperature	
					overTemp = 1;		
				}
	*/
			
				displayUpdateOt(temperature,OVER_TEMPERATURE_LIMIT_C);
			}		
			else{						//else if (temperature < OVER_TEMPERATURE_LIMIT_C){
				overTempHysteresisVal_C = 0;//reset hysteresis to allow full range operation until "TEMPERATURE_MAX_LIMIT_C" is reached
				overTempFlag  = 0;		//unset  overT flag
				if (!lock_controls){	//this is 0 by default and can be changed vis SerialCmds "lock" and "unlock"
					//////////////////////////////////// KNOB DIRECTION DECODER ////////////////////////////////////////////
			
					//static uint8_t clkwise_cnt=0,anticlkwise_cnt=0; //may be the divisor part can be made part of the class direction decoder method.
					//uint8_t div=2; //the only viable approach to tackle the double increment per click has been this, I have tried the _delay_us/ms_ in the ISR already.
					switch(voltage_knob.encoderDir()){
						case CLOCK:
										#ifdef PRINT_DEBUG_MSG
											Serial.println(F_STR("VOLTS KNOB CLOCKWISE"));
										#endif
							
										//pot_increase(pot_channel);
										//clockwise();					
					
										/*clkwise_cnt++;	//the knob sometimes provides two transitions per click, so have added a divisor to tackle that, although it is a dirty trick.
										if (clkwise_cnt==div){
											volts_increase();
											clkwise_cnt=0;
										}*/
										voltage_knob.increase();
							
							break;
						case ANTICLOCK:
										#ifdef PRINT_DEBUG_MSG
											Serial.println(F_STR("VOLTS KNOB ANTI-CLOCKWISE"));
										#endif
					
										//pot_decrease(pot_channel);
										//anti();		
							
										/*anticlkwise_cnt++;
										if (anticlkwise_cnt==div){
											decrease();
											anticlkwise_cnt=0;
										}*/
										voltage_knob.decrease();
							break;
				
						//case BUFFER_EMPTY:	break;
						default: break;
			
					}
		
					//displayUpdateKnobTest(encoder_val);
					//delay(500);
		
					switch(current_knob.encoderDir()){
						case CLOCK:
							#ifdef PRINT_DEBUG_MSG
								Serial.println(F_STR("CURRENT LIMIT KNOB CLOCKWISE"));
							#endif
							current_knob.increase();
							break;
						case ANTICLOCK:
							#ifdef PRINT_DEBUG_MSG
								Serial.println(F_STR("CURRENT LIMIT KNOB ANTI-CLOCKWISE"));
							#endif
							current_knob.decrease();
							break;
						case BUFFER_EMPTY: break;
						default: break;
					}
		
					//////////////////////////////////// KNOB DIRECTION DECODER ////////////////////////////////////////////		
		
					//for(uint8_t i=0;i<255;i++)
					//{
						//tcaWriteP0(i);
						//delay(500);
						//Serial.print("TCA P0 Val: "); Serial.print(i); Serial.print(", Voltage: "); Serial.println(ads.readADC_Differential_0_1()*multiplier,3);
					//}
		
			


					if (output_enable_sw.getState()==false && (millis()-output_enable_sw_debounce_timestamp)>SWITCH_DEBOUNCE_DELAY_ms)
					{
						output_enable_relay.toggle();
						while(output_enable_sw.getState()==false); //wait until switch is released
						output_enable_sw_debounce_timestamp=millis();
					}



		/*
					//Temporarily using Voltage knob switch for relay toggling
					if (voltage_knob_sw.getState()==false && (millis()-voltage_knob_sw_debounce_timestamp)>SWITCH_DEBOUNCE_DELAY_ms)
					{
						output_enable_relay.toggle();
						while(voltage_knob_sw.getState()==false); //wait until switch is released
						voltage_knob_sw_debounce_timestamp=millis();
					}
		*/

					//Temporarily using Current knob switch for LCD back-light toggling
					if (current_knob_sw.getState()==false && (millis()-current_knob_sw_debounce_timestamp)>SWITCH_DEBOUNCE_DELAY_ms)
					{
						lcd_bl.toggle();
						while(current_knob_sw.getState()==false); //wait until switch is released
						current_knob_sw_debounce_timestamp=millis();
					}
				}

		
				//use a code to check if there is change in value and only they call setVoltage(), same should apply for setCurrentLimit()
				if (voltage_knob.encoder_val != voltage_set_prev_state_val){
					setVoltage(voltage_knob.encoder_val);
					voltage_set_prev_state_val = voltage_knob.encoder_val;
				}
		
				if (current_knob.encoder_val != current_limit_prev_state_val){
					setCurrentLimit(current_knob.encoder_val);
					current_limit_prev_state_val = current_knob.encoder_val;
				}
	
				displayUpdateVi((float)voltage_knob.encoder_val/1000,(float)current_knob.encoder_val/1000);	//update Voltage and Current on the display
			}

		}
		else{
			if(millis()-disp_update_timestamp > DISPLAY_UPDATE_PERIOD_ms){
				
				lcd.clearDisplay();
				lcd.setRotation(2);
				lcd.setTextColor(BLACK);
				lcd.setTextSize(1);
				lcd.println(F_STR("OVER-CURRENT"));
				lcd.println(F_STR("DETECTED"));
				lcd.println(F_STR("POWER CYCLE"));
				lcd.println(F_STR("THIS EQUIPMENT"));
				lcd.display();
				disp_update_timestamp = millis();
			}
		}
	
}

int main(void)
{
	wdt_reset();		//reset Watchdog timer
	wdt_disable();
	//wdt_enable(WDTO_4S);	//enable Watchdog timer with a Value of 4 seconds
	
	initPlatform();		//Initialize all platform devices like UART and Display and other peripherals.

	system_configuration_init(); //initialize all system components specific to project
		
	while(1)
	{
		main_system_loop(); //poll required devices, this needs to be active for all features to work well
		//loop(); // is generally used for generating calibration output
		readSerialCmd();
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

	 

	Serial.print("Setting Voltage,");Serial.print("INA VBUS Reading,");Serial.println("INA Shunt Current");
	//Serial.print("INA Current:"); Serial.print(ina.readShuntCurrent()); Serial.print(" INA Bus Voltage:"); Serial.print(ina.readBusVoltage()); Serial.print(" INA Bus Power:"); Serial.print(ina.readBusPower());
	//Serial.println();
	for(uint16_t voltage_test_mV = VOLTAGE_SET_RANGE_MIN_mV; voltage_test_mV < VOLTAGE_SET_RANGE_MAX_mV; voltage_test_mV+=100 )
	{
		//Serial.print("Setting Voltage: "); Serial.print(voltage_test_mV); Serial.println("mV");
		
		setVoltage(voltage_test_mV);
		
		delay(500);  //2s not required any more we have a 100ohm load on output of power module now, //2s delay is for stabilization as I have a constant current load, not a resistive load
		
		//Serial.print("INA Voltage Read Back: "); Serial.print(ina.readBusVoltage(),3); Serial.println("V");	

		//Serial.print("ADS1115 Voltage Reading: "); Serial.print(ads.readADC_Differential_0_1() * multiplier); Serial.println("mV");
		
		//dummy reads to throw out previous conversions
		for (uint8_t i = 0;i<10;i++)
		{
			ina.readBusVoltage();
		}
		//have also enabled hard averaging value of 16 in INA260 configuration
		Serial.print((float)voltage_test_mV/1000,3);Serial.print(","); Serial.print(ina.readBusVoltage(),3); Serial.print(","); Serial.println(ina.readShuntCurrent(),3);
		
		//Nokia LCD output test
		displayUpdateVi((float)voltage_test_mV/1000,5000.0/1000);
		
	}
	

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
	output_enable_relay.Low();
	setCurrentLimit(CURRENT_LIMIT_RANGE_MIN_mA);
	_delay_ms(10);		//for protecting the load and power path from out-rush from load, when voltage on the device decreases suddenly and if relay is still connected.
	setVoltage(VOLTAGE_SET_RANGE_MIN_mV);		
	overCurrentFlag = 1;	//this flag is not reset, as of now, only way to reset the system is to actually apply an arduino reset or power cycle the system, or issue a reset command if connected already via bt-uart
}




/*
	TODO:
	1.Modify the PCB to add the 12bit DAC
	2.Update the code as per the new DAC
	3.Add a temperature sensor and a FAN with ON/OFF capability if we are going forward with the 100 ohm resistor load
	4.Display power draw in the display using INA260/INA226
	5.Add the serialCmd feature to set voltage, current limit, Power limit etc
	6.Add the USB connector with a comparator+pfet+reference+output-ON_indication_led circuit on-board which can detect over-voltage and disconnect the USB output.
	7.Add a CC/CV indicator led, I think we can directly use the comparator output to drive the led.
	8.Add a Over Power limit indicator led, for this probably we can use the Alert pin directly as it is open-drain. or set the alert pin for over-voltage condition rather than power trigger, as it can sometimes protect devices from catastrphic failures in case the DAC is damaged 
	or the system becomes unstable.
	9.Add a switch
	10.If required add a short-circuit protection mechanism using a comparator + led to indicate it, when short-circuit event occurs, it should turn-off output relay.
	
	See if we need a bigger display, we can probably add other things like real-time power output display, system up-time and input supply voltage if we have a bigger display
	
	We have 180-13 = 167 voltage set points but we have 0-255 value, using that we can probably come up with a bit better transfer function to achieve at least accurate voltage output
	at 0.1V steps. Try reverse transfer function or use manual tuning calibration.
	
	Tried a bunch of reverse transfer functions, like full range single trf, then tried to correlate the error after applying trf and got another 2nd level trf and then tried piece wise range trf 1, until 1.4V to 10V and then 
	after analyzing all I decided to use a single one, since anyway when the voltage is lower then the granularity decreases as we need to apply higher voltage and where we have lower granularity.
	
	Regarding the fan speed control or On-off control I was thinking if we use a NTC thermistor with a transistor biased in such a way that when temperature increases the fan speed automatically increases (voltage increases)
	or at-least turns on beyond certain threshold, and probably we can we have a control to the set-point at which the fan speed is can be software controlled if an input to the transistor driver's bias network can be an I2C DAC.
	
	
	Now I have integrated the 12Bit I2C DAC using 5V input supply as its input Vref and then used 3k/1k to scale the output to 1/4th or close to 1250.
	It seems we have some inaccuracies in the set-point calculation using MCP4725 or the DAC is non-linear (has a bad INL). After some analysis I have understood
	a single linear reverse transfer function will not work to correct the inaccuracies. We will need multiple piece wise linear functions for that. I have also tried
	to fit it using polynomial 2nd order model even that didn't give a very good fit.
	
	Have decided to use a comparator with hysteresis and a LDO within enable control to turn the fan at 3.3V or 4V, at this voltage the fan does not make much noise and 
	is also able to handle the temperature well. If required we can later connect the two PWM+LRC DAC or a discrete DAC to both control the comparator set-point and/or
	control the voltage of the FAN by driving the FB of the LDO. But I don't feel that as necessary, only thing required will be to characterize and arrive at proper
	hysteresis values and FAN voltage for a given FAN and LOAD temperature.
	
*/