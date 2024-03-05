/*
 * Serial_CMD_REPL.cpp
 *
 * Created: 8/10/2019 12:13:56 AM
 *  Author: neutron
 */ 
#include "Serial_CMD_REPL.h"

char btCmdReplyBuffer[CMD_REPLY_BUFFER_LENGTH]={0};
uint8_t uartBufCharCount=0;
uint8_t case_sensitivity_status=0;
uint8_t backspace_det=0;
uint8_t lock_controls=0;




////////////////////////////////SERIAL CMD INTERPRETER VARS START/////////////////////////////////////////////////////

//lets try to keep all command keywords within 4 letters, to improve performance by reducing search time,
//however it will be better if we can using strcmp_P and make sure that commands are executed when followed by cr,lf etc. --done
const char PROGMEM cmd1[]="on";			//Turn output on
const char PROGMEM cmd2[]="off";		//Turn output off
const char PROGMEM cmd3[]="vbus?";		//Read Voltage
const char PROGMEM cmd4[]="ibus?";		//Read Current
const char PROGMEM cmd5[]="pbus?";		//Read Power
const char PROGMEM cmd6[]="vset?";		//Read Voltage set point
const char PROGMEM cmd7[]="iset?";		//Read Current Limit set point
const char PROGMEM cmd8[]="vset";		//Set Voltage
const char PROGMEM cmd9[]="iset";		//Set Current Limit
const char PROGMEM cmd10[]="bloff";		//Turn back-light off
const char PROGMEM cmd11[]="blon?";		//Read Back-light status
const char PROGMEM cmd12[]="blon";		//Turn back-light on
const char PROGMEM cmd13[]="ison?";		//Read Output Relay status
const char PROGMEM cmd14[]="help";		//will print available commands and their usage
const char PROGMEM cmd15[]="lock";		//lock on device controls, remote access only
const char PROGMEM cmd16[]="unlock";	//unlock on device controls

const char PROGMEM cmd17[]="reset";		//reset device
const char PROGMEM cmd18[]="bspace_e";	//enable backspace detection
const char PROGMEM cmd19[]="bspace_d";	//disable backspace detection
const char PROGMEM cmd20[]="case_e";	//Enable case sensitivity of commands, does not auto-convert the case of input string
const char PROGMEM cmd21[]="case_d";	//Disable case sensitivity of commands, automatically converts input command string to lower case
const char PROGMEM cmd22[]="ate";		//Enable terminal echo
const char PROGMEM cmd23[]="atd";		//Disable terminal echo

//const char PROGMEM cmd24[]="fanp";		
//const char PROGMEM cmd25[]="fanf";
const char PROGMEM cmd26[]="temp?";
//const char PROGMEM cmd27[]="fanon";
//const char PROGMEM cmd28[]="fanoff";

const char PROGMEM cmd28[]="fans?";	//read fan speed
const char PROGMEM cmd29[]="fans";	//set fan speed, works only if override is enabled
const char PROGMEM cmd30[]="fano";	//set fan control to manual override
const char PROGMEM cmd31[]="fana";	//set fan control to automatic mode


////////////////////////////////SERIAL CMD INTERPRETER VARS END/////////////////////////////////////////////////////

void usage(){
	
	Serial.println(F_STR("This is the help message, you will find usage instructions below:"));

	Serial.println(F_STR("on								:	Turn output on"));
	Serial.println(F_STR("off								:	Turn output off"));
	Serial.println(F_STR("vbus?								:	Read Voltage"));
	Serial.println(F_STR("ibus?								:	Read Current"));
	Serial.println(F_STR("pbus?								:	Read Power"));
	Serial.println(F_STR("vset	<set voltage in mV>					:	Set Voltage"));	//these tabs have been adjusted as per putty output and using notepad++ "View->Symobols->show all characters" options
	Serial.println(F_STR("iset	<set current limit in mA>				:	Set Current Limit"));
	Serial.println(F_STR("vset?								:	Read Voltage set point"));
	Serial.println(F_STR("iset?								:	Read Current Limit set point"));
	Serial.println(F_STR("bloff								:	Turn back-light off"));
	Serial.println(F_STR("blon								:	Turn back-light on"));
	Serial.println(F_STR("blon?								:	Read Back-light status"));
	Serial.println(F_STR("ison?								:	Read Output Relay status"));
	Serial.println(F_STR("lock								:	lock on device controls, remote access only"));
	Serial.println(F_STR("unlock								:	unlock on device controls"));
	Serial.println(F_STR("temp?								:	Read the internal dummy load temperature"));
	Serial.println(F_STR("fans?								:	Read Fan Speed"));
	Serial.println(F_STR("fans <speed: 0-150>						:	set fan speed between 0 to 150"));
	Serial.println(F_STR("fano								:	set fan control override, required for \"fans\" to work"));
	Serial.println(F_STR("fana								:	set fan control to automatic mode"));
	
	Serial.println(F_STR("reset								:	reset device"));
	Serial.println(F_STR("help								:	print usage"));

	Serial.println(F_STR("\r\nHelper Commands for interactive mode:"));
	Serial.println(F_STR("bspace_e <optional: bspace_key_code: 127 | 8(default) >		:	enable backspace detection"));
	Serial.println(F_STR("bspace_d							:	disable backspace detection"));
	Serial.println(F_STR("case_e								:	Enable case sensitivity of commands, does not auto-convert the case of input string"));
	Serial.println(F_STR("case_d								:	Disable case sensitivity of commands, automatically converts input command string to lower case"));
	Serial.println(F_STR("ate								:	Enable terminal echo"));
	Serial.println(F_STR("atd								:	Disable terminal echo"));
}



//////////////////////////////////////// Serial Cmd interpreter, a lot improved version than the Original DDAT/////////////////////////////////////////////////////////
/*
* Where ever data acquisition is involved from a sensor or an interface, I have made sure mostly that the sensor is polled only in one place throughout the code
* and a variable which gets updated in main loop is used to report back the value, so there is not inconsistency in reported value vs actual value and we have
* better insight and there is sync in the reported waveforms.
*/


void readSerialCmd()
{
		//char *pos;
		char numeric[12];
		uint8_t token_char = '\r';
		//uint8_t num;
		
		/*
			Don't Provide both CR and LF after cmds provide only CR. LF is not honored and is taken up as a character thus, if anywhere
			whole word match is required that condition will fail.
		*/

#ifdef SERIAL_CR_DETECT
	if(Serial.crStatusDetected())	//experimental feature.
#endif
		if(Serial.available())
		{
			#ifdef PRINT_DEBUG_MSG
				Serial.println(F_STR("data available"));
			#endif
			
				
				uartBufCharCount = readUntilToken(btCmdReplyBuffer,token_char,CMD_REPLY_BUFFER_LENGTH);	
				
				if (case_sensitivity_status)
				{
					strlwr(btCmdReplyBuffer); //convert string to lower case
				}
				
								
				#ifdef PRINT_DEBUG_MSG
					Serial.print("Char Count ");
					Serial.println(uartBufCharCount);
				#endif
				
				if(uartBufCharCount == 0)
				{
					#ifdef PRINT_DEBUG_MSG
						Serial.println("No Character found");
					#endif
					
					return;				//i.e. the '\r' was not found in string/array
				}
				
				/*
				//Read until return is detected or 12 bytes from serial Fifo to buffer.
				uartBufCharCount = Serial.readBytes(btCmdReplyBuffer,12);
				
				
				
				//terminate the string at the point where we find the token character which is '\r' <CR> in this case.
				uint8_t char_pos;
				for(char_pos=0;char_pos<uartBufCharCount;char_pos++)
				{
					if(btCmdReplyBuffer[char_pos]==token_char)
					{
						btCmdReplyBuffer[char_pos]='\0';		//terminate the string at that point so that unnecessary string processing
						break;									//can be avoided.
					}
					
				}
				
				//when 'char_pos' value is same as total character count that means it never found the character,
				//otherwise a break instruction would have been executed. 
				if(char_pos==uartBufCharCount)
				{
					#ifdef PRINT_DEBUG_MSG
						Serial.println("No Carriage return detected");
					#endif
					return;				//i.e. the '\r' was not found in string/array
				}
				*/
				
			#ifdef PRINT_DEBUG_MSG
				uint8_t i=0;
				while(i<=uartBufCharCount)
				{
					Serial.print(btCmdReplyBuffer[i], HEX);
					Serial.print(' ');
					i++;
				}
				//serial debug messages here
			#endif // PRINT_DEBUG_MSG			
			
			
			if (strcmp_P(btCmdReplyBuffer,cmd1)==0)
			{	
				if(!overTempFlag){
					#ifdef PRINT_DEBUG_MSG
						Serial.println(F_STR("OUTPUT: ON"));
					#endif
					output_enable_relay.High();
					replyOK();	
				}
				else{
					Serial.println(F_STR("OVERT"));
				}

			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd2)==0)
			{
				if(!overTempFlag){
					#ifdef PRINT_DEBUG_MSG
						Serial.println(F_STR("OUTPUT: OFF"));
					#endif
					output_enable_relay.Low();
				}
				else{
					Serial.println(F_STR("OVERT"));
				}

			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd3)==0)
			{
				//Serial.print(ina.readBusVoltage(),3); Serial.println("V");
				Serial.print(busVoltage,3); Serial.println("V");
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd4)==0)
			{
				//Serial.print(ina.readShuntCurrent(),3); Serial.println("A");
				Serial.print(shuntCurrent,3); Serial.println("A");
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd5)==0)
			{
				//Serial.print(ina.readBusPower(),3); Serial.println("W");
				Serial.print(busPower,3); Serial.println("W");
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd6)==0)
			{
				Serial.print((float)voltage_knob.encoder_val/1000,3); Serial.println("V");
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd7)==0)
			{
				Serial.print((float)current_knob.encoder_val/1000,3); Serial.println("A");
			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd8,4)==0)
			{
				if (!overTempFlag){
					uint16_t vSetVal = 0;
					strncpy(numeric,btCmdReplyBuffer+4,uartBufCharCount-(4));
					
					vSetVal=atoi(numeric);
					
					#ifdef PRINT_DEBUG_MSG
					Serial.print(F_STR("V_set val is "));
					Serial.println(vSetVal);
					#endif
					
					if (vSetVal > VOLTAGE_SET_RANGE_MAX_mV || vSetVal < VOLTAGE_SET_RANGE_MIN_mV)
					{
						Serial.println(F_STR("V_set value out of Range"));
						return;
					}
					
					voltage_knob.encoder_val = vSetVal;
					
					replyOK();
				}
				else{
					Serial.println(F_STR("OVERT"));
				}
				

			}
			
			else if(strncmp_P(btCmdReplyBuffer,cmd9,4)==0)
			{
				if (!overTempFlag){
					uint16_t iSetVal = 0;
					strncpy(numeric,btCmdReplyBuffer+4,uartBufCharCount-(4));
									
					iSetVal=atoi(numeric);
									
					#ifdef PRINT_DEBUG_MSG
					Serial.print(F_STR("I_Limit_set val is "));
					Serial.println(iSetVal);
					#endif
									
					if (iSetVal > CURRENT_LIMIT_RANGE_MAX_mA || iSetVal < CURRENT_LIMIT_RANGE_MIN_mA)
					{
						Serial.println(F_STR("I_Limit_set value out of Range"));
						return;
					}
									
					current_knob.encoder_val = iSetVal;
									
					replyOK();
					
				}
				else{
					Serial.println(F_STR("OVERT"));
				}

			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd10)==0)
			{
				lcd_bl.Low();
				replyOK();
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd11)==0)
			{
				if (lcd_bl.getState())
				{
					Serial.println(F_STR("ON"));
				}
				else{
					Serial.println(F_STR("OFF"));
				}
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd12)==0)
			{
				lcd_bl.High();
				replyOK();
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd13)==0)
			{
				if (output_enable_relay.getState())
				{
					Serial.println(F_STR("ON"));
				}
				else{
					Serial.println(F_STR("OFF"));
				}
			
			}

			else if (strcmp_P(btCmdReplyBuffer,cmd15)==0)
			{
				voltage_knob.disable();
				current_knob.disable();
				lock_controls=1;
				replyOK();
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd16)==0)
			{
				//need to empty the encoder buffers or else it will immediately update output values wrt content in buffer.
				//instead of the above I added methods to disable the encoder interrupt
				voltage_knob.enable();
				current_knob.enable();
				lock_controls=0;
				replyOK();
			}			

			else if(strcmp_P(btCmdReplyBuffer,cmd14)==0)
			{
				
				usage();
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd17)==0)
			{
				replyOK();
				delay(30);	//when resetSysUsingWDT executes, before "OK" from above command is sent out of Serial TX Buffer device is reset, so adding delay here.
				resetSysUsingWDT(); //reset device using WDT.
			}


////////////////////////////////////////////FAN testing //////////////////////////////////////////////////////////////////	
			/*
			else if (strncmp_P(btCmdReplyBuffer,cmd24,4)==0)
			{
				uint16_t fanPwmSetVal = 0;
				strncpy(numeric,btCmdReplyBuffer+4,uartBufCharCount-(4));
				
				fanPwmSetVal=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("fan_pwm_set val is "));
				Serial.println(fanPwmSetVal);
				#endif
				
				if (fanPwmSetVal > 255 || fanPwmSetVal < 0)
				{
					Serial.println(F_STR("fan_pwm_set value out of Range"));
					return;
				}
				
				analogWrite(10,fanPwmSetVal);
				
				replyOK();
			}
			
			else if (strncmp_P(btCmdReplyBuffer,cmd25,4)==0)
			{
				uint16_t fanPwmFreqSetVal = 0;
				strncpy(numeric,btCmdReplyBuffer+4,uartBufCharCount-(4));
				
				fanPwmFreqSetVal=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
				Serial.print(F_STR("fan_pwm_freq_set val is "));
				Serial.println(fanPwmFreqSetVal);
				#endif
				
				if (fanPwmFreqSetVal > 1024 || fanPwmFreqSetVal < 1)
				{
					Serial.println(F_STR("fan_pwm_freq_set value out of Range"));
					return;
				}
				
				setPwmFrequency(10,fanPwmFreqSetVal);
				
				replyOK();
			}
			*/
			else if (strcmp_P(btCmdReplyBuffer,cmd26)==0)
			{
				/*
				analogReference(DEFAULT);
				Serial.println(4.88*analogRead(A6));
				*/
				//Serial.print(ntcThermistorTemp(),3); Serial.println('C');
				Serial.print(temperaturePrecise,3); Serial.println('C');
			}
			
			/*
			else if (strcmp_P(btCmdReplyBuffer,cmd27)==0)
			{
				//fan.Low(); //this is unable to revert to GPIO pin from PWM, so used arduino built in function
				pinMode(10,OUTPUT);
				digitalWrite(10,LOW);
				replyOK();
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd28)==0)
			{
				//fan.High();
				pinMode(10,OUTPUT);
				digitalWrite(10,HIGH);
				replyOK();
			}*/
			
			else if (strcmp_P(btCmdReplyBuffer,cmd28)==0)
			{
				Serial.println(fanspeed_val);
			}
			
			else if (strncmp_P(btCmdReplyBuffer,cmd29,4)==0)
			{
				if (fanOverride==0){
					Serial.println(F_STR("set fan override first! it is in automatic mode"));
					return;
				}
				
				uint16_t fanSpeedSetVal = 0;
				strncpy(numeric,btCmdReplyBuffer+4,uartBufCharCount-(4));
				
				fanSpeedSetVal=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
					Serial.print(F_STR("fan_speed_set val is "));
					Serial.println(fanSpeedSetVal);
				#endif
				
				//if (fanSpeed(fanSpeedSetVal)==0){
				if (fanSpeedSetVal > FAN_SPEED_MAX || fanSpeedSetVal < FAN_SPEED_OFF){ //not using FAN_SPEED_MIN here 
					//instead using FAN_SPEED_OFF so that we can completely turn off the fan if required.
					Serial.println(F_STR("fan_speed_set value out of Range"));
					return;
				}
				else{
					fanspeed_val = fanSpeedSetVal;	//applying this is handled in main_system_loop()->fanControlLoop()
					replyOK();
				}
				

			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd30)==0)
			{
				fanOverride = 1;
				replyOK();
			}
			
			else if (strcmp_P(btCmdReplyBuffer,cmd31)==0)
			{
				fanOverride = 0;
				replyOK();
			}
			
					
////////////////////////////////////////////FAN testing //////////////////////////////////////////////////////////////////
			
			////////////////////////////NON-ESSENTIAL FUNCTIONS_START////////////////////////////////
			
			else if (strncmp_P(btCmdReplyBuffer,cmd18,8)==0)
			{
				unsigned char backspace_key_decoded = 0;
				strncpy(numeric,btCmdReplyBuffer+8,uartBufCharCount-(8));
								
				backspace_key_decoded=atoi(numeric);
				
				#ifdef PRINT_DEBUG_MSG
					Serial.print(F_STR("backspace_key_decoded is "));
					Serial.println(backspace_key_decoded);
				#endif
				
				//only two options are available and default is minicom option, because that is IBM pc standard option
				if(backspace_key_decoded == BACKSPACE_ASCII_PUTTY){
					backspace_key = BACKSPACE_ASCII_PUTTY;
				}
				else{
					backspace_key = BACKSPACE_ASCII_MINICOM;
				}

				backspace_det = 1;
				Serial.println(F_STR("Backspace detection Enabled"));
				replyOK();
			}	
			else if (strcmp_P(btCmdReplyBuffer,cmd19)==0)
			{
				backspace_det = 0;
				Serial.println(F_STR("Backspace detection Disabled"));
				replyOK();
			}
			
			
						
			else if (strcmp_P(btCmdReplyBuffer,cmd20)==0)
			{
				case_sensitivity_status = 0;
				Serial.println(F_STR("Case Sensitivity Enabled"));
				replyOK();
			}			
			else if (strcmp_P(btCmdReplyBuffer,cmd21)==0)
			{
				case_sensitivity_status = 1;
				Serial.println(F_STR("Case Sensitivity Disabled"));
				replyOK();
			}
						
			

			else if(strcmp_P(btCmdReplyBuffer,cmd22)==0)	
			{
				Serial.enableEcho();
				//ate_enabled=1;
				Serial.println(F_STR("ECHO_ENABLED"));
				replyOK();
			}			
			else if(strcmp_P(btCmdReplyBuffer,cmd23)==0)	
			{
				Serial.disableEcho();
				Serial.println(F_STR("ECHO_DISABLED"));
				replyOK();
			}
			
			////////////////////////////NON-ESSENTIAL FUNCTIONS_START////////////////////////////////



			else
			{
				Serial.println(F_STR("ERROR(!)"));	
			}
									
		}
		
}




uint8_t readUntilToken(char* buffer, uint8_t token, uint8_t maxlen)
{
	/*	The Serial.readBytesUntil() doesn't work that well for me because it does a signed character check, which sometimes
		could be a problem for me because it doesn't work well sometime, especially when I tell it read looking for CR it goes mad
		and returns random characters.
		
		So i decided to do my own version.
	*/
	
	/*
		Another most important thing is that we should not use a mixture of both my uart_function and Serial class
		it creates lot of problems and wastes lot of time to debug, use only one of them in whole program.
		
		One more thing is the Serial.readUntil thing also filled the buffer with the token which was the problem
		some places I have used code which looks to match the whole word in that case it fails.
		
		I used to get only half the codes present in buffer like for hello i got only 6C 6F 0D 00 because 
		the hexbyte used uart_write() internally which creates problem since it uses blocking mode not interrupt
		mode like Serial class and interrupt is already enabled for UDRE so it messes things up. So don't use it
		here anymore.
		
		But the Serial.readBytesUntil is advanced in that it can wait for until we actually send the character 
		to the uart and the max wait period can be set to timeout value. So it can actually let you get all
		the characters into a buffer which we specify until the token is detected, and buffer can be bigger than
		even serialreceive buffer of 64bytes.
		
		But since we are doing parsing only after detecting CR so we don't need this feature, not useful for me
		right now. May be we can use this if required for exclusive SPI and I2C mode to specify data strings.
		
		But my code in this function is slightly faster than Serial.readBytesUntil because it doesn't wait for
		any timeout and does use timedRead();
	*/ 
	
	uint8_t char_count=0;
	
//	buffer[0]=0;	//since we don't know what data is present in that location we clear it. So that we don't have to use
					//any complicated tricks in the while condition.
	
	if (maxlen < 1)
	{
		return 0;
	}
	
	//delay(1000);
	
	while(Serial.available()>0 && char_count < maxlen)
	{
		buffer[char_count]=Serial.read();
		
		if (buffer[char_count]==token)
		{
			buffer[char_count]='\0';		//terminate the string
			return char_count;				//return characters read, doesn't include NULL character.
		}
		
		char_count++;
	}
	
	buffer[char_count]='\0';
	
	return 0;		//returning zero tells that token was not found however we have read whatever characters we could get 
					//from buffer, i.e. emptied the buffer or we just ran out of maxlength specified.
}

void replyOK()
{
	Serial.println("OK");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////