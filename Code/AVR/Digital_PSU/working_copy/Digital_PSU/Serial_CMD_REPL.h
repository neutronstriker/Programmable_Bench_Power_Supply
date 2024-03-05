/*
 * Serial_CMD_REPL.h
 * All commands should be followed by a CR (carriage return) only. If you send both LF (line feed) and CR it won't recognise those commands
 * Created: 8/10/2019 12:14:22 AM
 *  Author: neutron
 */ 


#ifndef SERIAL_CMD_REPL_H_
#define SERIAL_CMD_REPL_H_


#include "project_config.h"
#include "Arduino.h"
#include "neo_core_aux.h"
#include "GPIO.h"
#include "INA260/INA260.h"
#include "Rotary_Encoder.h"
#include "PowerModule.h"
#include "fanCtrl.h"

#define CMD_REPLY_BUFFER_LENGTH		SERIAL_BUFFER_SIZE_NEO		//initially it was 32, but in order to accommodate i2cwrite instruction, it has been increased to 64.

extern Rotary_Encoder voltage_knob;
extern Rotary_Encoder current_knob;
extern GPIO lcd_bl;
extern GPIO output_enable_relay;
//extern INA260 ina;
extern float busPower, shuntCurrent, busVoltage;

extern uint8_t lock_controls;

//extern uint8_t setPwmFrequency(int pin, int divisor);
//extern float ntcThermistorTemp();
//extern uint8_t fanSpeed(uint8_t speedVal);
//extern GPIO fan;
//extern uint8_t fanOverride, uint8_t fanspeed_val;



void readSerialCmd();
uint8_t readUntilToken(char* buffer, uint8_t token, uint8_t maxlen);
void replyOK();
void usage();



#endif /* SERIAL_CMD_REPL_H_ */