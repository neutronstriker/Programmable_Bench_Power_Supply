/*
 * rotary_encoder_no_class.h
 *
 * Created: 7/25/2019 1:03:50 AM
 *  Author: neutron
 */ 


#ifndef ROTARY_ENCODER_NO_CLASS_H_
#define ROTARY_ENCODER_NO_CLASS_H_

#include "fifo.h"
#include <util/delay.h>

#define ENCODER_COUNT_MAX	18000
#define ENCODER_COUNT_MIN	1400
#define	ENCODER_INIT_VALUE	3300
#define	ENCODER_STEP_SIZE	100

#define	ENCODER_PORT_DIR_REG	DDRD
#define	ENCODER_PORT_WRITE_REG	PORTD
#define	ENCODER_PORT_READ_REG	PIND
#define	ENCODER_CLK_PIN			4
#define ENCODER_DATA_PIN		5

#define ENCODER_PIN_CHANGE_INT_MASK_REG		PCMSK2
#define ENCODER_PIN_CHANGE_INT_NUMBER		PCIE2

#define ENCODER_ISR_VECTOR	PCINT2_vect

//re-entry blocking delay
#define ISR_STABILIZATION_DELAY_us	0 //100
#define	ISR_STABILIZATION_DELAY_ms	0 //50 //its useless it limits the speed not the dual flips per click

#define CLOCK		2
#define ANTICLOCK	3

void init_encoder();

void increase();
void decrease();


#endif /* ROTARY_ENCODER_NO_CLASS_H_ */