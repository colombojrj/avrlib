/*
 * gpio.h
 *
 *  Created on: 10/06/2017
 *      Author: junior
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timers.h"
#include "adc.h"

// This does not work for ATmega64 or higher
#define DDR(x) (*(&x - 1))
#define PIN(x) (*(&x - 2))

#define INPUT			0
#define INPUT_PULLUP	1
#define OUTPUT			2
#define ANALOG_INPUT	3	// TODO: add support
#define PWM				4	// adding support
#define PWM_8			PWM	//  8 bit PWM
#define PWM_9			5	//  9 bit PWM
#define PWM_10			6	// 10 bit PWM
#define PWM_11			7	// 11 bit PWM
#define PWM_12			8	// 12 bit PWM
#define PWM_13			9	// 13 bit PWM
#define PWM_14			10	// 14 bit PWM
#define PWM_15			11	// 15 bit PWM
#define PWM_16			12	// 16 bit PWM
#define LOW_LEVEL		13	// low level triggers external interrupt
#define ANY_CHANGE		14	// any logical change trigger external interrupt
#define RISING_EDGE		15	// rising edge triggers external interrupt
#define FALLING_EDGE	16	// falling edge triggers external interrupt

#define LOW				0
#define	HIGH			1

class gpio {
private:
	// Variables with pin information
	volatile uint8_t *_port;
	uint8_t _pin, _mode;

public:
	// Constructors
	gpio(volatile uint8_t *port, uint8_t pin, uint8_t mode, uint8_t state = LOW);

	// Set pin mode
	void set_pin_mode(uint8_t mode, uint8_t state = LOW);

	// Writing functions
	void write_high();
	void write_low();
	void write(uint8_t state);
	void toggle();

	// Reading function
	uint8_t digital_read();      // faster then read()
	uint16_t analog_raw_read();  // return raw analog pin voltage
	float analog_read();         // return analog pin voltage
	uint16_t read();             // slower, but generic (it decides if return analog or digital automatically)

	// Functions to deal with PCINT interrupt
	void attach_pcint_interrupt();
	void detach_pcint_interrupt();

	// Functions to deal with EXT_INT interrupt (usually INT0 and INT1)
	void attach_ext_int(uint8_t sensible_edge = RISING_EDGE);
	void detach_ext_int();

	// Function to PWM mode (only for PWM capable pins)
	void set_duty(uint16_t duty);

};



#endif /* GPIO_H_ */

