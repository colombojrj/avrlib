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
#include "HAL.h"

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

