/*
 * Encoder.h
 *
 *  Created on: 05/07/2017
 *      Author: junior
 */

#ifndef AVRLIB_DRIVERS_ENCODER_H_
#define AVRLIB_DRIVERS_ENCODER_H_

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include "../avrlib.h"
#include "../HAL/HAL.h"

// Interrupt types
#define EXT_INT		0
#define PC_INT		1

namespace drivers {

/*
 * OnePinEncoder
 *
 * Make sure to configure the respective timer as NORMAL operation
 *
 */
class OnePinEncoder {
private:
	gpio *_pin;
	uint8_t _what_timer;
	float _to_seconds;
	float _frequency;
	volatile uint8_t _stopped;

	// Internal methods
	float calculate_timer_clock(uint8_t timer_clk);
	uint16_t get_timer_count();

public:
	OnePinEncoder(gpio *pin, uint8_t int_type, uint8_t what_timer);

	// Available methods
	void update(uint8_t stopped);
	float get_frequency();
	void set_frequency(float f);
	uint8_t get_stopped();
};

} /* namespace drivers */

#endif /* AVRLIB_DRIVERS_ENCODER_H_ */
