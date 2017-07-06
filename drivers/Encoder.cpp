/*
 * Encoder.cpp
 *
 *  Created on: 05/07/2017
 *      Author: junior
 */

#include "Encoder.h"

namespace drivers {

OnePinEncoder::OnePinEncoder(gpio *pin, uint8_t int_type, uint8_t what_timer) {
	// Initilize internal variables
	_frequency = 0.;
	_stopped = 1;

	// Initialize the hardware
	_pin = pin;
	_what_timer = what_timer;

	// Set interrupt on pin
	if (int_type == EXT_INT) 	_pin->attach_ext_int(ANY_CHANGE);
	else 						_pin->attach_pcint_interrupt();

	// Initialize the timer
	if (_what_timer == TIMER0A)
	{
		INIT_TIMER0A();
		_to_seconds = 1./calculate_timer_clock(TIMER0_CLOCK);
	}
	else if (_what_timer == TIMER0B)
	{
		INIT_TIMER0B();
		_to_seconds = 1./calculate_timer_clock(TIMER0_CLOCK);
	}
	else if (_what_timer == TIMER1A)
	{
		INIT_TIMER1A();
		_to_seconds = 1./calculate_timer_clock(TIMER1_CLOCK);
	}
	else if (_what_timer == TIMER1B)
	{
		INIT_TIMER1B();
		_to_seconds = 1./calculate_timer_clock(TIMER1_CLOCK);
	}
	else if (_what_timer == TIMER2A)
	{
		INIT_TIMER2A();
		_to_seconds = 1./calculate_timer_clock(TIMER2_CLOCK);
	}
	else if (_what_timer == TIMER2B)
	{
		INIT_TIMER2B();
		_to_seconds = 1./calculate_timer_clock(TIMER2_CLOCK);
	}
}

float OnePinEncoder::calculate_timer_clock(uint8_t timer_clk)
{
	switch (timer_clk)
	{
		case NO_PREESCALE: 	return (float) F_CPU;
		case CLK_8:			return (float) F_CPU/8;
		case CLK_32:		return (float) F_CPU/32;
		case CLK_64:		return (float) F_CPU/64;
		case CLK_128:		return (float) F_CPU/128;
		case CLK_256:		return (float) F_CPU/256;
		case CLK_1024:		return (float) F_CPU/1024;
		default:			return 0.0;					// NO_CLOCK
	}
}

uint16_t OnePinEncoder::get_timer_count()
{
	uint16_t count;
	if ((_what_timer == TIMER0A) || (_what_timer == TIMER0B))
	{
		count = TCNT0;
		TCNT0 = 0;
	}
	else if ((_what_timer == TIMER1A) || (_what_timer == TIMER1B))
	{
		count = TCNT1;
		TCNT1 = 0;
	}
	else
	{
		count = TCNT2;
		TCNT2 = 0;
	}

	return count;
}

void OnePinEncoder::update(uint8_t stopped)
{
	// Get timer count
	uint16_t count = get_timer_count();

	// Is the encoder stopped?
	if ((_stopped == 1) && (stopped == 1))
	{
		_frequency = 0;
	}
	else if ((_stopped == 0) && (stopped == 0))
	{
		_frequency = 1./(_to_seconds*count);
	}
	_stopped = stopped;
}

float OnePinEncoder::get_frequency()
{
	return _frequency;
}

void OnePinEncoder::set_frequency(float f)
{
	_frequency = f;
}

uint8_t OnePinEncoder::get_stopped()
{
	return _stopped;
}

} /* namespace drivers */


