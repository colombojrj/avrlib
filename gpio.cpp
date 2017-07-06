/*
 * gpio.cpp
 *
 *  Created on: 10/06/2017
 *      Author: junior
 */

#include "gpio.h"

gpio::gpio(volatile uint8_t *port, uint8_t pin, uint8_t mode, uint8_t state)
{
	// Initialize variables
	_port = port;
	_pin  = pin;
	_mode = mode;

	// Configure the pin
	set_pin_mode(mode, state);
}

void gpio::set_pin_mode(uint8_t mode, uint8_t state)
{
	// Pin configuration
	if  (_mode == INPUT)
	{
		DDR(*_port) = DDR(*_port) & ~(1 << _pin);
	}
	else if (_mode == INPUT_PULLUP)
	{
		DDR(*_port)  = DDR(*_port)  & ~(1 << _pin);
		*_port = *_port | (1 << _pin);
	}
	else if (_mode == OUTPUT)
	{
		DDR(*_port) = DDR(*_port) | (1 << _pin);
		write(state);
	}
	else if (_mode == PWM)
	{
		// Configure pin as output
		DDR(*_port) = DDR(*_port) | (1 << _pin);

		// Timer configuration
		#if defined (__AVR_ATmega328P__)
			if (*_port == PORTD)
			{
				if (_pin == PD6)
				{
					INIT_TIMER0A();
				}
				else if (_pin == PD5)
				{
					INIT_TIMER0B();
				}
			}
			else if (*_port == PORTB)
			{
				if (_pin == PB1)
				{
					INIT_TIMER1A();
				}
				else if (_pin == PB2)
				{
					INIT_TIMER1B();
				}
			}
		#endif
	}
	else if (_mode == ANALOG_INPUT)
	{
		// Set pin as input
		DDR(*_port) = DDR(*_port) & ~(1 << _pin);

		// Initialize the A/D converter
		INIT_ADC(_pin);
	}
}

void gpio::write(uint8_t state)
{
	if (state == HIGH)
	{
		*_port = *_port | (1 << _pin);
	}
	else
	{
		*_port = *_port & ~(1 << _pin);
	}
}

/*
 * Function to toggle the state of an output pin
 *
 */
void gpio::toggle()
{
	*_port = *_port ^ (1 << _pin);
}

/*
 * Function to read a pin
 *
 * Returns logic state of the pin (1 or 0)
 */
uint8_t gpio::digital_read()
{
	return (uint8_t) ((PIN(*_port) & (1 << _pin)) >> _pin);
}

/*
 * Function to read the raw analog value on the pin
 *
 * Returns the analog value (uint16_t)
 */
uint16_t gpio::analog_raw_read()
{
	return ANALOG_READ(_pin);
}

/*
 * Function to read the analog value on the pin
 *
 * Returns the analog value (float)
 */
float gpio::analog_read()
{
	float reading = ANALOG_READ(_pin);
	return (ADC_REFERENCE_VOLTAGE*reading)/1023.;
}

// Generic function to read the pin value
uint16_t gpio::read()
{
	if (_mode == ANALOG_INPUT)
	{
		return analog_read();
	}
	else
	{
		return digital_read();
	}
}

/*
 * Function to attach interrupt to the pin
 *
 */
void gpio::attach_pcint_interrupt()
{
	if (*_port == PORTB)
	{
		PCICR = PCICR | (1 << PCIE0);
		PCMSK0 = PCMSK0 | (1 << _pin);
	}
	else if (*_port == PORTC)
	{
		PCICR = PCICR | (1 << PCIE1);
		PCMSK1 = PCMSK1 | (1 << _pin);
	}
	else if (*_port == PORTD)
	{
		PCICR = PCICR | (1 << PCIE2);
		PCMSK2 = PCMSK2 | (1 << _pin);
	}

	sei();
}

void gpio::detach_pcint_interrupt()
{
	if (*_port == PORTB)
	{
		PCMSK0 = PCMSK0 | (1 << _pin);
	}
	else if (*_port == PORTC)
	{
		PCMSK1 = PCMSK1 & ~(1 << _pin);
	}
	else if (*_port == PORTD)
	{
		PCMSK2 = PCMSK2 & ~(1 << _pin);
	}
}

void gpio::attach_ext_int(uint8_t sensible_edge)
{
	if (_pin == PD2)
	{
		if (sensible_edge == LOW_LEVEL)
		{
			EICRA &= ~((1 << ISC01) | (1 << ISC00));
		}
		else if (sensible_edge == ANY_CHANGE)
		{
			EICRA &= ~(1 << ISC01);
			EICRA |= (1 << ISC00);
		}
		else if (sensible_edge == FALLING_EDGE)
		{
			EICRA |= (1 << ISC01);
			EICRA &= ~(1 << ISC00);
		}
		else
		{
			EICRA |= (1 << ISC01) | (1 << ISC00);
		}
		EIMSK |= (1 << INT0);
	}
	else if (_pin == PD3)
	{
		if (sensible_edge == LOW_LEVEL)
		{
			EICRA &= ~((1 << ISC11) | (1 << ISC10));
		}
		else if (sensible_edge == ANY_CHANGE)
		{
			EICRA &= ~(1 << ISC11);
			EICRA |= (1 << ISC10);
		}
		else if (sensible_edge == FALLING_EDGE)
		{
			EICRA |= (1 << ISC11);
			EICRA &= ~(1 << ISC10);
		}
		else
		{
			EICRA |= (1 << ISC11) | (1 << ISC10);
		}
		EIMSK |= (1 << INT1);
	}
	sei();
}
void gpio::detach_ext_int()
{

}

// TODO: fix extreme values of PWM on PORTD (as done in PORTB)
void gpio::set_duty(uint16_t duty)
{
	#if defined (__AVR_ATmega328P__)
		if (*_port == PORTD)
		{
			if (_pin == PD6)
			{
				TIMER0A_SET_OCR(duty & 0x00FF);
			}
			else if (_pin == PD5)
			{
				TIMER0B_SET_OCR(duty & 0x00FF);
			}

		}
		else if (*_port == PORTB)
		{
			if (_pin == PB1) // A channel
			{
				if (duty == 255)
				{
					TCCR1A = TCCR1A & ~(1 << COM1A1);
					#if TIMER1A_POLATIRY == NORMAL
						write(1);
					#else
						write(0);
					#endif
				}
				else if (duty == 0)
				{
					TCCR1A = TCCR1A & ~(1 << COM1A1);
					#if TIMER1A_POLATIRY == NORMAL
						write(0);
					#else
						write(1);
					#endif
				}
				else
				{
					TCCR1A = TCCR1A | (1 << COM1A1);
					TIMER1A_SET_OCR(duty);
				}
			}
			else if (_pin == PB2) // B channel
			{
				if (duty == 255)
				{
					TCCR1A = TCCR1A & ~(1 << COM1B1);
					#if TIMER1B_POLATIRY == NORMAL
						write(1);
					#else
						write(0);
					#endif
				}
				else if (duty == 0)
				{
					TCCR1A = TCCR1A & ~(1 << COM1B1);
					#if TIMER1B_POLATIRY == NORMAL
						write(0);
					#else
						write(1);
					#endif
				}
				else
				{
					TCCR1A = TCCR1A | (1 << COM1B1);
					TIMER1B_SET_OCR(duty);
				}
			}
		}
	#endif
}


