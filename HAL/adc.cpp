/*
  adc.c - Library to handle the ADC module of AVR micro-controllers

  Copyright (c) 2013 - José Roberto Colombo Junior

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "adc.h"

void INIT_ADC(uint8_t pin) {
	// Enable the A/D converter
	ADCSRA = (1 << ADEN) | PREESCALE;
	ADMUX = ADC_REFERENCE | DATA_ALIGN;

	// Disable input digital buffer (saves power)
	#if defined (__AVR_ATmega328P__)
		DIDR0 = (1 << pin);
	#endif

	// TODO: add support to the free running mode
}

uint16_t ANALOG_READ(uint8_t pin) {
    // Select the pin
    CHANGE_ADMUX (pin);

	#if ADC_OPERATION_MODE == OFF
	    return 0;
    #elif ADC_OPERATION_MODE == SINGLE
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC)) {};
		CHANGE_ADMUX (0);
		return ADC;

	#elif ADC_OPERATION_MODE == NOISE_REDUCTION
		ADCSRA |= _BV(ADIE); // Generate an interrupt when conversion is ready
		set_sleep_mode(SLEEP_MODE_ADC);
		sleep_enable();

		do {
			sei();
			sleep_cpu();
			cli();
		}
		while (ADCSRA & (1<<ADSC));

		sleep_disable();
		sei();

		// Disable ADC interrupt
		ADCSRA &= ~ _BV(ADIE);
		return(ADC);

	#elif ADC_OPERATION_MODE == FREE_RUNNING
		return 0;
	#endif
}

void CHANGE_ADMUX (uint8_t pino) {
	ADMUX &= 0b11110000;
	ADMUX |= (pino & 0b00001111);
}

void SET_REFERENCE_VOLTAGE(uint8_t source)
{
	ADMUX = source;
}
