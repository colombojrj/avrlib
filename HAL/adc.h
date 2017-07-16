/*
  adc.h - A/D converter library

  Copyright (c) 2012 - José Roberto Colombo Junior (colombojrj [at] gmail [dot] com)

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

#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "defines.h"
#include "../config.h"

//
// Internal defines (do not change them!)
//
#define ADC_OFF                 0
#define ADC_SINGLE_CONVERSION   1
#define ADC_NOISE_REDUCTION	    2
#define ADC_FREE_RUNNING        3
#define ADC_REF_EXT             0
#define ADC_REF_INT             ((1 << REFS1) | (1 << REFS0))
#define ADC_REF_VCC				(1 << REFS0)
#define ADC_DIV_BY_2			1
#define ADC_DIV_BY_4			2
#define ADC_DIV_BY_8			3
#define ADC_DIV_BY_16			4
#define ADC_DIV_BY_32			5
#define ADC_DIV_BY_64			6
#define ADC_DIV_BY_128			7
#define ADC_LEFT				(1 << ADLAR)
#define ADC_RIGHT				0

// Available functions
extern void INIT_ADC(uint8_t pin);
extern uint16_t ANALOG_READ(uint8_t pin);
extern void CHANGE_ADMUX(uint8_t pin);
extern void SET_REFERENCE_VOLTAGE(uint8_t source);
#if defined (__AVR_ATmega328P__)
	extern int16_t READ_TEMPERATURE();
#endif

#endif /* ADC_H_ */
