/*
  adc.c - Library to handle the ADC module of AVR microcontrollers

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

void adcSetClock()
{
    ADCSRA = ADCSRA & ~static_cast<uint8_t>(adcClock_t::setState);
    ADCSRA = ADCSRA | static_cast<uint8_t>(adcClock);
}

void adcSetRefVoltage(adcRefVoltage_t adcRefVoltage)
{
    // Clear reference voltage configuration
    ADMUX = ADMUX & ~static_cast<uint8_t>(adcRefVoltage_t::setState);

    // Load reference voltage configuration
    ADMUX = ADMUX | static_cast<uint8_t>(adcRefVoltage);
}

void adcSetDataAlign(adcDataAlign_t config)
{
    ADMUX = ADMUX & ~static_cast<uint8_t>(adcDataAlign_t::setState);
    ADMUX = ADMUX | static_cast<uint8_t>(adcDataAlign);
}

void adcInit(uint8_t pin)
{
	// Enable the A/D converter
    ADCSRA = (1 << ADEN);
    adcSetClock();
    ADMUX = 0;
    adcSetRefVoltage(adcReferenceConfig);
    adcSetDataAlign(adcDataAlign);

    #if defined (PRR)
        // Disable input digital buffer (saves power)
        DIDR0 = (1 << pin);

        // Disable power reduction
        PRR = PRR & ~(1 << PRADC);
    #endif

    // TODO: add support to free running operation mode
    if (adcConfig == adcConfig_t::freeRunning)
    {
    }
}

uint16_t adcRead(uint8_t pin)
{
    if (adcConfig == adcConfig_t::off)
    {
        return 0;
    }

    // Select the pin
    adcChangeAdmux (pin);

    if (adcConfig == adcConfig_t::singleConversion)
    {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC)) {};
		adcChangeAdmux (0);
		return ADC;
    }

	else if (adcConfig == adcConfig_t::noiseReduction)
	{
		// TODO before enter in sleep mode, it is necessary to wait the uart to finish actual transmission

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
		ADCSRA &= ~_BV(ADIE);
		return(ADC);
	}
    // TODO add support to free running mode
	else // i.e., adcConfig == freeRunning
	{
		return 1;
	}
}

void adcChangeAdmux (uint8_t pin)
{
	ADMUX &= 0b11110000;
	ADMUX |= (pin & 0b00001111);
}

int16_t adcReadTemperature()
{
    #if defined (__AVR_ATmega328P__)
        return adcRead(8);
    #else
        return 0;
    #endif
}

