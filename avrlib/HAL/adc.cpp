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

    // Scan mode requires interrupt
    if (adcConfig == adcConfig_t::scanMode)
    {
        ADCSRA = ADCSRA | (1 << ADIE);
        sei();
        ADCSRA = ADCSRA | (1 << ADSC);
    }
}

uint16_t adcRead(uint8_t pin)
{
    uint16_t convertedValue = 0;
    if (adcConfig != adcConfig_t::off)
    {
        // Select the pin
        adcChangeAdmux(pin);

        if (adcConfig == adcConfig_t::singleConversion)
        {
            ADCSRA |= (1 << ADSC);
            while (ADCSRA & (1 << ADSC)) {};
            adcChangeAdmux(0);
            convertedValue = ADC;
        }

        else if (adcConfig == adcConfig_t::noiseReduction)
        {
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
            convertedValue = ADC;
        }

        else // i.e., adcConfig == freeRunning
        {
            convertedValue = 1;
        }
    }

    return convertedValue;
}

void adcChangeAdmux (uint8_t pin)
{
	ADMUX = ADMUX & ~static_cast<uint8_t>(adcAdmux_t::setState);
	ADMUX = ADMUX | (pin & static_cast<uint8_t>(adcAdmux_t::setState));
}

int16_t adcReadTemperature()
{
    #if defined (__AVR_ATmega328P__)
        return adcRead(static_cast<uint8_t>(adcAdmux_t::temperatureSensor));
    #else
        return 0;
    #endif
}

