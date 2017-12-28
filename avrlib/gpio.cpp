#include "gpio.h"

// Pin class constructor
Pin::Pin(volatile uint8_t *port, uint8_t pin) :
        _port(port), _pin(pin) {}

void Pin::setAsOutput()
{
    gpioAsOutput(_port, _pin);
}

void Pin::setAsInput(uint8_t withPullUp)
{
    gpioAsInput(_port, _pin, withPullUp);
}

// AnalogPin constructor
AnalogPin::AnalogPin(volatile uint8_t *port, uint8_t pin) :
        Pin(port, pin)
{
	// Set pin as input
	gpioAsAdc(port, pin);
}

uint16_t AnalogPin::rawRead()
{
	return adcRead(_pin);
}

float AnalogPin::read()
{
	float reading = adcRead(_pin);
	return (ADC_REFERENCE_VOLTAGE*reading)/1023.;
}

// DigitalPin constructor
DigitalPin::DigitalPin(volatile uint8_t *port, uint8_t pin, gpioConfig_t mode, uint8_t state) :
        Pin(port, pin)
{
    // Configure the pin
    setPinMode(mode, state);
}

void DigitalPin::setPinMode(gpioConfig_t mode, uint8_t state)
{
    // Pin configuration
    if (mode == gpioConfig_t::output)
    {
        setAsOutput();
        gpioWrite(_port, _pin, state);
    }
    else // i.e., INPUT or INPUT_PULLUP
    {
        if (mode == gpioConfig_t::inputWithPullUp)
            setAsInput(1);
        else
            setAsInput(0);
    }
}

void DigitalPin::writeHigh()
{
    gpioWriteHigh(_port, _pin);
}

void DigitalPin::writeLow()
{
    gpioWriteLow(_port, _pin);
}

void DigitalPin::write(uint8_t state)
{
    if (state == LOW)
        gpioWriteLow(_port, _pin);
    else
        gpioWriteHigh(_port, _pin);
}

/*
 * Function to toggle the state of an output pin
 *
 */
void DigitalPin::toggle()
{
    gpioToggle(_port, _pin);
}

/*
 * Function to read a pin
 *
 * Returns logic state of the pin (HIGH or LOW)
 */
uint8_t DigitalPin::read()
{
    return gpioRead(_port, _pin);
}

void DigitalPin::enablePCINT()
{
    gpioEnablePCINT(_port, _pin);
    sei();
}

void DigitalPin::disablePCINT()
{
    gpioDisablePCINT(_port, _pin);
}

void DigitalPin::enableINT(gpioInt_t trigger)
{
    gpioEnableINT(_port, _pin, trigger);
}

void DigitalPin::disableINT()
{
    gpioDisableINT(_port, _pin);
}


PwmPin::PwmPin(volatile uint8_t *port, uint8_t pin, uint8_t duty) :
		Pin(port, pin)
{
    gpioAsPwm(port, pin);
}

void PwmPin::setDuty(uint16_t duty)
{
	gpioSetDuty(_port, _pin, duty);
}
