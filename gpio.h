#ifndef GPIO_H_
#define GPIO_H_

/**
 * This the high level gpio abstraction layer available in avrlib.
 *
 * Each instance of gpio handles a single pin of the microcontroller.
 * If the pin is PWM capable, it is only necessary to configure the
 * respective timer in config.h and then instantiate a gpio object
 * for the pin. The same logic is applicable for ADC pins.
 */

extern "C"
{
	#include <stdio.h>
	#include <avr/io.h>
	#include <avr/interrupt.h>
}

#include "HAL/HAL.h"

class Pin {
protected:
    // Variables with pin information
    volatile uint8_t *_port;
    uint8_t _pin;

public:
    Pin(volatile uint8_t *port, uint8_t pin);
    void setAsOutput();
    void setAsInput();
};

class AnalogPin : private Pin {
public:
    // Constructors
    AnalogPin(volatile uint8_t *port, uint8_t pin);

    // Reading function
    uint16_t rawRead();  // return raw analog pin voltage
    float read();        // return analog pin voltage
};

class DigitalPin : private Pin {
private:
    // Mode information
    uint8_t _mode;

public:
    // Constructors
    DigitalPin(volatile uint8_t *port, uint8_t pin, uint8_t mode, uint8_t state = LOW);

    // Set pin mode
    void setPinMode(uint8_t mode, uint8_t state = LOW);

    // Writing functions
    void writeHigh();
    void writeLow();
    void write(uint8_t state);
    void toggle();

    // Reading function
    uint8_t read();

    // Functions to deal with PCINT interrupt
    void enablePCINT();
    void disablePCINT();

    // Functions to deal with EXT_INT interrupt (usually INT0 and INT1)
    void enableINT(uint8_t sensible_edge = RISING_EDGE);
    void disableINT();
};

/**
 * PwmPin class
 *
 * Used to use a pin as PWM generator
 *
 * @param volatile uint8_t *port is a pointer to pin port (example, &PORTB)
 * @param uint8_t pin is what pin to use (example, PB1)
 * @param uint8_t mode is
 */
class PwmPin : private Pin {
private:

public:
    // Constructors
    PwmPin(volatile uint8_t *port, uint8_t pin, uint8_t duty = 0);

    // Function to PWM mode (only for PWM capable pins)
    void setDuty(uint16_t duty);
};



#endif /* GPIO_H_ */

