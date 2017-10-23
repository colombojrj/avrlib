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

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "HAL/HAL.h"

class Pin {
protected:
    // Variables with pin information
    uint8_t *_port;
    uint8_t _pin;

public:
    Pin(uint8_t *port, uint8_t pin);
    void setAsOutput();
    void setAsInput();
};

class AnalogPin : private Pin {
public:
    // Constructors
    AnalogPin(uint8_t *port, uint8_t pin);

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
    DigitalPin(uint8_t *port, uint8_t pin, uint8_t mode, uint8_t state = LOW);

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

class pwmPin {
private:
    // Variables with pin information
    volatile uint8_t *_port;
    uint8_t _pin;

public:
    // Constructors
    pwmPin(volatile uint8_t *port, uint8_t pin, uint8_t mode, uint8_t state = LOW);

    // Function to PWM mode (only for PWM capable pins)
    void setDuty(uint16_t duty);
};

/// Old class, kept by compatibility issues (it will be removed soon)
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

