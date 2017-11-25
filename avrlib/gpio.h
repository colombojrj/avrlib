#ifndef GPIO_H_
#define GPIO_H_

/**
 * This the high level gpio abstraction layer available in avrlib.
 *
 * Each instance of these classes can handle a single pin of the
 * microcontroller. If the pin is PWM capable, it is only necessary
 * to configure the respective timer in @ref config.h and then
 * instantiate a PwmPin object for the pin. The same logic is
 * applicable for ADC pins (@ref AnalogPin) and general purpose I/O
 * (@ref DigitalPin).
 */

extern "C"
{
	#include <stdio.h>
	#include <avr/io.h>
	#include <avr/interrupt.h>
}

#include "HAL/HAL.h"
#include "config.h"

/**
 * This is the mother class of all other classes. The @ref Pin has
 * the purpose of being extremely generic, having only very few
 * methods. But it stores two very important variables:
 * * _port
 * * _pin
 *
 * These variables are protected and can not be accessed outer this
 * class.
 *
 * It is worth noting this class alone is almost useless, since it
 * can not manipulate digital or analog signals. But it will be used
 * as base to another classes:
 * * @ref AnalogPin
 * * @ref DigitalPin
 * * @ref PwmPin
 *
 * Since the ATmega microcontrollers that I work doesn't have a DAC
 * module, the AnalogPin can only read analog signals.
 */
class Pin {
protected:
    volatile uint8_t *_port;
    const uint8_t _pin;

public:
    /**
     * Constructor
     *
     * @param port is a pointer to the port of the pin (example: &PORTB)
     * @param pin is the pin number (example PB5 or just 5)
     */
    Pin(volatile uint8_t *port, uint8_t pin);

    /**
     * setAsOutput configures the \b pin number (\b _pin) placed on \b
     * _port as output. This method is built on the @ref gpioAsOutput
     * function available in HAL.
     *
     * @see HAL/gpio.h
     */
    void setAsOutput();

    /**
     * setAsInput configures \b pin number (\b _pin) placed on \b
     * _port as input. This method is built on the @ref gpioAsInput
     * function available in HAL.
     *
     * @param withPullUp can be used to decide if the pull-up
     *        resistor will be used with the pin. If any argument
     *        is provided, then the default value is to keep the
     *        resistor disabled.
     *
     * @see HAL/gpio.h
     */
    void setAsInput(uint8_t withPullUp = 0);
};

/**
 * AnalogPin is a class derived from @ref Pin. This class has the
 * purpose of handling the ADC. The idea is to provide a simple
 * interface for configuring a single pin as analog input.
 *
 * \warning Do not forget to enable the ADC module in the @ref
 *          config.h file. This class won't work properly if the
 *          ADC is disabled!
 *
 * Since the ATmega microcontrollers that I work does not have
 * a DAC module, this class supports only reading analog signals.
 */
class AnalogPin : private Pin {
public:
    /**
     * Constructor
     *
     * This method will automatically initialize the gpio as
     * analog input. For this, it will set the pin as input
     * without the pull-up resistor and then call the
     * @ref gpioAsAdc function, which may disable (if supported
     * by the microcontroller) the digital registers, trying to
     * minimize possible noise on the measurements.
     *
     * @param port is a pointer to the port of the pin (example: &PORTC)
     * @param pin is the pin number (example PC2 or just 2)
     */
    AnalogPin(volatile uint8_t *port, uint8_t pin);

    /**
     * rawRead() is a method to read the analog value attached to the
     * \b pin on \b port.
     *
     * @return the raw value, i.e., a non-negative number from 0 until
     *         1023 (or the ADC resolution).
     */
    uint16_t rawRead();

    /**
     * read() is a simple method to read a analog value attached to
     * the \b pin on \b port. Differently from @ref read, this method
     * returns a float number representing the actual measured
     * Voltage in the pin.
     *
     * \warning If the configuration in @ref config.h is wrong, then
     *          unsafe values will be returned by this method.
     *
     * @return a float number which represents the measured voltage
     *         in the pin.
     */
    float read();

    /**
     * safeRead() is a more complex (and slower) method than read().
     * It does the following: the designer decided to use the 1.1V
     * internal reference voltage. However, due bad calculation or
     * any other factor, the pin voltage is 1.2V and read would
     * return only 1.1V, which contains an error.
     *
     * This method implements the following algorithm:
     * * run the read method
     * * if readed value is smaller than 1.1V, then return it
     * * else change reference voltage to supply voltage (usually
     *   3.3V or 5V) and read it again and return the new read value.
     *
     * @return a float number which represents the measured voltage
     *         in the pin.
     */
    float safeRead();
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

