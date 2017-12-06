#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

extern "C"
{
	#include <avr/io.h>
	#include <avr/interrupt.h>
    #include "adc.h"
}
#include "defines.h"
#include "timers.h"

/**
 * @defgroup hal_gpio_group gpio
 *
 */

/**@{*/

/**
 * gpioAsOutput(volatile uint8_t *port, const uint8_t pin)
 *
 * Configures the \b pin placed on \b port as output (there is
 * support only for push-pull configuration)
 *
 * \todo Add support for open-drain configuration
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 */
extern void gpioAsOutput(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioAsInput(volatile uint8_t *port, const uint8_t pin, const uint8_t pullUp)
 *
 * Configures the \b pin placed on \b port as input
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 * @param pullUp defines if a pull-up resistor will be
 * added to the pin, use 1 to yes or 0 to no
 */
extern void gpioAsInput(volatile uint8_t *port, const uint8_t pin, const uint8_t pullUp = 0);

/**
 * gpioDirection(volatile uint8_t *port, const uint8_t pin, const uint8_t dir)
 *
 * Configures the \b pin placed on \b port as input
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 */
extern void gpioDirection(volatile uint8_t *port, const uint8_t pin, const uint8_t dir);

/**
 * gpioAsAdc(volatile uint8_t *port, const uint8_t pin)
 *
 * Configures the \b pin placed on \b port as ADC input.
 * It will automatically disable digital registers and
 * set the pin as INPUT (\b without the pull-up resistor)
 *
 * @param port is a pointer to the port of the pin (example: &PORTC)
 * @param pin is the pin number (example: PC0 or just 0)
 */
extern void gpioAsAdc(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioAsPwm(volatile uint8_t *port, const uint8_t pin)
 *
 * Configures the \b pin placed on \b port as PWM output
 *
 * @param port is a pointer to the port of the pin (example: &PORTD)
 * @param pin is the pin number (example: PC5 or just 5)
 */
extern void gpioAsPwm(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioWriteHigh(volatile uint8_t *port, const uint8_t pin)
 *
 * Write high logic level on the \b pin placed on \b port as output (there is
 * support only for push-pull configuration)
 *
 * \todo Add support for open-drain configuration
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 */
extern void gpioWriteHigh(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioWriteLow(volatile uint8_t *port, const uint8_t pin)
 *
 * Write low logic level on the \b pin placed on \b port as output (there is
 * support only for push-pull configuration)
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 */
extern void gpioWriteLow(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioWrite(volatile uint8_t *port, const uint8_t pin, const uint8_t level)
 *
 * Write logic \b level on the \b pin placed on \b port as output (there is
 * support only for push-pull configuration)
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 * @param level is the logic level to write (true or false, HIGH or LOW, 1 or 0)
 */
extern void gpioWrite(volatile uint8_t *port, const uint8_t pin, const uint8_t level);

/**
 * gpioToggle(volatile uint8_t *port, const uint8_t pin)
 *
 * Toggles logic level on the \b pin placed on \b port as output (there is
 * support only for push-pull configuration)
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 */
extern void gpioToggle(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioFastRead(volatile uint8_t *port, const uint8_t pin)
 *
 * Reads the logic level on the \b pin placed on \b port as output
 * This function is faster than @ref gpioFastRead because it returns
 * a positive integer (1, 2, 4, 8, 16, 32, 64 or 128) if the logic
 * level on the pin is HIGH. However, LOW level is always zero.
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 * @return a non-negative number with the logic level on the pin
 *         (if >= 1 then it is HIGH and 0 if LOW)
 */
extern uint8_t gpioFastRead(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioFast(volatile uint8_t *port, const uint8_t pin)
 *
 * Reads the logic level on the \b pin placed on \b port as output
 * This function is faster than @ref gpioFastRead because it returns
 * 1 if the logic level on the pin is HIGH and 0 if LOW level.
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 * @return a non-negative number with the logic level on the pin
 *         (1 if it is HIGH and 0 if LOW)
 */
extern uint8_t gpioRead(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioEnablePCINT(volatile uint8_t *port, const uint8_t pin)
 *
 * Enables the PCINT interrupt on the \b pin on \b port. This
 * function isn't available if the employed microcontroller
 * doesn't support this feature (ATmega8 doesn't support it,
 * for example).
 *
 * \warning This kind of interrupt will raise event signals
 *          for any logic change of any configured pins on
 *          the same port. For example, if enabled PB3 and
 *          PB6, any logic change in any of these pins will
 *          trigger the same interrupt. It is necessary to
 *          configure the ISR to detect the interested pin.
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 */
extern void gpioEnablePCINT(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioDisablePCINT(volatile uint8_t *port, const uint8_t pin)
 *
 * Disables the PCINT interrupt on the \b pin on \b port. This
 * function isn't available if the employed microcontroller
 * doesn't support this feature (ATmega8, for example)
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 */
extern void gpioDisablePCINT(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioEnableINT(volatile uint8_t *port, const uint8_t pin, const uint8_t edge = RISING_EDGE)
 *
 * Enables the INT interrupt on the \b pin on \b port. This
 * function isn't available if the employed pin, then it
 * doesn't work. It is worth nothing that ATmega328P has two
 * INT, the INT0 and INT1.
 *
 * It is possible to configure the sensible edge (RISING_EDGE
 * or FALLING_EDGE, respectively)
 *
 * @param port is a pointer to the port of the pin (example: &PORTD)
 * @param pin is the pin number (example: PD2 or just 2)
 * @param edge can be RISING_EDGE or FALLING_EDGE
 */
extern void gpioEnableINT(volatile uint8_t *port, const uint8_t pin, const uint8_t edge = RISING_EDGE);

/**
 * gpioDisableINT(volatile uint8_t *port, const uint8_t pin)
 *
 * Disables the INT interrupt on the \b pin on \b port.
 *
 * @param port is a pointer to the port of the pin (example: &PORTD)
 * @param pin is the pin number (example: PD2 or just 2)
 */
extern void gpioDisableINT(volatile uint8_t *port, const uint8_t pin);

/**
 * gpioSetDuty(volatile uint8_t *port, const uint8_t pin, const uint16_t duty)
 *
 * This function sets the duty cycle of a PWM channel.
 *
 * \todo Add support to software PWM to increase PWM channels on microcontroller
 *
 * \warning The \b duty variable is of type uint16_t. However, if the hardware
 *          supports only 8 bits, then it won't work as 16 bits PWM.
 *
 * @param port is a pointer to the port of the pin (example: &PORTB)
 * @param pin is the pin number (example: PB5 or just 5)
 * @param duty is the duty cycle value
 */
extern void gpioSetDuty(volatile uint8_t *port, const uint8_t pin, const uint16_t duty);

/**@}*/

#endif /* HAL_GPIO_H_ */
