#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

extern "C"
{
	#include <avr/interrupt.h>
}
#include "defines.h"
#include "timers.h"
#include "adc.h"

/**
 * @defgroup hal_gpio_group gpio
 *
 * Some functions were replaced with macros. The main advantage is
 * the code size (macros use less flash memory)
 *
 * See https://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_port_pass
 */

/**@{*/

/**
 * gpioAsOutput(gpio)
 *
 * Configures the pin placed on port as output (ATmega devices support only
 * push-pull configuration)
 *
 * @param gpio is a pointer to the pin port (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioAsOutput(gpio) (*(*gpio).ddr=*(*gpio).ddr|(1<<(*gpio).pinNumber))

/**
 * gpioAsInput(gpio)
 *
 * Configures the gpio pin as input
 *
 * @param gpio is a pointer to the gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioAsInput(gpio) (*(*gpio).ddr=*(*gpio).ddr&(~(1<<(*gpio).pinNumber)))

/**
 * gpioPullUpEnable(gpio)
 *
 * Enable gpio pull up resistor.
 *
 * @param gpio is a pointer to gpio pin whose pull up
 *        resistor will be enabled (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioPullUpEnable(gpio) (*(*gpio).port=*(*gpio).port|(1<<(*gpio).pinNumber))

/**
 * gpioPullUpDisable(gpio)
 *
 * Disable gpio pull up resistor.
 *
 * @param gpio is a pointer to gpio pin whose pull up
 *        resistor will be disabled (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioPullUpDisable(gpio) (*(*gpio).port=*(*gpio).port&(~(1<<(*gpio).pinNumber)))

/**
 * gpioDirection(volatile uint8_t *gpio, const uint8_t dir)
 *
 * Configures the gpio pin as input or output
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 * @param dir is the direction (0 is input and 1 is output)
 *
 * @todo migrate from const uint8_t dir to enum
 *
 * @see gpio_t
 */
extern void gpioDirection(gpio_t* gpio, const uint8_t dir);

/**
 * Macro for gpio writing
 *
 * gpioWriteHigh(gpio)
 *
 * Write high logic level on the gpio pin configured as output (ATmega devices
 * there is support only push-pull configuration)
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioWriteHigh(gpio) (*(*gpio).port=*(*gpio).port|(1<<(*gpio).pinNumber))

/**
 * Macro for gpio writing
 *
 * gpioWriteLow(gpio)
 *
 * Write low logic level on the gpio pin configured as output (ATmega devices
 * there is support only push-pull configuration)
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioWriteLow(gpio) (*(*gpio).port=*(*gpio).port&(~(1<<(*gpio).pinNumber)))

/**
 * Macro for gpio writing
 *
 * gpioToggle(gpio)
 *
 * Toggles an output pin logic level
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioToggle(gpio) (*(*gpio).port=*(*gpio).port^(1<<(*gpio).pinNumber))

/**
 * Macro for gpio reading
 *
 * uint8_t level = gpioFast(gpio)
 *
 * Reads the logic level on the gpio pin
 *
 * \warning This macro "returns" 0 for low logic level or a
 * positive number for high logic level
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 * @return a non-negative number with the logic level on the pin
 *         (0 if LOW or a positive number if HIGH)
 *
 * @see gpio_t
 */
#define gpioRead(gpio) (*(*gpio).pin&(1<<(*gpio).pinNumber))

/**
 * gpioWrite(gpio_t* gpio, const uint8_t level)
 *
 * Write logic level on the gpio pin configured as output (ATmega devices support
 * only push-pull configuration)
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 * @param level is the logic level to write (true or false, HIGH or LOW, 1 or 0)
 */
extern void gpioWrite(gpio_t* gpio, const uint8_t level);

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
#if defined (PCMSK0) || defined(PCMSK1) || defined(PCMSK2)
extern void gpioEnablePCINT(volatile uint8_t *port, const uint8_t pin);
#endif

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
#if defined (PCMSK0) || defined(PCMSK1) || defined(PCMSK2)
extern void gpioDisablePCINT(volatile uint8_t *port, const uint8_t pin);
#endif

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
extern void gpioEnableINT(volatile uint8_t *port, const uint8_t pin, const gpioInt_t trigger);

/**
 * Disables the INT interrupt on the pin on port.
 *
 * @param gpio is a pointer to gpio pin (example: PinD0)
 *
 * @warning If the informed gpio pin has no support to interrupt, then
 *          nothing happens. Please consult the microcontroller datasheet
 *          for more information about the pins with this kind of
 *          interruption.
 */
extern void gpioDisableINT(gpio_t* gpio);

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
