#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

#include "devices/device.h"
#include <avr/interrupt.h>
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
 * Configures the gpio pin as output (ATmega devices support only
 * push-pull configuration)
 *
 * @param gpio is a pointer to the pin port (example: PinB5)
 *
 * @see gpio_t
 *
 * @todo It is possible to emulated open-drain output configuration
 *       by employing the input mode with the pull-up resistor.
 */
#define gpioAsOutput(gpio) (*(*gpio).regs.direction=*(*gpio).regs.direction|(1<<(*gpio).pinNumber))

/**
 * gpioAsInput(gpio)
 *
 * Configures the gpio pin as input
 *
 * @param gpio is a pointer to the gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioAsInput(gpio) (*(*gpio).regs.direction=*(*gpio).regs.direction&(~(1<<(*gpio).pinNumber)))

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
#define gpioPullUpEnable(gpio) (*(*gpio).regs.port=*(*gpio).regs.port|(1<<(*gpio).pinNumber))

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
#define gpioPullUpDisable(gpio) (*(*gpio).regs.port=*(*gpio).regs.port&(~(1<<(*gpio).pinNumber)))

/**
 * Configures the gpio pin as input or output
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 * @param dir is the direction (input and output)
 *
 * @see gpioDir_t
 * @see gpio_t
 */
extern void gpioSetDir(const gpio_t* gpio, gpioDir_t dir);

/**
 * Write high logic level on the gpio pin configured as output (ATmega devices
 * there is support only push-pull configuration)
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioWriteHigh(gpio) (*(*gpio).regs.outputData=*(*gpio).regs.outputData|(1<<(*gpio).pinNumber))

/**
 * Write low logic level on the gpio pin configured as output (ATmega devices
 * there is support only push-pull configuration)
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioWriteLow(gpio) (*(*gpio).regs.outputData=*(*gpio).regs.outputData&(~(1<<(*gpio).pinNumber)))

/**
 * Toggles an output pin logic level
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
#define gpioToggle(gpio) (*(*gpio).regs.outputData=*(*gpio).regs.outputData^(1<<(*gpio).pinNumber))

/**
 * Returns the logic level on the gpio pin
 *
 * \warning This macro "returns" 0 for low logic level and a
 *          positive number for high logic level.
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 * @return 0 for low logic level or a positive number for high
 *         logic level
 *
 * @see gpio_t
 */
#define gpioRead(gpio) (*(*gpio).regs.inputData&(1<<(*gpio).pinNumber))

/**
 * Write logic level on the gpio pin configured as output (ATmega devices support
 * only push-pull configuration)
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 * @param level is the logic level to write (true or false, HIGH or LOW, 1 or 0)
 *
 * @see gpio_t
 */
extern void gpioWrite(const gpio_t* gpio, const uint8_t level);

/**
 * If supported by the employed device, this function will enable
 * the PCINT (pin change interrupt) on the pin.
 *
 * \warning This kind of interrupt raises interrupts for any logic
 *          change on any pin of the employed port (if enabled).
 *          For example, if enabled PinB3 and PinB6 have the PCINT
 *          enabled, then any logic change in any of these pins will
 *          trigger the interrupt.
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
extern void gpioEnablePCINT(const gpio_t* gpio);

/**
 * If supported by the employed device, this function will disable
 * the PCINT (pin change interrupt) on the pin.
 *
 * @param gpio is a pointer to gpio pin (example: PinB5)
 *
 * @see gpio_t
 */
extern void gpioDisablePCINT(const gpio_t* gpio);

/**
 * If supported, this function enables the INT interrupt on
 * the gpio pin.
 *
 * The trigger source of the employed device is specified through the
 * @ref gpioInt enum. Use the auto completion of your favorite IDE to show
 * you the available options.
 *
 * @warning The raised interrupt will call ISR(INTx_vect), where x is
 * the interrupt vector (Example: INT0_vect). It is important to note
 * that avrlib does nothing inside INTx_vect.
 *
 * @param gpio is a pointer to gpio pin (example: PinD0)
 * @param trigger is from gpioInt enum specifying the trigger source
 *
 * @see gpio_t
 * @see gpioInt
 */
extern void gpioEnableINT(gpio_t* gpio, gpioInt trigger);

/**
 * Disables the INT interrupt on the gpio pin.
 *
 * @param gpio is a pointer to gpio pin (example: PinD0)
 *
 * @see gpio_t
 */
extern void gpioDisableINT(gpio_t* gpio);

/**
 * This function sets the duty cycle of a PWM channel
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
