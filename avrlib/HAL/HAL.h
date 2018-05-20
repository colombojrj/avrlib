#ifndef AVRLIB_HAL_H_
#define AVRLIB_HAL_H_

/**
 * @defgroup hal_group HAL (Hardware Abstraction Layer)
 *
 * This library can be imported separately by invoking:
 *
 * @code #include <avrlib/HAL.h> @endcode
 *
 * @brief A simple Hardware Abstraction Layer for the
 *        ATmega microcontrollers
 *
 * This library implements a low level API to handle
 * the ATmega microcontrollers internal peripherals.
 * It is not necessary to use the high level API (i.e.,
 * the classes @ref Pin, @ref DigitalPin, @ref AnalogPin
 * and @ref PwmPin.
 *
 * Right now there is support for:
 * * gpio.h (reading, writing and interruptions)
 * * adc.h (reading analog values)
 * * timers.h (with PWM generation)
 * * spi.h
 * * uart.h (thanks to Peter Fleury)
 *
 * The classes implementation are totally independent of
 * these low level drivers. These drivers were built with
 * the following goals:
 *
 * * Make easier to program these microcontrollers
 * * Build faster prototypes
 * * Serve as class material for my classes
 * * Use as little flash memory as possible
 *
 * With the purpose of using as little flash memory as
 * possible, it is assumed that:
 *
 * * The microcontroller has enough peripherals for the
 *   desired application
 * * The peripherals are configured just once (for example:
 *   if TIMER0 is used as timer ticker, then it is just the
 *   timer ticker It will not be turned off and then begin as PWM
 *   generator). It is worth noting that \b you can do it
 *   (i.e., turn it off and reconfigure it). But <b> there
 *   are no </b> helper functions in this library to assist
 *   you for this purpose
 * * This is the main motivation to the @ref config.h file, where
 *   the designer can configure all the peripherals of the
 *   microcontroller. But that configuration is static
 */

/**@{*/

//namespace HAL
//{

#include "defines.h"
#include "gpio.h"
#include "timers.h"
#include "uart.h"
#include "adc.h"
#include "spi.h"

/**
 * initHAL()
 *
 * @brief This function has the purpose of configuring the
 *        microcontrollers peripherals, as specified in the
 *        \ref config.h file.
 *
 * Make sure to run this function in the main program before
 * start to use the microcontroller peripherals.
 */
extern void initHAL();

//} // namespace HAL

/**@}*/

#endif /* AVRLIB_HAL_H_ */
