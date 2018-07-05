#ifndef AVRLIB_H_
#define AVRLIB_H_

/**
 *  @defgroup avrlib
 *  @code #include <avrlib.h> @endcode
 *
 *  This the main avrlib file. By including this file, all the abstraction
 *  functions, data structures (and more) from this library will be available
 *  on your code.
 *
 *  AVRLIB is written in C/C++ and is divided in three layers:
 *  - Device: microcontroller registers have funny names and change from device
 *            to device. This layer defines standard names and data structures
 *            for these registers
 *  - HAL: initializes and work with hardware. This layer accesses the data
 *         structures defined on device (raw registers are accessed directly)
 *
 *  @author José Roberto Colombo Junior colombojrj@gmail.com
 */

/**@{*/

/*!
 *
 * \mainpage avrlib
 *
 * @section welcome AVRLIB a high level library for AVR micro-controllers
 *
 * @subsection supported_hardware Supported micro-controllers
 *
 * Actually the following microcontrollers are supported:
 * * ATmega328P (recall that I develop this library for free)
 *
 * @subsection recommendations General recomendations
 *
 * It is worth noting that this documentation has the purpose of guiding
 * the developer only. It doesn't replace the original AVR micro-controller
 * datasheet.
 *
 */

#define AVRLIB

#include <util/delay.h>
#include "devices/device.h"

/**@}*/

#endif /* AVRLIB_H_ */
