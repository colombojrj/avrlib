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
 *  To use avrlib you need to configure the file config.h (do not worry,
 *  an examples is shipped with this library :-)
 *
 *  AVRLIB is written in C/C++ and is divided in three layers:
 *  - Device: microcontroller registers have funny names and change from device
 *            to device. This layer defines standard names and data structures
 *            for these registers
 *  - HAL: initializes and work with hardware. This layer accesses the data
 *         structures defined on device (raw registers are accessed directly)
 *  - High-end: most abstract layer. Everything is a class
 *
 *  @author José Roberto Colombo Junior colombojrj@gmail.com
 */

/**@{*/

#include <util/delay.h>

#define AVRLIB

#include "config.h"
#include "HAL/HAL.h"


#endif /* AVRLIB_H_ */
