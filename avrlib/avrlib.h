/*
 * avrlib.h
 *
 *  Created on: 12/06/2017
 *      Author: junior
 */

#ifndef AVRLIB_H_
#define AVRLIB_H_

/**
 *  @defgroup avrlib
 *  @code #include <avrlib.h> @endcode
 *
 *  This the main avrlib file. Including this file, you will have access
 *  to all the abstraction functions from this library.
 *
 *  To use avrlib you need to configure the file config.h (do not worry,
 *  an examples is shipped with this library :-)
 *
 *  AVRLIB is written in C++ and is divided in two layers:
 *  - HAL (talk to the micro-controller registers)
 *  - top (use the HAL)
 *
 *  @author José Roberto Colombo Junior colombojrj@gmail.com
 */

/**@{*/

#include <util/delay.h>

#define AVRLIB

#include "config.h"
#include "HAL/HAL.h"
#include "gpio.h"


#endif /* AVRLIB_H_ */
