#ifndef AVRLIB_HAL_H_
#define AVRLIB_HAL_H_

/**
 * @defgroup hal_group HAL (Hardware Abstraction Layer)
 *
 */

#include "defines.h"
#include "gpio.h"
#include "timers.h"
#include "uart.h"
#include "adc.h"
#include "spi.h"

/**
 * This function has the purpose of configuring the micro-controllers
 * peripherals, as configured in the \ref config.h file.

 * Make sure to run this function in the main program.
 */
extern void HAL_init();


#endif /* AVRLIB_HAL_H_ */
