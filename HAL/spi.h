#ifndef AVRLIB_HAL_SPI_H_
#define AVRLIB_HAL_SPI_H_

/**
 * @defgroup hal_spi SPI Library
 *
 * @code #include <spi.h> @endcode
 *
 * A library to handle the SPI module of AVR microcontrollers
 *
 * The supported modes of operation are:
 * * OFF
 * * MASTER
 * * SLAVE (not supported yet)
 *
 * In MASTER mode, the SPI pins are automatically configured as:
 * * MOSI: OUTPUT
 * * MISO: INPUT
 * * SCK: OUTPUT
 *
 * The SS pin must be configured by the user of this library manually.
 * The intention behind this is to allow the library to become more
 * flexible.
 *
 * In SLAVE mode, the SPI pins are automatically configured as:
 * * MOSI: INPUT
 * * MISO: OUTPUT
 * * SCK: INPUT
 * * SS: INPUT
 *
 * @author José Roberto Colombo Junior
 */

/**@{*/

#include <stdlib.h>
#include <avr/io.h>
#include "defines.h"
#include "../config.h"

#if defined (__AVR_ATmega328P__)
    #define _DDR_SPI    DDRB
    #define _PIN_SS     PB2
    #define _PIN_MOSI   PB3
    #define _PIN_MISO   PB4
    #define _PIN_SCK    PB5
#endif

/**
 * @brief  This function has the purpose of configuring
 *          the SPI clock source.
 *
 * There are no input or output parameters for this function
 *
 * @param  none
 * @return none
 */
extern void SPI_SET_CLK();

/**
 * @brief  This function has the purpose of initialize
 *         the SPI micro-controller module. It is called
 *         automatically during HAL configuration step
 *         @see HAL_INIT.
 *
 * There are no input or output parameters for this function.
 *
 * @param  none
 * @return none
 */
extern void SPI_INIT();

/**
 * Function to send data from SPI.
 * This function will block the CPU
 *
 * The input parameter is:
 * @param uint8_t data
 *
 * There are no output parameters
 * @return none
 */
extern void SPI_MASTER_SEND(uint8_t data);

/**
 * Function to send and receive data from SPI.
 * This function will block the CPU.
 *
 * The input parameter is:
 * @param uint8_t data
 *
 * @return uint8_t received data
 */
extern uint8_t SPI_MASTER_TRANCEIVER(uint8_t data);

/**
 * Function to send data from SPI.
 * This function will block the CPU
 *
 * The input parameter is:
 * @param uint8_t data
 *
 * There are no output parameters
 * @return none
 */
extern void SPI_OFF();

/**@}*/

#endif /* AVRLIB_HAL_SPI_H_ */
