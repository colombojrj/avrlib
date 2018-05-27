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
 * SPI polarity definitions:
 * @image html spi_cpha_0.gif
 *
 * @author José Roberto Colombo Junior
 */

/**@{*/

#include "gpio.h"
#include "defines.h"
#include "../config.h"

#if defined(SUPPORT_TO_SPI)

/**
 * @brief  This function has the purpose of configuring
 *          the SPI clock source.
 *
 * There are no input or output parameters for this function
 *
 * @param  none
 * @return none
 */
extern void spiSetClock(spiClock_t spiClock);

/**
 * @brief  This function has the purpose of initialize
 *         the SPI microcontroller module. It is called
 *         automatically during HAL configuration step
 *         @see initHAL.
 *
 * @param  none
 * @return none
 */
extern void spiInit(spiClock_t spiClock);

/**
 * Function to send data from SPI.
 * This function will block the CPU
 *
 * @param uint8_t data
 *
 * @return none
 */
extern void spiSend(uint8_t data);
// TODO make spiMasterSend return if send was successful (use WCOL from SPSR)

/**
 * Function to send and receive data from SPI.
 * This function will block the CPU.
 *
 * @param uint8_t data
 *
 * @return uint8_t received data
 */
extern uint8_t spiTransceiver(uint8_t data);

/**
 * Function to send data from SPI.
 * This function will block the CPU
 *
 * @param uint8_t data
 *
 * @return none
 */
extern void spiOff();

/**
 * spiBusy is a boolean variable informing if the
 * spi module is busy. This implementation is totally
 * in software.
 */
extern bool spiBusy;

/**
 * Returns if the microcontroller's SPI module is busy
 *
 * @return a bool type, if true then the SPI is busy.
 *         If false, then the SPI is free.
 */
extern bool spiIsBusy();

/**@}*/

#endif // SUPPORT_TO_SPI

#endif /* AVRLIB_HAL_SPI_H_ */
