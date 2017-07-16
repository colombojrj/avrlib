/*
 * spi.h
 *
 *  Created on: 16/07/2017
 *      Author: junior
 */

#ifndef AVRLIB_HAL_SPI_H_
#define AVRLIB_HAL_SPI_H_

#include <stdlib.h>
#include <avr/io.h>
#include "defines.h"
#include "../config.h"

extern void SPI_SET_CLK();
extern void SPI_INIT();
extern void SPI_MASTER_SEND(uint8_t data);
extern uint8_t SPI_MASTER_TRANCEIVER(uint8_t data);
extern void SPI_OFF();

#endif /* AVRLIB_HAL_SPI_H_ */
