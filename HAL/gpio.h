/*
 * gpio.h
 *
 *  Created on: 23/10/2017
 *      Author: junior
 */

#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "defines.h"
#include "../config.h"

// Functions relating pin direction (INPUT or OUTPUT)
void gpioAsOutput(uint8_t *port, uint8_t pin);
void gpioAsInput(uint8_t *port, uint8_t pin);
void gpioDirection(uint8_t *port, uint8_t pin, uint8_t dir);

// Functions to write on pin
void gpioWriteHigh(uint8_t *port, uint8_t pin);
void gpioWriteLow(uint8_t *port, uint8_t pin);
void gpioWrite(uint8_t *port, uint8_t pin, uint8_t level);
void gpioToggle(uint8_t *port, uint8_t pin);

// Functions to read pin state
uint8_t gpioFastRead(uint8_t *port, uint8_t pin);
uint8_t gpioRead(uint8_t *port, uint8_t pin);

// Functions relating PCINT interrupts
void enablePCINT(uint8_t *port, uint8_t pin);
void disablePCINT(uint8_t *port, uint8_t pin);

// Functions relating INT interrupts
void enableINT(uint8_t *port, uint8_t pin, uint8_t edge = RISING_EDGE);
void disableINT(uint8_t *port, uint8_t pin);


#endif /* HAL_GPIO_H_ */
