/*
 * gpio.h
 *
 *  Created on: 23/10/2017
 *      Author: junior
 */

#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "defines.h"
//#include "../config.h"
#include "timers.h"
#include "adc.h"

namespace HAL {

/// To configure pin alternative functions (example: INPUT, OUTPUT, ADC)
void gpioAsOutput(volatile uint8_t *port, uint8_t pin);
void gpioAsInput(volatile uint8_t *port, uint8_t pin);
void gpioDirection(volatile uint8_t *port, uint8_t pin, uint8_t dir);
void gpioAsADC(volatile uint8_t *port, uint8_t pin);
void gpioAsPWM(volatile uint8_t *port, uint8_t pin);

/// Functions to write on pin
void gpioWriteHigh(volatile uint8_t *port, uint8_t pin);
void gpioWriteLow(volatile uint8_t *port, uint8_t pin);
void gpioWrite(volatile uint8_t *port, uint8_t pin, uint8_t level);
void gpioToggle(volatile uint8_t *port, uint8_t pin);

// Functions to read pin state
uint8_t gpioFastRead(volatile uint8_t *port, uint8_t pin);
uint8_t gpioRead(volatile uint8_t *port, uint8_t pin);

// Functions relating PCINT interrupts
void enablePCINT(volatile uint8_t *port, uint8_t pin);
void disablePCINT(volatile uint8_t *port, uint8_t pin);

// Functions relating INT interrupts
void enableINT(volatile uint8_t *port, uint8_t pin, uint8_t edge = RISING_EDGE);
void disableINT(volatile uint8_t *port, uint8_t pin);

// PWM relating functions
void setDuty(volatile uint8_t *port, uint8_t pin, uint16_t duty);


} // namespace HAL

#endif /* HAL_GPIO_H_ */
