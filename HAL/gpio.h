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
#include "timers.h"
#include "adc.h"

namespace HAL {

/// To configure pin alternative functions (example: INPUT, OUTPUT, ADC)
extern void gpioAsOutput(volatile uint8_t *port, const uint8_t pin);
extern void gpioAsInput(volatile uint8_t *port, const uint8_t pin);
extern void gpioDirection(volatile uint8_t *port, const uint8_t pin, const uint8_t dir);
extern void gpioAsADC(volatile uint8_t *port, uint8_t pin);
extern void gpioAsPWM(volatile uint8_t *port, uint8_t pin);

/// Functions to write on pin
extern void gpioWriteHigh(volatile uint8_t *port, const uint8_t pin);
extern void gpioWriteLow(volatile uint8_t *port, const uint8_t pin);
extern void gpioWrite(volatile uint8_t *port, const uint8_t pin, const uint8_t level);
extern void gpioToggle(volatile uint8_t *port, const uint8_t pin);

// Functions to read pin state
extern uint8_t gpioFastRead(volatile uint8_t *port, const uint8_t pin);
extern uint8_t gpioRead(volatile uint8_t *port, const uint8_t pin);

// Functions relating PCINT interrupts
extern void enablePCINT(volatile uint8_t *port, const uint8_t pin);
extern void disablePCINT(volatile uint8_t *port, const uint8_t pin);

// Functions relating INT interrupts
extern void enableINT(volatile uint8_t *port, const uint8_t pin, const uint8_t edge = RISING_EDGE);
extern void disableINT(volatile uint8_t *port, const uint8_t pin);

// PWM relating functions
extern void setDuty(volatile uint8_t *port, const uint8_t pin, const uint16_t duty);


} // namespace HAL

#endif /* HAL_GPIO_H_ */
