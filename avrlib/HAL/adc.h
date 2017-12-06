/*
  adc.h - Library to handle the ADC module of AVR micro-controllers

  Copyright (c) 2013 - José Roberto Colombo Junior

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef ADC_H_
#define ADC_H_

/**
 * @defgroup hal_adc ADC Library
 *
 * @code #include <adc.h> @endcode
 *
 * A library to handle the ADC module of AVR microcontrollers
 *
 * The supported modes of operation are:
 * * OFF
 * * SINGLE_CONVERSION
 * * NOISE_REDUCTION (to do)
 * * FREE_RUNNING (to do)
 *
 * In the FREE_RUNNING mode, when a conversion is ready an interruption
 * is triggered. Make sure to declare the ISR(ADC_vect). Here is an example:
 *
 * \code
 * volatile uint16_t voltage;
 * ISR(ADC_vect) {
 *     voltage = ADC;
 * }
 * \endcode
 *
 */

/**@{*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "defines.h"
#include "../config.h"

// Available functions

/**
 * @brief  This function is employed to initialize the ADC module as configured in \ref config.h
 *
 * In OFF mode this function start energy saving.
 *
 * @param  uint8_t pin: the ADC pin number (ex: PC0)
 * @return none
 */
extern void adcInit(uint8_t pin);

/**
 * @brief  This function is employed to configure the ADC clock source configured in \ref config.h
 *
 * @param  none
 * @return none
 */
extern void adcSetClock();

/**
 * @brief This function set the reference voltage source.
 *
 * The possible values are:
 * * INTERNAL (verify the micro-controller manual to verify the value in volts)
 * * EXTERNAL (extern value connected through pin)
 * * VCC      (microcontroller VCC voltage)
 *
 *
 * @param  uint8_t source
 * @return none
 */
extern void adcSetRefVoltage(uint8_t config);

/**
 * @brief  This function is employed to configure the ADC data alignment configured in \ref config.h
 *
 * @param  uint8_t config: data align configuration  (see \ref config.h)
 * @return none
 */
extern void adcSetDataAlign(uint8_t config);

/**
 * @brief  This function start an analog conversion. It will block
 *         the code execution until not finished.
 *
 * @param  uint8_t pin: the ADC pin number (ex: PC0)
 * @return uint16_t converted value (12 bits)
 */
extern uint16_t adcRead(uint8_t pin);

/**
 * @brief  This function is employed to change the analog channel in
 *         \ref FREE_RUNING mode.
 *
 * @param  uint8_t pin: the ADC pin number (ex: PC0)
 * @return none
 */
extern void adcChangeAdmux(uint8_t pin);

/**
 * @brief  This function read the temperature sensor available in some
 *         micro-controllers.
 *
 * Known micro-controllers that possesses this hardware:
 * * ATmega328P
 *
 * @param  none
 * @return none
 */
extern int16_t adcReadTemperature();

/**@}*/

#endif /* ADC_H_ */
