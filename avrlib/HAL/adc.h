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
 * * NOISE_REDUCTION
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

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "defines.h"
#include "../config.h"

/**
 * @brief  This function is employed to initialize the ADC module as configured in \ref config.h
 *
 * In OFF mode this function start energy saving.
 *
 * @param  uint8_t pin: the ADC pin number (ex: PC0)
 */
extern void adcInit(uint8_t pin);

/**
 * @brief  This function is employed to configure the ADC clock source configured in \ref config.h
 *
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
 * @param  adcReferenceVoltage_t adcRefVoltage is the configuration for the reference voltage
 *
 * @see adcReferenceVoltage_t
 */
extern void adcSetRefVoltage(adcRefVoltage_t adcRefVoltage);

/**
 * @brief  This function is employed to configure the ADC data alignment configured in \ref config.h
 *
 * @param  adcDataAlign_t adcDataAlign: data align configuration  (see \ref config.h)
 */
extern void adcSetDataAlign(adcDataAlign_t adcDataAlign);

/**
 * @brief  This function start an analog conversion. It will block
 *         the code execution until not finished.
 *
 * @param  uint8_t pin: the ADC pin number (ex: PC0)
 * @return uint16_t converted value (10 bits)
 */
extern uint16_t adcRead(uint8_t pin);

/**
 * @brief  This function is employed to change the analog channel in
 *         \ref FREE_RUNING mode.
 *
 * @param  uint8_t pin: the ADC pin number (ex: PC0)
 */
extern void adcChangeAdmux(uint8_t pin);

/**
 * @brief  This function read the temperature sensor available in some
 *         micro-controllers.
 *
 * Known micro-controllers that possesses this hardware:
 * * ATmega328P
 *
 * @return uint16_t converted value (12 bits)
 */
extern int16_t adcReadTemperature();

/**@}*/

#endif /* ADC_H_ */
