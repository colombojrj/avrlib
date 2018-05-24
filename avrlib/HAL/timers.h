/*
  timers.h - Light library to manage the TIMERS of ATMEGA microcontrollers

  Copyright (c) 2015 - José Roberto Colombo Junior (colombojrj [at] gmail.com)

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

#ifndef TIMERS_H_
#define TIMERS_H_

extern "C"
{
	#include <avr/io.h>
	#include <avr/interrupt.h>
}
#include "gpio.h"
#include "defines.h"
#include "../config.h"

/**
 * Definitions:
 * - BOTTOM: The counter reaches the BOTTOM when it becomes 0x00
 * - MAX: The counter reaches its MAXimum when it becomes 0xFF (decimal 255)
 * - TOP: The counter reaches the TOP when it becomes equal to the highest value in the
 *        count sequence. The TOP value can be assigned to be the fixed value 0xFF
 *        (MAX) or the value stored in the OCR0A Register. The assignment is depen-
 *        dent on the mode of operation.
 *
 *
 * Organizational idea of this library:
 *
 * - The file defines.h keeps the definitions of the actual
 *   microcontroller timers (such as what pin and port the
 *   PWM is connected to)
 *
 * - This library uses those definitions to map the PWM
 *   pins. If a new microcontroller needs to be added, then
 *   it remains to add the port definition on defines.h
 *
 * - It is assumed that a timer has a single configuration
 *   during production. With this, defines were inserted
 *   to allow the compiler to select what configuration is
 *   needed and then, generate code only for that
 *   application. The main concern here is to keep the code
 *   size smallest as possible. If "dynamic" functions were
 *   used the compiler would have to build all unused code,
 *   occupying memory of the microcontroller.
 *
 * - Main philosophy: use the right microcontroller for the
 *   application. If it needs more peripherals, then it is
 *   not the right microcontroller for the application.
 */

#if defined(SUPPORT_TO_TIMER0)
/**
 * Functions to handle the TIMER0 module
 */
extern void timer0Init(timer0Config_t config,
                       timer0Clock_t clock,
                       timer0OutputConfig_t outputConfig = timer0OutputConfig_t::off);
extern void timer0ASetDuty(uint8_t OCR, timer0OutputConfig_t outputConfig);
extern void timer0BSetDuty(uint8_t OCR, timer0OutputConfig_t outputConfig);
extern void TIMER0_OFF();

#endif

#if defined(SUPPORT_TO_TIMER1)

/**
 * Functions to handle the TIMER1 module
 */
extern void timer1Init(timer1Config_t config,
                       timer1Clock_t clock,
                       timer1OutputConfig_t outputConfig,
                       uint16_t topValue = 255);
extern void timer1ASetDuty(uint16_t duty, timer1OutputConfig_t output, uint16_t top = 255);
extern void timer1BSetDuty(uint16_t duty, timer1OutputConfig_t output, uint16_t top = 255);
extern void enableInputCapture(timer1InputCaptureEdge_t config);
extern void TIMER1_OFF();

#endif

#if defined(SUPPORT_TO_TIMER2)

/**
 * Functions to handle the TIMER2 module
 */
void timer2Init(timer2Config_t config,
                timer2Clock_t clock,
                timer2OutputConfig_t outputConfig);
extern void timer2ASetDuty(uint8_t OCR, timer2OutputConfig_t outputConfig);
extern void timer2BSetDuty(uint8_t duty, timer2OutputConfig_t outputConfig);
extern void TIMER2_OFF();

#endif /* defined(SUPPORT_TO_TIMER2) */

#endif /* TIMER_H_ */
