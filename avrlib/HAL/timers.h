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

/**
 * Functions to handle the TIMER0 module
 */
extern void timer0Init(timer0Config_t config,
                       timer0Clock_t clock,
                       timer0APolarity_t polarityChannelA = timer0APolarity_t::normal,
                       timer0BPolarity_t polarityChannelB = timer0BPolarity_t::normal);
extern void timer0Init(timer0Config_t config,
                       timer0Clock_t clock,
                       timer0BPolarity_t polarityChannelB);
extern void timer0ASetDuty(uint8_t OCR, timer0APolarity_t polarity = timer0APolarity_t::normal);
extern void timer0BSetDuty(uint8_t OCR, timer0BPolarity_t polarity = timer0BPolarity_t::normal);
extern void TIMER0_OFF();

/**
 * Functions to handle the TIMER1 module
 */
extern void INIT_TIMER1A ();
extern void INIT_TIMER1B ();
extern void TIMER1A_SET_OCR (uint16_t OCR);
extern void TIMER1B_SET_OCR (uint16_t OCR);
extern void SET_INPUT_CAPTURE_EDGE(uint8_t type);
extern void TIMER1_OFF();

/**
 * Functions to handle the TIMER2 module
 */
extern void INIT_TIMER2A ();
extern void INIT_TIMER2B ();
extern void TIMER2_SET_CLK(uint8_t config);
extern void TIMER2A_SET_OCR (uint8_t OCR);
extern void TIMER2B_SET_OCR (uint8_t OCR);
extern void TIMER2_OFF();

#endif /* TIMER_H_ */
