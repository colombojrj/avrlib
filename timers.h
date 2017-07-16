/*
  timers.h - Light library to manage the TIMERS of ATMEGA microcontrollers

  Copyright (c) 2015 - José Roberto Colombo Junior (colombojrj@gmail.com)

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

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "general_config.h"

// Useful defines
#define OFF				0 // TIMER without configuration
#define NORMAL			1
#define CTC				2
#define PWM_A			3 // only channel A (example OCR0A, OCR1A, ...) works as fast-PWM
#define PWM_B			4 // only channel B (example OCR0B, OCR1B, ...) works as fast-PWM
#define PWM_AB			5 // both channels work as fast-PWM
#define PHASE_CORRECT	6
#define INVERTED		7
#define TIMER0A			0
#define TIMER0B			1
#define TIMER1A			2
#define TIMER1B			3
#define TIMER2A			4
#define TIMER2B			5

// Clock division
#define NO_CLOCK		0
#define NO_PREESCALE	1
#define CLK_8			2
#define CLK_32			8 // only for TIMER2
#define CLK_64			3
#define CLK_128			9 // only for TIMER2
#define CLK_256			4
#define CLK_1024		5
#define T0_FALLING		6
#define T0_RAISING		7

/*
 * FUNCTIONS TO HANDLE THE TIMER0 MODULE
 */
extern void INIT_TIMER0A(uint8_t auxiliary_call = 0);
extern void INIT_TIMER0B(uint8_t auxiliary_call = 0);
extern void TIMER0A_SET_OCR(uint8_t OCR);
extern void TIMER0B_SET_OCR(uint8_t OCR);
extern void TIMER0_OFF();

/*
 * FUNCTIONS TO HANDLE THE TIMER1 MODULE
 */
extern void INIT_TIMER1A (uint8_t auxiliary_call = 0);
extern void INIT_TIMER1B (uint8_t auxiliary_call = 0);
extern void TIMER1A_SET_OCR (uint16_t OCR);
extern void TIMER1B_SET_OCR (uint16_t OCR);
extern void TIMER1_OFF();

/*
 * FUNCTIONS TO HANDLE THE TIMER2 MODULE
 */
extern void INIT_TIMER2A (uint8_t auxiliary_call = 0);
extern void INIT_TIMER2B (uint8_t auxiliary_call = 0);
extern void TIMER2A_SET_OCR (uint8_t OCR);
extern void TIMER2B_SET_OCR (uint8_t OCR);
extern void TIMER2_OFF();

#endif /* TIMER_H_ */
