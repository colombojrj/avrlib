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
#include "defines.h"
#include "../config.h"
#include "gpio.h"

/*
 * FUNCTIONS TO HANDLE THE TIMER0 MODULE
 */
extern void INIT_TIMER0A();
extern void INIT_TIMER0B();
extern void TIMER0A_SET_OCR(uint8_t OCR);
extern void TIMER0B_SET_OCR(uint8_t OCR);
extern void TIMER0_OFF();

/*
 * FUNCTIONS TO HANDLE THE TIMER1 MODULE
 */
extern void INIT_TIMER1A ();
extern void INIT_TIMER1B ();
extern void TIMER1A_SET_OCR (uint16_t OCR);
extern void TIMER1B_SET_OCR (uint16_t OCR);
extern void SET_INPUT_CAPTURE_EDGE(uint8_t type);
extern void TIMER1_OFF();

/*
 * FUNCTIONS TO HANDLE THE TIMER2 MODULE
 */
extern void INIT_TIMER2A ();
extern void INIT_TIMER2B ();
extern void TIMER2_SET_CLK(uint8_t config);
extern void TIMER2A_SET_OCR (uint8_t OCR);
extern void TIMER2B_SET_OCR (uint8_t OCR);
extern void TIMER2_OFF();

#endif /* TIMER_H_ */
