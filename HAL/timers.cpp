/*
  timer.cpp - Light library to manage the TIMERS of ATMEGA microcontrollers

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

#include "timers.h"

//
// TODO: add these other operation modes to the library
//

/*****************
 * TIMER0 MODULE
 */

/*
void timer0_init() {
    // set up timer with external clock source
    TCCR0 |= (1 << CS02) | (1 << CS01) | (0 << CS00);
    // initialize counter
    TCNT0 = 0;
    // enable overflow interrupt
    TIMSK |= (1 << TOIE0);
}

void INIT_TIMER1 () {
    OCR1A = 0xF423; // 100ms => 0x1869 and 1000ms => 0xF423
    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A
    TIMSK |= (1 << OCIE1A);
    //Set interrupt on compare match
    TCCR1B |= (1 << CS12);
    // set prescaler to 1024 and start the timer
    sei();
}

 */

#if TIMER0_CONFIG == OFF
	void INIT_TIMER0A (uint8_t auxiliary_call) {}
	void INIT_TIMER0B (uint8_t auxiliary_call) {}
	void TIMER0A_SET_OCR (uint8_t OCR) {}
	void TIMER0B_SET_OCR (uint8_t OCR) {}
#endif

#if TIMER0_CONFIG == NORMAL
	#if defined (__AVR_ATmega8__)
		void INIT_TIMER0 () {
			TIMSK = (1 << TOIE0); // Enable overflow interrupt
			TCCR0 = TIMER0_CLOCK;
			TCNT0 = 0;
			sei();
		}
	#endif
#endif

#if TIMER0_CONFIG == CTC
	#if defined (__AVR_ATmega328P__)
		void INIT_TIMER0 (uint8_t TOP_OCR0A) {
			TIMSK0 = (1 << OCIE0A); // Enable output compare interrupt
			TCCR0A = (1 << WGM01);
			TCCR0B = TIMER0_CLOCK;
			TCNT0 = 0;
			OCR0A = TOP_OCR0A;
			sei();
		}
	#endif
#endif

#if TIMER0_CONFIG == PWM_A || TIMER0_CONFIG == PWM_AB
	void TIMER0A_SET_OCR(uint8_t OCR)
	{
	    if (OCR >= 255)
	    {
	        // Disable compare match
	        TCCR0A = TCCR0A & ~((1 << COM0A1) | (1 << COM0A0));

            #if TIMER0A_POLATIRY == NORMAL
	            gpioWriteHigh(&PORTD, PD6);
            #else
	            gpioWriteLow(&PORTD, PD6);
            #endif
	    }
	    else if (OCR <= 0)
	    {
	        // Disable compare match
	        TCCR0A = TCCR0A & ~((1 << COM0A1) | (1 << COM0A0));

            #if TIMER0A_POLATIRY == NORMAL
	            gpioWriteLow(&PORTD, PD6);
            #else
	            gpioWriteHigh(&PORTD, PD6);
            #endif
	    }
	    else
	    {
	        // Enable compare match
            #if TIMER0A_POLATIRY == NORMAL
	            TCCR0A = TCCR0A | (1 << COM0A1);
            #else
	            TCCR0A = TCCR0A | (1 << COM0A1) | (1 << COM0A0);
            #endif

	        // Set the output compare register
	        OCR0A = OCR;
	    }
	}

	/*
	 * Function: INIT_TIMER0A ()
	 * Purpose:  start TIMER0 channel A as FAST PWM
	 * Receives: 0
	 * Returns:  nothing
	 */
	#if defined (__AVR_ATmega328P__)
		void INIT_TIMER0A(uint8_t auxiliary_call)
		{
			// Set pin as output
			gpioAsOutput(&PORTD, PD6);

			// Clear TIMER0 configuration registers
			TCCR0A = 0;
			TCCR0B = 0;

			#if TIMER0_CONFIG == PWM_AB
				if (auxiliary_call == 0) INIT_TIMER0B(1);
			#endif
			#if TIMER0A_POLATIRY == NORMAL
				TCCR0A = TCCR0A | (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
			#else
				TCCR0A = TCCR0A | (1 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
			#endif
			TCCR0B = TIMER0_CLOCK;
			TCNT0 = 0;

			// Set initial duty cycle
			TIMER0A_SET_OCR(TIMER0A_INITIAL_OCR);
		}
	#endif

	// If configured only channel A, the gpio.h needs the INIT_TIMER0B() function
	#if TIMER0_CONFIG == PWM_A
		void INIT_TIMER0B(uint8_t auxiliary_call) {}
		void TIMER0B_SET_OCR (uint8_t OCR) {}
	#endif
#endif

#if TIMER0_CONFIG == PWM_B || TIMER0_CONFIG == PWM_AB
	void TIMER0B_SET_OCR (uint8_t OCR)
	{
	    if (OCR >= 255)
	    {
	        // Disable compare match
	        TCCR0B = TCCR0B & ~((1 << COM0B1) | (1 << COM0B0));

	        #if TIMER0B_POLATIRY == NORMAL
	            gpioWriteHigh(&PORTD, PD5);
            #else
                gpioWriteLow(&PORTD, PD5);
            #endif
	    }
	    else if (OCR <= 0)
	    {
	        // Disable compare match
	        TCCR0B = TCCR0B & ~((1 << COM0B1) | (1 << COM0B0));

	        #if TIMER0B_POLATIRY == NORMAL
	            gpioWriteHigh(&PORTD, PD5);
            #else
                gpioWriteLow(&PORTD, PD5);
            #endif
	    }
	    else
	    {
	        // Enable compare match
            #if TIMER0B_POLATIRY == NORMAL
	            TCCR0B = TCCR0B | (1 << COM0B1);
            #else
	            TCCR0B = TCCR0B | (1 << COM0B1) | (1 << COM0B0);
            #endif

	        // Set output compare register
	        OCR0B = OCR;
	    }
	}

	/*
	 * Function: INIT_TIMER0B ()
	 * Purpose:  start TIMER0 channel B as FAST PWM
	 * Receives: 0
	 * Returns:  nothing
	 */
	#if defined (__AVR_ATmega328P__)
		void INIT_TIMER0B(uint8_t auxiliary_call)
		{
			// Set pin as output
			gpioAsOutput(&PORTD, PD5);

			// Clear TIMER0 configuration registers
			TCCR0A = 0;
			TCCR0B = 0;

			// If using both channels, make sure the other channel was configured
			#if TIMER0_CONFIG == PWM_AB
				if (auxiliary_call == 0) INIT_TIMER0A(1);
			#endif
			#if TIMER0A_POLATIRY == NORMAL
				TCCR0A = TCCR0A | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
			#else
				TCCR0A = TCCR0A | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM01) | (1 << WGM00);
			#endif
			TCCR0B = TIMER0_CLOCK;
			TCNT0 = 0;

			// Set initial duty cycle
			TIMER0B_SET_OCR(TIMER0B_INITIAL_OCR);
		}
	#endif

	// If configured only channel B, the gpio.h needs the INIT_TIMER0A() function
	#if TIMER0_CONFIG == PWM_B
		void INIT_TIMER0A(uint8_t auxiliary_call) {}
		void TIMER0A_SET_OCR (uint8_t OCR) {}
	#endif
#endif

#if TIMER0_CONFIG == PHASE_CORRECT
#endif
/*
 * Function: TIMER0_OFF ()
 * Purpose:  stop TIMER0 and deactivate the interrupts
 * Receives: nothing
 * Returns:  nothing
 */
#if TIMER0_CONFIG != OFF
	#if defined (__AVR_ATmega8__)
	void TIMER0_OFF() {
			TCCR0  = 0;
			TCNT0  = 0;
			TIMSK &= ~(1 << TOIE0);
			TIFR  &= ~(1 << TOV0);
		}
	#endif
	#if defined (__AVR_ATmega328P__)
		void TIMER0_OFF() {
			TCCR0A = 0;
			TCCR0B = 0;
			TCNT0 = 0;
			OCR0A = 0;
			OCR0B = 0;
			TIMSK0 = 0;
			TIFR0 = 0;
		}
	#endif
#endif

//
///////////////////
// TIMER1 MODULE //
///////////////////
//

#if TIMER1_CONFIG == OFF
	void INIT_TIMER1A (uint8_t auxiliary_call) {}
	void INIT_TIMER1B (uint8_t auxiliary_call) {}
	void TIMER1A_SET_OCR (uint16_t OCR) {}
	void TIMER1B_SET_OCR (uint16_t OCR) {}
	void SET_INPUT_CAPTURE_EDGE(uint8_t type) {}
	void TIMER1_OFF() {}
#endif

#if TIMER1_CONFIG == NORMAL || TIMER1_CONFIG == NORMAL_WITH_IN_CAP
    void INIT_TIMER1A (uint8_t auxiliary_call)
    {
        TCCR1A = 0;
        #if TIMER1_CONFIG == NORMAL
            TCCR1B = TIMER1_CLOCK;
            TIMSK1 = (1 << TOIE1);
        #elif TIMER1_CONFIG == NORMAL_WITH_IN_CAP
            TCCR1B = TIMER1_CLOCK | (1 << ICNC1);
            TIMSK1 = (1 << TOIE1) | (1 << ICIE1);

            // Configure Input Capture Pin as input without pull-up resistor
            DDRB = DDRB & ~(1 << PB0);
            PORTB = PORTB & ~(1 << PB0);
        #endif

    }

	void INIT_TIMER1B (uint8_t auxiliary_call)
	{
		INIT_TIMER1A (0);
	}

	void TIMER1A_SET_OCR (uint16_t OCR) {}
	void TIMER1B_SET_OCR (uint16_t OCR) {}
    #if TIMER1_CONFIG == NORMAL
	    void SET_INPUT_CAPTURE_EDGE(uint8_t type) {}
    #elif TIMER1_CONFIG == NORMAL_WITH_IN_CAP
	    void SET_INPUT_CAPTURE_EDGE(uint8_t type)
        {
            if (type == RISING_EDGE)
            {
                TCCR1B = TCCR1B | (1 << ICES1);
            }
            else
            {
                TCCR1B = TCCR1B & ~(1 << ICES1);
            }
        }
    #endif
#endif

#if TIMER1_CONFIG == CTC
	#if defined (__AVR_ATmega8__)
		void INIT_TIMER1 (uint16_t TOP_OCR1A) {
			OCR1A   = TOP_OCR1A;
			TCCR1B |= (1 << WGM12) | TIMER1_CLOCK;
			TIMSK  |= (1 << OCIE1A); // Enable output compare interrupt
			TCCR1A  = 0;
			TCNT1   = 0;
			sei();
		}
	#endif
	#if defined (__AVR_ATmega328P__)
		void INIT_TIMER1A (uint8_t auxiliary_call) {
			TIMSK1 = (1 << OCIE1A);
			TCCR1A = 0;
			TCCR1B = (1 << WGM12) | TIMER1_CLOCK;
			TCNT1 = 0;
			OCR1A = TIMER1A_INITIAL_OCR;
			sei();
		}
		void INIT_TIMER1B (uint8_t auxiliary_call) {
			TIMSK1 = (1 << OCIE1A);
			TCCR1A = 0;
			TCCR1B = (1 << WGM12) | TIMER1_CLOCK;
			TCNT1 = 0;
			OCR1A = TIMER1A_INITIAL_OCR;
			sei();
		}
	#endif
#endif

#if TIMER1_CONFIG == PWM_A || TIMER1_CONFIG == PWM_AB
	void TIMER1A_SET_OCR (uint16_t OCR)
	{
	    if (OCR >= 255)
        {
	        // Disable compare match
            TCCR1A = TCCR1A & ~((1 << COM1A1) | (1 << COM1A0));

            #if TIMER1A_POLATIRY == NORMAL
                gpioWriteHigh(&PORTB, PB1);
            #else
                gpioWriteLow(&PORTB, PB1);
            #endif
        }
        else if (OCR <= 0)
        {
            // Disable compare match
            TCCR1A = TCCR1A & ~((1 << COM1A1) | (1 << COM1A0));

            #if TIMER1A_POLATIRY == NORMAL
                gpioWriteLow(&PORTB, PB1);
            #else
                gpioWriteHigh(&PORTB, PB1);
            #endif
        }
        else
        {
            // Enable compare match
            #if TIMER1A_POLATIRY == NORMAL
                TCCR1A = TCCR1A | (1 << COM1A1);
            #else
                TCCR1A = TCCR1A | (1 << COM1A1) | (1 << COM1A0);
            #endif

            // Set output compare register
            OCR1A = OCR;
        }
	}
    #if defined (__AVR_ATmega328P__)
        void INIT_TIMER1A (uint8_t auxiliary_call)
        {
            // Set pin as output
            gpioAsOutput(&PORTB, PB1);

            // Clear TIMER0 configuration registers
            TCCR1A = 0;
            TCCR1B = 0;

            // If using both channels, make sure the other channel was configured
            #if TIMER1_CONFIG == PWM_AB
                if (auxiliary_call == 0) INIT_TIMER1B(1);
            #endif

            #if TIMER1_RESOLUTION == 8 && TIMER1A_POLATIRY == NORMAL
                TCCR1A = TCCR1A | (1 << COM1A1) | (1 << WGM10);
                TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;
            #elif TIMER1_RESOLUTION == 8 && TIMER1A_POLATIRY == INVERTED
                TCCR1A = TCCR1A | (1 << COM1A1) | (1 << COM1A0) | (1 << WGM10);
                TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;

            // TODO add support for the other resolutions
            #elif TIMER1_RESOLUTION == 9 && TIMER1A_POLATIRY == NORMAL
                TCCR1A = TCCR1A | (1 << COM1A1) | (1 << WGM11);
                TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;
            #elif TIMER1_RESOLUTION == 9 && TIMER1A_POLATIRY == INVERTED
            #elif TIMER1_RESOLUTION == 10 && TIMER1A_POLATIRY == NORMAL
                TCCR1A = TCCR1A | (1 << COM1A1) | (1 << WGM10) | (1 << WGM11);
                TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;
            #elif TIMER1_RESOLUTION == 10 && TIMER1A_POLATIRY == INVERTED
            #elif TIMER1_RESOLUTION > 10 && TIMER1A_POLATIRY == NORMAL
                TCCR1A = TCCR1A | (1 << COM1A1) | (1 << WGM11);
                TCCR1B = TCCR1B | (1 << WGM13) | (1 << WGM12) | TIMER1_CLOCK;
            #elif TIMER1_RESOLUTION > 10 && TIMER1A_POLATIRY == INVERTED
            #endif
            TCNT1 = 0;
            TIMER1A_SET_OCR (TIMER1A_INITIAL_OCR);
        }
	#endif

    // If configured only channel A, the gpio.h needs the INIT_TIMER0B() function
    #if TIMER1_CONFIG == PWM_A
        void INIT_TIMER1B(uint8_t auxiliary_call) {}
        void TIMER1B_SET_OCR (uint16_t OCR) {}
    #endif
#endif

#if TIMER1_CONFIG == PWM_B || TIMER1_CONFIG == PWM_AB
	void TIMER1B_SET_OCR (uint16_t OCR)
	{
	    if (OCR >= 255)
        {
	        // Disable compare match
            TCCR1B = TCCR1B & ~((1 << COM1B1) | (1 << COM1B0));

            #if TIMER1B_POLATIRY == NORMAL
                gpioWriteHigh(&PORTB, PB2);
            #else
                gpioWriteLow(&PORTB, PB2);
            #endif
        }
        else if (OCR <= 0)
        {
            // Disable compare match
            TCCR1B = TCCR1B & ~((1 << COM1B1) | (1 << COM1B0));

            #if TIMER1B_POLATIRY == NORMAL
                gpioWriteLow(&PORTB, PB2);
            #else
                gpioWriteHigh(&PORTB, PB2);
            #endif
        }
        else
        {
            // Enable compare match
            #if TIMER1B_POLATIRY == NORMAL
                TCCR1B = TCCR1B | (1 << COM1B1);
            #else
                TCCR1B = TCCR1B | (1 << COM1B1) | (1 << COM1B0);
            #endif

            // Set output compare register
            OCR1B = OCR;
        }
	}
	void INIT_TIMER1B (uint8_t auxiliary_call)
	{
		// Set pin as output
		gpioAsOutput(&PORTB, PB2);

		// Clear TIMER0 configuration registers
        TCCR1A = 0;
        TCCR1B = 0;

        // If using both channels, make sure the other channel was configured
        #if TIMER0_CONFIG == PWM_AB
            if (auxiliary_call == 0) INIT_TIMER1A(1);
        #endif

        #if TIMER1_RESOLUTION == 8 && TIMER1A_POLATIRY == NORMAL
            TCCR1A = TCCR1A | (1 << COM1B1) | (1 << WGM10);
            TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;
        #elif TIMER1_RESOLUTION == 8 && TIMER1A_POLATIRY == INVERTED
            TCCR1A = TCCR1A | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10);
            TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;
        #elif TIMER1_RESOLUTION == 9 && TIMER1A_POLATIRY == NORMAL
            TCCR1A = TCCR1A | (1 << COM1B1) | (1 << WGM11);
            TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;
        #elif TIMER1_RESOLUTION == 9 && TIMER1A_POLATIRY == INVERTED
        #elif TIMER1_RESOLUTION == 10 && TIMER1A_POLATIRY == NORMAL
            TCCR1A = TCCR1A | (1 << COM1B1) | (1 << WGM10) | (1 << WGM11);
            TCCR1B = TCCR1B | (1 << WGM12) | TIMER1_CLOCK;
        #elif TIMER1_RESOLUTION == 10 && TIMER1A_POLATIRY == INVERTED
        #elif TIMER1_RESOLUTION > 10 && TIMER1A_POLATIRY == NORMAL
            TCCR1A = TCCR1A | (1 << COM1B1) | (1 << WGM11);
            TCCR1B = TCCR1B | (1 << WGM13) | (1 << WGM12) | TIMER1_CLOCK;
        #elif TIMER1_RESOLUTION > 10 && TIMER1A_POLATIRY == INVERTED
        #endif
        TCNT1 = 0;
        TIMER1B_SET_OCR (TIMER1B_INITIAL_OCR);
    }

    // If configured only channel B, the gpio.h needs the functions relating TIMER1A
    #if TIMER1_CONFIG == PWM_B
        void INIT_TIMER1A(uint8_t auxiliary_call) {}
        void TIMER1A_SET_OCR(uint16_t OCR) {}
    #endif
#endif

#if TIMER1_CONFIG == PHASE_CORRECT
#endif

#if TIMER1_CONFIG != OFF
#if defined (__AVR_ATmega8__)
	void TIMER1_OFF() {
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1 = 0;
		OCR1A = 0;
		OCR1B = 0;
		ICR1 = 0;
	}
	#endif
	#if defined (__AVR_ATmega328P__)
	void TIMER1_OFF() {
		TCCR1A = 0;
		TCCR1B = 0;
		TCNT1 = 0;
		OCR1A = 0;
		OCR1B = 0;
		TIMSK1 = 0;
		TIFR1 = 0;
		ICR1 = 0;
	}
	#endif
#endif

/*
 * TIMER2 MODULE
 */

void TIMER2_SET_CLK(uint8_t config)
{
    if (config <= CLK_8)
        TCCR2B = config;
    else if (config == CLK_32)
        TCCR2B = 3;
    else if (config == CLK_64)
        TCCR2B = 4;
    else if (config == CLK_128)
        TCCR2B = 5;
    else if (config == CLK_256)
        TCCR2B = 6;
    else
        TCCR2B = 7;
}

#if TIMER2_CONFIG == OFF
	void INIT_TIMER2A (uint8_t auxiliary_call) {}
	void INIT_TIMER2B (uint8_t auxiliary_call) {}
	void TIMER2A_SET_OCR (uint8_t OCR) {}
	void TIMER2B_SET_OCR (uint8_t OCR) {}
#endif

#if TIMER2_CONFIG == NORMAL
#endif

#if TIMER2_CONFIG == CTC
	#if defined (__AVR_ATmega328P__)
		void INIT_TIMER2A (uint8_t auxiliary_call) {
			TIMSK2 = (1 << OCIE2A);
			#if TIMER2A_POLATIRY == NORMAL
				TCCR2A = (1 << COM2A1) | (1 << WGM21);
			#else
				TCCR2A = (1 << COM2A1) | (1 << COM2A0) | (1 << WGM21);
			#endif
			TIMER2_SET_CLK(TIMER2_CLOCK);
			TCNT2 = 0;
			OCR2A = TIMER2A_INITIAL_OCR;
			sei();
		}

		// Functions needed by the gpio.h
		void INIT_TIMER2B (uint8_t auxiliary_call) {}
		void TIMER2A_SET_OCR (uint8_t OCR) {}
		void TIMER2B_SET_OCR (uint8_t OCR) {}
	#endif
#endif

#if TIMER2_CONFIG == PWM_A || TIMER2_CONFIG == PWM_AB
    void TIMER2A_SET_OCR (uint16_t OCR)
    {
        if (OCR >= 255)
        {
            // Disable compare match
            TCCR2A = TCCR2A & ~((1 << COM2A1) | (1 << COM2A0));

            #if TIMER2A_POLATIRY == NORMAL
                gpioWriteHigh(&PORTB, PB1);
            #else
                gpioWriteLow(&PORTB, PB1);
            #endif
        }
        else if (OCR <= 0)
        {
            // Disable compare match
            TCCR2A = TCCR2A & ~((1 << COM2A1) | (1 << COM2A0));

            #if TIMER2A_POLATIRY == NORMAL
                gpioWriteLow(&PORTB, PB1);
            #else
                gpioWriteHigh(&PORTB, PB1);
            #endif
        }
        else
        {
            // Enable compare match
            #if TIMER2A_POLATIRY == NORMAL
                TCCR2A = TCCR2A | (1 << COM2A1);
            #else
                TCCR2A = TCCR2A | (1 << COM2A1) | (1 << COM2A0);
            #endif

            // Set output compare register
            OCR2A = OCR;
        }
    }

    #if defined (__AVR_ATmega328P__)
        void INIT_TIMER2A (uint8_t auxiliary_call)
        {
            // Set pin as output
            gpioAsOutput(&PORTB, PB3);

            // Clear TIMER2 configuration registers
            TCCR2A = 0;
            TCCR2B = 0;

            // If using both channels, make sure the other channel was configured
            #if TIMER2_CONFIG == PWM_AB
                if (auxiliary_call == 0) INIT_TIMER2B(1);
            #endif

            #if TIMER2A_POLATIRY == NORMAL
                TCCR2A = TCCR2A | (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
                TCCR2B = TCCR2B | (1 << WGM12);
            #else // i.e., TIMER2A_POLATIRY == INVERTED
                TCCR2A = TCCR2A | (1 << COM2A1) | (1 << COM2A0) | (1 << WGM21) | (1 << WGM20);
            #endif
            TIMER2_SET_CLK(TIMER2_CLOCK);
            TCNT2 = 0;
            TIMER2A_SET_OCR (TIMER2A_INITIAL_OCR);
        }
    #endif

    // If configured only channel A, the gpio.h needs the INIT_TIMER2B() function
    #if TIMER2_CONFIG == PWM_A
        void INIT_TIMER2B(uint8_t auxiliary_call) {}
        void TIMER2B_SET_OCR (uint16_t OCR) {}
    #endif
#endif

#if TIMER2_CONFIG == PWM_B || TIMER2_CONFIG == PWM_AB
    void TIMER2B_SET_OCR (uint16_t OCR)
    {
        if (OCR >= 255)
        {
            // Disable compare match
            TCCR2B = TCCR2B & ~((1 << COM2B1) | (1 << COM2B0));

            #if TIMER2B_POLATIRY == NORMAL
                gpioWriteHigh(&PORTD, PD3);
            #else
                gpioWriteLow(&PORTD, PD3);
            #endif
        }
        else if (OCR <= 0)
        {
            // Disable compare match
            TCCR2A = TCCR2A & ~((1 << COM2A1) | (1 << COM2A0));

            #if TIMER2B_POLATIRY == NORMAL
                gpioWriteLow(&PORTD, PD3);
            #else
                gpioWriteHigh(&PORTD, PD3);
            #endif
        }
        else
        {
            // Enable compare match
            #if TIMER2B_POLATIRY == NORMAL
                TCCR2B = TCCR2B | (1 << COM2B1);
            #else
                TCCR2B = TCCR2B | (1 << COM2B1) | (1 << COM2B0);
            #endif

            // Set output compare register
            OCR2B = OCR;
        }
    }

    void INIT_TIMER2B (uint8_t auxiliary_call)
    {
        // Set pin as output
        gpioAsOutput(&PORTD, PD3);

        // Clear TIMER2 configuration registers
        TCCR2A = 0;
        TCCR2B = 0;

        // If using both channels, make sure the other channel was configured
        #if TIMER2_CONFIG == PWM_AB
            if (auxiliary_call == 0) INIT_TIMER2A(1);
        #endif

        #if TIMER2B_POLATIRY == NORMAL
            TCCR2A = TCCR2A | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
            TCCR2B = TCCR2B | (1 << WGM12);
        #else // i.e., TIMER2B_POLATIRY == INVERTED
            TCCR2A = TCCR2A | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM21) | (1 << WGM20);
        #endif
        TIMER2_SET_CLK(TIMER2_CLOCK);
        TCNT2 = 0;
        TIMER2B_SET_OCR(TIMER2B_INITIAL_OCR);
    }

    // If configured only channel B, the gpio.h needs the functions relating TIMER1A
    #if TIMER2_CONFIG == PWM_B
        void INIT_TIMER2A(uint8_t auxiliary_call) {}
        void TIMER2A_SET_OCR(uint16_t OCR) {}
    #endif
#endif

#if TIMER2_CONFIG == PHASE_CORRECT
#endif




