/*
  timer.cpp - Light library to manage the TIMERS of ATMEGA microcontrollers

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

#include "timers.h"

//
///////////////////
// TIMER0 MODULE //
///////////////////
//

void timer0ASetDuty(uint8_t OCR, timer0APolarity_t polarity)
{
    if (OCR >= 255)
    {
        // Disable compare match
        TCCR0A = TCCR0A & ~static_cast<uint8_t>(timer0APolarity_t::setState);

        if (polarity == timer0APolarity_t::normal)
            gpioWriteHigh(&OC0A_PORT, OC0A_PIN);
        else
            gpioWriteLow(&OC0A_PORT, OC0A_PIN);
    }

    else if (OCR <= 0)
    {
        // Disable compare match
        TCCR0A = TCCR0A & ~static_cast<uint8_t>(timer0APolarity_t::setState);

        if (polarity == timer0APolarity_t::normal)
            gpioWriteLow(&OC0A_PORT, OC0A_PIN);
        else
            gpioWriteHigh(&OC0A_PORT, OC0A_PIN);
    }

    else
    {
        // Enable compare match
        TCCR0A = TCCR0A | static_cast<uint8_t>(polarity);

        // Set the output compare register
        OCR0A = OCR;
    }
}

void timer0BSetDuty(uint8_t OCR, timer0BPolarity_t polarity)
{
    if (OCR >= 255)
    {
        // Disable compare match
        TCCR0B = TCCR0B & ~static_cast<uint8_t>(timer0BPolarity_t::setState);

        if (polarity == timer0BPolarity_t::normal)
            gpioWriteHigh(&OC0B_PORT, OC0B_PIN);
        else
            gpioWriteLow(&OC0B_PORT, OC0B_PIN);
    }

    else if (OCR <= 0)
    {
        // Disable compare match
        TCCR0B = TCCR0B & ~static_cast<uint8_t>(timer0BPolarity_t::setState);

        if (polarity == timer0BPolarity_t::normal)
            gpioWriteLow(&OC0B_PORT, OC0B_PIN);
        else
            gpioWriteHigh(&OC0B_PORT, OC0B_PIN);
    }

    else
    {
        // Enable compare match
        TCCR0B = TCCR0B | static_cast<uint8_t>(polarity);

        // Set the output compare register
        OCR0B = OCR;
    }
}

void timer0Init(timer0Config_t config,
                timer0Clock_t clock,
                timer0APolarity_t polChA,
                timer0BPolarity_t polChB)
{
    #if defined (PRR)
        // Enables tTimer/Counter0 module
        PRR = PRR & ~(1 << PRTIM0);
    #endif

    // Set timer0 configuration
	#if defined (TCCR0A)
        uint8_t polarity = static_cast<uint8_t>(polChA) | static_cast<uint8_t>(polChB);
        if (config == timer0Config_t::off || config == timer0Config_t::normal || config == timer0Config_t::ctc)
            TCCR0A = static_cast<uint8_t>(config);
        else
            TCCR0A = static_cast<uint8_t>(config) | polarity;

    #elif defined (TCCR0) // ATmega8, for example
	#endif

    // Set timer0 clock source
    TCCR0B = static_cast<uint8_t>(clock);
    if (clock == timer0Clock_t::externT0FallingEdge || clock == timer0Clock_t::externT0RisingEdge)
    	gpioAsInput(&T0_PORT, T0_PIN, 1);

    // Timer0 specific configurations
    if (config == timer0Config_t::normal)
    {
        // Enable overflow interrupt
        TIMSK0 = (1 << TOIE0);
        sei();
    }

    else if (config == timer0Config_t::ctc)
    {
        // Enable output compare interrupt
        TIMSK0 = (1 << OCIE0A);
        OCR0A = timer0AInitialOcr;
        sei();
    }

    else if (config == timer0Config_t::pwmChannelA || config == timer0Config_t::pwmPhaseCorrectA)
    {
        // GPIO configuration
        gpioAsOutput(&OC0A_PORT, OC0A_PIN);

        // Initial duty cycle
        timer0ASetDuty(timer0AInitialOcr, polChA);
    }

    else if (config == timer0Config_t::pwmChannelB || config == timer0Config_t::pwmPhaseCorrectB)
    {
        // GPIO Configuration
        gpioAsOutput(&OC0B_PORT, OC0B_PIN);

        // Initial duty cycle
        timer0BSetDuty(timer0BInitialOcr, polChB);
    }

    else if (config == timer0Config_t::pwmChannelsAB || config == timer0Config_t::pwmPhaseCorrectAB)
    {
        // GPIO Configuration
        gpioAsOutput(&OC0A_PORT, OC0A_PIN);
        gpioAsOutput(&OC0B_PORT, OC0B_PIN);

        // Initial duty cycle
        timer0ASetDuty(timer0AInitialOcr, polChA);
        timer0BSetDuty(timer0BInitialOcr, polChB);
    }
}

void timer0Init(timer0Config_t config,
                timer0Clock_t clock,
                timer0BPolarity_t polarityChannelB)
{
    timer0Init(config, clock, timer0APolarity_t::normal, polarityChannelB);
}

/*
 * Function: TIMER0_OFF ()
 * Purpose:  stop TIMER0 and deactivate the interrupts
 * Receives: nothing
 * Returns:  nothing
 */
#if defined (__AVR_ATmega8__)
void TIMER0_OFF()
{
        TCCR0  = 0;
        TCNT0  = 0;
        TIMSK &= ~(1 << TOIE0);
        TIFR  &= ~(1 << TOV0);
    }
#endif
#if defined (__AVR_ATmega328P__)
    void TIMER0_OFF()
    {
        TCCR0A = 0;
        TCCR0B = 0;
        TCNT0  = 0;
        OCR0A  = 0;
        OCR0B  = 0;
        TIMSK0 = 0;
        TIFR0  = 0;

        // Enables power reduction
        PRR = PRR | (1 << PRTIM0);
    }
#endif

//
///////////////////
// TIMER1 MODULE //
///////////////////
//

void timer1Init(timer1Config_t config,
                timer1Clock_t clock,
                timer1OutputConfig_t outputConfig,
                uint16_t topValue)
{
    #if defined (PRR)
        // Enables Timer/Counter1 module
        PRR = PRR & ~(1 << PRTIM1);
    #endif

    // Pin configuration
    if (outputConfig == timer1OutputConfig_t::channelAnormal   ||
        outputConfig == timer1OutputConfig_t::channelAinverted ||
        outputConfig == timer1OutputConfig_t::channelABnormal  ||
        outputConfig == timer1OutputConfig_t::channelABinverted)
    {
        gpioAsOutput(&OC1A_PORT, OC1A_PIN);
    }

    if (outputConfig == timer1OutputConfig_t::channelBnormal   ||
        outputConfig == timer1OutputConfig_t::channelBinverted ||
        outputConfig == timer1OutputConfig_t::channelABnormal  ||
        outputConfig == timer1OutputConfig_t::channelABinverted)
    {
        gpioAsOutput(&OC1B_PORT, OC1B_PIN);
    }

    // Set timer1 configuration
    if (config == timer1Config_t::normal)
    {
        timer1AsNormal helperNormal(outputConfig);

        // Enable overflow interrupt
        TIMSK1 = (1 << TOIE1);
        sei();
    }

    else if (config == timer1Config_t::ctc)
    {
        timer1AsCTC helperCTC(outputConfig);

        // Enable compare match channel A interrupt
        TIMSK1 = (1 << OCIE1A);
        sei();
    }

    else if (config == timer1Config_t::pwm8Bits)
    {
        timer1As8bitPwm helper(outputConfig);
    }

    else if (config == timer1Config_t::pwm9Bits)
    {
        timer1As9bitPwm helper(outputConfig);
    }

    else if (config == timer1Config_t::pwm10Bits)
    {
        timer1As10bitPwm helper(outputConfig);
    }

    else if (config == timer1Config_t::pwmDefinedTop)
    {
        timer1As16bitPwm helper(outputConfig, topValue);
    }

    else if (config == timer1Config_t::pwmPhaseCorrect8Bits)
    {
        timer1As8bitPhaseCorrectPwm helper(outputConfig);
    }

    else if (config == timer1Config_t::pwmPhaseCorrect9Bits)
    {
        timer1As9bitPhaseCorrectPwm helper(outputConfig);
    }

    else if (config == timer1Config_t::pwmPhaseCorrect10Bits)
    {
        timer1As10bitPhaseCorrectPwm helper(outputConfig);
    }

    else if (config == timer1Config_t::pwmPhaseCorrectDefinedTop)
    {
        timer1As16bitPhaseCorrectPwm helper(outputConfig, topValue);
    }

    // Set timer 1 clock source
    TCCR1B = TCCR1B | static_cast<uint8_t>(clock);

    // Reset counting
    TCNT1 = 0;

}

void timer1ASetDuty(uint16_t duty, timer1OutputConfig_t output, uint16_t top)
{
    if (duty >= top)
    {
        // Disable compare match
        TCCR1A = TCCR1A & ~static_cast<uint8_t>(timer1OutputConfig_t::channelAsetState);

        if (output == timer1OutputConfig_t::channelAnormal ||
            output == timer1OutputConfig_t::channelABnormal)
            gpioWriteHigh(&OC1A_PORT, OC1A_PIN);
        else
            gpioWriteLow(&OC1A_PORT, OC1A_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR1A = TCCR1A & ~static_cast<uint8_t>(timer1OutputConfig_t::channelAsetState);

        if (output == timer1OutputConfig_t::channelAnormal ||
            output == timer1OutputConfig_t::channelABnormal)
            gpioWriteLow(&OC1A_PORT, OC1A_PIN);
        else
            gpioWriteHigh(&OC1A_PORT, OC1A_PIN);
    }

    else
    {
        // Enable compare match
        TCCR1A = TCCR1A | static_cast<uint8_t>(output);

        // Set the output compare register
        OCR1A = duty;
    }
}

void timer1BSetDuty(uint16_t duty, timer1OutputConfig_t output, uint16_t top)
{
    if (duty >= top)
    {
        // Disable compare match
        TCCR1A = TCCR1A & ~static_cast<uint8_t>(timer1OutputConfig_t::channelBsetState);

        if (output == timer1OutputConfig_t::channelBnormal ||
            output == timer1OutputConfig_t::channelABnormal)
            gpioWriteHigh(&OC1B_PORT, OC1B_PIN);
        else
            gpioWriteLow(&OC1B_PORT, OC1B_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR1A = TCCR1A & ~static_cast<uint8_t>(timer1OutputConfig_t::channelBsetState);

        if (output == timer1OutputConfig_t::channelBnormal ||
            output == timer1OutputConfig_t::channelABnormal)
            gpioWriteLow(&OC1B_PORT, OC1B_PIN);
        else
            gpioWriteHigh(&OC1B_PORT, OC1B_PIN);
    }

    else
    {
        // Enable compare match
        TCCR1A = TCCR1A | static_cast<uint8_t>(output);

        // Set the output compare register
        OCR1B = duty;
    }
}

void enableInputCapture(timer1InputCaptureEdge_t config)
{
    TCCR1B = TCCR1B & ~static_cast<uint8_t>(timer1InputCaptureEdge_t::setState);
    TCCR1B = TCCR1B |  static_cast<uint8_t>(config);
}

#if defined (__AVR_ATmega328P__)
    void TIMER1_OFF() {
        TCCR1A = 0;
        TCCR1B = 0;
        TCNT1  = 0;
        OCR1A  = 0;
        OCR1B  = 0;
        TIMSK1 = 0;
        TIFR1  = 0;
        ICR1   = 0;

        // Enables power save
        PRR = PRR | (1 << PRTIM1);
    }
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
            // Disables power save
            PRR = PRR & ~(1 << PRTIM2);

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
    #endif
#endif

#if TIMER2_CONFIG == PWM_A || TIMER2_CONFIG == PWM_AB
    void TIMER2A_SET_OCR (uint8_t OCR)
    {
        if (OCR >= 255)
        {
            // Disable compare match
            TCCR2A = TCCR2A & ~((1 << COM2A1) | (1 << COM2A0));

            #if TIMER2A_POLATIRY == NORMAL
                gpioWriteHigh(&OC2A_PORT, OC2A_PIN);
            #else
                gpioWriteLow(&OC2A_PORT, OC2A_PIN);
            #endif
        }
        else if (OCR <= 0)
        {
            // Disable compare match
            TCCR2A = TCCR2A & ~((1 << COM2A1) | (1 << COM2A0));

            #if TIMER2A_POLATIRY == NORMAL
                gpioWriteLow(&OC2A_PORT, OC2A_PIN);
            #else
                gpioWriteHigh(&OC2A_PORT, OC2A_PIN);
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
        void INIT_TIMER2A()
        {
            // Disables power save
            PRR = PRR & ~(1 << PRTIM2);

            // Set pin as output
            gpioAsOutput(&OC2A_PORT, OC2A_PIN);

            #if TIMER2A_POLATIRY == NORMAL
                TCCR2A = TCCR2A | (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
                TCCR2B = TCCR2B | (1 << WGM22);
            #else // i.e., TIMER2A_POLATIRY == INVERTED
                TCCR2A = TCCR2A | (1 << COM2B1) | (1 << COM2A0) | (1 << WGM21) | (1 << WGM20);
            #endif
            TIMER2_SET_CLK(TIMER2_CLOCK);
            TCNT2 = 0;
            TIMER2A_SET_OCR (TIMER2A_INITIAL_OCR);
        }
    #endif
#endif

#if TIMER2_CONFIG == PWM_B || TIMER2_CONFIG == PWM_AB
    void TIMER2B_SET_OCR (uint8_t OCR)
    {
        if (OCR >= 255)
        {
            // Disable compare match
            TCCR2B = TCCR2B & ~((1 << COM2B1) | (1 << COM2B0));

            #if TIMER2B_POLATIRY == NORMAL
                gpioWriteHigh(&OC2B_PORT, OC2B_PIN);
            #else
                gpioWriteLow(&OC2B_PORT, OC2B_PIN);
            #endif
        }
        else if (OCR <= 0)
        {
            // Disable compare match
            TCCR2A = TCCR2A & ~((1 << COM2A1) | (1 << COM2A0));

            #if TIMER2B_POLATIRY == NORMAL
                gpioWriteLow(&OC2B_PORT, OC2B_PIN);
            #else
                gpioWriteHigh(&OC2B_PORT, OC2B_PIN);
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

    void INIT_TIMER2B()
    {
        // Disables power save
        PRR = PRR & ~(1 << PRTIM2);

        // Set pin as output
        gpioAsOutput(&OC2B_PORT, OC2B_PIN);

        #if TIMER2B_POLATIRY == NORMAL
            TCCR2A = TCCR2A | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
            TCCR2B = TCCR2B | (1 << WGM22);
        #else // i.e., TIMER2B_POLATIRY == INVERTED
            TCCR2A = TCCR2A | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM21) | (1 << WGM20);
        #endif
        TIMER2_SET_CLK(TIMER2_CLOCK);
        TCNT2 = 0;
        TIMER2B_SET_OCR(TIMER2B_INITIAL_OCR);
    }
#endif

#if TIMER2_CONFIG == PHASE_CORRECT
#endif

#if TIMER2_CONFIG == OFF
    void INIT_TIMER2A() {}
    void INIT_TIMER2B() {}
#endif

void TIMER2_OFF()
{
    // Clear all TIMER2 configuration registers

    #if defined (__AVR_ATmega328P__)
        TCCR2A = 0;
        TCCR2B = 0;
        TCNT0  = 0;
        OCR2A  = 0;
        OCR2B  = 0;
        TIMSK2 = 0;

        // Disables power save
        PRR = PRR | (1 << PRTIM2);
    #endif
}


