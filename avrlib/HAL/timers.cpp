#include "timers.h"

uint8_t timer0WhatOutputAConfig;
uint8_t timer0WhatOutputBConfig;
uint8_t timer1WhatOutputAConfig;
uint8_t timer1WhatOutputBConfig;
uint8_t timer2WhatOutputAConfig;
uint8_t timer2WhatOutputBConfig;
uint8_t timer0MaxCount;
uint16_t timer1MaxCount;
uint8_t timer2MaxCount;

void timerInit(const timer8b* timer,
               uint16_t mode,
               uint16_t clockConf,
               uint8_t outputAConf,
               uint8_t outputBConf,
               uint8_t interruptConf)
{
    // Enables the timer module
    #if defined (PRR)
        PRR = PRR & ~(1 << timer->regs->whatPRR);
    #endif

    // Configures gpio peripheral control
    if (outputAConf != 0)
        gpioAsOutput(timer->regs->ocRegs->pinA);
    if (outputBConf != 0)
        gpioAsOutput(timer->regs->ocRegs->pinB);

    // Configure timer operation mode and if gpio control
    *timer->regs->control = mode | timer->ocAConfs[io8Conf(outputAConf)]
                                 | timer->ocBConfs[io8Conf(outputBConf)];

    // Save output configuration
    *timer->outputConfA = timer->ocAConfs[io8Conf(outputAConf)];
    *timer->outputConfB = timer->ocBConfs[io8Conf(outputBConf)];

    // Clock source configuration
    *timer->regs->control = *timer->regs->control | clockConf;

    // Configure interrupt
    *timer->regs->interruptMask = interruptConf;
    if (interruptConf != 0)
        sei();

    // Sets timer max count
    *timer->maxCount = 255;
}

void timerInit(const timer16b* timer,
               uint16_t mode,
               uint16_t clockConf,
               uint8_t outputAConf,
               uint8_t outputBConf,
               uint8_t interruptConf)
{
    // Enables the timer module
    #if defined (PRR)
        PRR = PRR & ~(1 << timer->regs->whatPRR);
    #endif

    // Configures gpio peripheral control
    if (outputAConf != 0)
        gpioAsOutput(timer->regs->outputPinA);
    if (outputBConf != 0)
        gpioAsOutput(timer->regs->outputPinB);

    // Configure timer operation mode and if gpio control
    *timer->regs->control = mode | outputAConf | outputBConf;

    // Save the configured output configuration
    *timer->outputConfA = outputAConf;
    *timer->outputConfB = outputBConf;

    // Clock source configuration
    *timer->regs->control = *timer->regs->control | clockConf;

    // Configure interrupt
    *timer->regs->interruptMask = interruptConf;
    if (interruptConf != 0)
        sei();

    // @todo Finish the maxCount initialization
    *timer->maxCount = 0xFFFF;
    if (mode != 0)
    {

    }
}

void timerSetDutyA(const timer8b* timer, uint8_t duty)
{
    if (duty >= *timer->maxCount)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocASetState;

        if (*timer->outputConfA == timer->ocAConfs[io8Conf(timerOutputCompareMode::normal)])
            gpioWriteHigh(timer->regs->ocRegs->pinA);
        else
            gpioWriteLow(timer->regs->ocRegs->pinA);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocASetState;

        if (*timer->outputConfA == timer->ocAConfs[io8Conf(timerOutputCompareMode::normal)])
            gpioWriteLow(timer->regs->ocRegs->pinA);
        else
            gpioWriteHigh(timer->regs->ocRegs->pinA);
    }

    else
    {
        // Enable compare match
        *timer->regs->control |= io8Conf(*timer->outputConfA);

        // Set the output compare register
        *timer->regs->ocRegs->compareValueA = duty;
    }
}

void timerSetDutyB(const timer8b* timer, uint8_t duty)
{
    if (duty >= *timer->maxCount)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocBSetState;

        if (*timer->outputConfB == timer->ocBConfs[io8Conf(timerOutputCompareMode::normal)])
            gpioWriteHigh(timer->regs->ocRegs->pinB);
        else
            gpioWriteLow(timer->regs->ocRegs->pinB);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocBSetState;

        if (*timer->outputConfB == timer->ocBConfs[io8Conf(timerOutputCompareMode::normal)])
            gpioWriteLow(timer->regs->ocRegs->pinB);
        else
            gpioWriteHigh(timer->regs->ocRegs->pinB);
    }

    else
    {
        // Enable compare match
        *timer->regs->control |= io8Conf(*timer->outputConfB);

        // Set the output compare register
        *timer->regs->ocRegs->compareValueB = duty;

    }
}


#if defined(SUPPORT_TO_TIMER0)

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

#endif

//
///////////////////
// TIMER1 MODULE //
///////////////////
//

#if defined(SUPPORT_TO_TIMER1)

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

#endif

//
///////////////////
// TIMER2 MODULE //
///////////////////
//

#if defined(SUPPORT_TO_TIMER2)

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

#endif
