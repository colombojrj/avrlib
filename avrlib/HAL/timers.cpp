#include "timers.h"

uint8_t timer0MaxCount;
uint16_t timer1MaxCount;
uint8_t timer2MaxCount;
uint8_t timerActualMode[3];

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
    if (outputAConf != TIMER_OC_DISCONNECTED)
        gpioAsOutput(timer->ocA->pin);
    if (outputBConf != TIMER_OC_DISCONNECTED)
        gpioAsOutput(timer->ocB->pin);

    // Save output configuration
    *timer->ocA->actualOutputConf = outputAConf;
    *timer->ocB->actualOutputConf = outputBConf;

    // Configure timer operation mode and the gpio control
    *timer->actualMode = mode;
    *timer->regs->control = timer->availableModes[mode]             |
                            timer->ocA->availableConfs[outputAConf] |
                            timer->ocB->availableConfs[outputBConf];

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
        gpioAsOutput(timer->ocA->pin);
    if (outputBConf != 0)
        gpioAsOutput(timer->ocB->pin);

    // Configure timer operation mode and if gpio control
    *timer->regs->control = mode | outputAConf | outputBConf;

    // Save the configured output configuration
    *timer->ocA->actualOutputConf = outputAConf;
    *timer->ocB->actualOutputConf = outputBConf;

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
    uint8_t ocNormalConf = timer->ocA->availableConfs[TIMER_OC_NORMAL];

    if (duty >= *timer->maxCount)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocA->availableConfs[TIMER_OC_SET_STATE];

        if (*timer->ocA->actualOutputConf == ocNormalConf)
            gpioWriteHigh(timer->ocA->pin);
        else
            gpioWriteLow(timer->ocA->pin);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocA->availableConfs[TIMER_OC_SET_STATE];

        if (*timer->ocA->actualOutputConf == ocNormalConf)
            gpioWriteLow(timer->ocA->pin);
        else
            gpioWriteHigh(timer->ocA->pin);
    }

    else
    {
        // Enable compare match
        *timer->regs->control |= io8(*timer->ocA->actualOutputConf);

        // Set the output compare register
        *timer->ocA->compareValue = duty;
    }
}

void timerSetDutyB(const timer8b* timer, uint8_t duty)
{
    uint8_t ocNormalConf = timer->ocB->availableConfs[TIMER_OC_NORMAL];

    if (duty >= *timer->maxCount)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocB->availableConfs[TIMER_OC_SET_STATE];

        if (*timer->ocB->actualOutputConf == ocNormalConf)
            gpioWriteHigh(timer->ocB->pin);
        else
            gpioWriteLow(timer->ocB->pin);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocB->availableConfs[TIMER_OC_SET_STATE];

        if (*timer->ocB->actualOutputConf == ocNormalConf)
            gpioWriteLow(timer->ocB->pin);
        else
            gpioWriteHigh(timer->ocB->pin);
    }

    else
    {
        // Enable compare match
        *timer->regs->control |= *timer->ocB->actualOutputConf;

        // Set the output compare register
        *timer->ocB->compareValue = duty;

    }
}

void timerSetTop(const timer8b* timer, uint8_t top)
{
    *timer->maxCount = top;
    *timer->ocA->compareValue = top;
}

void timerSetTop(const timer16b* timer, uint8_t top)
{
    *timer->maxCount = top;

    if ((*timer->actualMode == TIMER_AS_CTC_TOP_ICR)     ||
        (*timer->actualMode == TIMER_AS_PWM_DEFINED_TOP) ||
        (*timer->actualMode == TIMER_AS_PWM_PHASE_CORRECT_DEFINED_TOP))
    {
        *timer->regs->inputCapture = top;
    }
    else
    {
        *timer->ocA->compareValue = top;
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
