#include "timers.h"

uint8_t whatTimer0OutputAConfig;
uint8_t whatTimer0OutputBConfig;
uint8_t whatTimer1OutputAConfig;
uint8_t whatTimer1OutputBConfig;
uint8_t whatTimer2OutputAConfig;
uint8_t whatTimer2OutputBConfig;
uint8_t timer0MaxCount;
uint16_t timer1MaxCount;
uint8_t timer2MaxCount;

void timerSetClockPreescaler(const timer8b* timer, uint16_t clockConf)
{
    *timer->regs->control = *timer->regs->control & ~0x0700;
    *timer->regs->control = *timer->regs->control | clockConf;
}

void timerSetClockPreescaler(const timer16b* timer, uint16_t clockConf)
{
    *timer->regs->control = *timer->regs->control & ~0x0700;
    *timer->regs->control = *timer->regs->control | clockConf;
}

void timerSetOutputCompareUnitA(const timer8b* timer, uint8_t whatConf)
{
    *timer->regs->control = *timer->regs->control & ~timer->ocASetState;
    *timer->regs->control = *timer->regs->control | timer->ocAConfs[io8Conf(whatConf)];
    *timer->outputConfA = timer->ocAConfs[io8Conf(whatConf)];
}

void timerSetOutputCompareUnitB(const timer8b* timer, uint8_t whatConf)
{
    *timer->regs->control = *timer->regs->control & ~timer->ocBSetState;
    *timer->regs->control = *timer->regs->control | timer->ocBConfs[io8Conf(whatConf)];
    *timer->outputConfB = timer->ocBConfs[io8Conf(whatConf)];
}

void timerSetOutputCompareUnitA(const timer16b* timer, uint8_t whatConf)
{
    *timer->regs->control = *timer->regs->control & ~timer->ocASetState;
    *timer->regs->control = *timer->regs->control | timer->ocAConfs[io8Conf(whatConf)];
    *timer->outputConfA = timer->ocAConfs[io8Conf(whatConf)];
}

void timerSetOutputCompareUnitB(const timer16b* timer, uint8_t whatConf)
{
    *timer->regs->control = *timer->regs->control & ~timer->ocBSetState;
    *timer->regs->control = *timer->regs->control | timer->ocBConfs[io8Conf(whatConf)];
    *timer->outputConfB = timer->ocBConfs[io8Conf(whatConf)];
}

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
        gpioAsOutput(timer->regs->outputPinA);
    if (outputBConf != 0)
        gpioAsOutput(timer->regs->outputPinB);

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
    *timer->regs->control = mode | outputAConf| outputBConf;

    // Save the configured output configuration
    *timer->outputConfA = outputAConf;
    *timer->outputConfB = outputBConf;

    // Clock source configuration
    *timer->regs->control = *timer->regs->control | clockConf;

    // Configure interrupt
    *timer->regs->interruptMask = interruptConf;
    if (interruptConf != 0)
        sei();
}

void timerSetDutyA(const timer8b* timer, uint8_t duty)
{
    if (duty >= *timer->maxCount)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocASetState;

        if (*timer->outputConfA == timer->ocAConfs[io8Conf(timerOutputConfig::normal)])
            gpioWriteHigh(timer->regs->outputPinA);
        else
            gpioWriteLow(timer->regs->outputPinA);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocASetState;

        if (*timer->outputConfA == timer->ocAConfs[io8Conf(timerOutputConfig::normal)])
            gpioWriteLow(timer->regs->outputPinA);
        else
            gpioWriteHigh(timer->regs->outputPinA);
    }

    else
    {
        // Enable compare match
        *timer->regs->control |= io8Conf(*timer->outputConfA);

        // Set the output compare register
        *timer->regs->outputCompareA = duty;
    }
}

void timerSetDutyB(const timer8b* timer, uint8_t duty)
{
    if (duty >= *timer->maxCount)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocBSetState;

        if (*timer->outputConfB == timer->ocBConfs[io8Conf(timerOutputConfig::normal)])
            gpioWriteHigh(timer->regs->outputPinB);
        else
            gpioWriteLow(timer->regs->outputPinB);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        *timer->regs->control &= ~timer->ocBSetState;

        if (*timer->outputConfB == timer->ocBConfs[io8Conf(timerOutputConfig::normal)])
            gpioWriteLow(timer->regs->outputPinB);
        else
            gpioWriteHigh(timer->regs->outputPinB);
    }

    else
    {
        // Enable compare match
        *timer->regs->control |= io8Conf(*timer->outputConfB);

        // Set the output compare register
        *timer->regs->outputCompareB = duty;
    }
}


#if defined(SUPPORT_TO_TIMER0)

void timer0ASetDuty(uint8_t duty, timer0OutputConfig_t outputConfig)
{
    if (duty >= 255)
    {
        // Disable compare match
        TCCR0A = TCCR0A & ~static_cast<uint8_t>(timer0OutputConfig_t::channelAsetState);

        if (outputConfig == timer0OutputConfig_t::channelAnormal ||
            outputConfig == timer0OutputConfig_t::channelABnormal)
            gpioWriteHigh(OC0A_PIN);
        else
            gpioWriteLow(OC0A_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR0A = TCCR0A & ~static_cast<uint8_t>(timer0OutputConfig_t::channelAsetState);

        if (outputConfig == timer0OutputConfig_t::channelAnormal ||
            outputConfig == timer0OutputConfig_t::channelABnormal)
            gpioWriteLow(OC0A_PIN);
        else
            gpioWriteHigh(OC0A_PIN);
    }

    else
    {
        // Enable compare match
        TCCR0A = TCCR0A | static_cast<uint8_t>(outputConfig);

        // Set the output compare register
        OCR0A = duty;
    }
}

void timer0BSetDuty(uint8_t duty, timer0OutputConfig_t outputConfig)
{
    if (duty >= 255)
    {
        // Disable compare match
        TCCR0A = TCCR0A & ~static_cast<uint8_t>(timer0OutputConfig_t::channelBsetState);

        if (outputConfig == timer0OutputConfig_t::channelBnormal ||
            outputConfig == timer0OutputConfig_t::channelABnormal)
            gpioWriteHigh(OC0B_PIN);
        else
            gpioWriteLow(OC0B_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR0A = TCCR0A & ~static_cast<uint8_t>(timer0OutputConfig_t::channelBsetState);

        if (outputConfig == timer0OutputConfig_t::channelBnormal ||
            outputConfig == timer0OutputConfig_t::channelABnormal)
            gpioWriteLow(OC0B_PIN);
        else
            gpioWriteHigh(OC0B_PIN);
    }

    else
    {
        // Enable compare match
        TCCR0A = TCCR0A | static_cast<uint8_t>(outputConfig);

        // Set the output compare register
        OCR0B = duty;
    }
}

void timer0Init(timer0Config_t config,
                timer0Clock_t clock,
                timer0OutputConfig_t outputConfig)
{
    #if defined (PRR)
        // Enables tTimer/Counter0 module
        PRR = PRR & ~(1 << PRTIM0);
    #endif

    // Pin configuration
    if (outputConfig == timer0OutputConfig_t::channelAnormal   ||
        outputConfig == timer0OutputConfig_t::channelAinverted ||
        outputConfig == timer0OutputConfig_t::channelABnormal  ||
        outputConfig == timer0OutputConfig_t::channelABinverted)
    {
        gpioAsOutput(OC0A_PIN);
    }

    if (outputConfig == timer0OutputConfig_t::channelBnormal   ||
        outputConfig == timer0OutputConfig_t::channelBinverted ||
        outputConfig == timer0OutputConfig_t::channelABnormal  ||
        outputConfig == timer0OutputConfig_t::channelABinverted)
    {
        gpioAsOutput(OC0B_PIN);
    }

    // Set timer 0 configuration
	#if defined (TCCR0A)
        TCCR0A = static_cast<uint8_t>(config) | static_cast<uint8_t>(outputConfig);

    #elif defined (TCCR0) // ATmega8, for example
	#endif

    // Set timer0 clock source
    TCCR0B = static_cast<uint8_t>(clock);
    if (clock == timer0Clock_t::externT0FallingEdge || clock == timer0Clock_t::externT0RisingEdge)
    {
    	gpioAsInput(T0_PIN);
    	gpioPullUpEnable(T0_PIN);
    }

    // Enable interrupts
    if (config == timer0Config_t::normal)
    {
        TIMSK0 = (1 << TOIE0); // overflow
        sei();
    }

    else if (config == timer0Config_t::ctc)
    {
        TIMSK0 = (1 << OCIE0A);
        sei();
    }

    // Set timer 2 clock source
    TCCR0B = static_cast<uint8_t>(clock);
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

#endif

//
///////////////////
// TIMER1 MODULE //
///////////////////
//

#if defined(SUPPORT_TO_TIMER1)

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
        gpioAsOutput(OC1A_PIN);
    }

    if (outputConfig == timer1OutputConfig_t::channelBnormal   ||
        outputConfig == timer1OutputConfig_t::channelBinverted ||
        outputConfig == timer1OutputConfig_t::channelABnormal  ||
        outputConfig == timer1OutputConfig_t::channelABinverted)
    {
        gpioAsOutput(OC1B_PIN);
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
    if (clock == timer1Clock_t::externT1FallingEdge || clock == timer1Clock_t::externT1RisingEdge)
    {
        gpioAsInput(T1_PIN);
        gpioPullUpEnable(T1_PIN);
    }

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
            gpioWriteHigh(OC1A_PIN);
        else
            gpioWriteLow(OC1A_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR1A = TCCR1A & ~static_cast<uint8_t>(timer1OutputConfig_t::channelAsetState);

        if (output == timer1OutputConfig_t::channelAnormal ||
            output == timer1OutputConfig_t::channelABnormal)
            gpioWriteLow(OC1A_PIN);
        else
            gpioWriteHigh(OC1A_PIN);
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
            gpioWriteHigh(OC1B_PIN);
        else
            gpioWriteLow(OC1B_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR1A = TCCR1A & ~static_cast<uint8_t>(timer1OutputConfig_t::channelBsetState);

        if (output == timer1OutputConfig_t::channelBnormal ||
            output == timer1OutputConfig_t::channelABnormal)
            gpioWriteLow(OC1B_PIN);
        else
            gpioWriteHigh(OC1B_PIN);
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

#endif

//
///////////////////
// TIMER2 MODULE //
///////////////////
//

#if defined(SUPPORT_TO_TIMER2)

void timer2Init(timer2Config_t config,
                timer2Clock_t clock,
                timer2OutputConfig_t outputConfig)
{
    #if defined (PRR)
        // Enables Timer/Counter2 module
        PRR = PRR & ~(1 << PRTIM2);
    #endif

    // Pin configuration
    if (outputConfig == timer2OutputConfig_t::channelAnormal   ||
        outputConfig == timer2OutputConfig_t::channelAinverted ||
        outputConfig == timer2OutputConfig_t::channelABnormal  ||
        outputConfig == timer2OutputConfig_t::channelABinverted)
    {
        gpioAsOutput(OC2A_PIN);
    }

    if (outputConfig == timer2OutputConfig_t::channelBnormal   ||
        outputConfig == timer2OutputConfig_t::channelBinverted ||
        outputConfig == timer2OutputConfig_t::channelABnormal  ||
        outputConfig == timer2OutputConfig_t::channelABinverted)
    {
        gpioAsOutput(OC2B_PIN);
    }

    // Set timer 2 configuration
    #if defined (TCCR2A)
        TCCR2A = static_cast<uint8_t>(config) | static_cast<uint8_t>(outputConfig);
    #elif defined (TCCR2) // ATmega8, for example
    #endif

    // Enable interrupts
    if (config == timer2Config_t::normal)
    {
        TIMSK2 = (1 << TOIE2); // overflow
        sei();
    }

    else if (config == timer2Config_t::ctc)
    {
        TIMSK2 = (1 << OCIE2A);
        sei();
    }

    // Set timer 2 clock source
    TCCR2B = static_cast<uint8_t>(clock);
}

void timer2ASetDuty(uint8_t duty, timer2OutputConfig_t outputConfig)
{
    if (duty >= 255)
    {
        // Disable compare match
        TCCR2A = TCCR2A & ~static_cast<uint8_t>(timer2OutputConfig_t::channelAsetState);

        if (outputConfig == timer2OutputConfig_t::channelAnormal ||
            outputConfig == timer2OutputConfig_t::channelABnormal)
            gpioWriteHigh(OC2A_PIN);
        else
            gpioWriteLow(OC2A_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR2A = TCCR2A & ~static_cast<uint8_t>(timer2OutputConfig_t::channelAsetState);

        if (outputConfig == timer2OutputConfig_t::channelAnormal ||
            outputConfig == timer2OutputConfig_t::channelABnormal)
            gpioWriteLow(OC2A_PIN);
        else
            gpioWriteHigh(OC2A_PIN);
    }

    else
    {
        // Enable compare match
        TCCR2A = TCCR2A | static_cast<uint8_t>(outputConfig);

        // Set the output compare register
        OCR2A = duty;
    }
}

void timer2BSetDuty(uint8_t duty, timer2OutputConfig_t outputConfig)
{
    if (duty >= 255)
    {
        // Disable compare match
        TCCR2A = TCCR2A & ~static_cast<uint8_t>(timer2OutputConfig_t::channelBsetState);

        if (outputConfig == timer2OutputConfig_t::channelBnormal ||
            outputConfig == timer2OutputConfig_t::channelABnormal)
            gpioWriteHigh(OC2B_PIN);
        else
            gpioWriteLow(OC2B_PIN);
    }

    else if (duty <= 0)
    {
        // Disable compare match
        TCCR2A = TCCR2A & ~static_cast<uint8_t>(timer2OutputConfig_t::channelBsetState);

        if (outputConfig == timer2OutputConfig_t::channelBnormal ||
            outputConfig == timer2OutputConfig_t::channelABnormal)
            gpioWriteLow(OC2B_PIN);
        else
            gpioWriteHigh(OC2B_PIN);
    }

    else
    {
        // Enable compare match
        TCCR2A = TCCR2A | static_cast<uint8_t>(outputConfig);

        // Set the output compare register
        OCR2B = duty;
    }
}

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
