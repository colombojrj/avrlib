#include "gpio.h"

void gpioDirection(gpio_t* gpio, const uint8_t dir)
{
    if (dir == OUTPUT)
        gpioAsOutput(gpio);
    else
        gpioAsInput(gpio);
}

void gpioWrite(gpio_t* gpio, const uint8_t level)
{
    if (level == LOW)
        gpioWriteLow(gpio);
    else
        gpioWriteHigh(gpio);
}

#if defined (PCMSK0) || defined(PCMSK1) || defined(PCMSK2)
void gpioEnablePCINT(volatile uint8_t *port, uint8_t pin)
{
    if (*port == PORTB)
    {
        PCICR = PCICR | (1 << PCIE0);
        PCMSK0 = PCMSK0 | (1 << pin);
    }
    else if (*port == PORTC)
    {
        PCICR = PCICR | (1 << PCIE1);
        PCMSK1 = PCMSK1 | (1 << pin);
    }
    else if (*port == PORTD)
    {
        PCICR = PCICR | (1 << PCIE2);
        PCMSK2 = PCMSK2 | (1 << pin);
    }
}

void gpioDisablePCINT(volatile uint8_t *port, uint8_t pin)
{
    if (*port == PORTB)
    {
        PCMSK0 = PCMSK0 & ~(1 << pin);

        /* If any pin have an interrupt attached, then there is
         * no need to keep the interrupt generator active
         */
        if (PCMSK0 == 0)
            PCICR = PCICR & ~(1 << PCIE0);
    }

    else if (*port == PORTC)
    {
        PCMSK1 = PCMSK1 & ~(1 << pin);

        if (PCMSK1 == 0)
            PCICR = PCICR & ~(1 << PCIE1);
    }

    else if (*port == PORTD)
    {
        PCMSK2 = PCMSK2 & ~(1 << pin);

        if (PCMSK2 == 0)
            PCICR = PCICR & ~(1 << PCIE2);
    }
}
#endif

void gpioEnableINT(gpio_t* gpio, gpioInt_t trigger)
{
    EICRA = EICRA & ~static_cast<uint8_t>(gpioInt_t::setState);
    EICRA = EICRA | static_cast<uint8_t>(trigger);
    EIMSK = EIMSK | (gpio->hasInt << gpio->whatInt);
}

void gpioDisableINT(gpio_t* gpio)
{
	EIMSK = EIMSK & ~(gpio->hasInt << gpio->whatInt);
}

void gpioSetDuty(gpio_t* gpio, const uint16_t duty)
{
    #if defined(SUPPORT_TO_TIMER0)
        if (*gpio->port == *OC0A_PIN.port && gpio->pinNumber == OC0A_PIN.pinNumber)
            timer0ASetDuty((uint8_t) (duty & 0xFF), timer0OutputConfig);

        else if (*gpio->port == *OC0B_PIN.port && gpio->pinNumber == OC0B_PIN.pinNumber)
            timer0BSetDuty((uint8_t) (duty & 0xFF), timer0OutputConfig);
    #endif

    #if defined(SUPPORT_TO_TIMER1)
        if (*gpio.port == *OC1A_PIN.port && gpio.number == OC1A_PIN.number)
            timer1ASetDuty(duty, timer1OutputConfig, timer1TopValue);

        else if (*gpio.port == *OC1B_PIN.port && gpio.number == OC1B_PIN.number)
            timer1BSetDuty(duty, timer1OutputConfig, timer1TopValue);
    #endif

    #if defined(SUPPORT_TO_TIMER2)
        if (*gpio.port == *OC2A_PIN.port && gpio.number == OC2A_PIN.number)
            timer2ASetDuty((uint8_t) (duty & 0xFF), timer2OutputConfig);

        else if (*gpio.port == *OC2B_PIN.port && gpio.number == OC2B_PIN.number)
            timer2BSetDuty((uint8_t) (duty & 0xFF), timer2OutputConfig);
    #endif
}
