#include "gpio.h"

/**
 * @todo Migrate from const uint8_t dir (OUTPUT or INPUT) to enum
 */
void gpioDirection(port_t* port, const uint8_t pinNumber, const uint8_t dir)
{
    if (dir == OUTPUT)
        gpioAsOutput(port, pinNumber);
    else
        gpioAsInput(port, pinNumber);
}

void gpioWrite(port_t* port, const uint8_t pinNumber, const uint8_t level)
{
    if (level == LOW)
        gpioWriteLow(port, pinNumber);
    else
        gpioWriteHigh(port, pinNumber);
}

#if defined (__AVR_ATmega48__)  || defined(__AVR_ATmega48P__) || \
            (__AVR_ATmega88__)  || defined(__AVR_ATmega88P__) || \
            (__AVR_ATmega168__)  || defined(__AVR_ATmega168P__) || \
            (__AVR_ATmega328__)  || defined(__AVR_ATmega328P__)
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

void gpioEnableINT(volatile uint8_t *port, uint8_t pin, gpioInt_t trigger)
{
    if (pin == static_cast<uint8_t>(gpioIntPin_t::int0))
    {
        EICRA = EICRA & ~static_cast<uint8_t>(gpioInt_t::setState);
        EICRA = EICRA | static_cast<uint8_t>(trigger);
        EIMSK |= (1 << INT0);
    }
    else if (pin == static_cast<uint8_t>(gpioIntPin_t::int1)) // i.e. INT1
    {
        EICRA = EICRA & ~(static_cast<uint8_t>(gpioInt_t::setState) << 2);
        EICRA = EICRA | (static_cast<uint8_t>(trigger) << 2);
        EIMSK |= (1 << INT1);
    }
}

void gpioDisableINT(volatile uint8_t *port, uint8_t pin)
{
    if (pin == static_cast<uint8_t>(gpioIntPin_t::int0))
    {
        EIMSK = EIMSK & ~(1 << INT0);
    }
    else if (pin == static_cast<uint8_t>(gpioIntPin_t::int1))
    {
        EIMSK = EIMSK & ~(1 << INT1);
    }
}

void gpioSetDuty(gpio_t gpio, const uint16_t duty)
{
    if (*gpio.port == *OC0A_PIN.port && gpio.number == OC0A_PIN.number)
        timer0ASetDuty((uint8_t) (duty & 0xFF), timer0OutputConfig);

    else if (*gpio.port == *OC0B_PIN.port && gpio.number == OC0B_PIN.number)
        timer0BSetDuty((uint8_t) (duty & 0xFF), timer0OutputConfig);

    else if (*gpio.port == *OC1A_PIN.port && gpio.number == OC1A_PIN.number)
        timer1ASetDuty(duty, timer1OutputConfig, timer1TopValue);

    else if (*gpio.port == *OC1B_PIN.port && gpio.number == OC1B_PIN.number)
        timer1BSetDuty(duty, timer1OutputConfig, timer1TopValue);

    else if (*gpio.port == *OC2A_PIN.port && gpio.number == OC2A_PIN.number)
        timer2ASetDuty((uint8_t) (duty & 0xFF), timer2OutputConfig);

    else if (*gpio.port == *OC2B_PIN.port && gpio.number == OC2B_PIN.number)
        timer2BSetDuty((uint8_t) (duty & 0xFF), timer2OutputConfig);
}
