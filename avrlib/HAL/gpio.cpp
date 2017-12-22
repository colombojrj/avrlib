#include "gpio.h"

void gpioAsOutput(volatile uint8_t *port, const uint8_t pin)
{
    DDR(*port) = DDR(*port) | (1 << pin);
}

void gpioAsInput(volatile uint8_t *port, const uint8_t pin, const uint8_t pullUp)
{
    DDR(*port) = DDR(*port) & ~(1 << pin);
    if (pullUp > 0)
        gpioWriteHigh(port, pin);
}

void gpioAsAdc(volatile uint8_t *port, const uint8_t pin)
{
	gpioAsInput(port, pin, 0);
	adcInit(pin);
}

void gpioAsPwm(volatile uint8_t *port, const uint8_t pin)
{
	gpioAsOutput(port, pin);
}

void gpioDirection(volatile uint8_t *port, const uint8_t pin, const uint8_t dir)
{
    if (dir == OUTPUT)
        gpioAsOutput(port, pin);
    else
        gpioAsInput(port, pin, dir);
}

void gpioWriteHigh(volatile uint8_t *port, const uint8_t pin)
{
    *port = *port | (1 << pin);
}

void gpioWriteLow(volatile uint8_t *port, const uint8_t pin)
{
    *port = *port & ~(1 << pin);
}

void gpioWrite(volatile uint8_t *port, const uint8_t pin, const uint8_t state)
{
    if (state == LOW)
        gpioWriteLow(port, pin);
    else
        gpioWriteHigh(port, pin);
}

void gpioToggle(volatile uint8_t *port, const uint8_t pin)
{
    *port = *port ^ (1 << pin);
}

uint8_t gpioFastRead(volatile uint8_t *port, uint8_t pin)
{
    return (PIN(*port) & (1 << pin));
}

uint8_t gpioRead(volatile uint8_t *port, uint8_t pin)
{
    if (gpioFastRead(port, pin) > 0)
        return 1;
    else
        return 0;
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

void gpioSetDuty(volatile uint8_t *port, uint8_t pin, uint16_t duty)
{
    if (*port == OC0A_PORT && pin == OC0A_PIN)
        timer0ASetDuty((uint8_t) (duty & 0xFF), timer0OutputConfig);

    else if (*port == OC0B_PORT && pin == OC0B_PIN)
        timer0BSetDuty((uint8_t) (duty & 0xFF), timer0OutputConfig);

    else if (*port == OC1A_PORT && pin == OC1A_PIN)
        timer1ASetDuty(duty, timer1OutputConfig, timer1TopValue);

    else if (*port == OC1B_PORT && pin == OC1B_PIN)
        timer1BSetDuty(duty, timer1OutputConfig, timer1TopValue);

    else if (*port == OC2A_PORT && pin == OC2A_PIN)
        timer2ASetDuty((uint8_t) (duty & 0xFF), timer2OutputConfig);

    else if (*port == OC2B_PORT && pin == OC2B_PIN)
        timer2BSetDuty((uint8_t) (duty & 0xFF), timer2OutputConfig);
}
