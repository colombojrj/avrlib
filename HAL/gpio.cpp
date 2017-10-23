#include "gpio.h"

void gpioAsOutput(uint8_t *port, uint8_t pin)
{
    DDR(*port) = DDR(*port) | (1 << pin);
}

void gpioAsInput(uint8_t *port, uint8_t pin)
{
    DDR(*port) = DDR(*port) & ~(1 << pin);
}

void gpioDirection(uint8_t *port, uint8_t pin, uint8_t dir)
{
    if (dir == OUTPUT)
        gpioAsOutput(port, pin);
    else
        gpioAsInput(port, pin);
}

void gpioWriteHigh(uint8_t *port, uint8_t pin)
{
    *port = *port | (1 << pin);
}

void gpioWriteLow(uint8_t *port, uint8_t pin)
{
    *port = *port & ~(1 << pin);
}

void gpioWrite(uint8_t *port, uint8_t pin, uint8_t state)
{
    if (state == HIGH)
        gpioWriteHigh(port, pin);
    else
        gpioWriteLow(port, pin);
}

void gpioToggle(uint8_t *port, uint8_t pin)
{
    *port = *port ^ (1 << pin);
}

uint8_t gpioFastRead(uint8_t *port, uint8_t pin)
{
    return (PIN(*port) & (1 << pin));
}

uint8_t gpioRead(uint8_t *port, uint8_t pin)
{
    if (gpioFastRead(port, pin))
        return 1;
    else
        return 0;
}

void enablePCINT(uint8_t *port, uint8_t pin)
{
    #if defined (__AVR_ATmega8__)
    #elif defined (__AVR_ATmega328P__)
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
        else // i.e., PORTD
        {
            PCICR = PCICR | (1 << PCIE2);
            PCMSK2 = PCMSK2 | (1 << pin);
        }
    #endif
}

void disablePCINT(uint8_t *port, uint8_t pin)
{
    #if defined (__AVR_ATmega8__)
    #elif defined (__AVR_ATmega328P__)
        if (*port == PORTB)
        {
            PCMSK0 = PCMSK0 & ~(1 << pin);
        }
        else if (*port == PORTC)
        {
            PCMSK1 = PCMSK1 & ~(1 << pin);
        }
        else // i.e., PORTD
        {
            PCMSK2 = PCMSK2 & ~(1 << pin);
        }
    #endif
}

void enableINT(uint8_t *port, uint8_t pin, uint8_t edge)
{

    #if defined (__AVR_ATmega8__)
    #elif defined (__AVR_ATmega328P__)
        if (pin == PD2) // i.e., INT0
        {
            if (edge == LOW_LEVEL)
            {
                EICRA &= ~((1 << ISC01) | (1 << ISC00));
            }
            else if (edge == ANY_CHANGE)
            {
                EICRA &= ~(1 << ISC01);
                EICRA |= (1 << ISC00);
            }
            else if (edge == FALLING_EDGE)
            {
                EICRA |= (1 << ISC01);
                EICRA &= ~(1 << ISC00);
            }
            else
            {
                EICRA |= (1 << ISC01) | (1 << ISC00);
            }
            EIMSK |= (1 << INT0);
        }
        else // i.e. PD3 (INT1)
        {
            if (edge == LOW_LEVEL)
            {
                EICRA &= ~((1 << ISC11) | (1 << ISC10));
            }
            else if (edge == ANY_CHANGE)
            {
                EICRA &= ~(1 << ISC11);
                EICRA |= (1 << ISC10);
            }
            else if (edge == FALLING_EDGE)
            {
                EICRA |= (1 << ISC11);
                EICRA &= ~(1 << ISC10);
            }
            else
            {
                EICRA |= (1 << ISC11) | (1 << ISC10);
            }
            EIMSK |= (1 << INT1);
        }
    #endif
}

void disableINT(uint8_t *port, uint8_t pin)
{
    #if defined (__AVR_ATmega8__)
    #elif defined (__AVR_ATmega328P__)
        if (pin == PD2)
            EIMSK = EIMSK & ~(1 << INT0);
        else
            EIMSK = EIMSK & ~(1 << INT1);
    #endif
}
