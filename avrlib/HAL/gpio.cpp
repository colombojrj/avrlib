#include "gpio.h"

void gpioAsOutput(volatile uint8_t *port, const uint8_t pin)
{
    DDR(*port) = DDR(*port) | (1 << pin);
}

void gpioAsInput(volatile uint8_t *port, const uint8_t pin)
{
    DDR(*port) = DDR(*port) & ~(1 << pin);
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

void gpioEnablePCINT(volatile uint8_t *port, uint8_t pin)
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

void gpioDisablePCINT(volatile uint8_t *port, uint8_t pin)
{
    #if defined (__AVR_ATmega8__)
    #elif defined (__AVR_ATmega328P__)
        if (*port == PORTB)
            PCMSK0 = PCMSK0 & ~(1 << pin);
        else if (*port == PORTC)
            PCMSK1 = PCMSK1 & ~(1 << pin);
        else // i.e., PORTD
            PCMSK2 = PCMSK2 & ~(1 << pin);
    #endif
}

void gpioEnableINT(volatile uint8_t *port, uint8_t pin, uint8_t edge)
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

void gpioDisableINT(volatile uint8_t *port, uint8_t pin)
{
    #if defined (__AVR_ATmega8__)
    #elif defined (__AVR_ATmega328P__)
        if (pin == PD2)
            EIMSK = EIMSK & ~(1 << INT0);
        else
            EIMSK = EIMSK & ~(1 << INT1);
    #endif
}

void gpioSetDuty(volatile uint8_t *port, uint8_t pin, uint16_t duty)
{
    #if defined (__AVR_ATmega328P__)
        if (*port == PORTD)
        {
            if (pin == PD6)      // OC0A
                TIMER0A_SET_OCR((uint8_t) (duty & 0x00FF));
            else if (pin == PD5) // OC0B
                TIMER0B_SET_OCR((uint8_t) (duty & 0x00FF));
            else if (pin == PD3) // OC2B
                TIMER2B_SET_OCR((uint8_t) (duty & 0x00FF));
        }
        else // i.e., *port == PORTB
        {
            if (pin == PB1)      // OC1A
                TIMER1A_SET_OCR(duty);
            else if (pin == PB2) // OC1B
                TIMER1B_SET_OCR(duty);
            else if (pin == PB3) // OC2A
                TIMER2A_SET_OCR((uint8_t) (duty & 0x00FF));
        }
    #endif
}
