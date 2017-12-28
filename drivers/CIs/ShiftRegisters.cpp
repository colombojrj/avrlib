#include "ShiftRegisters.h"

ShiftRegister74HC595::ShiftRegister74HC595(DigitalPin *serialDataInput,
                                           DigitalPin *shiftRegisterClock,
                                           DigitalPin *storageRegisterClock,
                                           DigitalPin *outputEnable,
                                           DigitalPin *masterReset)
{
    // Pin configuration
    sdi = serialDataInput;
    shcp = shiftRegisterClock;
    stcp = storageRegisterClock;
    oe = outputEnable;
    mr = masterReset;

    // Initializes the 74LS595 pins
    sdi->setPinMode(gpioConfig_t::output, LOW);
    shcp->setPinMode(gpioConfig_t::output, LOW);
    stcp->setPinMode(gpioConfig_t::output, LOW);

    // Initializes the 74LS595 pins
    if (outputEnable != nullptr)
        oe->setPinMode(gpioConfig_t::output, HIGH);

    if (outputEnable != nullptr)
        mr->setPinMode(gpioConfig_t::output, HIGH);

    write(actualValue);
}

void ShiftRegister74HC595::write(uint8_t value)
{
    uint8_t i;
    actualValue = value;

    for (i = 0; i < 8; i++)
    {
        // Data
        sdi->write(value & (1 << i));

        // Clock toggle
        shcp->writeHigh();
        shcp->writeLow();
    }

    stcp->writeHigh();
    stcp->writeLow();
}

void ShiftRegister74HC595::setPin(uint8_t pin)
{
    actualValue = actualValue | (1 << pin);
    write(actualValue);
}

void ShiftRegister74HC595::clearPin(uint8_t pin)
{
    actualValue = actualValue & ~(1 << pin);
    write(actualValue);
}

void ShiftRegister74HC595::write(uint8_t pin, uint8_t value)
{
    if (value == 0)
        clearPin(pin);
    else
        setPin(pin);
}


void ShiftRegister74HC595::enableOutput()
{
    if (oe != nullptr)
        oe->writeLow();
}

void ShiftRegister74HC595::disableOutput()
{
    if (oe != nullptr)
        oe->writeHigh();
}

void ShiftRegister74HC595::enableHighImpedance()
{
    mr->writeLow();
    oe->writeHigh();
}

void ShiftRegister74HC595::disableHighImpedance()
{
    mr->writeHigh();
    oe->writeLow();
}
