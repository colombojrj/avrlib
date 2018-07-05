#include "gpio.h"

void gpioSetDir(const gpio_t* gpio, GpioDir dir)
{
    if (dir == GpioDir::output)
        gpioAsOutput(gpio);
    else
        gpioAsInput(gpio);
}

void gpioWrite(const gpio_t* gpio, const uint8_t level)
{
    if (level == 0)
        gpioWriteLow(gpio);
    else
        gpioWriteHigh(gpio);
}

void gpioEnablePCINT(const gpio_t* gpio)
{
    #if defined (PCMSK0) || defined(PCMSK1) || defined(PCMSK2)
        // Enable interrupt generator
        PCICR = PCICR | (1 << gpio->regs.whatPCI);

        // Unmask pin interrupt
        *gpio->regs.pcmsk = *gpio->regs.pcmsk | (1 << gpio->pinNumber);
    #endif
}

void gpioDisablePCINT(const gpio_t* gpio)
{
    #if defined (PCMSK0) || defined(PCMSK1) || defined(PCMSK2)
        // Mask pin interrupt
        *gpio->regs.pcmsk = *gpio->regs.pcmsk & ~(1 << gpio->pinNumber);

        /*
         * If the pcmsk register is empty, then the interrupt generator may
         * be disabled to save some power.
         */
        if (*gpio->regs.pcmsk == 0)
            PCICR = PCICR & ~(1 << gpio->regs.whatPCI);
#endif
}

void gpioEnableINT(const gpio_t* gpio, gpioTrigger trigger)
{
    // Clean previous configuration
    EICRA = EICRA & ~static_cast<uint8_t>(gpioTrigger::setState);

    EICRA = EICRA | static_cast<uint8_t>(trigger);
    EIMSK = EIMSK | (gpio->hasInt << gpio->whatInt);
}

void gpioDisableINT(const gpio_t* gpio)
{
	EIMSK = EIMSK & ~(gpio->hasInt << gpio->whatInt);
}
