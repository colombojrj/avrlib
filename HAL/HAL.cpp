#include "HAL.h"
#include "../config.h"

void initHAL()
{
    // Initialize the TIMERs
    TIMER0_OFF();
    INIT_TIMER0A();
    INIT_TIMER0B();

    TIMER1_OFF();
    INIT_TIMER1A();
    INIT_TIMER1B();

    TIMER2_OFF();
    INIT_TIMER2A();
    INIT_TIMER2B();

    // Initialize the ADC


    // Initialize the SPI
    // TODO
    // Make the SPI INIT function adaptable (as in timers lib) to save memory
    if (SPI_MODE == OFF)
        SPI_OFF();
    else
        SPI_INIT();
}


