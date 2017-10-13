#include "HAL.h"
#include "../config.h"

void HAL_init()
{
    // Initialize the TIMERs
    INIT_TIMER0A(0);
    INIT_TIMER0B(0);
    INIT_TIMER1A(0);
    INIT_TIMER1B(0);
    INIT_TIMER2A(0);
    INIT_TIMER2B(0);

    // Initialize the ADC


    // Initialize the SPI
    SPI_INIT();
}


