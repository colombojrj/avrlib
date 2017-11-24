#include "HAL.h"
#include "../config.h"

void HAL_init()
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
    SPI_INIT();
}


