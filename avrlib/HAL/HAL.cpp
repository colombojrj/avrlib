#include "HAL.h"
#include "../config.h"

void initHAL()
{
    // Initialize the TIMERs
    if (timer0Config == timer0Config_t::off)
        TIMER0_OFF();
    else
        timer0Init(timer0Config, timer0Clock, timer0APolarity, timer0BPolarity);

    if (timer1Config == timer1Config_t::off)
        TIMER1_OFF();
    else
        timer1Init(timer1Config, timer1Clock, timer1OutputConfig, timer1TopValue);

    TIMER2_OFF();
    INIT_TIMER2A();
    INIT_TIMER2B();

    // Initialize the ADC


    // Initialize the SPI
    // TODO
    // Make the SPI INIT function adaptable (as in timers lib) to save memory
    spiInit();

    // Initialize the UART
    #if UART_MODE == UART_DOUBLE_SPEED
        uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(115200, F_CPU));
        sei();
    #elif UART_MODE == UART_NORMAL_SPEED
        uart_init(UART_BAUD_SELECT(115200, F_CPU));
        sei();
    #endif
}


