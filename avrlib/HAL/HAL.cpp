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

    if (timer2Config == timer2Config_t::off)
        TIMER2_OFF();
    else
        timer2Init(timer2Config, timer2Clock, timer2OutputConfig);

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


