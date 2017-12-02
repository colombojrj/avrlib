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


