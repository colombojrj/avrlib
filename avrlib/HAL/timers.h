#ifndef TIMERS_H_
#define TIMERS_H_

#include <avr/io.h>
#include "gpio.h"
#include "defines.h"

/**
 * @defgroup hal_timer_group Timer
 *
 * @brief
 *
 * Timers. These very peripherals are very simple and versatile. For sure,
 * they are one of the most important peripherals in the microcontroller. If
 * well used, they can save lots of processing power, freeing the CPU to
 * work on other tasks.
 *
 * Despite their simplicity, it was true nightmare to add support to it.
 * The ATmega microcontrollers may have 8 bits and 16 bits timers. In
 * addition, some timers may work in input capture mode or even with
 * asynchronous events.
 *
 * Therefore, the following design definitions were made:
 * - A timer is a device that counts. Just it.
 * - The ATmega devices have an output compare unit (OC). This peripheral
 *   is attached to some timer and a gpio pin. This unit has the ability
 *   of controlling the gpio unit through the timer events
 * - BOTTOM: The counter reaches the BOTTOM when it becomes 0x00
 * - MAX: The counter reaches its MAXimum when it becomes 0xFF (decimal 255)
 * - TOP: The counter reaches the TOP when it becomes equal to the highest value
 *        in the count sequence. The TOP value can be assigned to be the fixed
 *        value 0xFF (MAX) or the value stored in the OCR0A Register. The
 *        assignment is dependent on the mode of operation.
 *
 */

/**@{*/

/// What is the output compare channel A of timer 0 configuration
extern uint8_t timer0WhatOutputAConfig;

/// What is the output compare channel B of timer 0 configuration
extern uint8_t timer0WhatOutputBConfig;

/// What is the output compare channel A of timer 1 configuration
extern uint8_t timer1WhatOutputAConfig;

/// What is the output compare channel B of timer 1 configuration
extern uint8_t timer1WhatOutputBConfig;

/// What is the output compare channel A of timer 2 configuration
extern uint8_t timer2WhatOutputAConfig;

/// What is the output compare channel B of timer 2 configuration
extern uint8_t timer2WhatOutputBConfig;

/// Timer 0 max count value
extern uint8_t timer0MaxCount;

/// Timer 1 max count value
extern uint16_t timer1MaxCount;

/// Timer 2 max count value
extern uint8_t timer2MaxCount;

/**
 * @brief This structure models the output compare unit of an 8 bits timer
 */
struct timer8bOC
{
    const uint8_t* ocAConfs;         //!< This is a pointer to a list of all available output compare channel A configurations.
    const uint8_t* ocBConfs;           //!< This is a pointer to a list of all available output compare channel B configurations.
    const gpio_t* pinA;                //!< The output compare unit channel A controls this gpio pin
    const gpio_t* pinB;                //!< The output compare unit channel B controls this gpio pin
    volatile uint8_t* compareValueA;   //!< The value stored in this register may trigger an interrupt or change the gpio pin state
    volatile uint8_t* compareValueB;   //!< The value stored in this register may trigger an interrupt or change the gpio pin state
    const uint8_t setStateA;           //!< Output compare channel A set state (for library use only)
    const uint8_t setStateB;           //!< Output compare channel B set state (for library use only)
};

/**
 * @brief This structure models the output compare unit of an 16 bits timer
 */
struct timer16bOC
{
    const uint8_t* ocAConfs;           //!< This is a pointer to a list of all available output compare channel A configurations.
    const uint8_t* ocBConfs;           //!< This is a pointer to a list of all available output compare channel B configurations.
    const gpio_t* pinA;                //!< The output compare unit channel A controls this gpio pin
    const gpio_t* pinB;                //!< The output compare unit channel B controls this gpio pin
    volatile uint16_t* compareValueA;  //!< The value stored in this register may trigger an interrupt or change the gpio pin state
    volatile uint16_t* compareValueB;  //!< The value stored in this register may trigger an interrupt or change the gpio pin state
    const uint8_t setStateA;           //!< Output compare channel A set state (for library use only)
    const uint8_t setStateB;           //!< Output compare channel B set state (for library use only)
};

/**
 * @brief A structure to hold the 8 bits timer registers
 */
struct timer8bRegs
{
    volatile uint16_t* control;        //!< Address of the timer control register. Because in AVR architecture the two control registers are aligned in memory map, these registers are treated as a single 16bit register
    volatile uint8_t* counter;         //!< Address of the timer counter register
    const timer8bOC* ocRegs;           //!< Output compare unit of the respective timer
    volatile uint8_t* interruptMask;   //!< Address of the interrupt mask register
    volatile uint8_t* interruptFlag;   //!< Address of the interrupt flag register
    volatile uint8_t* asyncStatus;     //!< Address of the interrupt asynchronous status register
    volatile uint8_t* asyncControl;    //!< Address of the interrupt asynchronous control register
    const uint8_t whatPRR;             //!< It inform what bit in PRR the timer has
};

/**
 * @brief Generic 16 bits timer registers
 */
struct timer16bRegs
{
    volatile uint16_t* control;        //!< Address of the timer control register. Because in AVR architecture the two control registers are aligned in memory map, these registers are treated as a single 16bit register
    volatile uint16_t* counter;        //!< Address of the timer counter register
    volatile uint16_t* outputCompareA; //!< Address of the output compare A register
    volatile uint16_t* outputCompareB; //!< Address of the output compare B register
    volatile uint16_t* inputCapture;   //!< Address of the input capture register
    volatile uint8_t* interruptMask;   //!< Address of the interrupt mask register
    volatile uint8_t* interruptFlag;   //!< Address of the interrupt flag register
    const gpio_t* outputPinA;          //!< Gpio pin controlled by output channel A of timer
    const gpio_t* outputPinB;          //!< Gpio pin controlled by output channel B of timer
    const uint8_t whatPRR;             //!< It inform what bit in PRR the timer has
};



/**
 * @brief An abstract 8 bits timer configuration structure
 */
struct timer8b
{
    const timer8bRegs* regs;    //!< Low level registers
    const uint8_t* ocAConfs;    //!< This is a pointer to a list of all available output compare channel A configurations.
    const uint8_t* ocBConfs;    //!< This is a pointer to a list of all available output compare channel B configurations.
    uint8_t* outputConfA;       //!< Address of variable containing the actual output channel A configuration
    uint8_t* outputConfB;       //!< Address of variable containing the actual output channel B configuration
    const uint8_t ocASetState;  //!< Output compare channel A set state (for library use only)
    const uint8_t ocBSetState;  //!< Output compare channel B set state (for library use only)
    uint8_t* maxCount;          //!< Max count (only if applicable)
};

/**
 * @brief An abstract 16 bits timer configuration structure
 */
struct timer16b
{
    const timer16bRegs* regs;   //!< Low level registers
    const uint8_t* ocAConfs;    //!< This is a pointer to a list of all available output compare channel A configurations.
    const uint8_t* ocBConfs;    //!< This is a pointer to a list of all available output compare channel B configurations.
    uint8_t* outputConfA;       //!< Address of variable containing the actual output channel A configuration
    uint8_t* outputConfB;       //!< Address of variable containing the actual output channel B configuration
    const uint8_t ocASetState;  //!< Output compare channel A set state (for library use only)
    const uint8_t ocBSetState;  //!< Output compare channel B set state (for library use only)
    uint16_t* maxCount;         //!< Max count (only if applicable)
};

/**
 * The ATmega timers share an output compare match unit. This enumerator defines
 * the available possible output configurations.
 */
enum class timerOutputCompareMode : uint8_t
{
    disconnected = 0, //!< Output compare unit does not control the gpio pin
    normal,           //!< Output compare unit controls the gpio pin in normal mode, i.e., the output signal is cleared after a compare match
    inverted,         //!< Output compare unit controls the gpio pin in inverted mode, i.e., the output signal is set after a compare match
    setState          //!< For library purposes
};

/**
 * Configures the clock preescaler of a 8 bits timer
 *
 * @param timer8b is a pointer to a 8 bits timer register descriptor
 *        @see timer8b
 * @param clockConf is the clock configuration defined with enum. Although
 *        this parameter type is uint16_t, the values that can be used depend on
 *        microcontroller and the employed timer. The enums have the following
 *        name: timerxClock.
 *        @see timer0Clock, for example
 *
 * Programming example on ATmega328P:
 *
 * @code
 *        timerSetClockPreescaler(Timer0, io16Conf(timer0Clock::divideBy64))
 * @endcode
 *
 * @see io16Conf
 */
#define timerSetClockPreescaler(timer, clockConf) { \
    *(*timer).regs->control |= clockConf;           \
}


#define timerSetMode(timer, modeConf) {             \
    *(*timer).regs->control |= modeConf;            \
}

#define timerSetOutputCompareUnitA(timer, outputConf) {                 \
    *(*timer).outputConfA = (*timer).ocAConfs[io8Conf(outputConf)];     \
    *(*timer).regs->control &= ~(*timer).ocASetState;                   \
    *(*timer).regs->control |= *(*timer).outputConfA;                   \
}

#define timerSetOutputCompareUnitB(timer, outputConf) {                 \
    *(*timer).outputConfB = (*timer).ocBConfs[io8Conf(outputConf)];     \
    *(*timer).regs->control &= ~(*timer).ocBSetState;                   \
    *(*timer).regs->control |= *(*timer).outputConfB;                   \
}

/**
 * Found solution: I am defining that 16 bits timer will always employ
 * ICR register as top register for PWMs and CTC modes
 */
#define timerSetTop(timer, top) {  \
    *(*timer).maxCount = top; \
}


void timerSetDutyA(const timer8b* timer, uint8_t duty);


void timerSetDutyB(const timer8b* timer, uint8_t duty);


void timerSetDutyA(const timer16b* timer, uint16_t duty);


void timerSetDutyB(const timer16b* timer, uint16_t duty);

/**
 * This function has the purpose of initialize an ATmega timer peripheral.
 * The user has to provide:
 * @param timer is the timer which will be configured
 * @param mode is the desired timer mode
 * @param clockConf is the desired clock source configuration
 * @param outputAConf is the configuration of the output compare unit A
 * @param outputAConf is the configuration of the output compare unit B
 * @param interruptConf is the interrupt configuration
 *
 * Here is an example of how to configure the Timer0 of the ATmega328P
 * to run in CTC mode generating an interrupt on a compare match:
 *
 * \code
 *  timerInit(Timer0,
 *            io16Conf(timer0Mode::ctc),
 *            io16Conf(timer0Clock::divideBy64),
 *            io8Conf(timerOutputConfig::off),
 *            io8Conf(timerOutputConfig::off),
 *            io8Conf(timer0Interrupt::onCompareMatchA));
 *  timer
 * \endcode
 *
 * @see timer8b
 *
 */
void timerInit(const timer8b* timer,
               uint16_t mode,
               uint16_t clockConf,
               uint8_t outputAConf,
               uint8_t outputBConf,
               uint8_t interruptConf);


void timerInit(const timer16b* timer,
               uint16_t mode,
               uint16_t clockConf,
               uint8_t outputAConf,
               uint8_t outputBConf,
               uint8_t interruptConf);






#if defined(SUPPORT_TO_TIMER0)
/**
 * Functions to handle the TIMER0 module
 */
extern void timer0Init(timer0Config_t config,
                       timer0Clock_t clock,
                       timer0OutputConfig_t outputConfig = timer0OutputConfig_t::off);
extern void timer0ASetDuty(uint8_t OCR, timer0OutputConfig_t outputConfig);
extern void timer0BSetDuty(uint8_t OCR, timer0OutputConfig_t outputConfig);
extern void TIMER0_OFF();

#endif

#if defined(SUPPORT_TO_TIMER1)

/**
 * Functions to handle the TIMER1 module
 */
extern void timer1Init(timer1Config_t config,
                       timer1Clock_t clock,
                       timer1OutputConfig_t outputConfig,
                       uint16_t topValue = 255);
extern void timer1ASetDuty(uint16_t duty, timer1OutputConfig_t output, uint16_t top = 255);
extern void timer1BSetDuty(uint16_t duty, timer1OutputConfig_t output, uint16_t top = 255);
extern void enableInputCapture(timer1InputCaptureEdge_t config);
extern void TIMER1_OFF();

#endif

#if defined(SUPPORT_TO_TIMER2)

/**
 * Functions to handle the TIMER2 module
 */
void timer2Init(timer2Config_t config,
                timer2Clock_t clock,
                timer2OutputConfig_t outputConfig);
extern void timer2ASetDuty(uint8_t OCR, timer2OutputConfig_t outputConfig);
extern void timer2BSetDuty(uint8_t duty, timer2OutputConfig_t outputConfig);
extern void TIMER2_OFF();

#endif /* defined(SUPPORT_TO_TIMER2) */

/**@}*/

#endif /* TIMER_H_ */
