#ifndef TIMERS_H_
#define TIMERS_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "gpio.h"
#include "defines.h"

/**
 * @defgroup hal_timer_group Timer
 *
 * Definitions:
 * - BOTTOM: The counter reaches the BOTTOM when it becomes 0x00
 * - MAX: The counter reaches its MAXimum when it becomes 0xFF (decimal 255)
 * - TOP: The counter reaches the TOP when it becomes equal to the highest value in the
 *        count sequence. The TOP value can be assigned to be the fixed value 0xFF
 *        (MAX) or the value stored in the OCR0A Register. The assignment is depen-
 *        dent on the mode of operation.
 *
 *
 * Organizational idea of this library:
 *
 * - The file defines.h keeps the definitions of the actual
 *   microcontroller timers (such as what pin and port the
 *   PWM is connected to)
 *
 * - This library uses those definitions to map the PWM
 *   pins. If a new microcontroller needs to be added, then
 *   it remains to add the port definition on defines.h
 *
 * - It is assumed that a timer has a single configuration
 *   during production. With this, defines were inserted
 *   to allow the compiler to select what configuration is
 *   needed and then, generate code only for that
 *   application. The main concern here is to keep the code
 *   size smallest as possible. If "dynamic" functions were
 *   used the compiler would have to build all unused code,
 *   occupying memory of the microcontroller
 */

/**@{*/

/**
 * @brief Generic 8 bits timer
 */
struct timer8bRegs
{
    volatile uint16_t* control;        //!< Address of the timer control register. Because in AVR architecture the two control registers are aligned in memory map, these registers are treated as a single 16bit register
    volatile uint8_t* counter;         //!< Address of the timer counter register
    volatile uint8_t* outputCompareA;  //!< Address of the output compare A register
    volatile uint8_t* outputCompareB;  //!< Address of the output compare B register
    volatile uint8_t* interruptMask;   //!< Address of the interrupt mask register
    volatile uint8_t* interruptFlag;   //!< Address of the interrupt flag register
    volatile uint8_t* asyncStatus;     //!< Address of the interrupt asynchronous status register
    volatile uint8_t* asyncControl;    //!< Address of the interrupt asynchronous control register
    const gpio_t* outputPinA;          //!< Gpio pin controlled by output channel A of timer
    const gpio_t* outputPinB;          //!< Gpio pin controlled by output channel B of timer
    const uint8_t hasPRR;              //!< It informs if the timer has a power enable on PRR
    const uint8_t whatPRR;             //!< It inform what bit in PRR the timer has
};

/**
 * @brief Generic 16 bits timer
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
    const uint8_t hasPRR;              //!< It informs if the timer has a power enable on PRR
    const uint8_t whatPRR;             //!< It inform what bit in PRR the timer has
};

/**
 * An abstract 8 bits timer configuration struct
 */
struct timer8b
{
    const timer8bRegs* regs;   // Low level registers
    uint16_t timerConf;        // Timer configuration
    uint8_t outputConfA;       // Gpio controlled by timer channel A configuration
    uint8_t outputConfB;       // Gpio controlled by timer channel B configuration
    uint8_t interruptConf;     // Interrupt configuration
    uint8_t maxCount;          // Max count (only if applicable)
};

/**
 * An abstract 16 bits timer configuration struct
 */
struct timer16b
{
    const timer16bRegs* regs;  // Low level registers
    uint16_t timerConf;        // Clock configuration
    uint8_t outputConfA;       // Gpio controlled by timer channel A configuration
    uint8_t outputConfB;       // Gpio controlled by timer channel B configuration
    uint8_t interruptConf;     // Interrupt configuration
    uint16_t maxCount;         // Max count (only if applicable)
};

/**
 * Configures the clock preescaler of a 8 bits timer
 *
 * @param timer8b is a pointer to a 8 bits timer register descriptor
 *        @see timer8b
 * @param clockConf is the clock configuration defined with enum. Although
 *        this parameter type is uint8_t, the values that can be used depend on
 *        microcontroller and the employed timer. The enums have the following
 *        name: timerxClock.
 *        @see timer0Clock, for example
 *
 * Programming example on ATmega328P:
 *
 * @code
 *        timerSetClockPreescaler(&Timer0, io8Conf(timer0Clock::divideBy64))
 * @endcode
 *
 * @see io8Conf
 *
 * @todo add support to select frequency automatically (ctc mode only)
 */
void timerSetClockPreescaler(const timer8b* timer, uint16_t clockConf);

/**
 * Configures the clock preescaler of a 16 bits timer
 *
 * @param timer16b is a pointer to a 16 bits timer register descriptor
 *        @see timer16b
 * @param clockConf is the clock configuration defined with enum. Although
 *        this parameter type is uint8_t, the values that can be used depend on
 *        microcontroller and the employed timer. The enums have the following
 *        name: timerxClock.
 *        @see timer1Clock, for example
 *
 * Programming example on ATmega328P:
 *
 * @code
 *        timerSetClockPreescaler(&Timer1, io16Conf(timer0Clock::divideBy64))
 * @endcode
 *
 * @see io16Conf
 *
 * @todo add support to select frequency automatically (ctc mode only)
 */
void timerSetClockPreescaler(const timer16b* timer, uint16_t clockConf);


void timerSetMode(const timer8b* timer, uint16_t mode);


void timerSetMode(const timer16b* timer, uint16_t mode);


void timerSetOutputA(const timer8b* timer, uint8_t outputConfig);


void timerSetOutputA(const timer16b* timer, uint8_t outputConfig);


void timerSetOutputB(const timer8b* timer, uint8_t outputConfig);


void timerSetOutputB(const timer16b* timer, uint8_t outputConfig);


void timerSetTop(const timer8b* timer, uint8_t top);


void timerSetTop(const timer16b* timer, uint16_t top);


void timerSetDuty(const timer8b* timer, uint8_t duty);


void timerSetDuty(const timer16b* timer, uint16_t duty);


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
