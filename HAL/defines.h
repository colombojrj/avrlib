/*
 * defines.h
 *
 *  Created on: 16/07/2017
 *      Author: junior
 */

#ifndef AVRLIB_HAL_DEFINES_H_
#define AVRLIB_HAL_DEFINES_H_

// Useful defines
#define OFF                 0 // TIMER turned off
#define NORMAL              1
#define CTC                 2
#define PWM_A               3 // only channel A (example OCR0A, OCR1A, ...) works as fast-PWM
#define PWM_B               4 // only channel B (example OCR0B, OCR1B, ...) works as fast-PWM
#define PWM_AB              5 // both channels work as fast-PWM
#define PHASE_CORRECT       6
#define NORMAL_WITH_IN_CAP  7 // only for TIMER1
#define INVERTED            7
#define TIMER0A             0
#define TIMER0B             1
#define TIMER1A             2
#define TIMER1B             3
#define TIMER2A             4
#define TIMER2B             5

// Clock division
#define NO_CLOCK            0
#define NO_PREESCALE        1
#define CLK_8               2
#define CLK_32              8 // only for TIMER2
#define CLK_64              3
#define CLK_128             9 // only for TIMER2
#define CLK_256             4
#define CLK_1024            5
#define T0_FALLING          6
#define T0_RISING           7

// Input capture
#define RISING_EDGE         1
#define FALLING_EDGE        0




//////////
// GPIO //
//////////

// This does not work for ATmega64 or higher
#define DDR(x) (*(&x - 1))
#define PIN(x) (*(&x - 2))

#define INPUT           0
#define INPUT_PULLUP    1
#define OUTPUT          2
#define ANALOG_INPUT    3   // TODO: add support
#define PWM             4   // adding support
#define PWM_8           PWM //  8 bit PWM
#define PWM_9           5   //  9 bit PWM
#define PWM_10          6   // 10 bit PWM
#define PWM_11          7   // 11 bit PWM
#define PWM_12          8   // 12 bit PWM
#define PWM_13          9   // 13 bit PWM
#define PWM_14          10  // 14 bit PWM
#define PWM_15          11  // 15 bit PWM
#define PWM_16          12  // 16 bit PWM
#define LOW_LEVEL       13  // low level triggers external interrupt
#define ANY_CHANGE      14  // any logical change trigger external interrupt
//#define RISING_EDGE     15  // rising edge triggers external interrupt
//#define FALLING_EDGE    16  // falling edge triggers external interrupt

#define LOW             0
#define HIGH            1




#endif /* AVRLIB_HAL_DEFINES_H_ */
