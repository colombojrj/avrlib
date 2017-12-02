#ifndef AVRLIB_HAL_DEFINES_H_
#define AVRLIB_HAL_DEFINES_H_

/////////////////////////
// GENERAL DEFINITIONS //
/////////////////////////
#define LOW                     0
#define HIGH                    1
#define HIGHZ                   2
#define RISING_EDGE             1
#define FALLING_EDGE            0
#define TRUE                    1
#define FALSE                   0

////////////
// TIMERS //
////////////
#define OFF                     0 // TIMER turned off
#define NORMAL                  1
#define CTC                     2
#define PWM_A                   3 // only channel A (example OCR0A, OCR1A, ...) works as fast-PWM
#define PWM_B                   4 // only channel B (example OCR0B, OCR1B, ...) works as fast-PWM
#define PWM_AB                  5 // both channels work as fast-PWM
#define PHASE_CORRECT           6
#define NORMAL_WITH_IN_CAP      7 // only for TIMER1
#define INVERTED                7
#define TIMER0A                 0
#define TIMER0B                 1
#define TIMER1A                 2
#define TIMER1B                 3
#define TIMER2A                 4
#define TIMER2B                 5

// Microcontroller specifics
#if defined (__AVR_ATmega328P__)
    // Timer 0
    #define OC0A_PIN            PD6
    #define OC0A_PORT           PORTD
    #define OC0B_PIN            PD5
    #define OC0B_PORT           PORTB

    // Timer 1
    #define OC1A_PIN            PB1
    #define OC1A_PORT           PORTB
    #define OC1B_PIN            PB2
    #define OC1B_PORT           PORTB

    // Timer 2
    #define OC2A_PIN            PB3
    #define OC2A_PORT           PORTB
    #define OC2B_PIN            PD3
    #define OC2B_PORT           PORTD
#endif

///////////
// CLOCK //
///////////
#define NO_CLOCK                0
#define NO_PREESCALE            1
#define CLK_2                   12
#define CLK_4                   10
#define CLK_8                   2
#define CLK_16                  11
#define CLK_32                  8 // only for TIMER2
#define CLK_64                  3
#define CLK_128                 9 // only for TIMER2
#define CLK_256                 4
#define CLK_1024                5
#define T0_FALLING              6
#define T0_RISING               7


//////////
// GPIO //
//////////
#define DDR(x)                  (*(&x - 1)) // This does not work for ATmega64 or higher
#define PIN(x)                  (*(&x - 2)) // This does not work for ATmega64 or higher

#define INPUT                   0
#define INPUT_PULLUP            1
#define OUTPUT                  2
#define ANALOG_INPUT            3
#define PWM                     4
#define PWM_8_BITS              PWM //  8 bit PWM
#define PWM_9_BITS              5   //  9 bit PWM
#define PWM_10_BITS             6   // 10 bit PWM
#define PWM_11_BITS             7   // 11 bit PWM
#define PWM_12_BITS             8   // 12 bit PWM
#define PWM_13_BITS             9   // 13 bit PWM
#define PWM_14_BITS             10  // 14 bit PWM
#define PWM_15_BITS             11  // 15 bit PWM
#define PWM_16_BITS             12  // 16 bit PWM
#define LOW_LEVEL               13  // low level triggers external interrupt
#define ANY_CHANGE              14  // any logical change trigger external interrupt

/////////////////////////////////
// ANALOG TO DIGITAL CONVERTER //
/////////////////////////////////
#define SINGLE_CONVERSION   1
#define NOISE_REDUCTION     2
#define FREE_RUNNING        3
#define EXTERN              0
#define INTERN              1
#define REF_VCC             2
#define LEFT                0
#define RIGHT               1

///* Configure the microcontroller reference voltage
#if defined (__AVR_ATmega328__)  || (__AVR_ATmega328P__) || \
            (__AVR_ATtiny13__)   || (__AVR_ATtiny13A__)  || \
            (__AVR_ATtiny24__)   || (__AVR_ATtiny24A__)  || \
            (__AVR_ATtiny44__)   || (__AVR_ATtiny44A__)  || \
            (__AVR_ATtiny84__)   || (__AVR_ATtiny84A__)
    #define ADC_INTERNT_REF_VOLTAGE   1.1
#elif defined (__AVR_ATmega8__) || (__AVR_ATmega8A__)

	#define ADC_INTERNT_REF_VOLTAGE   2.56
#endif

// Automatic reference voltage selection
#if ADC_REFERENCE == INTERN
	#define ADC_REFERENCE_VOLTAGE ADC_INTERNT_REF_VOLTAGE
#else
	#define ADC_REFERENCE_VOLTAGE ADC_SUPPLY_VOLTAGE
#endif

/////////
// SPI //
/////////
#define SLAVE                   2
#define MASTER                  1
#define MSB_FIRST               0
#define LSB_FIRST               1

//////////
// UART //
//////////
#define UART_NORMAL_SPEED       1
#define UART_DOUBLE_SPEED       2

#endif /* AVRLIB_HAL_DEFINES_H_ */
