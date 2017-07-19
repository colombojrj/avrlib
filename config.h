/*
 * general_config.h
 *
 *  Created on: 16/07/2017
 *      Author: junior
 */

/**
 * \file config.h
 * Configure the peripherals of the micro-controller. All the possible
 * configuration defines are available in \ref defines.h
 */

/*!
 *
 * \mainpage avrlib
 *
 * @section welcome AVRLIB a high level library for AVR micro-controllers
 *
 * @subsection supported_hardware Supported micro-controllers
 *
 * Actually the following microcontrollers are supported:
 * * ATmega328P (recall that I develop this library for free)
 *
 * @subsection recommendations General recomendations
 *
 * It is worth noting that this documentation has the purpose of guiding
 * the developer only. It doesn't replace the original AVR micro-controller
 * datasheet.
 *
 * Before start using avrlib, it is necessary to configure
 * the peripherals of the current micro-controller in \ref config.h. When
 * configuring the peripherals, make sure to insert a valid configuration.
 * For example, if the Timerx preescale support on 1x, 8x and 32x, setting 4x
 * will not work and may generate unexpected results.
 *
 * @subsection conf_timers Timers
 *
 * Configure here all the timers available in the micro-controller.
 *
 * @param TIMERx_CONFIG can be configured with the following options:
 * * OFF: current timer is turned OFF (power saings will be enabled)
 * * NORMAL: current timer is configured as NORMAL operation
 *           (only counts until overflow and then generate an interrupt)
 * * CTC: current timer is configured as CTC (generate interrupt when count
 *        is greater than value setted in OCR (see @ref TIMER0A_INITIAL_OCR, for example))
 * * PWM_A: only channel A of the current timer works as fast PWM generator
 * * PWM_B: only channel B of the current timer works as fast PWM generator
 * * PWM_AB: both channels (A and B) of the current timer work as fast PWM generator
 * * PHASE_CORRECT: (to be done!)
 * * NORMAL_WITH_IN_CAP: (only for Timer 1) the input capture unit is enabled
 *
 * @param TIMERx_CLOCK can be configured with the following options <b>(make sure to verify
 *        if the current timer supports the desired preescaler factor!)</b>:
 * * NO_CLOCK: the current timer doens't receive any clock signal
 * * NO_PREESCALE: the current timer receives the CPU clock frequency
 * * CLK_2: the current timer receives the CPU clock frequency divided by a factor of 2x
 * * CLK_4: the current timer receives the CPU clock frequency divided by a factor of 4x
 * * CLK_8: the current timer receives the CPU clock frequency divided by a factor of 8x
 * * CLK_16: the current timer receives the CPU clock frequency divided by a factor of 16x
 * * CLK_32: the current timer receives the CPU clock frequency divided by a factor of 32x
 *           (supported only for TIMER2)
 * * CLK_64: the current timer receives the CPU clock frequency divided by a factor of 64x
 * * CLK_128: the current timer receives the CPU clock frequency divided by a factor of 128x
 *            (supported only for TIMER2)
 * * CLK_256: the current timer receives the CPU clock frequency divided by a factor of 256x
 * * CLK_1024: the current timer receives the CPU clock frequency divided by a factor of 1024x
 * * T0_FALLING: the current timer is sensible to the falling edge of the clock signal
 *               provided through the T0 pin
 * * T0_RISING: the current timer is sensible to the rising edge of the clock signal
 *               provided through the T0 pin
 *
 * @param TIMERxA_POLARITY can be configured with the following options
 * * NORMAL: in PWM mode, normal goes as: the grater the OCR value, the greater the duty cycle
 * * INVERTED: reverse of NORMAL mode
 *
 * @param TIMERxA_INITIAL_OCR is configured with the initial value of the OCR for channel A of
 *                            the current timer. Make sure to provide a uint8_t value for 8 bit
 *                            timers and a uint16_t value for 16 bit timers. A great tool for
 *                            calculating OCR values is available
 *                            <a href="http://kevin.rosenberg.net/">here</a>
 *
 * @param TIMERxB_POLATIRY can be configured with the following options
 * * NORMAL: in PWM mode, normal goes as: the grater the OCR value, the greater the duty cycle
 * * INVERTED: reverse of NORMAL mode
 *
 * @param TIMERxB_INITIAL_OCR is configured with the initial value of the OCR for channel A of
 *                            the current timer. Make sure to provide a uint8_t value for 8 bit
 *                            timers and a uint16_t value for 16 bit timers. A great tool for
 *                            calculating OCR values is available
 *                            <a href="http://kevin.rosenberg.net/">here</a>
 *
 * @param TIMER1_RESOLUTION Timer1 is a 16 bit timer. This configuration is required for some
 *                          operation modes (16 bit PWM, for example) <b>(however there isn't support
 *                          for it right now)
 *
 * @param TIMER1_PWM_TOP Timer1 top value (it may be required for some operation modes)
 *
 * \todo
 * * Add subsections of SPI, UART and ADC
 *
 * @subsection conf_spi SPI
 *
 *
 * @subsection conf_uart UART
 *
 *
 * @subsection conf_adc Analog to Digital Converter (ADC)
 * \todo
 * * renomear as definições do ADC
 *
 *
 *
 *
 */

#ifndef AVRLIB_CONFIG_H_
#define AVRLIB_CONFIG_H_

// Use this file to configure the micro-controller peripherals

#define TIMER0_CONFIG           PWM_B        ///< TIMER0 operation mode. For more information see \ref conf_timers
#define TIMER0_CLOCK            NO_PREESCALE ///< configure TIMER0 clock source. For more information see \ref conf_timers
#define TIMER0A_POLATIRY        NORMAL       ///< can be NORMAL or INVERTED. For more information see \ref conf_timers
#define TIMER0A_INITIAL_OCR     0            ///< Initial duty cycle. For more information see \ref conf_timers
#define TIMER0B_POLATIRY        NORMAL       ///< can be NORMAL or INVERTED. For more information see \ref conf_timers
#define TIMER0B_INITIAL_OCR     0            ///< Initial duty cycle. For more information see \ref conf_timers

#define TIMER1_CONFIG           NORMAL       ///< TIMER1 operation mode. For more information see \ref conf_timers
#define TIMER1_CLOCK            CLK_64       ///< configure TIMER1 clock source. For more information see \ref conf_timers
#define TIMER1_RESOLUTION       16           ///< in bits. For more information see \ref conf_timers
#define TIMER1_PWM_TOP          0xFFFF       ///< if using up to 16 bit resolution. For more information see \ref conf_timers
#define TIMER1A_POLATIRY        NORMAL       ///< can be NORMAL or INVERTED. For more information see \ref conf_timers
#define TIMER1A_INITIAL_OCR     0x0000       ///< Initial duty cycle. For more information see \ref conf_timers
#define TIMER1B_POLATIRY        NORMAL       ///< can be NORMAL or INVERTED. For more information see \ref conf_timers
#define TIMER1B_INITIAL_OCR     0x0000       ///< Initial duty cycle. For more information see \ref conf_timers

#define TIMER2_CONFIG           CTC          ///< TIMER2 operation mode. For more information see \ref conf_timers
#define TIMER2_CLOCK            CLK_128      ///< configure TIMER2 clock source. For more information see \ref conf_timers
#define TIMER2A_POLATIRY        NORMAL       ///< can be NORMAL or INVERTED. For more information see \ref conf_timers
#define TIMER2A_INITIAL_OCR     0x7C         ///< Initial duty cycle. For more information see \ref conf_timers
#define TIMER2B_POLATIRY        NORMAL       ///< can be NORMAL or INVERTED. For more information see \ref conf_timers
#define TIMER2B_INITIAL_OCR     0            ///< Initial duty cycle. For more information see \ref conf_timers



/////////
// SPI //
/////////
#define SPI_MODE                MASTER
#define SPI_CLK                 CLK_2
#define SPI_USE_INTERRUPT       FALSE
#define SPI_DATA_ORDER          MSB_FIRST


//////////
// UART //
//////////
#define UART_BAUD_RATE 115200


/////////////////////////////////
// ANALOG TO DIGITAL CONVERTER //
/////////////////////////////////

/**
 * Choose the A/D operation mode listed from below:
 *
 * OFF
 * SINGLE_CONVERSION
 * NOISE_REDUCTION
 * FREE_RUNNING
 */
#define ADC_OPERATION_MODE OFF


/**
 * Defina qual como será gerada a tensão de
 * referência do circuito. Os possíveis valores são:
 *
 * ADC_EXT
 * ADC_INT
 * ADC_VCC
 *
 */
#define ADC_REFERENCE INTERN

/**
 * Define the reference voltage and the analog circuit vcc supply voltage
 */
#define ADC_SUPPLY_VOLTAGE      5.0


/* É também preciso informar qual o fator de
 * divisão da frequência da CPU. Os possíveis
 * valores são:
 *
 * CLK_2
 * CLK_4
 * CLK_8
 * CLK_16
 * CLK_32
 * CLK_64
 * CLK_128
 */
#define PREESCALE CLK_32

/* Data alignment:
 *
 * ADC_LEFT
 * ADC_RIGHT
 */
#define DATA_ALIGN RIGHT

#endif /* AVRLIB_CONFIG_H_ */
