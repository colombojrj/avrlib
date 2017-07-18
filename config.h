/*
 * general_config.h
 *
 *  Created on: 16/07/2017
 *      Author: junior
 */

#ifndef AVRLIB_CONFIG_H_
#define AVRLIB_CONFIG_H_

// Use this file to configure the micro-controller peripherals

////////////
// TIMERS //
////////////
#define TIMER0_CONFIG           PWM_B        // TIMER0 operation mode
#define TIMER0_CLOCK            NO_PREESCALE // configure TIMER0 clock source
#define TIMER0A_POLATIRY        NORMAL       // can be NORMAL or INVERTED
#define TIMER0A_INITIAL_OCR     0            // Initial duty cycle
#define TIMER0B_POLATIRY        NORMAL       // can be NORMAL or INVERTED
#define TIMER0B_INITIAL_OCR     0            // Initial duty cycle

#define TIMER1_CONFIG           NORMAL       // TIMER1 operation mode
#define TIMER1_CLOCK            CLK_64       // configure TIMER1 clock source
#define TIMER1_RESOLUTION       16           // in bits
#define TIMER1_PWM_TOP          0xFFFF       // if using up to 16 bit resolution
#define TIMER1A_POLATIRY        NORMAL       // can be NORMAL or INVERTED
#define TIMER1A_INITIAL_OCR     0x0000       // Initial duty cycle
#define TIMER1B_POLATIRY        NORMAL       // can be NORMAL or INVERTED
#define TIMER1B_INITIAL_OCR     0x0000       // Initial duty cycle

#define TIMER2_CONFIG           CTC          // TIMER2 operation mode
#define TIMER2_CLOCK            CLK_128      // configure TIMER2 clock source
#define TIMER2A_POLATIRY        NORMAL       // can be NORMAL or INVERTED
#define TIMER2A_INITIAL_OCR     0x7C         // Initial duty cycle
#define TIMER2B_POLATIRY        NORMAL       // can be NORMAL or INVERTED
#define TIMER2B_INITIAL_OCR     0            // Initial duty cycle


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

/*
 * Choose the A/D operation mode listed from below:
 *
 * ADC_OFF
 * ADC_SINGLE_CONVERSION
 * ADC_NOISE_REDUCTION
 * ADC_FREE_RUNNING
 */
#define OPERATION_MODE ADC_OFF


/*
 * Defina qual como será gerada a tensão de
 * referência do circuito. Os possíveis valores são:
 *
 * ADC_EXT
 * ADC_INT
 * ADC_VCC
 */
#define ADC_REFERENCE ADC_REF_INT

/*
 * Define the reference voltage and the analog circuit vcc supply voltage
 */
#define ADC_REFERENCE_VOLTAGE   1.1
#define ADC_SUPPLY_VOLTAGE      5.0


/* É também preciso informar qual o fator de
 * divisão da frequência da CPU. Os possíveis
 * valores são:
 *
 * ADC_DIV_BY_2
 * ADC_DIV_BY_4
 * ADC_DIV_BY_8
 * ADC_DIV_BY_16
 * ADC_DIV_BY_32
 * ADC_DIV_BY_64
 * ADC_DIV_BY_128
 */
#define PREESCALE ADC_DIV_BY_32

/* Data alignment:
 *
 * ADC_LEFT
 * ADC_RIGHT
 */
#define DATA_ALIGN ADC_RIGHT

#endif /* AVRLIB_CONFIG_H_ */
