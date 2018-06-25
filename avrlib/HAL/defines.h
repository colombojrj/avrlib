#ifndef AVRLIB_HAL_DEFINES_H_
#define AVRLIB_HAL_DEFINES_H_

/**
 * @defgroup hal_defines_group Defines
 *
 * HAL defines.
 */

/**@{*/

/// Decodes from 8 bits enum hardware configuration
#define io8Conf(x)              static_cast<uint8_t>(x)

/// Decodes from 16 bits enum hardware configuration
#define io16Conf(x)             static_cast<uint16_t>(x)




/////////////////////////////////
// ANALOG TO DIGITAL CONVERTER //
/////////////////////////////////

///* Configure the microcontroller reference voltage
#if defined (__AVR_ATmega88__)  || (__AVR_ATmega88P__)   || \
            (__AVR_ATmega168__)  || (__AVR_ATmega168P__) || \
            (__AVR_ATmega328__)  || (__AVR_ATmega328P__) || \
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
	#define ADC_REFERENCE_VOLTAGE adcSupplyVoltage
#endif

//////////
// UART //
//////////
#define UART_NORMAL_SPEED       1
#define UART_DOUBLE_SPEED       2

/**@}*/

#endif /* AVRLIB_HAL_DEFINES_H_ */
