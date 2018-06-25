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
 */

#ifndef AVRLIB_CONFIG_H_
#define AVRLIB_CONFIG_H_

// Include microcontroller peripheral definitions
//#include "HAL/devices/device.h"

/////////
// SPI //
/////////
/*
constexpr spiConfig_t           spiConfig           = spiConfig_t::off;
constexpr spiMode_t             spiMode             = spiMode_t::mode0;
constexpr spiClock_t            spiClock            = spiClock_t::divideBy16;
constexpr spiUseInterrupt_t     spiUseInterrupt     = spiUseInterrupt_t::no;
constexpr spiDataOrder_t        spiDataOrder        = spiDataOrder_t::msbFisrt;
*/
//////////
// UART //
//////////
#define UART_MODE       UART_NORMAL_SPEED
#define UART_BAUD_RATE  115200


/////////////////////////////////
// ANALOG TO DIGITAL CONVERTER //
/////////////////////////////////
/*
constexpr adcConfig_t           adcConfig           = adcConfig_t::off;
constexpr adcRefVoltage_t       adcReferenceConfig  = adcRefVoltage_t::avcc;
constexpr float                 adcSupplyVoltage    = 5.0;
constexpr adcClock_t            adcClock            = adcClock_t::divideBy32;
constexpr adcDataAlign_t        adcDataAlign        = adcDataAlign_t::right;
*/

#endif /* AVRLIB_CONFIG_H_ */
