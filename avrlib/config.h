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
#include "HAL/devices/device.h"


constexpr timer0Config_t        timer0Config        = timer0Config_t::off;
constexpr timer0Clock_t         timer0Clock         = timer0Clock_t::noPreescale;
constexpr timer0OutputConfig_t  timer0OutputConfig  = timer0OutputConfig_t::channelAnormal;
constexpr uint8_t               timer0AInitialOcr   = 0;
constexpr uint8_t               timer0BInitialOcr   = 0;

constexpr timer1Config_t        timer1Config        = timer1Config_t::off;
constexpr timer1Clock_t         timer1Clock         = timer1Clock_t::divideBy8;
constexpr timer1OutputConfig_t  timer1OutputConfig  = timer1OutputConfig_t::channelAnormal;
constexpr uint16_t              timer1TopValue      = 255; /// If 8 bits, it is 255, if 9 bits it is 511, if 10 bits it is 1023, if DefinedTop users may choose maximum value
constexpr uint16_t              timer1AInitialOcr   = 0;
constexpr uint16_t              timer1BInitialOcr   = 0;

constexpr timer2Config_t        timer2Config        = timer2Config_t::off;
constexpr timer2Clock_t         timer2Clock         = timer2Clock_t::divideBy8;
constexpr timer2OutputConfig_t  timer2OutputConfig  = timer2OutputConfig_t::channelAnormal;
constexpr uint8_t               timer2AInitialOcr   = 0;
constexpr uint8_t               timer2BInitialOcr   = 0;



/////////
// SPI //
/////////
constexpr spiConfig_t           spiConfig           = spiConfig_t::off;
constexpr spiMode_t             spiMode             = spiMode_t::mode0;
constexpr spiClock_t            spiClock            = spiClock_t::divideBy16;
constexpr spiUseInterrupt_t     spiUseInterrupt     = spiUseInterrupt_t::no;
constexpr spiDataOrder_t        spiDataOrder        = spiDataOrder_t::msbFisrt;

//////////
// UART //
//////////
#define UART_MODE       UART_NORMAL_SPEED
#define UART_BAUD_RATE  115200


/////////////////////////////////
// ANALOG TO DIGITAL CONVERTER //
/////////////////////////////////
constexpr adcConfig_t           adcConfig           = adcConfig_t::off;
constexpr adcRefVoltage_t       adcReferenceConfig  = adcRefVoltage_t::avcc;
constexpr float                 adcSupplyVoltage    = 5.0;
constexpr adcClock_t            adcClock            = adcClock_t::divideBy32;
constexpr adcDataAlign_t        adcDataAlign        = adcDataAlign_t::right;

#endif /* AVRLIB_CONFIG_H_ */
