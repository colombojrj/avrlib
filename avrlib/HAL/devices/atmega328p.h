#ifndef AVRLIB_AVRLIB_HAL_DEVICES_ATMEGA328P_H_
#define AVRLIB_AVRLIB_HAL_DEVICES_ATMEGA328P_H_

/**@{*/

#include <stdlib.h>
#include <avr/io.h>

// TODO documentation of this file

/**
 * adcConfig_t
 */
enum class adcConfig_t : uint8_t
{
    off,              //!< off
    singleConversion, //!< singleConversion
    noiseReduction,   //!< noiseReduction
    freeRunning       //!< freeRunning
};

/**
 * @brief reference voltage enum
 *
 * It is defined:
 * * aref is when extern voltage is connected to aref pin
 * * external is an alias to aref
 * * avcc is when adc vdd voltage is used
 * * vcc is an alias to avcc
 * * internal is when internal voltage generator is used
 */
enum class adcRefVoltage_t : uint8_t
{
    aref     = 0,                           //!< aref
    external = aref,                        //!< external (alias to aref)
    avcc     = (1 << REFS0),                //!< avcc
    vcc      = avcc,                        //!< vcc (alias to avcc)
    internal = (1 << REFS1) | (1 << REFS0), //!< internal
    setState = (1 << REFS1) | (1 << REFS0)  //!< reserved (to be used by the API)
};

/**
 * adcClock_t
 */
enum class adcClock_t : uint8_t
{
    divideBy2   = (1 << ADPS0),                               //!< divideBy2
    divideBy4   = (1 << ADPS1),                               //!< divideBy4
    divideBy8   = (1 << ADPS1) | (1 << ADPS0),                //!< divideBy8
    divideBy16  = (1 << ADPS2),                               //!< divideBy16
    divideBy32  = (1 << ADPS2) | (1 << ADPS0),                //!< divideBy32
    divideBy64  = (1 << ADPS2) | (1 << ADPS1),                //!< divideBy64
    divideBy128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0), //!< divideBy128
    setState    = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)  //!< reserved (to be used by the API)
};

/**
 *
 */
enum class adcDataAlign_t : uint8_t
{
    right = 0,              //!< right
    left = (1 << ADLAR),    //!< left
    setState = (1 << ADLAR) //!< reserved (to be used by the API)
};

/**
 *
 */
enum class adcAdmux_t : uint8_t
{
    adc0 = 0b0000,              //!< adc0
    adc1 = 0b0001,              //!< adc1
    adc2 = 0b0010,              //!< adc2
    adc3 = 0b0011,              //!< adc3
    adc4 = 0b0100,              //!< adc4
    adc5 = 0b0101,              //!< adc5
    adc6 = 0b0110,              //!< adc6
    adc7 = 0b0111,              //!< adc7
    temperatureSensor = 0b1000, //!< temperatureSensor
    internalReference = 0b1110, //!< internalReference
    gnd = 0b1111,               //!< gnd
    setState = 0b1111           //!< setState
};

///////////////////
/// GPIO module ///
///////////////////

enum class gpioIntPin_t : uint8_t
{
    int0 = PD2,
    int1 = PD3
};

enum class gpioInt_t : uint8_t
{
    lowLevel    = 0b00,
    anyChange   = 0b01,
    fallingEdge = 0b10,
    risingEdge  = 0b11,
    setState    = 0b11
};

//////////////////
/// SPI module ///
//////////////////

/**
 *
 */

#define _SPI_PORT   PORTB
#define _SPI_SS     PB2
#define _SPI_MOSI   PB3
#define _SPI_MISO   PB4
#define _SPI_SCK    PB5

struct spiClockDivideBy
{
    spiClockDivideBy(uint8_t rSPI2X, uint8_t rSPR1, uint8_t rSPR0)
    {
        SPSR = SPSR | (rSPI2X << SPI2X);
        SPCR = SPCR | (rSPR1 << SPR1) | (rSPR0 << SPR0);
    }
};

struct spiClockDivideBy2 : spiClockDivideBy
{
    spiClockDivideBy2() : spiClockDivideBy(1, 0, 0) {}
};

struct spiClockDivideBy4 : spiClockDivideBy
{
    spiClockDivideBy4() : spiClockDivideBy(0, 0, 0) {}
};

struct spiClockDivideBy8 : spiClockDivideBy
{
    spiClockDivideBy8() : spiClockDivideBy(1, 0, 1) {}
};

struct spiClockDivideBy16 : spiClockDivideBy
{
    spiClockDivideBy16() : spiClockDivideBy(0, 0, 1) {}
};

struct spiClockDivideBy32 : spiClockDivideBy
{
    spiClockDivideBy32() : spiClockDivideBy(1, 1, 0) {}
};

struct spiClockDivideBy64 : spiClockDivideBy
{
    spiClockDivideBy64() : spiClockDivideBy(0, 1, 0) {}
};

struct spiClockDivideBy128 : spiClockDivideBy
{
    spiClockDivideBy128() : spiClockDivideBy(0, 1, 1) {}
};

struct spiClockResetState
{
    spiClockResetState()
    {
        SPSR = SPSR & ~(1 << SPI2X);
        SPCR = SPCR & ~((1 << SPR1) | (1 << SPR0));
    }
};

enum class spiClock_t : uint8_t
{
    divideBy2,
    divideBy4,
    divideBy8,
    divideBy16,
    divideBy32,
    divideBy64,
    divideBy128
};

enum class spiConfig_t : uint8_t
{
    master,
    slave,
    off
};

enum class spiMode_t : uint8_t
{
    mode0 = 0,
    mode1 = (1 << CPHA),
    mode2 = (1 << CPOL),
    mode3 = (1 << CPOL) | (1 << CPHA)
};

enum class spiDataOrder_t : uint8_t
{
    msbFisrt = 0,
    lsbFirst = (1 << DORD)
};

enum class spiUseInterrupt_t : uint8_t
{
    no = 0,
    yes = (1 << SPIE)
};

////////////////////
/// Timer module ///
////////////////////

struct timer0RegisterConfig
{
    timer0RegisterConfig(uint8_t rWGM02, uint8_t rWGM01, uint8_t rWGM00)
    {
        TCCR0A = TCCR0A | (rWGM01 << WGM01) | (rWGM00 << rWGM00);
        TCCR0B = TCCR0B | (rWGM02 << WGM02);
    }
};

struct timer0AsNormal : timer0RegisterConfig
{
    timer0AsNormal() : timer0RegisterConfig(0, 0, 0) {}
};

struct timer0AsCTC : timer0RegisterConfig
{
    timer0AsCTC() : timer0RegisterConfig(0, 1, 0) {}
};

struct timer0AsPwm : timer0RegisterConfig
{
    timer0AsPwm() : timer0RegisterConfig(0, 1, 1) {}
};

struct timer0AsPwmPhaseCorrect : timer0RegisterConfig
{
    timer0AsPwmPhaseCorrect() : timer0RegisterConfig(0, 0, 1) {}
};

// TODO add support to select frequency automatically (ctc mode only)
enum class timer0Config_t : uint8_t
{
    off = 0,
    normal,
    ctc,
    pwmChannelA   = (1 << COM0A1),
    pwmChannelB   = (1 << COM0B1),
    pwmChannelsAB = (1 << COM0A1) | (1 << COM0B1),
    pwmPhaseCorrectA,
    pwmPhaseCorrectB,
    pwmPhaseCorrectAB
};

enum class timer0APolarity_t : uint8_t
{
    normal,
    inverted
};

enum class timer0BPolarity_t : uint8_t
{
    normal,
    inverted
};

enum class timer0Clock_t : uint8_t
{
    off                 = 0,
    noPreescale         = (1 << CS00),
	divideBy8           = (1 << CS01),
	divideBy64          = (1 << CS01) | (1 << CS00),
	divideBy256         = (1 << CS02),
	divideBy1024        = (1 << CS02) | (1 << CS00),
	externT0FallingEdge = (1 << CS02) | (1 << CS01),
	externT0RisingEdge  = (1 << CS02) | (1 << CS01) | (1 << CS00),
	setState            = (1 << CS02) | (1 << CS01) | (1 << CS00)
};

/**@}*/

#endif /* AVRLIB_AVRLIB_HAL_DEVICES_ATMEGA328P_H_ */
