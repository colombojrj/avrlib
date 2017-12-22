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

// Timer 0
#define OC0A_PIN            PD6
#define OC0A_PORT           PORTD
#define OC0B_PIN            PD5
#define OC0B_PORT           PORTD
#define T0_PIN              PD4
#define T0_PORT             PORTD

// Timer 1
#define OC1A_PIN            PB1
#define OC1A_PORT           PORTB
#define OC1B_PIN            PB2
#define OC1B_PORT           PORTB
#define T1_PIN              PD5
#define T1_PORT             PORTD

// Timer 2
#define OC2A_PIN            PB3
#define OC2A_PORT           PORTB
#define OC2B_PIN            PD3
#define OC2B_PORT           PORTD

enum class timer0OutputConfig_t
{
    off                = 0,
    channelAnormal     = (1 << COM0A1),
    channelAinverted   = (1 << COM0A1) | (1 << COM0A0),
    channelBnormal     = (1 << COM0B1),
    channelBinverted   = (1 << COM0B1) | (1 << COM0B0),
    channelAsetState   = (1 << COM0A1) | (1 << COM0A0),
    channelBsetState   = (1 << COM0B1) | (1 << COM0B0),
    channelABnormal    = channelAnormal | channelBnormal,
    channelABinverted  = channelAinverted | channelBinverted
};

// TODO add support to select frequency automatically (ctc mode only)
enum class timer0Config_t : uint8_t
{
    off = 0,
    normal = 0,
    ctc = (1 << WGM01),
    pwmChannelA = (1 << WGM01) | (1 << WGM00),
    pwmChannelB = (1 << WGM01) | (1 << WGM00),
    pwmChannelsAB = (1 << WGM01) | (1 << WGM00),
    pwmPhaseCorrectA = (1 << WGM00),
    pwmPhaseCorrectB = (1 << WGM00),
    pwmPhaseCorrectAB = (1 << WGM00)
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

////////////////////
// TIMER 1 MODULE //
////////////////////
enum class timer1Config_t : uint8_t
{
    off,
    normal,
    ctc,
    pwm8Bits,
    pwm9Bits,
    pwm10Bits,
    pwmDefinedTop,
    pwmPhaseCorrect8Bits,
    pwmPhaseCorrect9Bits,
    pwmPhaseCorrect10Bits,
    pwmPhaseCorrectDefinedTop
};

enum class timer1OutputConfig_t
{
    off                = 0,
    channelAnormal     = (1 << COM1A1),
    channelAinverted   = (1 << COM1A1) | (1 << COM1A0),
    channelBnormal     = (1 << COM1B1),
    channelBinverted   = (1 << COM1B1) | (1 << COM1B0),
    channelAsetState   = (1 << COM1A1) | (1 << COM1A0),
    channelBsetState   = (1 << COM1B1) | (1 << COM1B0),
    channelABnormal    = channelAnormal | channelBnormal,
    channelABinverted  = channelAinverted | channelBinverted
};

enum class timer1Clock_t : uint8_t
{
    off                 = 0,
    noPreescale         = (1 << CS10),
    divideBy8           = (1 << CS11),
    divideBy64          = (1 << CS11) | (1 << CS10),
    divideBy256         = (1 << CS12),
    divideBy1024        = (1 << CS12) | (1 << CS10),
    externT1FallingEdge = (1 << CS12) | (1 << CS11),
    externT1RisingEdge  = (1 << CS12) | (1 << CS11) | (1 << CS10),
    setState            = (1 << CS12) | (1 << CS11) | (1 << CS10)
};

enum class timer1InputCaptureEdge_t : uint8_t
{
    risingEdge  = (1 << ICNC1) | (1 << ICES1),
    fallingEdge = (1 << ICNC1),
    setState    = (1 << ICNC1) | (1 << ICES1)
};

struct timer1RegisterConfig
{
    timer1RegisterConfig(uint8_t rWGM13, uint8_t rWGM12, uint8_t rWGM11, uint8_t rWGM10, timer1OutputConfig_t output)
    {
        TCCR1A = (rWGM11 << WGM11) | (rWGM10 << WGM10) | static_cast<uint8_t>(output);
        TCCR1B = (rWGM13 << WGM13) | (rWGM12 << WGM12);
    }
};

struct timer1AsNormal : timer1RegisterConfig
{
    timer1AsNormal(timer1OutputConfig_t output) : timer1RegisterConfig(0, 0, 0, 0, output) {}
};

struct timer1AsCTC : timer1RegisterConfig
{
    timer1AsCTC(timer1OutputConfig_t output) : timer1RegisterConfig(0, 1, 0, 0, output) {}
};

struct timer1As8bitPwm : timer1RegisterConfig
{
    timer1As8bitPwm(timer1OutputConfig_t output) : timer1RegisterConfig(0, 1, 0, 1, output) {}
};

struct timer1As9bitPwm : timer1RegisterConfig
{
    timer1As9bitPwm(timer1OutputConfig_t output) : timer1RegisterConfig(0, 1, 1, 0, output) {}
};

struct timer1As10bitPwm : timer1RegisterConfig
{
    timer1As10bitPwm(timer1OutputConfig_t output) : timer1RegisterConfig(0, 1, 1, 1, output) {}
};

struct timer1As16bitPwm : timer1RegisterConfig
{
    timer1As16bitPwm(timer1OutputConfig_t output, uint16_t rICR1) : timer1RegisterConfig(1, 1, 1, 0, output)
    {
        ICR1 = rICR1;
    }
};

struct timer1As8bitPhaseCorrectPwm : timer1RegisterConfig
{
    timer1As8bitPhaseCorrectPwm(timer1OutputConfig_t output) : timer1RegisterConfig(0, 0, 0, 1, output) {}
};

struct timer1As9bitPhaseCorrectPwm : timer1RegisterConfig
{
    timer1As9bitPhaseCorrectPwm(timer1OutputConfig_t output) : timer1RegisterConfig(0, 0, 1, 0, output) {}
};

struct timer1As10bitPhaseCorrectPwm : timer1RegisterConfig
{
    timer1As10bitPhaseCorrectPwm(timer1OutputConfig_t output) : timer1RegisterConfig(0, 0, 1, 1, output) {}
};

struct timer1As16bitPhaseCorrectPwm : timer1RegisterConfig
{
    timer1As16bitPhaseCorrectPwm(timer1OutputConfig_t output, uint16_t rICR1) : timer1RegisterConfig(1, 0, 0, 0, output)
    {
        ICR1 = rICR1;
    }
};

////////////////////
// TIMER 2 MODULE //
////////////////////
enum class timer2Config_t : uint8_t
{
    off             = 0,
    normal          = 0,
    ctc             = (1 << WGM21),
    pwm             = (1 << WGM21) | (1 << WGM20),
    pwmPhaseCorrect = (1 << WGM20)
};

enum class timer2OutputConfig_t
{
    off                = 0,
    channelAnormal     = (1 << COM2A1),
    channelAinverted   = (1 << COM2A1) | (1 << COM2A0),
    channelBnormal     = (1 << COM2B1),
    channelBinverted   = (1 << COM2B1) | (1 << COM2B0),
    channelAsetState   = (1 << COM2A1) | (1 << COM2A0),
    channelBsetState   = (1 << COM2B1) | (1 << COM2B0),
    channelABnormal    = channelAnormal | channelBnormal,
    channelABinverted  = channelAinverted | channelBinverted
};

enum class timer2Clock_t : uint8_t
{
    off                = 0,
    noPreescale        = (1 << CS20),
    divideBy8          = (1 << CS21),
    divideBy32         = (1 << CS21) | (1 << CS20),
    divideBy64         = (1 << CS22),
    divideBy128        = (1 << CS22) | (1 << CS20),
    divideBy256        = (1 << CS22) | (1 << CS21),
    divideBy1024       = (1 << CS22) | (1 << CS21) | (1 << CS20),
    setState           = (1 << CS22) | (1 << CS21) | (1 << CS20)
};

/**@}*/

#endif /* AVRLIB_AVRLIB_HAL_DEVICES_ATMEGA328P_H_ */
