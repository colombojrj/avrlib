#ifndef AVRLIB_AVRLIB_HAL_DEVICES_ATMEGA328P_H_
#define AVRLIB_AVRLIB_HAL_DEVICES_ATMEGA328P_H_

/**
 * @defgroup hal_device_atmega328p_group ATmega88P/ATmega168P/ATmega328P
 *
 * Some functions were replaced with macros. The main advantage is
 * the code size (macros use less flash memory)
 *
 * See https://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_port_pass
 */

/**@{*/

#include <stdlib.h>
#include <avr/io.h>

// Supported stuff
#define SUPPORT_TO_GPIO
#define SUPPORT_TO_SPI
#define SUPPORT_TO_ADC
//#define SUPPORT_TO_I2C
//#define SUPPORT_TO_TIMER0
//#define SUPPORT_TO_TIMER1
//#define SUPPORT_TO_TIMER2
//#define SUPPORT_TO_HAL

#define DDR(port) (*(&port - 1)) // This does not work for ATmega64 or higher
#define PIN(port) (*(&port - 2)) // This does not work for ATmega64 or higher

/**
 * gpio_t
 *
 * @brief Basic pin structure.
 *
 * This struct holds information about an specific pin:
 * @param pinNumber is the pin number
 * @param port is the port address
 * @param ddr is the direction register address
 * @param pin is the pin value register address
 * @param pcmsk is the pin change interrupt register address
 */
typedef struct Gpio
{
    const uint8_t pinNumber;
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t* pin;
    volatile uint8_t* pcmsk;
} gpio_t;

/// Pin PB0 declaration
constexpr gpio_t PinB0 = {PB0, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PB1 declaration
constexpr gpio_t PinB1 = {PB1, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PB2 declaration
constexpr gpio_t PinB2 = {PB2, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PB3 declaration
constexpr gpio_t PinB3 = {PB3, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PB4 declaration
constexpr gpio_t PinB4 = {PB4, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PB5 declaration
constexpr gpio_t PinB5 = {PB5, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PB6 declaration
constexpr gpio_t PinB6 = {PB6, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PB7 declaration
constexpr gpio_t PinB7 = {PB7, &PORTB, &DDRB, &PINB, &PCMSK0};

/// Pin PC0 declaration
constexpr gpio_t PinC0 = {PC0, &PORTC, &DDRC, &PINC, &PCMSK1};

/// Pin PC1 declaration
constexpr gpio_t PinC1 = {PC1, &PORTC, &DDRC, &PINC, &PCMSK1};

/// Pin PC2 declaration
constexpr gpio_t PinC2 = {PC2, &PORTC, &DDRC, &PINC, &PCMSK1};

/// Pin PC3 declaration
constexpr gpio_t PinC3 = {PC3, &PORTC, &DDRC, &PINC, &PCMSK1};

/// Pin PC4 declaration
constexpr gpio_t PinC4 = {PC4, &PORTC, &DDRC, &PINC, &PCMSK1};

/// Pin PC5 declaration
constexpr gpio_t PinC5 = {PC5, &PORTC, &DDRC, &PINC, &PCMSK1};

/// Pin PC6 declaration
constexpr gpio_t PinC6 = {PC6, &PORTC, &DDRC, &PINC, &PCMSK1};

/// Pin PD0 declaration
constexpr gpio_t PinD0 = {PD0, &PORTD, &DDRD, &PIND, &PCMSK2};

/// Pin PD1 declaration
constexpr gpio_t PinD1 = {PD1, &PORTD, &DDRD, &PIND, &PCMSK2};

/// Pin PD2 declaration
constexpr gpio_t PinD2 = {PD2, &PORTD, &DDRD, &PIND, &PCMSK2};

/// Pin PD3 declaration
constexpr gpio_t PinD3 = {PD3, &PORTD, &DDRD, &PIND, &PCMSK2};

/// Pin PD4 declaration
constexpr gpio_t PinD4 = {PD4, &PORTD, &DDRD, &PIND, &PCMSK2};

/// Pin PD5 declaration
constexpr gpio_t PinD5 = {PD5, &PORTD, &DDRD, &PIND, &PCMSK2};

/// Pin PD6 declaration
constexpr gpio_t PinD6 = {PD6, &PORTD, &DDRD, &PIND, &PCMSK2};

/// Pin PD7 declaration
constexpr gpio_t PinD7 = {PD7, &PORTD, &DDRD, &PIND, &PCMSK2};

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

enum class gpioConfig_t : uint8_t
{
    input,
    inputWithPullUp,
    output
};

/**
 * spi_t
 *
 * @brief SPI registers structure.
 *
 * This struct holds information about an specific pin:
 * @param spiControl is the SPI registers control. ATmega devices usually have two main
 *        registers to control the SPI peripheral: SPCR and SPSR. Because these registers
 *        are aligned in the memory map (see page 425 of the datasheet) they can be used
 *        as a single 16 bits registers. With this definition, this variable is located
 *        in 0x2C (SPCR address)
 * @param data is the SPI data register (ATmega datasheet calls it as SPDR)
 */

//////////////////
/// SPI module ///
//////////////////

typedef struct Spi
{
    /// SPCR and SPSR with address of SPCR
    volatile uint16_t* spiControl;

    /// SPDR address
    volatile uint8_t* data;
} spi_t;

/// SPI declaration
constexpr spi_t spi = {(uint16_t*) &SPCR, &SPDR};

enum class spiClock_t : uint16_t
{
    divideBy2 = (1 << (SPI2X+8)),
    divideBy4 = 0,
    divideBy8 = (1 << SPR0) | (1 << (SPI2X+8)),
    divideBy16 = (1 << SPR0),
    divideBy32 = (1 << SPR1) | (1 << (SPI2X+8)),
    divideBy64 = (1 << SPR1),
    divideBy128 = (1 << SPR1) | (1 << SPR0),
    setState = (1 << SPR1) | (1 << SPR0) | (1 << (SPI2X+8))
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

/// SPI SS pin definition
#define _SPI_SS_PIN      PinB2

/// SPI MOSI pin definition
#define _SPI_MOSI_PIN    PinB3

/// SPI MISO pin definition
#define _SPI_MISO_PIN    PinB4

/// SPI SCK pin definition
#define _SPI_SCK_PIN     PinB5

//////////////////
/// ADC Module ///
//////////////////

/**
 * adcConfig_t
 */
enum class adcConfig_t : uint8_t
{
    off,              //!< off
    singleConversion, //!< singleConversion
    noiseReduction,   //!< noiseReduction
    scanMode          //!< Scan mode
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






////////////////////
/// Timer module ///
////////////////////

// Timer 0
#define OC0A_PIN            PinD6
#define OC0B_PIN            PinD5
#define T0_PIN              PinD4

// Timer 1
#define OC1A_PIN            PinB1
#define OC1B_PIN            PinB2
#define T1_PIN              PinD5

// Timer 2
#define OC2A_PIN            PinB3
#define OC2B_PIN            PinD3

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
