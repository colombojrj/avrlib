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
//#define SUPPORT_TO_SPI
//#define SUPPORT_TO_ADC
//#define SUPPORT_TO_I2C
//#define SUPPORT_TO_TIMER0
//#define SUPPORT_TO_TIMER1
//#define SUPPORT_TO_TIMER2
//#define SUPPORT_TO_HAL

/**
 * @brief Basic port registers structure.
 */
struct GpioRegs
{
    volatile uint8_t* outputData; //!< It is the address of the output data register (in AVR architecture it is called usually as PORT)
    volatile uint8_t* direction;  //!< It is the address of the direction register (in AVR architecture it is called usually as DDR)
    volatile uint8_t* inputData;  //!< It is the address of input data register (in AVR architecture it is called usually as PIN)
    volatile uint8_t* pcmsk;      //!< It is the address of pin change interrupt register
    const uint8_t whatPCI;        //!< It is the Port Change Interrupt bit
};

/// Port B registers declaration
constexpr GpioRegs RegsPortB = {&PORTB, &DDRB, &PINB, &PCMSK0, PCIE0};

/// Port C registers declaration
constexpr GpioRegs RegsPortC = {&PORTC, &DDRC, &PINC, &PCMSK1, PCIE1};

/// Port D registers declaration
constexpr GpioRegs RegsPortD = {&PORTD, &DDRD, &PIND, &PCMSK2, PCIE2};

/**
 * @brief Basic gpio register structure.
 */
struct Gpio
{
    const uint8_t pinNumber;  //!< It is the pin number
    const GpioRegs regs;      //!< Gpio registers structure @see GpioRegs
    const uint8_t hasInt;     //!< It informs if the current pin has INT associated
    const uint8_t whatInt;    //!< It informs what INT is associated (INT0 or INT1)
};

/// NewGpio_t type definition
typedef Gpio gpio_t;

/// Pin PB0 declaration
constexpr gpio_t _PinB0 = {PB0, RegsPortB, 0, 0};

/// Pin PB1 declaration
constexpr gpio_t _PinB1 = {PB1, RegsPortB, 0, 0};

/// Pin PB2 declaration
constexpr gpio_t _PinB2 = {PB2, RegsPortB, 0, 0};

/// Pin PB3 declaration
constexpr gpio_t _PinB3 = {PB3, RegsPortB, 0, 0};

/// Pin PB4 declaration
constexpr gpio_t _PinB4 = {PB4, RegsPortB, 0, 0};

/// Pin PB5 declaration
constexpr gpio_t _PinB5 = {PB5, RegsPortB, 0, 0};

/// Pin PB6 declaration
constexpr gpio_t _PinB6 = {PB6, RegsPortB, 0, 0};

/// Pin PB7 declaration
constexpr gpio_t _PinB7 = {PB7, RegsPortB, 0, 0};

/// Pin PC0 declaration
constexpr gpio_t _PinC0 = {PC0, RegsPortC, 0, 0};

/// Pin PC1 declaration
constexpr gpio_t _PinC1 = {PC1, RegsPortC, 0, 0};

/// Pin PC2 declaration
constexpr gpio_t _PinC2 = {PC2, RegsPortC, 0, 0};

/// Pin PC3 declaration
constexpr gpio_t _PinC3 = {PC3, RegsPortC, 0, 0};

/// Pin PC4 declaration
constexpr gpio_t _PinC4 = {PC4, RegsPortC, 0, 0};

/// Pin PC5 declaration
constexpr gpio_t _PinC5 = {PC5, RegsPortC, 0, 0};

/// Pin PC6 declaration
constexpr gpio_t _PinC6 = {PC6, RegsPortC, 0, 0};

/// Pin PD0 declaration
constexpr gpio_t _PinD0 = {PD0, RegsPortD, 1, INT0};

/// Pin PD1 declaration
constexpr gpio_t _PinD1 = {PD1, RegsPortD, 1, INT1};

/// Pin PD2 declaration
constexpr gpio_t _PinD2 = {PD2, RegsPortD, 0, 0};

/// Pin PD3 declaration
constexpr gpio_t _PinD3 = {PD3, RegsPortD, 0, 0};

/// Pin PD4 declaration
constexpr gpio_t _PinD4 = {PD4, RegsPortD, 0, 0};

/// Pin PD5 declaration
constexpr gpio_t _PinD5 = {PD5, RegsPortD, 0, 0};

/// Pin PD6 declaration
constexpr gpio_t _PinD6 = {PD6, RegsPortD, 0, 0};

/// Pin PD7 declaration
constexpr gpio_t _PinD7 = {PD7, RegsPortD, 0, 0};

/// Friendly PinB0 definition
#define PinB0 &_PinB0

/// Friendly PinB1 definition
#define PinB1 &_PinB1

/// Friendly PinB2 definition
#define PinB2 &_PinB2

/// Friendly PinB3 definition
#define PinB3 &_PinB3

/// Friendly PinB4 definition
#define PinB4 &_PinB4

/// Friendly PinB5 definition
#define PinB5 &_PinB5

/// Friendly PinB6 definition
#define PinB6 &_PinB6

/// Friendly PinB7 definition
#define PinB7 &_PinB7

/// Friendly PinC0 definition
#define PinC0 &_PinC0

/// Friendly PinB1 definition
#define PinC1 &_PinC1

/// Friendly PinB2 definition
#define PinC2 &_PinC2

/// Friendly PinB3 definition
#define PinC3 &_PinC3

/// Friendly PinB4 definition
#define PinC4 &_PinC4

/// Friendly PinB5 definition
#define PinC5 &_PinC5

/// Friendly PinB6 definition
#define PinC6 &_PinC6

/// Friendly PinD0 definition
#define PinD0 &_PinD0

/// Friendly PinD1 definition
#define PinD1 &_PinD1

/// Friendly PinD2 definition
#define PinD2 &_PinD2

/// Friendly PinD3 definition
#define PinD3 &_PinD3

/// Friendly PinD4 definition
#define PinD4 &_PinD4

/// Friendly PinD5 definition
#define PinD5 &_PinD5

/// Friendly PinD6 definition
#define PinD6 &_PinD6

/// Friendly PinD7 definition
#define PinD7 &_PinD7

/// Interrupt INT0 definition
#define gpioInt0 PinD2

/// Interrupt INT1 definition
#define gpioInt1 PinD3

/**
 * @brief Gpio interrupt trigger
 */
enum class gpioInt : uint8_t
{
    lowLevel    = 0b00, //!< Interrupt triggered on pin low level
    anyChange   = 0b01, //!< Interrupt triggered on any pin change (falling or rising)
    fallingEdge = 0b10, //!< Interrupt triggered on falling edge
    risingEdge  = 0b11, //!< Interrupt triggered on rising edge
    setState    = 0b11  //!< Reserved for library purposes
};

/**
 * @brief Configures the GPIO as input or output
 *
 * @note The ATmega328P supports only push-pull configuration.
 *       However, open drain configuration may be obtained through
 *       software emulation.
 *
 * @todo Add support to open-drain configuration
 */
enum class GpioDir : uint8_t
{
    input = 0,  //!< configures the gpio pin as input
    output = 1  //!< configures the gpio pin as output
};

/// Define gpioConfig_t type
typedef GpioDir gpioDir_t;

/**
 * @brief Controls the pull up resistor of each GPIO pin
 */
enum class GpioPullResistor_t : uint8_t
{
    disable = 0, //!< disables the gpio pull up resistor
    enable = 1   //!< enables the gpio pull up resistor
};

/// Define gpioPullResistor_t type
typedef GpioPullResistor_t gpioPullResistor_t;

/**
 * @brief SPI registers structure.
 *
 * This struct holds the SPI registers.
 *
 * @param control is the SPI registers control. ATmega devices
 *        usually have two main registers to control the SPI
 *        peripheral: SPCR and SPSR. Because these registers
 *        are aligned in the memory map (see page 425 of the
 *        datasheet) they can be used as a single 16 bits
 *        registers. With this definition, this variable is
 *        located in 0x2C (SPCR address)
 *
 * @param data is the SPI data register (ATmega datasheet calls it as SPDR)
 */
struct SpiRegs
{
    /// SPCR and SPSR with address of SPCR
    volatile uint16_t* control;

    /// SPDR address
    volatile uint8_t* data;
};

typedef SpiRegs spiRegs_t;

/// SPI declaration
constexpr spiRegs_t spiRegs = {(uint16_t*) &SPCR, &SPDR};

/**
 * Available SPI clock divider configurations
 */
enum class SpiClock : uint16_t
{
    divideBy2   = (1 << (SPI2X+8)),                            //!< SPI is driven with CPU clock divided by 2
    divideBy4   = 0,                                           //!< SPI is driven with CPU clock divided by 4
    divideBy8   = (1 << SPR0) | (1 << (SPI2X+8)),              //!< SPI is driven with CPU clock divided by 8
    divideBy16  = (1 << SPR0),                                 //!< SPI is driven with CPU clock divided by 16
    divideBy32  = (1 << SPR1) | (1 << (SPI2X+8)),              //!< SPI is driven with CPU clock divided by 32
    divideBy64  = (1 << SPR1),                                 //!< SPI is driven with CPU clock divided by 64
    divideBy128 = (1 << SPR1) | (1 << SPR0),                   //!< SPI is driven with CPU clock divided by 128
    setState    = (1 << SPR1) | (1 << SPR0) | (1 << (SPI2X+8)) //!< only for library purposes
};

/// Define spiClock_t type
typedef SpiClock spiClock_t;

/**
 * @brief Available SPI operation modes
 */
enum class SpiMode : uint8_t
{
    master = (1 << MSTR), //!< SPI as master
    slave = 0             //!< SPI as slave
};

/// Define spiMode_t type
typedef SpiMode spiMode_t;

/// SPI clock polarity
typedef enum class SpiClockPolarity_t : uint8_t
{
    normal = 0,
    inverted = (1 << CPOL)
} spiClockPolarity_t;

/// SPI clock phase
typedef enum class SpiClockPhase_t : uint8_t
{
    rising = 0,
    falling = (1 << CPHA)
} spiClockPhase_t;

/**
 * spiDataOrder_t
 *
 * @param msbFirst
 * @param lsbFirst
 */
typedef enum class SpiDataOrder_t : uint8_t
{
    msbFisrt = 0,
    lsbFirst = (1 << DORD)
} spiDataOrder_t;

/// SPI driven with interrupts?
typedef enum class SpiUseInterrupt_t : uint8_t
{
    no = 0,
    yes = (1 << SPIE)
} spiUseInterrupt_t;

/**
 * spiConfig_t
 *
 * @brief SPI configuration struct
 *
 * The SPI hardware configuration is done with this struct.
 *
 * In order to make programming easier, all the internal variables
 * are enum classes, what allows for IDE auto completition to assist
 * the programmer.
 *
 * The ATmega328P supports:
 * @param spiClock_t clock configures the SPI clock divider
 * @param spiMode_t mode configures the SPI as master or slave
 * @param spiClockPhase_t spiClockPhase configures the clock phase
 * @param spiClockPolarity_t spiClockPolarity configures the clock polarity
 * @param spiDataOrder_t dataOrder configures data order (MSB or LSB first)
 * @param spiUseInterrupt_t useInterrupt configures if the SPI is driven with interrupts
 */
typedef struct SpiConfig
{
    /// SPI clock divider
    spiClock_t clock;

    /// SPI mode
    spiMode_t mode;

    /// SPI clock phase
    spiClockPhase_t spiClockPhase;

    /// SPI clock polarity
    spiClockPolarity_t spiClockPolarity;

    /// SPI data order
    spiDataOrder_t dataOrder;

    /// SPI driven with interrupts?
    spiUseInterrupt_t useInterrupt;
} spiConfig_t;

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
