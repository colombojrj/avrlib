#ifndef AVRLIB_AVRLIB_HAL_DEVICESLL_ATMEGA328P_H_
#define AVRLIB_AVRLIB_HAL_DEVICESLL_ATMEGA328P_H_

#include <stdlib.h>
#include <avr/io.h>

#include "../HAL/timers.h"

/**
 * @defgroup hal_device_atmega328p_group ATmega88P/ATmega168P/ATmega328P
 *
 * Some functions were replaced with macros. The main advantage is
 * the code size (macros use less flash memory)
 *
 * See https://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_port_pass
 */

/**@{*/

/// AVRLIB supports GPIO
#define SUPPORT_TO_GPIO

/// AVRLIB supports SPI
//#define SUPPORT_TO_SPI

/// AVRLIB supports ADC
//#define SUPPORT_TO_ADC

/// AVRLIB supports I2C
//#define SUPPORT_TO_I2C

/// AVRLIB supports TIMER 0
//#define SUPPORT_TO_TIMER0

/// AVRLIB supports TIMER 1
//#define SUPPORT_TO_TIMER1

/// AVRLIB supports TIMER 2
//#define SUPPORT_TO_TIMER2

/// Port B registers declaration
constexpr GpioRegs RegsPortB = {&PORTB, &DDRB, &PINB, &PCMSK0, PCIE0};

/// Port C registers declaration
constexpr GpioRegs RegsPortC = {&PORTC, &DDRC, &PINC, &PCMSK1, PCIE1};

/// Port D registers declaration
constexpr GpioRegs RegsPortD = {&PORTD, &DDRD, &PIND, &PCMSK2, PCIE2};

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
constexpr gpio_t _PinD0 = {PD0, RegsPortD, 0, 0};

/// Pin PD1 declaration
constexpr gpio_t _PinD1 = {PD1, RegsPortD, 0 ,0};

/// Pin PD2 declaration
constexpr gpio_t _PinD2 = {PD2, RegsPortD, 1, INT0};

/// Pin PD3 declaration
constexpr gpio_t _PinD3 = {PD3, RegsPortD, 1, INT1};

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

/// Timer 0 comparator output A pin
#define OC0A_PIN            PinD6

/// Timer 0 comparator output B pin
#define OC0B_PIN            PinD5

/// External clock input of Timer 0
#define T0_PIN              PinD4

/// Timer 1 comparator output A pin
#define OC1A_PIN            PinB1

/// Timer 1 comparator output A pin
#define OC1B_PIN            PinB2

/// External clock input of Timer 1
#define T1_PIN              PinD5

/// Timer 2 comparator output A pin
#define OC2A_PIN            PinB3

/// Timer 2 comparator output A pin
#define OC2B_PIN            PinD3

/**
 * @brief Timer 0 output compare unit available configurations
 */
constexpr uint8_t timer0OutputAConfig[] = {
        0,
        (1 << COM0A1),
        (1 << COM0A1) | (1 << COM0A0)
};

/**
 * @brief Timer 0 output compare unit available configurations
 */
constexpr uint8_t timer0OutputBConfig[] = {
        0,
        (1 << COM0B1),
        (1 << COM0B1) | (1 << COM0B0)
};

/**
 * @brief Timer 1 output compare unit available configurations
 */
constexpr uint8_t timer1OutputAConfig[] = {
        0,
        (1 << COM1A1),
        (1 << COM1A1) | (1 << COM1A0)
};

/**
 * @brief Timer 1 output compare unit available configurations
 */
constexpr uint8_t timer1OutputBConfig[] = {
        0,
        (1 << COM1B1),
        (1 << COM1B1) | (1 << COM1B0)
};

/**
 * @brief Timer 2 output compare unit available configurations
 */
constexpr uint8_t timer2OutputAConfig[] = {
        0,
        (1 << COM2A1),
        (1 << COM2A1) | (1 << COM2A0)
};

/**
 * @brief Timer 2 output compare unit available configurations
 */
constexpr uint8_t timer2OutputBConfig[] = {
        0,
        (1 << COM2B1),
        (1 << COM2B1) | (1 << COM2B0)
};

/// Timer 0 registers definition
constexpr timer8bRegs Timer0Regs = {
        (uint16_t*) &TCCR0A,
        &TCNT0,
        &OCR0A,
        &OCR0B,
        &TIMSK0,
        &TIFR0,
        nullptr,
        nullptr,
        PinD6,
        PinD5,
        PRTIM0
};

/// Timer 1 registers definition
constexpr timer16bRegs Timer1Regs = {
        (uint16_t*) &TCCR1A,
        &TCNT1,
        &OCR1A,
        &OCR1B,
        &ICR1,
        &TIMSK1,
        &TIFR1,
        PinB1,
        PinB2,
        PRTIM1
};

/// Timer 2 registers definition
constexpr timer8bRegs Timer2Regs = {
        (uint16_t*) &TCCR2A,
        &TCNT2,
        &OCR2A,
        &OCR2B,
        &TIMSK2,
        &TIFR2,
        &ASSR,
        &GTCCR,
        PinB3,
        PinD3,
        PRTIM2
};

/// Timer 0 config structure
constexpr timer8b _Timer0 = {
        &Timer0Regs,
        true,
        timer0OutputAConfig,
        timer0OutputBConfig,
        &whatTimer0OutputAConfig,
        &whatTimer0OutputBConfig,
        (1 << COM0A1) | (1 << COM0A0),
        (1 << COM0B1) | (1 << COM0B0),
        0,
        &timer0MaxCount
};

/// Timer 0 config structure
constexpr timer16b _Timer1 = {
        &Timer1Regs,
        true,
        timer1OutputAConfig,
        timer1OutputBConfig,
        &whatTimer1OutputAConfig,
        &whatTimer1OutputBConfig,
        (1 << COM1A1) | (1 << COM1A0),
        (1 << COM1B1) | (1 << COM1B0),
        0,
        0
};

/// Timer 0 config structure
constexpr timer8b _Timer2 = {
        &Timer2Regs,
        true,
        timer2OutputAConfig,
        timer2OutputBConfig,
        &whatTimer2OutputAConfig,
        &whatTimer2OutputBConfig,
        (1 << COM2A1) | (1 << COM2A0),
        (1 << COM2B1) | (1 << COM2B0),
        0,
        0
};

/// Timer 0 friendly definition
#define Timer0 &_Timer0

/// Timer 0 friendly definition
#define Timer1 &_Timer1

/// Timer 0 friendly definition
#define Timer2 &_Timer2

/**
 * @brief This structure holds the available timer configuration
 */
enum class timer0Mode : uint8_t
{
    normal          = 0,                           //!< Timer operates in normal mode
    ctc             = (1 << WGM01),                //!< Timer operates in CTC mode (clear on top)
    pwm             = (1 << WGM01) | (1 << WGM00), //!< Timer operates as pwm generator
    pwmPhaseCorrect = (1 << WGM00)                 //!< Timer operates in pwm phase correct mode
};

/**
 * @brief Available timer clock source configurations
 */
enum class timer0Clock : uint16_t
{
    off               = 0,                                                   //!< Timer has no clock (saves power)
    noPreescale       = (1 << (CS00+8)),                                     //!< CPU clock is applied directly on the timer
    divideBy8         = (1 << (CS01+8)),                                     //!< Divides CPU clock by 8
    divideBy64        = (1 << (CS01+8)) | (1 << (CS00+8)),                   //!< Divides CPU clock by 64
    divideBy256       = (1 << (CS02+8)),                                     //!< Divides CPU clock by 256
    divideBy1024      = (1 << (CS02+8)) | (1 << (CS00+8)),                   //!< Divides CPU clock by 1024
    externFallingEdge = (1 << (CS02+8)) | (1 << (CS01+8)),                   //!< Timer clock is driven from external source connected on pin but it is only sensible to falling edges
    externRisingEdge  = (1 << (CS02+8)) | (1 << (CS01+8)) | (1 << (CS00+8)), //!< Timer clock is driven from external source connected on pin but it is only sensible to rising edges
};

enum class timer0Interrupt : uint8_t
{
    none = 0,                        //!< Does not generate any interrupt
    onCompareMatchA = (1 << OCIE0A), //!< Generate an interrupt on compare match of channel A
    onCompareMatchB = (1 << OCIE0B), //!< Generate an interrupt on compare match of channel B
    onOverflow      = (1 << TOIE0)   //!< Generate an interrupt on after an overflow
};

enum class timer1Mode : uint16_t
{
    normal                    = 0,                                                  //!< Timer operates in normal mode
    ctcTopOnICR1              = (1 << (WGM13+8)) | (1 << (WGM12+8)),                //!< Timer operates in CTC mode (clear on input capture register (ICR1))
    ctcTopOnOCR1A             = (1 << (WGM12+8)),                                   //!< Timer operates in CTC mode (clear on output compare channel A register (OCR1A))
    pwm8Bits                  = (1 << (WGM12+8)) | (1 << WGM10),                    //!< Timer operates as pwm generator (8 bits)
    pwm9Bits                  = (1 << (WGM12+8)) | (1 << WGM11),                    //!< Timer operates as pwm generator (9 bits)
    pwm10Bits                 = (1 << (WGM12+8)) | (1 << WGM11)     | (1 << WGM10), //!< Timer operates as pwm generator (10 bits)
    pwmDefinedTop             = (1 << (WGM13+8)) | (1 << (WGM12+8)) | (1 << WGM11), //!< Timer operates as pwm generator (with defined top value (maximum of 16 bits))
    pwmPhaseCorrect8Bits      = (1 << WGM10),                                       //!< Timer operates in pwm phase correct mode (8 bits)
    pwmPhaseCorrect9Bits      = (1 << WGM11),                                       //!< Timer operates in pwm phase correct mode (9 bits)
    pwmPhaseCorrect10Bits     = (1 << WGM11)     | (1 << WGM10),                    //!< Timer operates in pwm phase correct mode (10 bits)
    pwmPhaseCorrectDefinedTop = (1 << (WGM13+8))                                    //!< Timer operates in pwm phase correct mode (with defined top value (maximum of 16 bits))
};

enum class timer1Clock : uint16_t
{
    off                 = 0,                                                  //!< Timer has no clock (saves power)
    noPreescale         = (1 << (CS10+8)),                                    //!< CPU clock is applied directly on the timer
    divideBy8           = (1 << (CS11+8)),                                    //!< Divides CPU clock by 8
    divideBy64          = (1 << (CS11+8)) | (1 << (CS10+8)),                  //!< Divides CPU clock by 64
    divideBy256         = (1 << (CS12+8)),                                    //!< Divides CPU clock by 256
    divideBy1024        = (1 << (CS12+8)) | (1 << (CS10+8)),                  //!< Divides CPU clock by 1024
    externT1FallingEdge = (1 << (CS12+8)) | (1 << (CS11+8)),                  //!< Timer clock is driven from external source connected on pin but it is only sensible to falling edges
    externT1RisingEdge  = (1 << (CS12+8)) | (1 << (CS11+8)) | (1 << (CS10+8)) //!< Timer clock is driven from external source connected on pin but it is only sensible to rising edges
};

enum class timer1Interrupt : uint8_t
{
    none = 0,                        //!< Does not generate any interrupt
    onCompareMatchA = (1 << OCIE1A), //!< Generate an interrupt on compare match of channel A
    onCompareMatchB = (1 << OCIE1B), //!< Generate an interrupt on compare match of channel B
    onOverflow      = (1 << TOIE1)   //!< Generate an interrupt on after an overflow
};

enum class timer1InputCaptureEdge : uint16_t
{
    risingEdge  = (1 << (ICES1+8)),
    fallingEdge = 0
};

/**
 * @brief Available timer 2 operation modes
 */
enum class timer2Mode : uint16_t
{
    normal          = 0,
    ctc             = (1 << WGM21),
    pwm             = (1 << WGM21) | (1 << WGM20),
    pwmPhaseCorrect = (1 << WGM20)
};

/**
 * @brief Available timer 2 clock source configurations
 */
enum class timer2Clock : uint16_t
{
    off                = 0,
    noPreescale        = (1 << (CS20+8)),                                     //!< CPU clock is applied directly on the timer
    divideBy8          = (1 << (CS21+8)),                                     //!<
    divideBy32         = (1 << (CS21+8)) | (1 << (CS20+8)),                   //!<
    divideBy64         = (1 << (CS22+8)),                                     //!<
    divideBy128        = (1 << (CS22+8)) | (1 << (CS20+8)),                   //!<
    divideBy256        = (1 << (CS22+8)) | (1 << (CS21+8)),                   //!<
    divideBy1024       = (1 << (CS22+8)) | (1 << (CS21+8)) | (1 << (CS20+8)), //!<
    setState           = (1 << (CS22+8)) | (1 << (CS21+8)) | (1 << (CS20+8))  //!<
};

enum class timer2Interrupt : uint8_t
{
    none = 0,                        //!< Does not generate any interrupt
    onCompareMatchA = (1 << OCIE2A), //!< Generate an interrupt on compare match of channel A
    onCompareMatchB = (1 << OCIE2B), //!< Generate an interrupt on compare match of channel B
    onOverflow      = (1 << TOIE2)   //!< Generate an interrupt on after an overflow
};

/**@}*/

#endif /* AVRLIB_AVRLIB_HAL_DEVICESLL_ATMEGA328P_H_ */

