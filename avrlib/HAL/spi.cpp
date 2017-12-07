#include "spi.h"

bool spiBusy;

void spiSetClock()
{
    // Default values
    const spiClockResetState resetClockRegisters;

    // Configure clock
    if (spiClock == spiClock_t::divideBy2)
        const spiClockDivideBy2 helper;

    else if (spiClock == spiClock_t::divideBy4)
        const spiClockDivideBy4 helper;

    else if (spiClock == spiClock_t::divideBy8)
        const spiClockDivideBy8 helper;

    else if (spiClock == spiClock_t::divideBy16)
        const spiClockDivideBy16 helper;

    else if (spiClock == spiClock_t::divideBy32)
        const spiClockDivideBy32 helper;

    else if (spiClock == spiClock_t::divideBy64)
        const spiClockDivideBy64 helper;

    else // i.e. spiClock == spiClock_t::divideBy128
        spiClockDivideBy128 helper;
}

void spiInit()
{
    // SPI is initializing, then it is busy
    spiBusy = true;

    if (spiConfig != spiConfig_t::off)
    {
        // Make sure to reset SPI registers
        SPCR = 0;

        // Turn off power saving
        #if defined (PRR)
            PRR = PRR & ~(1 << PRSPI);
        #endif

        // Firstly it is necessary to configure the SS pin as output
        // As the datasheet specifies, if this pin is an input, then
        // any change in its logic state will cause the SPI behaviour
        // change automatically from master to slave
        if (spiConfig == spiConfig_t::master)
        {
            gpioAsOutput(&_SPI_PORT, _SPI_SS);
            SPCR = (1 << SPE) | (1 << MSTR);
        }
        else
        {
            gpioAsInput(&_SPI_PORT, _SPI_SS, 1);
            SPCR = (1 << SPE);
        }

        // SPI mode
        SPCR = SPCR | static_cast<uint8_t>(spiMode);

        // SPI clock
        spiSetClock();

        if (spiUseInterrupt == spiUseInterrupt_t::yes)
        {
            SPCR = SPCR | static_cast<uint8_t>(spiUseInterrupt);
            sei();
        }

        if (spiDataOrder == spiDataOrder_t::lsbFirst)
            SPCR = SPCR | static_cast<uint8_t>(spiDataOrder);

        // Configure the MOSI, MISO and SCK SPI pins
        if (spiConfig == spiConfig_t::master)
        {
            gpioAsOutput(&_SPI_PORT, _SPI_MOSI);
            gpioAsOutput(&_SPI_PORT, _SPI_SCK);
            gpioAsInput(&_SPI_PORT, _SPI_MISO);
        }
        else
        {
            gpioAsInput(&_SPI_PORT, _SPI_SCK);
            gpioAsInput(&_SPI_PORT, _SPI_MOSI);
            gpioAsOutput(&_SPI_PORT, _SPI_MISO);
        }

        // SPI is not busy
        spiBusy = false;
    }
    else
    {
        spiOff();
    }
}

/*
 * This function will block the CPU
 */
void spiSend(uint8_t data)
{
    // Wait to spi to be free
    while(spiBusy == true) {}

    // SPI is getting busy
    spiBusy = true;

    // Lock and load!
    SPDR = data;

    // Wait the transmission to be complete
    while (!(SPSR & (1 << SPIF)));

    // Release SPI
    spiBusy = false;
}

/*
 * This function will block the CPU
 */
uint8_t spiTransceiver(uint8_t data)
{
    spiSend(data);
    return SPDR;
}

void spiOff()
{
    SPCR = 0;

    // Enable power saving
    #if defined(PRR)
        PRR = PRR | (1 << PRSPI);
    #endif
}

bool spiIsBusy()
{
    return spiBusy;
}

// TODO add support to software SPI
// TODO add support to USI as SPI

