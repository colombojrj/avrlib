#include "spi.h"

#if defined(SUPPORT_TO_SPI)

bool spiBusy;

void spiSetClock(spiClock_t spiClock)
{
    // Clear SPI clock registers configuration
    *spi.spiControl = *spi.spiControl & ~static_cast<uint16_t>(spiClock_t::setState);

    // Configure SPI clock
    *spi.spiControl = *spi.spiControl | static_cast<uint16_t>(spiClock);
}

void spiInit(spiClock_t spiClock)
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
            gpioAsOutput(&_SPI_SS_PIN);
            SPCR = (1 << SPE) | (1 << MSTR);
        }
        else
        {
            gpioAsInput(&_SPI_SS_PIN);
            gpioPullUpEnable(& _SPI_SS_PIN);
            SPCR = (1 << SPE);
        }

        // SPI mode
        SPCR = SPCR | static_cast<uint8_t>(spiMode);

        // SPI clock
        spiSetClock(spiClock);

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
            gpioAsOutput(&_SPI_MOSI_PIN);
            gpioAsOutput(&_SPI_SCK_PIN);
            gpioAsInput(&_SPI_MISO_PIN);
        }
        else
        {
            gpioAsInput(&_SPI_SCK_PIN);
            gpioAsInput(&_SPI_MOSI_PIN);
            gpioAsOutput(&_SPI_MISO_PIN);
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

#endif // SUPPORT_TO_SPI

/// @todo add support to software SPI
/// @todo add support to USI as SPI (ATtiny devices)

