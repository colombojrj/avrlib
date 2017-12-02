#include "spi.h"

bool spiBusy;

void spiSetClock()
{
    #if SPI_CLK == CLK_2
        SPSR = SPSR | (1 << SPI2X);
        SPCR = SPCR & ~(1 << SPR1);
        SPCR = SPCR  & ~(1 << SPR0);
    #elif SPI_CLK == CLK_4
        SPSR = SPSR & ~(1 << SPI2X);
        SPCR = SPCR & ~(1 << SPR1);
        SPCR = SPCR & ~(1 << SPR0);
    #elif SPI_CLK == CLK_8
        SPSR = SPSR |  (1 << SPI2X);
        SPCR = SPCR & ~(1 << SPR1);
        SPCR = SPCR |  (1 << SPR0);
    #elif SPI_CLK == CLK_16
        SPSR = SPSR & ~(1 << SPI2X);
        SPCR = SPCR & ~(1 << SPR1);
        SPCR = SPCR |  (1 << SPR0);
    #elif SPI_CLK == CLK_32
        SPSR = SPSR |  (1 << SPI2X);
        SPCR = SPCR |  (1 << SPR1);
        SPCR = SPCR & ~(1 << SPR0);
    #elif SPI_CLK == CLK_64
        SPSR = SPSR & ~(1 << SPI2X);
        SPCR = SPCR |  (1 << SPR1);
        SPCR = SPCR  & ~(1 << SPR0);
    #else
        SPSR = SPSR & ~(1 << SPI2X);
        SPCR = SPCR |  (1 << SPR1)
        SPCR = SPCR | (1 << SPR0);
    #endif
}

void spiInit()
{
    // SPI is initializing, then it is busy
    spiBusy = true;

    #if SPI_MODE == MASTER || SPI_MODE == SLAVE

        // Turn off power saving
        PRR = PRR & ~(1 << PRSPI);

        // Reset configuration
        SPCR = (1 << SPE) | (1 << SPR0);

        // Pin configuration
        #if SPI_MODE == MASTER
            SPCR = (1 << MSTR);
            gpioAsOutput(&_SPI_PORT, _SPI_MOSI);
            gpioAsOutput(&_SPI_PORT, _SPI_SCK);
            gpioAsInput(&_SPI_PORT, _SPI_MISO);
        #else
            gpioAsInput(&_SPI_PORT, _SPI_SS);
            gpioAsInput(&_SPI_PORT, _SPI_SCK);
            gpioAsInput(&_SPI_PORT, _SPI_MOSI);
            gpioAsOutput(&_SPI_PORT, _SPI_MISO);
        #endif

        spiSetClock();

        #if SPI_USE_INTERRUPT == TRUE
            SPCR = SPCR | (1 << SPIE);
            sei();
        #endif

        #if SPI_DATA_ORDER == LSB_FIRST
            SPCR = SPCR | (1 << DORD);
        #endif

    #else // SPI is off
        spiOff();
    #endif

    // SPI is not busy
    spiBusy = false;
}

/*
 * This function will block the CPU
 */
void spiMasterSend(uint8_t data)
{
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
uint8_t spiMasterTransceiver(uint8_t data)
{
    spiMasterSend(data);
    return SPDR;
}

void spiOff()
{
    SPCR = 0;

    // Enable power saving
    PRR = PRR | (1 << PRSPI);
}

bool spiIsBusy()
{
    return spiBusy;
}

// TODO add support to software SPI
// TODO add support to USI as SPI

