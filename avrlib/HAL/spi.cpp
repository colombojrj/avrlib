#include "spi.h"

void SPI_SET_CLK()
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

// TODO update this code to use the gpio low level library
void SPI_INIT(uint8_t mode)
{
    // Turn off power saving
    PRR = PRR & ~(1 << PRSPI);

    // Reset configuration
    SPCR = (1 << SPE) | (1 << SPR0);

    if (mode == MASTER)
    {
        SPCR = (1 << MSTR);

        // Configure SPI pins as output
        _DDR_SPI = _DDR_SPI | (1 << _PIN_MOSI) | (1 << _PIN_SCK);
        _DDR_SPI = _DDR_SPI & ~(1 << _PIN_MISO);
    }
    else
    {
        // Configure SPI pins as output
        _DDR_SPI = _DDR_SPI & ~((1 << _PIN_SS) | (1 << _PIN_MOSI) | (1 << _PIN_SCK));
        _DDR_SPI = _DDR_SPI | (1 << _PIN_MISO);
    }

    SPI_SET_CLK();

    #if SPI_USE_INTERRUPT == TRUE
        SPCR = SPCR | (1 << SPIE);
        sei();
    #endif

    #if SPI_DATA_ORDER == LSB_FIRST
        SPCR = SPCR | (1 << DORD);
    #endif
}

/*
 * This function will block the CPU
 */
void SPI_MASTER_SEND(uint8_t data)
{
    // Lock and load!
    SPDR = data;

    // Wait the transmission to be complete
    while (!(SPSR & (1 << SPIF)));
}

/*
 * This function will block the CPU
 */
uint8_t SPI_MASTER_TRANCEIVER(uint8_t data)
{
    SPI_MASTER_SEND(data);
    return SPDR;
}

void SPI_OFF()
{
    SPCR = 0;

    // Enable power saving
    PRR = PRR | (1 << PRSPI);
}

