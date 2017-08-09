/*
 * spi.cpp
 *
 *  Created on: 16/07/2017
 *      Author: junior
 */

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

void SPI_INIT()
{
    // Turn off power saving
    PRR = PRR & ~(1 << PRSPI);

    // Configure SPI pins as output
    #if defined (__AVR_ATmega328P__)
        DDRB = DDRB | (1 << PB3) | (1 << PB4) | (1 << PB5);
    #endif

    // Reset configuration
    SPCR = 0;

    #if SPI_MODE == MASTER
        SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    #else
        SPCR = (1 << SPE) | (1 << SPR0);
    #endif

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

