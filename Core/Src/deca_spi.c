#include "stm32g4xx_hal.h"
#include "main.h"
#include "deca_spi.h"
#include "deca_usleep.h"

/*
 * Bit-banged SPI for DW3000
 * Mode 0: CPOL=0, CPHA=0
 * - CLK idle low
 * - MOSI valid before rising edge
 * - sample MISO on rising edge
 */

#ifndef SOFTSPI_HALF_PERIOD_US
#define SOFTSPI_HALF_PERIOD_US   1U
#endif

static inline void spi_delay_half(void)
{
    deca_usleep(SOFTSPI_HALF_PERIOD_US);
}

static inline void clk_low(void)
{
    HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET);
}

static inline void clk_high(void)
{
    HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET);
}

static inline void mosi_write(uint8_t bitv)
{
    HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin,
                      bitv ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline uint8_t miso_read(void)
{
    return (HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

static void spi_idle(void)
{
    clk_low();
    mosi_write(0);
    cs_high();
    deca_usleep(2);
}

static uint8_t spi_xfer_byte(uint8_t tx)
{
    uint8_t rx = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        /* Present next MOSI bit */
        mosi_write((tx & 0x80U) ? 1U : 0U);
        tx <<= 1;

        spi_delay_half();
        clk_high();
        spi_delay_half();

        rx <<= 1;
        rx |= miso_read();

        clk_low();
    }

    return rx;
}

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint32_t bodyLength, const uint8_t *bodyBuffer)
{
    uint16_t i;
    uint32_t j;

    spi_idle();
    cs_low();
    deca_usleep(1);

    for (i = 0; i < headerLength; i++)
    {
        (void)spi_xfer_byte(headerBuffer[i]);
    }

    for (j = 0; j < bodyLength; j++)
    {
        (void)spi_xfer_byte(bodyBuffer[j]);
    }

    deca_usleep(1);
    cs_high();
    deca_usleep(1);

    return 0;
}

int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer,
                uint32_t bodyLength, uint8_t *bodyBuffer)
{
    uint16_t i;
    uint32_t j;

    spi_idle();
    cs_low();
    deca_usleep(1);

    for (i = 0; i < headerLength; i++)
    {
        (void)spi_xfer_byte(headerBuffer[i]);
    }

    for (j = 0; j < bodyLength; j++)
    {
        bodyBuffer[j] = spi_xfer_byte(0x00U);
    }

    deca_usleep(1);
    cs_high();
    deca_usleep(1);

    return 0;
}
