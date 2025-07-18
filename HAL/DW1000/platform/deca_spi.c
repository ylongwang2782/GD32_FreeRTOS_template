/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "deca_spi.h"

#include "deca_device_api.h"
#include "gd32f4xx.h"


extern spi_parameter_struct spi_init_struct;
#define DW1000_SPI_Handle SPI1

// 初始化在main中执行，此处留空
int openspi(/*SPI_TypeDef* SPIx*/) { return 0; }

int closespi(void) { return 0; }

#pragma GCC optimize("O3")
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint32_t bodyLength, const uint8_t *bodyBuffer) {
    decaIrqStatus_t stat;
    stat = decamutexon();

    while (spi_i2s_flag_get(SPI3, SPI_FLAG_TRANS) == SET);
    gpio_bit_reset(GPIOE, GPIO_PIN_4);
    for (volatile int delay = 0; delay < 100; delay++);    // 粗略延迟几个微秒

    for (uint16_t i = 0; i < headerLength; i++) {
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_TBE) == RESET);
        spi_i2s_data_transmit(SPI3, headerBuffer[i]);
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_RBNE) == RESET);
        (void)spi_i2s_data_receive(SPI3);
    }
    for (uint32_t i = 0; i < bodyLength; i++) {
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_TBE) == RESET);
        spi_i2s_data_transmit(SPI3, bodyBuffer[i]);
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_RBNE) == RESET);
        (void)spi_i2s_data_receive(SPI3);
    }
    gpio_bit_set(GPIOE, GPIO_PIN_4);
    decamutexoff(stat);
    return 0;
}

#pragma GCC optimize("O3")
int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer,
                uint32_t readlength, uint8_t *readBuffer) {
    decaIrqStatus_t stat;
    stat = decamutexon();

    while (spi_i2s_flag_get(SPI3, SPI_FLAG_TRANS) == SET);
    gpio_bit_reset(GPIOE, GPIO_PIN_4);
    for (volatile int delay = 0; delay < 100; delay++);    // 粗略延迟几个微秒

    for (uint16_t i = 0; i < headerLength; i++) {
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_TBE) == RESET);
        spi_i2s_data_transmit(SPI3, headerBuffer[i]);
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_RBNE) == RESET);
        spi_i2s_data_receive(SPI3);
    }
    while (readlength-- > 0) {
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_TBE) == RESET);
        spi_i2s_data_transmit(SPI3, 0x00);
        while (spi_i2s_flag_get(SPI3, SPI_FLAG_RBNE) == RESET);
        *readBuffer++ = spi_i2s_data_receive(SPI3);
    }
    gpio_bit_set(GPIOE, GPIO_PIN_4);
    decamutexoff(stat);
    return 0;
}
