/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    oled.h
  * @author  MCU Driver Team
  * @brief   oled process Header
  */

#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "stm32mp1xx.h"

/* OLED模式设置
 * 0: 4线串行模式  （模块的BS1，BS2均接GND）
 * 1: 并行8080模式 （模块的BS1，BS2均接VCC）
 */
#define OLED_MODE       0   /* 默认使用8080并口模式 */

/********************************OLED SPI模式引脚 定义************************************************/
/* 注意:这里仅定义了 OLED 4线SPI模式驱动时的 引脚定义. 8位并口访问, 由于引脚太多,就不单独定义了. */

#define OLED_SPI_RST_PORT               GPIOE
#define OLED_SPI_RST_PIN                GPIO_PIN_3
static inline void OledSpiRstClkEnable(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

#define OLED_SPI_CS_PORT                GPIOE
#define OLED_SPI_CS_PIN                 GPIO_PIN_4
static inline void OledSpiCsClkEnable(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

#define OLED_SPI_RS_PORT                GPIOE
#define OLED_SPI_RS_PIN                 GPIO_PIN_5
static inline void OledSpiRsClkEnable(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

#define OLED_SPI_SCLK_PORT              GPIOE
#define OLED_SPI_SCLK_PIN               GPIO_PIN_2
static inline void OledSpiSclkClkEnable(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

#define OLED_SPI_SDIN_PORT              GPIOE
#define OLED_SPI_SDIN_PIN               GPIO_PIN_6
static inline void OledSpiSdinClkEnable(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

/********************************IC库引脚定义************************************************/
#define  ROM_IN_PORT                  GPIOB
#define  ROM_IN_PIN                   GPIO_PIN_15  /* ROM_IN */

#define ROM_SCK_PORT                  GPIOB
#define ROM_SCK_PIN                   GPIO_PIN_13

#define ROM_OUT_PORT                  GPIOB
#define ROM_OUT_PIN                   GPIO_PIN_14  /* ROM_OUT */

#define ROM_CS_PORT                   GPIOB
#define ROM_CS_PIN                    GPIO_PIN_12

/*****************************OLED SPI和8080并口模式相关端口控制函数 定义***************************************/

/*
 * 注意:OLED_RST/OLED_CS/OLED_RS,这三个是和80并口模式共用的,即80模式也必须实现这3个函数!
 */
static inline void OledRstSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(OLED_SPI_RST_PORT, OLED_SPI_RST_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(OLED_SPI_RST_PORT, OLED_SPI_RST_PIN, GPIO_PIN_SET);
    }
}

static inline void OledCsSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(OLED_SPI_CS_PORT, OLED_SPI_CS_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(OLED_SPI_CS_PORT, OLED_SPI_CS_PIN, GPIO_PIN_SET);
    }
}

static inline void OledRsSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(OLED_SPI_RS_PORT, OLED_SPI_RS_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(OLED_SPI_RS_PORT, OLED_SPI_RS_PIN, GPIO_PIN_SET);
    }
}

static inline void OledSclkSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(OLED_SPI_SCLK_PORT, OLED_SPI_SCLK_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(OLED_SPI_SCLK_PORT, OLED_SPI_SCLK_PIN, GPIO_PIN_SET);
    }
}

static inline void OledSdinSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(OLED_SPI_SDIN_PORT, OLED_SPI_SDIN_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(OLED_SPI_SDIN_PORT, OLED_SPI_SDIN_PIN, GPIO_PIN_SET);
    }
}

/**********************字库IC引脚定义*********************************************************/
static inline void OledRomInSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(ROM_IN_PORT, ROM_IN_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(ROM_IN_PORT, ROM_IN_PIN, GPIO_PIN_SET);
    }
}

static inline void OledRomSckSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(ROM_SCK_PORT, ROM_SCK_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(ROM_SCK_PORT, ROM_SCK_PIN, GPIO_PIN_SET);
    }
}

static inline void OledRomOutSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(ROM_OUT_PORT, ROM_OUT_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(ROM_OUT_PORT, ROM_OUT_PIN, GPIO_PIN_SET);
    }
}

static inline GPIO_PinState OledReadRomOut(void)
{
    return HAL_GPIO_ReadPin(ROM_OUT_PORT, ROM_OUT_PIN);
}

static inline void OledRomCsSet(uint8_t val)
{
    if (val == 0) {
        HAL_GPIO_WritePin(ROM_CS_PORT, ROM_CS_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(ROM_CS_PORT, ROM_CS_PIN, GPIO_PIN_SET);
    }
}

/*********************************命令/数据 定义*************************************************/

#define OLED_CMD        0       /* 写命令 */
#define OLED_DATA       1       /* 写数据 */

typedef enum {
    DISPLAY_NORMAL,
    DISPLAY_REVERSE,
} DisplayMode;

/*********************************函数声明*************************************************/

void OledInit(void);           /* OLED初始化 */
void OledClear(void);          /* OLED清屏 */
void OledClearFromStartYPixel(uint32_t startYPixel);
uint8_t OledShowString(uint8_t startColumn, uint8_t startPage, const char *text, uint8_t size, uint8_t type);

uint8_t OledShowMenuItem(uint8_t page, const char *context, uint8_t size, uint8_t reversed);
void OledShowMsg(uint8_t page, const char *context, uint8_t size);
void OledClearLine(uint8_t page, uint8_t begin, uint8_t end);

void SetGlobalDisaplyaMode(DisplayMode mode);
#endif
