/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2024. All rights reserved.
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
  * @file      system_init.c
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

#include "main.h"
#include "ioconfig.h"
#include "iocmg.h"

#define UART0_BAND_RATE 115200
#define CLK1M_Div 25

#define SPI0_FREQ_SCR 2
#define SPI0_FREQ_CPSDVSR 50

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_3;
    crg.pllFbDiv        = 0x30; /* PLL Multiplier 48 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_4;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    /* The 1 MHz freq is equal to the input clock frequency / (clk_1m_div + 1). */
    crg.handleEx.clk1MDiv = (CLK1M_Div - 1);

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void SPI0_Init(void)
{
    HAL_CRG_IpEnableSet(SPI0_BASE, IP_CLK_ENABLE); /* Enable SPI clock. */

    g_spiSampleHandle.baseAddress = SPI0;
    g_spiSampleHandle.mode = HAL_SPI_MASTER;
    g_spiSampleHandle.csMode = SPI_CHIP_SELECT_MODE_INTERNAL;
    g_spiSampleHandle.xFerMode = HAL_XFER_MODE_BLOCKING;
    g_spiSampleHandle.clkPolarity = HAL_SPI_CLKPOL_0; /* Set Polarity and Phase. */
    g_spiSampleHandle.clkPhase =  HAL_SPI_CLKPHA_0;
    g_spiSampleHandle.endian = HAL_SPI_BIG_ENDIAN;
    g_spiSampleHandle.frameFormat = HAL_SPI_MODE_MOTOROLA; /* Set motorola mode. */
    g_spiSampleHandle.dataWidth = SPI_DATA_WIDTH_16BIT;
    g_spiSampleHandle.freqScr = SPI0_FREQ_SCR;
    g_spiSampleHandle.freqCpsdvsr = SPI0_FREQ_CPSDVSR;
    g_spiSampleHandle.waitEn = BASE_CFG_DISABLE;
    g_spiSampleHandle.waitVal = 127; /* 127: Waiting time between receiving and sending. */
    g_spiSampleHandle.rxBuff = NULL;
    g_spiSampleHandle.txBuff = NULL;
    g_spiSampleHandle.transferSize = 0;
    g_spiSampleHandle.txCount = 0;
    g_spiSampleHandle.rxCount = 0; /* The count of transferred data. */
    g_spiSampleHandle.state = HAL_SPI_STATE_RESET;
    g_spiSampleHandle.rxIntSize =  SPI_RX_INTERRUPT_SIZE_1;
    g_spiSampleHandle.txIntSize =  SPI_TX_INTERRUPT_SIZE_1;
    g_spiSampleHandle.rxDMABurstSize =  SPI_RX_DMA_BURST_SIZE_1; /* DMA brust bit width */
    g_spiSampleHandle.txDMABurstSize =  SPI_TX_DMA_BURST_SIZE_1;
    g_spiSampleHandle.handleEx.line = SPI_DATA_1LINE;
    HAL_SPI_Init(&g_spiSampleHandle); /* Init SPI */
    HAL_SPI_ChipSelectChannelSet(&g_spiSampleHandle, SPI_CHIP_SELECT_CHANNEL_0);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);

    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE7;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE7;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;
    /* Config PIN6 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_3_AS_SPI0_CLK);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_3_AS_SPI0_CLK, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_3_AS_SPI0_CLK, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_3_AS_SPI0_CLK, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_3_AS_SPI0_CLK, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN3 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_0_AS_SPI0_MOSI);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_0_AS_SPI0_MOSI, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_0_AS_SPI0_MOSI, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_0_AS_SPI0_MOSI, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_0_AS_SPI0_MOSI, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN4 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_1_AS_SPI0_MISO);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_1_AS_SPI0_MISO, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_1_AS_SPI0_MISO, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_1_AS_SPI0_MISO, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_1_AS_SPI0_MISO, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN2 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_7_AS_SPI0_CSN0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_7_AS_SPI0_CSN0, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_7_AS_SPI0_CSN0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_7_AS_SPI0_CSN0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_7_AS_SPI0_CSN0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN5 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_2_AS_SPI0_CSN1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_2_AS_SPI0_CSN1, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_2_AS_SPI0_CSN1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_2_AS_SPI0_CSN1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_2_AS_SPI0_CSN1, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN52 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_3_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_3_AS_UART0_TXD, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_3_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_3_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_3_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN53 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_4_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_4_AS_UART0_RXD, PULL_UP);  /* Pull-up and Pull-down, UART RX recommend PULL_UP */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_4_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_4_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_4_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    SPI0_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}