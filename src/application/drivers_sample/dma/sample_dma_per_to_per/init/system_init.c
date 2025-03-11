/**
  * @copyright Copyright (c) 2024, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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

static void DMA_Channel0Init(void)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_PERIPH_TO_PERIPH_BY_DMAC; /* Dma direction is memory to peripherals. */
    dma_param.srcAddrInc = DMA_ADDR_UNALTERED;   /* Destination is peripherals, address does not increase. */
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;  /* Destination is peripherals, address does not increase. */
    dma_param.srcPeriph = DMA_REQUEST_UART1_RX;
    dma_param.destPeriph = DMA_REQUEST_UART0_TX;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = &g_dmac;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_ZERO); /* Dma parameter init. */
}

static void DMA_Init(void)
{
    HAL_CRG_IpEnableSet(DMA_BASE, IP_CLK_ENABLE);
    g_dmac.baseAddress = DMA;
    IRQ_Register(IRQ_DMA_TC, HAL_DMA_IrqHandlerTc, &g_dmac);
    IRQ_Register(IRQ_DMA_ERR, HAL_DMA_IrqHandlerError, &g_dmac);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
    HAL_DMA_Init(&g_dmac);
    DMA_Channel0Init();
    HAL_DMA_SetChannelPriorityEx(&g_dmac, DMA_CHANNEL_ZERO, DMA_PRIORITY_HIGHEST);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);  /* UART0 clock enable. */
    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_DMA;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE7;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE7;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

__weak void UART1_RXDMACallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_READ_DMA_FINISH */
    /* USER CODE END UART1_READ_DMA_FINISH */
}

__weak void UART1ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_READ_IT_FINISH */
    /* USER CODE END UART1_READ_IT_FINISH */
}

static void UART1_Init(void)
{
    HAL_CRG_IpEnableSet(UART1_BASE, IP_CLK_ENABLE);  /* UART1 clock enable. */
    g_uart1.baseAddress = UART1;

    g_uart1.baudRate = UART1_BAND_RATE;
    g_uart1.dataLength = UART_DATALENGTH_8BIT;
    g_uart1.stopBits = UART_STOPBITS_ONE;
    g_uart1.parity = UART_PARITY_NONE;
    g_uart1.txMode = UART_MODE_BLOCKING;
    g_uart1.rxMode = UART_MODE_DMA;     /* UART rx use dma mode. */
    g_uart1.fifoMode = BASE_CFG_ENABLE;
    g_uart1.fifoTxThr = UART_FIFODEPTH_SIZE7;
    g_uart1.fifoRxThr = UART_FIFODEPTH_SIZE7;
    g_uart1.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart1.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart1.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart1); /* UART parameter init. */
    g_uart1.dmaHandle = &g_dmac;
    g_uart1.uartDmaRxChn = 0; /* 0 is UART_DMA rx channel */
    HAL_UART_RegisterCallBack(&g_uart1, UART_READ_DMA_FINISH, (UART_CallbackType)DMA_PeriToPeriFinish);
    HAL_UART_RegisterCallBack(&g_uart1, UART_READ_IT_FINISH, (UART_CallbackType)UART1ReadInterruptCallback);
}

static void IOConfig(void)
{
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
    /* Config PIN22 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_4_AS_UART1_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_4_AS_UART1_TXD, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_4_AS_UART1_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_4_AS_UART1_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_4_AS_UART1_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN21 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_3_AS_UART1_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_3_AS_UART1_RXD, PULL_UP);  /* Pull-up and Pull-down, UART RX recommend PULL_UP */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_3_AS_UART1_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_3_AS_UART1_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_3_AS_UART1_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    DMA_Init();
    UART0_Init();
    UART1_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}