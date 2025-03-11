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
#include "iocmg_ip.h"

#define UART0_BAND_RATE 115200

static void DMA_Channel0Init(void *handle)
{
    DMA_ChannelParam dma_param;  /* Setting DMA-related parameters. */
    dma_param.direction = DMA_PERIPH_TO_MEMORY_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_UNALTERED;
    dma_param.destAddrInc = DMA_ADDR_INCREASE;
    dma_param.srcPeriph = DMA_REQUEST_I2C_RX;
    dma_param.destPeriph = DMA_REQUEST_MEM;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE; /* Set the moving position width. */
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_ZERO); /* DMA Channel initialization. */
}

static void DMA_Channel1Init(void *handle)
{
    DMA_ChannelParam dma_param;  /* Setting DMA-related parameters. */
    dma_param.direction = DMA_MEMORY_TO_PERIPH_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_INCREASE;
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_MEM;
    dma_param.destPeriph = DMA_REQUEST_I2C_TX;
    dma_param.srcWidth = DMA_TRANSWIDTH_WORD; /* Set the moving position width. */
    dma_param.destWidth = DMA_TRANSWIDTH_WORD;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_ONE); /* DMA Channel initialization. */
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
    DMA_Channel0Init((void *)(&g_smbus));
    HAL_DMA_SetChannelPriorityEx(&g_dmac, DMA_CHANNEL_ZERO, DMA_PRIORITY_HIGHEST);
    DMA_Channel1Init((void *)(&g_smbus));
    HAL_DMA_SetChannelPriorityEx(&g_dmac, DMA_CHANNEL_ONE, DMA_PRIORITY_HIGHEST);
}

__weak void SMBusTxCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN I2C0TxCallback */
    /* USER CODE END I2C0TxCallback */
}

__weak void SMBusRxCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN I2C0RxCallback */
    /* USER CODE END I2C0RxCallback */
}

__weak void SMBusStopCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN I2C0RxCallback */
    /* USER CODE END I2C0RxCallback */
}

__weak void SMBusErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN I2C0ErrorCallback */
    /* USER CODE END I2C0ErrorCallback */
}

static void SMBUS_Init(void)
{
    HAL_CRG_IpEnableSet(I2C0_BASE, IP_CLK_ENABLE);
    g_smbus.baseAddress = I2C0;
    g_smbus.functionMode = SMBUS_MODE_SELECT_SLAVE_ONLY;
    g_smbus.addrMode = SMBUS_7_BITS;
    g_smbus.sdaHoldTime = 10; /* 10: sad hold time. */
    g_smbus.freq = 400000; /* 400000: i2c speed. */
    g_smbus.transferBuff = NULL;
    g_smbus.ignoreAckFlag = BASE_CFG_DISABLE;
    g_smbus.handleEx.spikeFilterTime = 0;
    g_smbus.handleEx.sdaDelayTime = 0;
    g_smbus.timeout = 10000; /* 10000: timeout, unit: ms. */
    g_smbus.slaveOwnAddress = 82;  /* 82: slave own address. */
    g_smbus.handleEx.slaveOwnXmbAddressEnable = BASE_CFG_DISABLE;
    g_smbus.handleEx.slaveOwnXmbAddress = 0;
    g_smbus.generalCallMode = BASE_CFG_DISABLE;
    g_smbus.state = SMBUS_STATE_RESET;
    g_smbus.rxWaterMark = 1;   /* 1:Set the rx watermark. */
    g_smbus.txWaterMark = 12;  /* 12:Set the tx watermark. */
    g_smbus.dmaHandle = &g_dmac;
    g_smbus.txDmaCh = DMA_CHANNEL_ONE;
    g_smbus.rxDmaCh = DMA_CHANNEL_ZERO;
    HAL_SMBUS_Init(&g_smbus);
    /* Registering Callback Function */
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_SLAVE_TX_COMPLETE_CB_ID, SMBusTxCallback);
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_SLAVE_RX_COMPLETE_CB_ID, SMBusRxCallback);
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_SLAVE_DETECTED_STOP_COMPLETE_CB_ID, SMBusStopCallback);
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_ERROR_CB_ID, SMBusErrorCallback);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    g_uart0.baseAddress = UART0;    /* UART-related parameter configuration. */
    g_uart0.baudRate = UART0_BAND_RATE;  /* Setting the UART Baud Rate. */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE; /* Enable uart fifo mode. */
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);    /* Initializing the UART0. */
}

void SystemInit(void)
{
    DMA_Init();
    UART0_Init();
    SMBUS_Init();
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}