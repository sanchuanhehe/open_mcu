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

__weak void SMBusSendStopCallback(void *handle)
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
    g_smbus.functionMode = SMBUS_MODE_SELECT_MASTER_ONLY;
    g_smbus.addrMode = SMBUS_7_BITS;
    g_smbus.sdaHoldTime = 10; /* 10: sad hold time. */
    g_smbus.freq = 400000; /* 400000: i2c speed. */
    g_smbus.transferBuff = NULL;
    g_smbus.ignoreAckFlag = BASE_CFG_DISABLE;
    g_smbus.handleEx.spikeFilterTime = 0;
    g_smbus.handleEx.sdaDelayTime = 0;
    g_smbus.timeout = 10000; /* 10000: timeout, unit: ms. */
    g_smbus.state = SMBUS_STATE_RESET;
    g_smbus.rxWaterMark = 1;   /* 1:Set the rx watermark. */
    g_smbus.txWaterMark = 12;  /* 12:Set the tx watermark. */
    HAL_SMBUS_Init(&g_smbus);
    /* Registering Callback Function */
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_MASTER_TX_COMPLETE_CB_ID, SMBusTxCallback);
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_MASTER_RX_COMPLETE_CB_ID, SMBusRxCallback);
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_MSTAER_SEND_STOP_COMPLETE_CB_ID, SMBusSendStopCallback);
    HAL_SMBUS_RegisterCallback(&g_smbus, SMBUS_ERROR_CB_ID, SMBusErrorCallback);
    IRQ_Register(IRQ_I2C0, HAL_SMBUS_IrqHandler, &g_smbus);
    IRQ_SetPriority(IRQ_I2C0, 1);
    IRQ_EnableN(IRQ_I2C0); /* Enable interrupt. */
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
    UART0_Init();
    SMBUS_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}