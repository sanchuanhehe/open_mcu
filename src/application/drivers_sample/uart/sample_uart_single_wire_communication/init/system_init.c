/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
#define UART1_BAND_RATE 115200
#define UART2_BAND_RATE 115200

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

__weak void TIMER2CallbackFunction(void *handle)
{
    DCL_TIMER_IrqClear((TIMER_RegStruct *)handle);
    /* USER CODE BEGIN TIMER2 ITCallBackFunc */
    /* USER CODE END TIMER2 ITCallBackFunc */
}

static void TIMER2_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER2) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 500;

    HAL_CRG_IpEnableSet(TIMER2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER2_BASE, CRG_PLL_NO_PREDV);

    g_timer2.baseAddress = TIMER2;

    g_timer2.load        = load - 1; /* Set timer value immediately */
    g_timer2.bgLoad      = load - 1; /* Set timer value */
    g_timer2.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer2.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer2.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer2.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer2.dmaReqEnable = BASE_CFG_DISABLE;
    g_timer2.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer2);
    IRQ_Register(IRQ_TIMER2, HAL_TIMER_IrqHandler, &g_timer2);
    HAL_TIMER_RegisterCallback(&g_timer2, TIMER_PERIOD_FIN, TIMER2CallbackFunction);
    IRQ_SetPriority(IRQ_TIMER2, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER2);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

__weak void UART1WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_WRITE_IT_FINISH */
    /* USER CODE END UART1_WRITE_IT_FINISH */
}

__weak void UART1ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_READ_IT_FINISH */
    /* USER CODE END UART1_READ_IT_FINISH */
}

__weak void UART1InterruptErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART1_TRNS_IT_ERROR */
    /* USER CODE END UART1_TRNS_IT_ERROR */
}

static void UART1_Init(void)
{
    HAL_CRG_IpEnableSet(UART1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART1_BASE, CRG_PLL_NO_PREDV);
    /* Config uart param */
    g_uart1.baseAddress = UART1;

    g_uart1.baudRate = UART1_BAND_RATE; /* Set baud rate is 115200 */
    g_uart1.dataLength = UART_DATALENGTH_8BIT;
    g_uart1.stopBits = UART_STOPBITS_ONE;
    g_uart1.parity = UART_PARITY_NONE;
    /* Set tx mode */
    g_uart1.txMode = UART_MODE_INTERRUPT;
    /* Set rx mode */
    g_uart1.rxMode = UART_MODE_INTERRUPT;
    g_uart1.fifoMode = BASE_CFG_DISABLE;
    g_uart1.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart1.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart1.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart1);
    /* Config tx mode interupt */
    HAL_UART_RegisterCallBack(&g_uart1, UART_WRITE_IT_FINISH, UART1WriteInterruptCallback);
    /* Config rx mode interupt */
    HAL_UART_RegisterCallBack(&g_uart1, UART_READ_IT_FINISH, UART1ReadInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart1, UART_TRNS_IT_ERROR, UART1InterruptErrorCallback);
    IRQ_Register(IRQ_UART1, HAL_UART_IrqHandler, &g_uart1);
    IRQ_SetPriority(IRQ_UART1, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_UART1);
}

__weak void UART2WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_WRITE_IT_FINISH */
    /* USER CODE END UART2_WRITE_IT_FINISH */
}

__weak void UART2ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_READ_IT_FINISH */
    /* USER CODE END UART2_READ_IT_FINISH */
}

__weak void UART2InterruptErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART2_TRNS_IT_ERROR */
    /* USER CODE END UART2_TRNS_IT_ERROR */
}

static void UART2_Init(void)
{
    HAL_CRG_IpEnableSet(UART2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART2_BASE, CRG_PLL_NO_PREDV);
    /* Config uart param */
    g_uart2.baseAddress = UART2;

    g_uart2.baudRate = UART2_BAND_RATE; /* Set baud rate is 115200 */
    g_uart2.dataLength = UART_DATALENGTH_8BIT;
    g_uart2.stopBits = UART_STOPBITS_ONE;
    g_uart2.parity = UART_PARITY_NONE;
    /* Set tx mode */
    g_uart2.txMode = UART_MODE_INTERRUPT;
    /* Set rx mode */
    g_uart2.rxMode = UART_MODE_INTERRUPT;
    g_uart2.fifoMode = BASE_CFG_DISABLE;
    g_uart2.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart2.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart2.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart2);
    /* Config tx mode interupt */
    HAL_UART_RegisterCallBack(&g_uart2, UART_WRITE_IT_FINISH, UART2WriteInterruptCallback);
    /* Config rx mode interupt */
    HAL_UART_RegisterCallBack(&g_uart2, UART_READ_IT_FINISH, UART2ReadInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart2, UART_TRNS_IT_ERROR, UART2InterruptErrorCallback);
    IRQ_Register(IRQ_UART2, HAL_UART_IrqHandler, &g_uart2);
    IRQ_SetPriority(IRQ_UART2, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_UART2);
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;
    HAL_IOCMG_SetPinAltFuncMode(IO52_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO52_AS_UART0_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO52_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO52_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO52_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

 /* UART RX recommend PULL_UP */
    HAL_IOCMG_SetPinAltFuncMode(IO53_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO53_AS_UART0_RXD, PULL_UP);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO53_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO53_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO53_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO36_AS_UART2_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO36_AS_UART2_TXD, PULL_UP);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO36_AS_UART2_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO36_AS_UART2_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO36_AS_UART2_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO37_AS_UART2_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO37_AS_UART2_RXD, PULL_UP);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO37_AS_UART2_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO37_AS_UART2_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO37_AS_UART2_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO61_AS_UART1_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO61_AS_UART1_TXD, PULL_UP);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO61_AS_UART1_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO61_AS_UART1_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO61_AS_UART1_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO60_AS_UART1_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO60_AS_UART1_RXD, PULL_UP);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO60_AS_UART1_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO60_AS_UART1_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO60_AS_UART1_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    UART1_Init();
    UART2_Init();
    TIMER2_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}