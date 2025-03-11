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

#define QDM_MOTOR_LINE_NUMBER   1000
#define QDM_INPUT_FILTER_VALUE 0
#define QDM_INPUT_PALORITY     0x0

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

__weak void PtuCycleTrgCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_TSU_CYCLE */
    /* USER CODE END QDM_TSU_CYCLE */
}

__weak void SpeedLoseCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_SPEED_LOSE */
    /* USER CODE END QDM_SPEED_LOSE */
}

__weak void ZIndexLockedCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_INDEX_LOCKED */
    /* USER CODE END QDM_INDEX_LOCKED */
}

__weak void PositionCompareMatchCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_POS_MATCH */
    /* USER CODE END QDM_POS_MATCH */
}

__weak void PositionCompareReadyCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_POS_READY */
    /* USER CODE END QDM_POS_READY */
}

__weak void PositionCounterOverflowCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_POS_CNT_OVERFLOW */
    /* USER CODE END QDM_POS_CNT_OVERFLOW */
}

__weak void PositionCounterUnderflowCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE QDM_POS_CNT_UNDERFLOW */
    /* USER CODE QDM_POS_CNT_UNDERFLOW */
}

__weak void OrthogonalDirectionChangeCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_DIR_CHANGE */
    /* USER CODE END QDM_DIR_CHANGE */
}

__weak void OrthogonalPhaseErrorCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE QDM_PHASE_ERROR */
    /* USER CODE QDM_PHASE_ERROR */
}

__weak void PositionCounterErrorCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN QDM_POS_CNT_ERROR */
    /* USER CODE END QDM_POS_CNT_ERROR */
}

static void QDM_Init(void)
{
    HAL_CRG_IpEnableSet(QDM0_BASE, IP_CLK_ENABLE);

    g_qdmHandle.baseAddress = QDM0_BASE;

    /* emulation config */
    g_qdmHandle.emuMode = QDM_EMULATION_MODE_STOP_IMMEDIATELY;
    /* input config */
    g_qdmHandle.ctrlConfig.decoderMode = QDM_QUADRATURE_COUNT;
    g_qdmHandle.ctrlConfig.polarity = 0;
    g_qdmHandle.ctrlConfig.resolution = QDM_1X_RESOLUTION;
    g_qdmHandle.ctrlConfig.trgLockMode = QDM_TRG_BY_CYCLE;
    g_qdmHandle.ctrlConfig.swap = QDM_SWAP_DISABLE;
    g_qdmHandle.ctrlConfig.ptuMode = QDM_PTU_MODE_CYCLE;
    /* filter config */
    g_qdmHandle.inputFilter.qdmAFilterLevel = 0;
    g_qdmHandle.inputFilter.qdmBFilterLevel = 0;
    g_qdmHandle.inputFilter.qdmZFilterLevel = 0;
    /* other config */
    g_qdmHandle.lock_mode = QDM_LOCK_RESERVE;
    g_qdmHandle.pcntMode = QDM_PCNT_MODE_BY_DIR;
    g_qdmHandle.pcntRstMode = QDM_PCNT_RST_BY_PTU;
    g_qdmHandle.pcntIdxInitMode = QDM_IDX_INIT_DISABLE;
    g_qdmHandle.qcMax = 4294967295; /* 4294967295 is max count */
    g_qdmHandle.subModeEn = true;
    g_qdmHandle.tsuPrescaler = 0;
    g_qdmHandle.cevtPrescaler = QDM_CEVT_PRESCALER_DIVI1;
    g_qdmHandle.posMax = 4294967295; /* 4294967295 is max count */
    g_qdmHandle.posInit = 0;

    g_qdmHandle.period = 25000000; /* 25000000 is count period */
    g_qdmHandle.motorLineNum = 1000; /* 1000: line number */
    g_qdmHandle.interruptEn = QDM_INT_POS_CNT_ERROR |
        QDM_INT_PHASE_ERROR |
        QDM_INT_WATCHDOG |
        QDM_INT_DIR_CHANGE |
        QDM_INT_UNDERFLOW |
        QDM_INT_OVERFLOW |
        QDM_INT_POS_COMP_READY |
        QDM_INT_POS_COMP_MATCH |
        QDM_INT_INDEX_EVNT_LATCH |
        QDM_INT_UNIT_TIME_OUT;

    HAL_QDM_Init(&g_qdmHandle);
    /* Register PTU cycle Callback */
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_TSU_CYCLE, PtuCycleTrgCallback);
    /* Register Speed lose Callback */
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_SPEED_LOSE, SpeedLoseCallback);
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_INDEX_LOCKED, ZIndexLockedCallBack);
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_POS_MATCH, PositionCompareMatchCallBack);
    /* Register Positon Ready Callback */
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_POS_READY, PositionCompareReadyCallBack);
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_POS_CNT_OVERFLOW, PositionCounterOverflowCallBack);
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_POS_CNT_UNDERFLOW, PositionCounterUnderflowCallBack);
    /* Register Direction change Callback */
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_DIR_CHANGE, OrthogonalDirectionChangeCallBack);
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_PHASE_ERROR, OrthogonalPhaseErrorCallBack);
    HAL_QDM_RegisterCallback(&g_qdmHandle, QDM_POS_CNT_ERROR, PositionCounterErrorCallBack);
    IRQ_Register(IRQ_QDM0, HAL_QDM_IrqHandler, &g_qdmHandle);
    IRQ_SetPriority(IRQ_QDM0, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_QDM0); /* Enable IRQ */
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

static void IOConfig(void)
{
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

    HAL_IOCMG_SetPinAltFuncMode(IO57_AS_QDM_A);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO57_AS_QDM_A, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO57_AS_QDM_A, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO57_AS_QDM_A, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO57_AS_QDM_A, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO58_AS_QDM_B);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO58_AS_QDM_B, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO58_AS_QDM_B, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO58_AS_QDM_B, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO58_AS_QDM_B, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO1_AS_QDM_INDEX);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO1_AS_QDM_INDEX, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO1_AS_QDM_INDEX, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO1_AS_QDM_INDEX, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO1_AS_QDM_INDEX, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    QDM_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}