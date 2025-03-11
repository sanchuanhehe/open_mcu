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

static void CAPM0_Init(void)
{
    HAL_CRG_IpEnableSet(CAPM0_BASE, IP_CLK_ENABLE);

    g_capmAConfig.baseAddress = CAPM0;

    g_capmAConfig.deburrNum = 0; /* deburr value */
    g_capmAConfig.capMode = CAPM_CONTINUECAP; /* capture mode */
    g_capmAConfig.preScale = 0;
    g_capmAConfig.capRegConfig[CAPM_ECR_NUM1].capEvent = CAPM_RISING; /* capture rising edge */
    g_capmAConfig.capRegConfig[CAPM_ECR_NUM1].regReset = CAPM_NOTRESET;
    g_capmAConfig.capRegConfig[CAPM_ECR_NUM2].capEvent = CAPM_FALLING; /* capture falling edge */
    g_capmAConfig.capRegConfig[CAPM_ECR_NUM2].regReset = CAPM_NOTRESET;
    g_capmAConfig.useCapNum = 2; /* 2: use 2 ECR */
    g_capmAConfig.tscntDiv = 1 - 1;
    g_capmAConfig.inputSrc = CAPM_INPUT_SRC1; /* input source 1 */
    HAL_CAPM_Init(&g_capmAConfig);
}

static void CAPM1_Init(void)
{
    HAL_CRG_IpEnableSet(CAPM1_BASE, IP_CLK_ENABLE);

    g_capmBConfig.baseAddress = CAPM1;

    g_capmBConfig.deburrNum = 0; /* deburr value */
    g_capmBConfig.capMode = CAPM_CONTINUECAP; /* capture mode */
    g_capmBConfig.preScale = 0;
    g_capmBConfig.capRegConfig[CAPM_ECR_NUM1].capEvent = CAPM_RISING; /* capture rising edge */
    g_capmBConfig.capRegConfig[CAPM_ECR_NUM1].regReset = CAPM_NOTRESET;
    g_capmBConfig.capRegConfig[CAPM_ECR_NUM2].capEvent = CAPM_FALLING; /* capture falling edge */
    g_capmBConfig.capRegConfig[CAPM_ECR_NUM2].regReset = CAPM_NOTRESET;
    g_capmBConfig.useCapNum = 2; /* 2: use 2 ECR */
    g_capmBConfig.tscntDiv = 1 - 1;
    g_capmBConfig.inputSrc = CAPM_INPUT_SRC0; /* input source 1 */
    HAL_CAPM_Init(&g_capmBConfig);
}

static void CAPM2_Init(void)
{
    HAL_CRG_IpEnableSet(CAPM2_BASE, IP_CLK_ENABLE);

    g_capmCConfig.baseAddress = CAPM2;

    g_capmCConfig.deburrNum = 0; /* deburr value */
    g_capmCConfig.capMode = CAPM_CONTINUECAP; /* capture mode */
    g_capmCConfig.preScale = 0;
    g_capmCConfig.capRegConfig[CAPM_ECR_NUM1].capEvent = CAPM_RISING; /* capture rising edge */
    g_capmCConfig.capRegConfig[CAPM_ECR_NUM1].regReset = CAPM_NOTRESET;
    g_capmCConfig.capRegConfig[CAPM_ECR_NUM2].capEvent = CAPM_FALLING; /* capture falling edge */
    g_capmCConfig.capRegConfig[CAPM_ECR_NUM2].regReset = CAPM_NOTRESET;
    g_capmCConfig.useCapNum = 2; /* 2: use 2 ECR */
    g_capmCConfig.tscntDiv = 1 - 1;
    g_capmCConfig.inputSrc = CAPM_INPUT_SRC1; /* input source 1 */
    HAL_CAPM_Init(&g_capmCConfig);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE); /* enable uart's clock */
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0.baseAddress = UART0; /* uart0  base address */

    g_uart0.baudRate = UART0_BAND_RATE; /* uart bandrate */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING; /* TX mode */
    g_uart0.rxMode = UART_MODE_BLOCKING; /* RX mode */
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(IO4_AS_CAPM0_SRC1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO4_AS_CAPM0_SRC1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO4_AS_CAPM0_SRC1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO4_AS_CAPM0_SRC1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO4_AS_CAPM0_SRC1, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO54_AS_CAPM1_SRC0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO54_AS_CAPM1_SRC0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO54_AS_CAPM1_SRC0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO54_AS_CAPM1_SRC0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO54_AS_CAPM1_SRC0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO3_AS_CAPM2_SRC1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO3_AS_CAPM2_SRC1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO3_AS_CAPM2_SRC1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO3_AS_CAPM2_SRC1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO3_AS_CAPM2_SRC1, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO52_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO52_AS_UART0_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO52_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO52_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO52_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    /* UART RX recommend PULL_UP */
    HAL_IOCMG_SetPinAltFuncMode(IO53_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO53_AS_UART0_RXD, PULL_UP);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO53_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO53_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW); /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO53_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    CAPM0_Init();
    CAPM1_Init();
    CAPM2_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}