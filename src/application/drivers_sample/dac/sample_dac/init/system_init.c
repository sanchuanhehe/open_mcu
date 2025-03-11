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

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG; /* CRG base address */
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {  /* Init CRG */
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void DAC0_Init(void)
{
    HAL_CRG_IpEnableSet(DAC0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkDivSet(DAC0_BASE, CRG_DAC_DIV_1);

    g_dac.baseAddress = DAC0_BASE;  /* DAC0 base address */
    g_dac.dacValue = 120; /* 120: DAC Value */
    g_dac.handleEx.sineMode = BASE_CFG_UNSET;
    HAL_DAC_Init(&g_dac);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(IO16_AS_DAC0_ANA_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO16_AS_DAC0_ANA_OUT, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO16_AS_DAC0_ANA_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO16_AS_DAC0_ANA_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO16_AS_DAC0_ANA_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    DAC0_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}