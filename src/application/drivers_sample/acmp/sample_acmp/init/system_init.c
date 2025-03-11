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

static void ACMP1_Init(void)
{
    HAL_CRG_IpEnableSet(ACMP1_BASE, BASE_CFG_ENABLE); /* Enable ACMP clock */

    g_acmp1.baseAddress =  ACMP1_BASE; /* ACMP baseAddr */

    g_acmp1.inOutConfig.vinNNum = ACMP_VIN_MUX3;  /* input channel 3 */
    g_acmp1.inOutConfig.vinPNum = ACMP_VIN_MUX3;  /* input channel 3 */
    g_acmp1.inOutConfig.swVinPNum = ACMP_SW_VIN3;
    g_acmp1.inOutConfig.swVinNNum = ACMP_SW_VIN3;
    g_acmp1.inOutConfig.polarity = ACMP_OUT_NOT_INVERT; /* ACMP out not invert */
    g_acmp1.filterCtrl.filterMode = ACMP_FILTER_NONE;
    g_acmp1.hysteresisVol = ACMP_HYS_VOL_30MV; /* 30mv: without hysteresis */
    HAL_ACMP_Init(&g_acmp1);
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;
    HAL_IOCMG_SetPinAltFuncMode(IO2_AS_ACMP1_ANA_N2);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO2_AS_ACMP1_ANA_N2, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO2_AS_ACMP1_ANA_N2, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO2_AS_ACMP1_ANA_N2, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO2_AS_ACMP1_ANA_N2, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO5_AS_ACMP1_ANA_P2);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO5_AS_ACMP1_ANA_P2, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO5_AS_ACMP1_ANA_P2, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO5_AS_ACMP1_ANA_P2, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO5_AS_ACMP1_ANA_P2, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO10_AS_ACMP1_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO10_AS_ACMP1_OUT, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO10_AS_ACMP1_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO10_AS_ACMP1_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO10_AS_ACMP1_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    ACMP1_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}