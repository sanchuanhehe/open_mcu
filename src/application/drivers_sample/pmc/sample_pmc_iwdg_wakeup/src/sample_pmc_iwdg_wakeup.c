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
  * @file    sample_pmc_iwdg_wakeup.c
  * @author  MCU Driver Team
  * @brief   APT module sample of HAL API.
  *          This file provides some configuration example of PMC module's IWDG WAKEUP.
  */
#include "debug.h"
#include "sample_pmc_iwdg_wakeup.h"

#define DELAY_TIME_FOR_PRINTF 500

/**
  * @brief PVD sample.
  * @retval None.
  */
void PmcIwdgSample(void)
{
    SystemInit();
    DBG_PRINTF("PMC Module example iwdg wakeup!\r\n");
    BASE_FUNC_DELAY_MS(DELAY_TIME_FOR_PRINTF);

    DBG_PRINTF("statrt iwdg and wait wakeup\r\n");
    HAL_IWDG_Start(&g_iwdg); /* iwdg start */

    DBG_PRINTF("Enter deepsleep mode!\r\n");
    BASE_FUNC_DELAY_MS(DELAY_TIME_FOR_PRINTF);
    HAL_PMC_EnterDeepSleepMode(&g_pmc);
    DBG_PRINTF("Error printf this sentence!\r\n");
}