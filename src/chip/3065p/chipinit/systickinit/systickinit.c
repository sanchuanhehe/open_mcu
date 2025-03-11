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
  * @file      systickinit.c
  * @author    MCU Driver Team
  * @brief     systick init modlue.
  * @details   systick initialization function during startup
  */
#include "baseaddr.h"
#include "crg.h"
#include "timer.h"
#include "systick.h"
#include "systickinit.h"
#include "feature.h"
#ifdef NOS_TASK_SUPPORT
#include "interrupt.h"
#include "nosinit.h"
#endif

TIMER_Handle g_systickHandle;
#ifdef NOS_TASK_SUPPORT
#define NOS_TickPostDispatch OsHwiDispatchTick

static void SYSTICK_Default_Callback(void *handle)
{
    /* The default systick callback when using th nos task */
    BASE_FUNC_UNUSED(handle);
    NOS_TickPostDispatch();
}

void SYSTICK_IRQ_Enable(void)
{
    HAL_CRG_IpEnableSet(TIMER3_BASE, IP_CLK_ENABLE);
    /* Choose the config to support GetTick and Delay */
    g_systickHandle.baseAddress = TIMER3; /* use timer module */
    g_systickHandle.load = (HAL_CRG_GetIpFreq(SYSTICK_BASE) / CRG_FREQ_1MHz) * CFG_SYSTICK_TICKINTERVAL_US;
    g_systickHandle.bgLoad = (HAL_CRG_GetIpFreq(SYSTICK_BASE) / CRG_FREQ_1MHz) * CFG_SYSTICK_TICKINTERVAL_US;
    g_systickHandle.mode = TIMER_MODE_RUN_PERIODIC;
    g_systickHandle.prescaler = TIMERPRESCALER_NO_DIV;  /* frequency not divition */
    g_systickHandle.size = TIMER_SIZE_32BIT;
    g_systickHandle.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_systickHandle);
    HAL_TIMER_RegisterCallback(&g_systickHandle, TIMER_PERIOD_FIN, SYSTICK_Default_Callback);  /* nos task schedule */
    IRQ_SetPriority(IRQ_TIMER3, 1);  /* interrupt priority 1 */
    IRQ_Register(IRQ_TIMER3, HAL_TIMER_IrqHandler, &g_systickHandle);
    IRQ_EnableN(IRQ_TIMER3);
    HAL_TIMER_Start(&g_systickHandle);  /* start timer module */
}

unsigned int SYSTICK_GetTickInterval(void)
{
    /* Get the tick interval(the number of usecond per tick) */
    return CFG_SYSTICK_TICKINTERVAL_US;
}

#endif

unsigned int SYSTICK_GetTimeStampUs(void)
{
    /* Get the systick timestamp(convert from the systick value) */
    return DCL_SYSTICK_GetTick() / (SYSTICK_GetCRGHZ() / CRG_FREQ_1MHz);
}

/**
  * @brief   Init the systick
  * @param   None
  * @retval  None
  */
void SYSTICK_Init(void)
{
    SYSTICK->TIMER_CTRL.reg = 0;
    SYSTICK->TIMER_CTRL.BIT.enable = 1;
}

/**
  * @brief   Get the Systick frep(Hz)
  * @param   None
  * @retval  Clock frep of systick(Hz)
  */
unsigned int SYSTICK_GetCRGHZ(void)
{
#ifdef NOS_TASK_SUPPORT
    return HAL_CRG_GetCoreClkFreq();
#else
    /* Get the Systick IP */
    return HAL_CRG_GetIpFreq(SYSTICK_BASE);
#endif
}