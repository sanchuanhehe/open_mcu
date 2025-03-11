/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
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
  * @file    timer.c
  * @author  MCU Driver Team
  * @brief   Timer Init and start, get tick function
  */
#include "timer.h"
#include "stm32mp1xx_hal_tim.h"

/**
 * @brief       Timer Init
 * @retval      No
 */
void TimerInit(void)
{
    TIM_HandleTypeDef tim;                                 /* 定时器x句柄 */
    GtimTimxClkEnable();                                /* 使能TIMx时钟 */

    tim.Instance = GTIM_TIMX;                              /* 通用定时器x */
    tim.Init.Prescaler = 199;                              /* 分频199 */
    tim.Init.CounterMode = TIM_COUNTERMODE_UP;             /* 向上计数器 */
    tim.Init.Period = 0xFFFFFFFF;                          /* 自动装载值 */
    tim.Init.AutoReloadPreload = 1;                        /* 自动装载值 */
    tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;       /* 时钟分频因子 */
    HAL_TIM_Base_Init(&tim);

    HAL_TIM_Base_Start(&tim);           /* 使能定时器x和定时器x更新中断 */
}

/**
 * @brief       Get current tick
 * @retval      tick(us)
 */
unsigned int GetTimerTickUs(void)
{
    return (unsigned int)(GTIM_TIMX->CNT);
}

/**
 * @brief       Get Delta of two ticks
 * @retval      delta
 */
unsigned int GetDelta(unsigned int curTicks, unsigned int preTicks)
{
    if (curTicks >= preTicks) {
        return curTicks - preTicks;
    } else {
        return 0xFFFFFFFF - preTicks + curTicks + 1;
    }
}
