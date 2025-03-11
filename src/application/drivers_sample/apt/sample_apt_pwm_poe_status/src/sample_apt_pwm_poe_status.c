/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2024. All rights reserved.
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
 * @file    sample_apt_pwm_poe_status.c
 * @author  MCU Driver Team
 * @brief   APT sample module driver
 * @details This file provides APT sample of pwm and poe status.
 */

#include "sample_apt_pwm_poe_status.h"
#include "apt.h"
#include "debug.h"
#include "crg.h"
#include "main.h"


typedef enum {
    APT_CHA_PWM_CHB_PWM,
    APT_CHA_PWM_CHB_LOW,
    APT_CHA_LOW_CHB_HIGH,
    APT_CHA_LOW_CHB_LOW,
    APT_CHA_HIGH_CHB_LOW,
} AptAct;


static void APT_Config(APT_RegStruct* aptAddr, AptAct aptAct);
static bool CheckStatusPwmChAHighChBLow(APT_Handle *aptHandle);
static bool CheckStatusPwmChALowChBHigh(APT_Handle *aptHandle);
static bool POE_CheckStatus(APT_Handle *aptHandle);

/**
  * @brief Event interrupt callback function of U phase APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void APT0EventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* POE is manually triggered High, the output will meet the expectation. */
    bool g_testRes3 = POE_CheckStatus(&g_apt0);
    if (g_testRes3) {
        APT_PwmStatus aptChAStatus;
        APT_PwmStatus aptChBStatus;
        APT_PoeStatus POEStatus;
        /* Read pwm status and poe status */
        /* Desired output of CHA: APT_PWM_LOW_LEVEL */
        aptChAStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_A);
        /* Desired output of CHB: APT_PWM_LOW_LEVEL */
        aptChBStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_B);
        /* Desired output of POE: APT_POE_HIGH_LEVEL */
        POEStatus = HAL_APT_GetPoeStatus(aptHandle, APT_POE0);
        DBG_PRINTF("POE0 is triggered, PWM A status is %d, PWM B status is %d, POE status is %d, correct!\n", \
                   aptChAStatus, aptChBStatus, POEStatus);
    }
}

/**
  * @brief Timer interrupt callback function of APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void APT0TimerCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    HAL_APT_SetPWMDutyByNumber(&g_apt0, 50); /* Pwm duty is 50% */
}

/**
  * @brief Test pwm status and poe status in three condition.
  * @retval None.
  */
void APT_PwmPoeStatus(void)
{
    SystemInit();
    HAL_APT_StartModule(RUN_APT0 | RUN_APT0 | RUN_APT0);

    BASE_FUNC_DELAY_S(3); /* Delay 3s */
    /* Pwm channel A output High level and channel B output Low level */
    bool g_testRes1 = CheckStatusPwmChAHighChBLow(&g_apt0);
    if (g_testRes1) {
        DBG_PRINTF("PWM A is High, PWM B is Low, POE status is Low, correct!\n");
    }

    BASE_FUNC_DELAY_S(3); /* Delay 3s */
    /* Pwm channel A output Low level and channel B output High level */
    bool g_testRes2 = CheckStatusPwmChALowChBHigh(&g_apt0);
    if (g_testRes2) {
        DBG_PRINTF("PWM A is Low, PWM B is High, POE status is Low, correct!\n");
    }

    BASE_FUNC_DELAY_S(3); /* Delay 3s */
    /* Channel A and B output pwm */
    DCL_APT_DisableSwContPWMAction(g_apt0.baseAddress, APT_PWM_CHANNEL_A);
    DCL_APT_DisableSwContPWMAction(g_apt0.baseAddress, APT_PWM_CHANNEL_B);

    while (1) {
    }
}

/**
  * @brief Configure APT action.
  * @param aptAddr APT base address.
  * @param aptAct The APT action.
  */
static void APT_Config(APT_RegStruct* aptAddr, AptAct aptAct)
{
    switch (aptAct) {
        case APT_CHA_PWM_CHB_PWM:
            /* Channel A: 0 means not force output enable, channel A output PWM. */
            DCL_APT_DisableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            /* Channel B: 0 means not force output enable, channel B output PWM. */
            DCL_APT_DisableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            break;
        case APT_CHA_PWM_CHB_LOW:
            /* Channel A: 0 means not force output enable, channel A output PWM. */
            DCL_APT_DisableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            /* Channel B: 1 means force output enable. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            /* Channel B: 2 means channel B force output LOW due to the A_H_B_L invert. */
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;
        case APT_CHA_LOW_CHB_HIGH:
            /* Channel A: 1 means force output enable. */
            /* Channel A: 1 means channel A force output LOW. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            /* Channel B: 1 means force output enable. */
            /* Channel B: 1 means channel A force output HIGH due to the A_H_B_L invert. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;
        case APT_CHA_LOW_CHB_LOW:
            /* Channel A: 1 means force output enable. */
            /* Channel A: 1 means channel A force output LOW. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            /* Channel B: 1 means force output enable. */
            /* Channel B: 2 means channel A force output LOW due to the A_H_B_L invert. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;
        case APT_CHA_HIGH_CHB_LOW:
            /* Channel A: 1 means force output enable. */
            /* Channel A: 1 means channel A force output HIGH. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            /* Channel B: 1 means force output enable. */
            /* Channel B: 2 means channel A force output LOW due to the A_H_B_L invert. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;
        default:
            break;
    }
}

/**
  * @brief Check pwm and poe status when channel A output high, channel B output low.
  * @param aptHandle APT Handle.
  * @retval bool.
  *         true: The obtained PWM and POE status is consistent with the preset status.
  *         false: The obtained PWM and PO status is not consistent with the preset status.
  */
static bool CheckStatusPwmChAHighChBLow(APT_Handle *aptHandle)
{
    APT_PwmStatus aptChAStatus;
    APT_PwmStatus aptChBStatus;
    APT_PoeStatus POEStatus;

    APT_Config(aptHandle->baseAddress, APT_CHA_HIGH_CHB_LOW);
    /* delay 100 ms */
    BASE_FUNC_DELAY_MS(100);
    /* Read pwm status and poe status */
    aptChAStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_A); /* Desired output of CHA: APT_PWM_HIGH_LEVEL */
    aptChBStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_B); /* Desired output of CHB: APT_PWM_LOW_LEVEL */
    POEStatus = HAL_APT_GetPoeStatus(aptHandle, APT_POE0);      /* Desired output of POE: APT_POE_LOW_LEVEL */
    return (aptChAStatus == APT_PWM_HIGH_LEVEL && \
            aptChBStatus == APT_PWM_LOW_LEVEL && \
            POEStatus == APT_POE_LOW_LEVEL);
}

/**
  * @brief Check pwm and poe status when channel A output high, channel B output low.
  * @param aptHandle APT Handle.
  * @retval bool.
  *         true: The obtained PWM and POE status is consistent with the preset status.
  *         false: The obtained PWM and PO status is not consistent with the preset status.
  */
static bool CheckStatusPwmChALowChBHigh(APT_Handle *aptHandle)
{
    APT_PwmStatus aptChAStatus;
    APT_PwmStatus aptChBStatus;
    APT_PoeStatus POEStatus;
    APT_Config(aptHandle->baseAddress, APT_CHA_LOW_CHB_HIGH);
    /* delay 100 ms */
    BASE_FUNC_DELAY_MS(100);
    /* Read pwm status and poe status */
    aptChAStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_A); /* Desired output of CHA: APT_PWM_LOW_LEVEL */
    aptChBStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_B); /* Desired output of CHB: APT_PWM_HIGH_LEVEL */
    POEStatus = HAL_APT_GetPoeStatus(aptHandle, APT_POE0);      /* Desired output of POE: APT_POE_LOW_LEVEL */
    return (aptChAStatus == APT_PWM_LOW_LEVEL && \
            aptChBStatus == APT_PWM_HIGH_LEVEL && \
            POEStatus == APT_POE_LOW_LEVEL);
}

/**
  * @brief Check pwm and poe status when poe is trigger high manually.
  * @param aptHandle APT Handle.
  * @retval bool.
  *         true: The obtained PWM and POE status is consistent with the preset status.
  *         false: The obtained PWM and PO status is not consistent with the preset status.
  */
static bool POE_CheckStatus(APT_Handle *aptHandle)
{
    APT_PwmStatus aptChAStatus;
    APT_PwmStatus aptChBStatus;
    APT_PoeStatus POEStatus;
    /* Read pwm status and poe status */
    aptChAStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_A); /* Desired output of CHA: APT_PWM_LOW_LEVEL */
    aptChBStatus = HAL_APT_GetPwmStatus(aptHandle, APT_PWM_CHANNEL_B); /* Desired output of CHB: APT_PWM_LOW_LEVEL */
    POEStatus = HAL_APT_GetPoeStatus(aptHandle, APT_POE0);      /* Desired output of POE: APT_POE_HIGH_LEVEL */

    return (aptChAStatus == APT_PWM_LOW_LEVEL && \
            aptChBStatus == APT_PWM_LOW_LEVEL && \
            POEStatus == APT_POE_HIGH_LEVEL);
}
