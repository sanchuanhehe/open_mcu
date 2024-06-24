/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file   mcs_six_step.c
  * @author MCU Algorithm Team
  * @brief  This file provides functions to control the H bridge.
  */

#include "mcs_assert.h"
#include "mcs_six_step.h"

/* Mode for disabling or enabling software-based forced output */
#define APT_PWM_MODE    0
#define APT_FRC_MODE    1

#define PWM_SIGNAL      1

#define FRC_LOW         APT_OUT_CTRL_ACTION_LOW
#define FRC_HIGH        APT_OUT_CTRL_ACTION_HIGH

/**
  * @brief Force PWM output by APT
  * @param APTx The APT to use
  * @param PwmAout   Force to be high or low
  * @param PwmAenble PWMA signal software force mode
  * @param PwmBout   Force to be high or low
  * @param PwmBenble PWMB signal software force mode
  */
static void APT_ForcePwmByApt(APT_RegStruct *aptx, APT_Act actMode)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    switch (actMode) {
        case APT_CHA_PWM_CHB_HIGH:
            /* Channel A: 0 means not force output enable, channel A output PWM. */
            DCL_APT_DisableSwContPWMAction(aptx, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptx, APT_PWM_CHANNEL_A, APT_PWM_OUT_ALWAYS_LOW);
            /* Channel B: 1 means force output enable. */
            DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_B);
            /* Channel B: 2 means channel B force output LOW due to the A_H_B_L invert. */
            DCL_APT_SetSwContPWMAction(aptx, APT_PWM_CHANNEL_B, APT_PWM_OUT_ALWAYS_HIGH);
            break;
        case APT_CHA_LOW_CHB_HIGH:
            /* Channel A: 1 means force output enable. */
            /* Channel A: 1 means channel A force output LOW. */
            DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptx, APT_PWM_CHANNEL_A, APT_PWM_OUT_ALWAYS_LOW);
            /* Channel B: 1 means force output enable. */
            /* Channel B: 2 means channel B force output HIGH. */
            DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_B);
            DCL_APT_SetSwContPWMAction(aptx, APT_PWM_CHANNEL_B, APT_PWM_OUT_ALWAYS_HIGH);
            break;
        case APT_CHA_LOW_CHB_LOW:
            /* Channel A: 1 means force output enable. */
            /* Channel A: 1 means channel A force output LOW. */
            DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptx, APT_PWM_CHANNEL_A, APT_PWM_OUT_ALWAYS_LOW);
            /* Channel B: 1 means force output enable. */
            /* Channel B: 1 means channel A force output HIGH due to the A_H_B_L invert. */
            DCL_APT_EnableSwContPWMAction(aptx, APT_PWM_CHANNEL_B);
            DCL_APT_SetSwContPWMAction(aptx, APT_PWM_CHANNEL_B, APT_PWM_OUT_ALWAYS_LOW);
            break;
        default:
            break;
    }
}

/**
  * @brief Output six-step square wave.
  * @param handle The SixStepHandle.
  * @retval None.
  */
void SixStepPwm(const SixStepHandle *handle)
{
    MCS_ASSERT_PARAM(handle != NULL);
    APT_RegStruct *aptU = handle->controlApt.u->baseAddress; /* Get apt u\v\w baseaddress. */
    APT_RegStruct *aptV = handle->controlApt.v->baseAddress;
    APT_RegStruct *aptW = handle->controlApt.w->baseAddress;
    switch (handle->phaseStep) {
        case STEP1: /* U+, W- */
            APT_ForcePwmByApt(aptU, APT_CHA_PWM_CHB_HIGH);
            APT_ForcePwmByApt(aptV, APT_CHA_LOW_CHB_LOW);
            APT_ForcePwmByApt(aptW, APT_CHA_LOW_CHB_HIGH);
            break;
        case STEP2: /* V+, W- */
            APT_ForcePwmByApt(aptU, APT_CHA_PWM_CHB_HIGH);
            APT_ForcePwmByApt(aptV, APT_CHA_LOW_CHB_HIGH);
            APT_ForcePwmByApt(aptW, APT_CHA_LOW_CHB_LOW);
            break;
        case STEP3: /* V+, U- */
            APT_ForcePwmByApt(aptU, APT_CHA_LOW_CHB_HIGH);
            APT_ForcePwmByApt(aptV, APT_CHA_PWM_CHB_HIGH);
            APT_ForcePwmByApt(aptW, APT_CHA_LOW_CHB_LOW);
            break;
        case STEP4: /* W+, U- */
            APT_ForcePwmByApt(aptU, APT_CHA_LOW_CHB_LOW);
            APT_ForcePwmByApt(aptV, APT_CHA_PWM_CHB_HIGH);
            APT_ForcePwmByApt(aptW, APT_CHA_LOW_CHB_HIGH);
            break;
        case STEP5: /* W+, V- */
            APT_ForcePwmByApt(aptU, APT_CHA_LOW_CHB_LOW);
            APT_ForcePwmByApt(aptV, APT_CHA_LOW_CHB_HIGH);
            APT_ForcePwmByApt(aptW, APT_CHA_PWM_CHB_HIGH);
            break;
        case STEP6: /* U+, V- */
            APT_ForcePwmByApt(aptU, APT_CHA_LOW_CHB_HIGH);
            APT_ForcePwmByApt(aptV, APT_CHA_LOW_CHB_LOW);
            APT_ForcePwmByApt(aptW, APT_CHA_PWM_CHB_HIGH);
            break;
        default:
            break;
    }
}