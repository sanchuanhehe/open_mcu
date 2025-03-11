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
  * @file    apt_ex.c
  * @author  MCU Driver Team
  * @brief   apt module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the apt.
  *           + high resolution of pwm setting.
  */

#include "apt_ex.h"
/**
  * @brief HRPWM delay setting
  * @param aptHandle APT module handle.
  * @param pwmChannel Channel of HRPMWM.
  * @param riseDelayStep Rising Edge Delay Step.
  * @param fallDelayStep Falling Edge Delay Step.
  * @retval BASE_StatusType: OK, ERROR.
  */
BASE_StatusType HAL_APT_SetHRPWM(APT_Handle *aptHandle, APT_PWMChannel pwmChannel,
                                 APT_PWMDelayStep riseDelayStep, APT_PWMDelayStep fallDelayStep)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    APT_ASSERT_PARAM(aptHandle->baseAddress != NULL);
    APT_ASSERT_PARAM(IsAPTInstance(aptHandle->baseAddress));
    /* PWM Channel Check */
    APT_PARAM_CHECK_WITH_RET(pwmChannel >= APT_PWM_CHANNEL_A, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(pwmChannel <= APT_PWM_CHANNEL_B, BASE_STATUS_ERROR);
    /* PWM Delay Step Check */
    APT_PARAM_CHECK_WITH_RET(riseDelayStep >= DELAY_0_STEP, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(riseDelayStep <= DELAY_11_STEP, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(fallDelayStep >= DELAY_0_STEP, BASE_STATUS_ERROR);
    APT_PARAM_CHECK_WITH_RET(fallDelayStep <= DELAY_11_STEP, BASE_STATUS_ERROR);
    if (pwmChannel == APT_PWM_CHANNEL_A) {
        DCL_APT_EnableHRPWMA(aptHandle->baseAddress);
        aptHandle->baseAddress->HRPWMA_RSTEP.reg = riseDelayStep;   /* HrPwm A channel PWM Delay Setting. */
        aptHandle->baseAddress->HRPWMA_FSTEP.reg = fallDelayStep;
        return BASE_STATUS_OK;
    }
    if (pwmChannel == APT_PWM_CHANNEL_B) {
        DCL_APT_EnableHRPWMB(aptHandle->baseAddress);
        aptHandle->baseAddress->HRPWMB_RSTEP.reg = riseDelayStep;  /* HrPwm B channel PWM Delay Setting. */
        aptHandle->baseAddress->HRPWMB_FSTEP.reg = fallDelayStep;
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Protect of POEx flag.
  * @param aptHandle APT module handle.
  * @retval APT_PoeStatus: Status of POE.
  */
APT_PoeStatus HAL_APT_GetPoeStatus(APT_Handle *aptHandle, APT_POEx poex)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    /* POE Channel Check */
    APT_ASSERT_PARAM(poex >= APT_POE0);
    APT_ASSERT_PARAM(poex <= APT_POE2);
    /* POE select */
    if (poex == APT_POE0) {
        return DCL_APT_GetPoe0Status(aptHandle->baseAddress);
    } else if (poex == APT_POE1) {
        return DCL_APT_GetPoe1Status(aptHandle->baseAddress);
    } else if (poex == APT_POE2) {
        return DCL_APT_GetPoe2Status(aptHandle->baseAddress);
    }
    return APT_POE_LOW_LEVEL;
}

/**
  * @brief Get PWM Channelx output status.
  * @param aptHandle APT module handle.
  * @param pwmChannel Channel of HRPMWM.
  * @retval APT_PwmStatus: Status of PWM.
  */
APT_PwmStatus HAL_APT_GetPwmStatus(APT_Handle *aptHandle, APT_PWMChannel pwmChannel)
{
    APT_ASSERT_PARAM(aptHandle != NULL);
    /* PWM Channel Check */
    APT_ASSERT_PARAM(pwmChannel >= APT_PWM_CHANNEL_A);
    APT_ASSERT_PARAM(pwmChannel <= APT_PWM_CHANNEL_B);
    return (pwmChannel == APT_PWM_CHANNEL_A)? DCL_APT_GetPWMAStatus(aptHandle->baseAddress)\
                                            : DCL_APT_GetPWMBStatus(aptHandle->baseAddress);
}
