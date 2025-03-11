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
#ifndef McuMagicTag_APT_EX_H
#define McuMagicTag_APT_EX_H
#include "apt.h"

/**
  * @addtogroup APT_IP
  * @{
  */
typedef enum {
    APT_POE0 = 0,
    APT_POE1,
    APT_POE2,
} APT_POEx;
/**
  * @defgroup APT_EX_API_Declaration APT HAL API EX
  * @{
  */

BASE_StatusType HAL_APT_SetHRPWM(APT_Handle *aptHandle, APT_PWMChannel pwmChannel,
                                 APT_PWMDelayStep riseDelayStep, APT_PWMDelayStep fallDelayStep);

APT_PoeStatus HAL_APT_GetPoeStatus(APT_Handle *aptHandle, APT_POEx poex);

APT_PwmStatus HAL_APT_GetPwmStatus(APT_Handle *aptHandle, APT_PWMChannel pwmChannel);

/**
  * @}
  */

/**
  * @}
  */
#endif  /* McuMagicTag_APT_EX_H */