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
  * @file      mcs_power_mgmt.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of motor average power management.
  */


#include "mcs_power_mgmt.h"
#include "mcs_math.h"
#include "mcs_assert.h"

/**
  * @brief Init motor power management.
  * @param powerCalc Pointer of motor power handle.
  * @param vdqRef Pointer of vdqRef handle.
  * @param idqFbk Pointer of idqFbk handle.
  * @param fltCoeff Power filter coefficient.
  * @retval None.
  */
void MotorPowerInit(POWER_Handle *powerCalc, DqAxis *vdqRef, DqAxis *idqFbk, float fltCoeff)
{
    MCS_ASSERT_PARAM(powerCalc != NULL);
    MCS_ASSERT_PARAM(vdqRef != NULL);
    MCS_ASSERT_PARAM(idqFbk != NULL);
    /* Initialization. */
    powerCalc->power = 0.0f;
    /* Initialization. */
    powerCalc->vdqRef = vdqRef;
    powerCalc->idqFbk = idqFbk;
    powerCalc->fltCoeff = fltCoeff;
}

/**
  * @brief Power result value.
  * @param powerCalc Pointer of motor power handle.
  * @retval Motor power value (w).
  */
float MotorPowerCalc(POWER_Handle *powerCalc)
{
    MCS_ASSERT_PARAM(powerCalc != NULL);
    /* Calculate average power. */
    float actvPwr = 1.5f * (powerCalc->idqFbk->d * powerCalc->vdqRef->d + powerCalc->idqFbk->q * powerCalc->vdqRef->q);
    powerCalc->power = actvPwr * powerCalc->fltCoeff + powerCalc->power * (1.0f - powerCalc->fltCoeff);
    return powerCalc->power;
}

/**
  * @brief Clear motor power history value.
  * @param powerCalc Pointer of motor power handle.
  * @retval None.
  */
void MotorPowerClear(POWER_Handle *powerCalc)
{
    MCS_ASSERT_PARAM(powerCalc != NULL);
    /* Clear history value. */
    powerCalc->power = 0.0f;
}