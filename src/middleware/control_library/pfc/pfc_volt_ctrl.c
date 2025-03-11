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
  * @file      pfc_volt_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of power factor correction(PFC) voltage control
  */
#include "pfc_volt_ctrl.h"
#include "mcs_math.h"
#include "mcs_assert.h"


/**
  * @brief Init PFC voltage control structure.
  * @param voltCtrl PFC voltage control Handle.
  * @param piParam PI controller parameter table.
  * @param vdcRef Reference voltage.
  * @retval None.
  */
void PFC_VoltCtrlInit(PFC_VOLTCTRL_Handle *voltCtrl, PI_Param *piParam, float vdcRef, float ts)
{
    MCS_ASSERT_PARAM(voltCtrl != NULL);
    MCS_ASSERT_PARAM(piParam != NULL);
    MCS_ASSERT_PARAM(vdcRef > 0.0f);
    /* Reset pi parameters. */
    PID_Reset(&voltCtrl->voltPi);
    /* Init pi parameters. */
    voltCtrl->voltPi.kp = piParam->kp;
    voltCtrl->voltPi.ki = piParam->ki;
    voltCtrl->voltPi.upperLimit = piParam->upperLim;
    voltCtrl->voltPi.lowerLimit = piParam->lowerLim;
    voltCtrl->voltPi.ts = ts;
    /* Init user parameter and control parameter. */
    voltCtrl->iamp = 0.0f;
    voltCtrl->vdcRef = vdcRef;
}


/**
  * @brief Power factor correction(PFC) voltage controller PI calculation.
  * @param voltCtrl PFC voltage control structure
  * @retval None.
  */
void PFC_VoltCtrlExec(PFC_VOLTCTRL_Handle *voltCtrl, float vdcFbk)
{
    MCS_ASSERT_PARAM(voltCtrl != NULL);
    voltCtrl->vdcFbk = vdcFbk;
    /* Calculate the voltage error of power factor correction(PFC). */
    voltCtrl->voltPi.error = voltCtrl->vdcRef - voltCtrl->vdcFbk;
    /* Calculation the voltage loop control output of power factor correction(PFC). */
    voltCtrl->iamp = PI_Exec(&voltCtrl->voltPi);
}

/**
  * @brief Clear historical values of power factor correction(PFC) voltage controller.
  * @param voltCtrl PFC voltage control structure
  * @retval None.
  */
void PFC_VoltCtrlClear(PFC_VOLTCTRL_Handle *voltCtrl)
{
    MCS_ASSERT_PARAM(voltCtrl != NULL);
    voltCtrl->iamp = 0.0f;

    PID_Clear(&voltCtrl->voltPi);
}