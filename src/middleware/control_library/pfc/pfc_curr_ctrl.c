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
  * @file      pfc_curr_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of power factor correction(PFC) current control
  */
#include "pfc_curr_ctrl.h"
#include "mcs_assert.h"


/**
  * @brief Init PFC voltage control structure.
  * @param currCtrl PFC voltage control Handle.
  * @param piParam PI controller parameter table.
  * @param startCurr Current threshold for starting the current loop.
  * @param stopCurr Current threshold for stopping the current loop.
  * @param vacAmp Amplitude of input voltage.
  * @param ts Control period.
  * @retval None.
  */
void PFC_CurrCtrlInit(PFC_CURRCTRL_Handle *currCtrl, PI_Param *piParam, float startCurr, \
                      float stopCurr, float vacAmp, float ts)
{
    MCS_ASSERT_PARAM(currCtrl != NULL);
    MCS_ASSERT_PARAM(piParam != NULL);
    MCS_ASSERT_PARAM(startCurr > 0.0f);
    MCS_ASSERT_PARAM(stopCurr > 0.0f);
    /* Reset pi parameters. */
    PID_Reset(&currCtrl->currPi);
    /* Init pi parameters. */
    currCtrl->currPi.kp = piParam->kp;
    currCtrl->currPi.ki = piParam->ki;
    currCtrl->currPi.upperLimit = piParam->upperLim;
    currCtrl->currPi.lowerLimit = piParam->lowerLim;
    currCtrl->currPi.ts = ts;
    /* Init user parameter and control parameter. */
    currCtrl->maxCurr = 0.0f;
    currCtrl->startCurr = startCurr;
    currCtrl->stopCurr = stopCurr;
    currCtrl->unitCoeff = 1.0f / vacAmp;
}

/**
  * @brief Simplified power factor correction(PFC) current controller PI calculation.
  * @param currCtrl PFC current control structure
  * @retval None.
  */
void PFC_CurrCtrlExec(PFC_CURRCTRL_Handle *currCtrl, float iacFbk)
{
    MCS_ASSERT_PARAM(currCtrl != NULL);

    currCtrl->iacFbk = iacFbk;
    /* Calculate the current error of power factor correction(PFC). */
    currCtrl->currPi.error = currCtrl->iacRef - currCtrl->iacFbk;
    /* Calculate the output pwm duty of power factor correction(PFC) current. */
    currCtrl->pwmDuty = PI_Exec(&currCtrl->currPi);
}

/**
  * @brief Clear historical values of power factor correction(PFC) current controller.
  * @param currCtrl PFC current control structure
  * @retval None.
  */
void PFC_CurrCtrlClear(PFC_CURRCTRL_Handle *currCtrl)
{
    MCS_ASSERT_PARAM(currCtrl != NULL);
    /* Clear history value */
    currCtrl->pwmDuty = 0.0f;
    currCtrl->iacFbk = 0.0f;
    currCtrl->vacFbk = 0.0f;
    currCtrl->unitVacFbk = 0.0f;
    /* Clear pid */
    PID_Clear(&currCtrl->currPi);
}