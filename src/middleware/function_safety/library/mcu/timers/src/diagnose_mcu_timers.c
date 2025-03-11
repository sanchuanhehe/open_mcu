/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      diagnose_mcu_timers.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions for timers diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_timers.h"
#ifdef TIMER0
/**
  * @brief calculate interrupt interval of timer.
  * @param timersDiagnoseHandle Value of @ref TIMERS_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState TIMERS_DiagnoseInterruptIntervalAccuracy(void* timersDiagnoseHandle, DiagnoseMoment moment)
{
    TIMERS_DiagnoseHandle* handle = (TIMERS_DiagnoseHandle*)timersDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_TIMERS); /* run in timer irq handler */
    STATE_SetDiagnoseMoudule(&state, MODULE_TIMER); /* set diagnose subsys, module and feature to state */
    STATE_SetDiagnoseFeature(&state, FEATURE_TIMER_INTERRUPT_ACCURACY);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    if (!handle->timerIrqFlag) {
        STATE_SetDiagnoseResult(&state, RESULT_IS_RUNNING);  /* set result ongoing to state */
        return state;
    }
    handle->timerIrqFlag = BASE_CFG_UNSET;
    if (handle->durationUs > (handle->setTimeUs + handle->errRangeUs)) {  /* interrupt interval judgement */
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_INTERRUPT_ACCURACY);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    HAL_TIMER_Stop(handle->timerHandle);
    return state;
}
#endif