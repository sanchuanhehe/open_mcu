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
  * @file      failsafe_mcu_clock.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for failsafe diagnose.
  */

#include "diagnose_mcu_clock.h"
#include "failsafe_mcu_clock.h"
/**
  * @brief default failsafe measure of clock .
  * @param clockDiagnoseHandle Value of @ref CLOCK_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
__weak FunctionSafetyState CLOCK_DefaultFailSafe(void* clockFailSafeHandle, DiagnoseMoment moment)
{
    CLOCK_FailSafeHandle* handle = (CLOCK_FailSafeHandle*)clockFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    while (1) {   /* program stuck for doing something */
    }
    return state;
}
/**
  * @brief failsafe measure of clock accuracy.
  * @param clockDiagnoseHandle Value of @ref CLOCK_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CLOCK_AccuracyFailSafe(void* clockFailSafeHandle, DiagnoseMoment moment)
{
    CLOCK_FailSafeHandle* handle = (CLOCK_FailSafeHandle*)clockFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    BASE_FUNC_UNUSED(handle);
    if (moment == MOMENT_STARTUP) {  /* at moment startup */
    } else if (moment == MOMENT_RUNTIME) {  /* at moment runtime */
    } else if (moment == MOMENT_IRQ) {  /* at moment irq */
    }
    return state;
}
/**
  * @brief failsafe measure handler of clock.
  * @param clockDiagnoseHandle Value of @ref CLOCK_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CLOCK_FailSafeHandler(void* clockFailSafeHandle, DiagnoseMoment moment)
{
    CLOCK_FailSafeHandle* handle = (CLOCK_FailSafeHandle*)clockFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    switch (state.BIT.faultType) {
        case FAULT_OVER_ACCURACY:
            state = CLOCK_AccuracyFailSafe(handle, moment); /* clock accuracy fail safe handler */
            break;
        case FAULT_PLLREF_CLOCK_STOP:
            break;
        default:
            state = CLOCK_DefaultFailSafe(handle, moment);  /* clock default fail safe handler */
            break;
    }
    return state;
}