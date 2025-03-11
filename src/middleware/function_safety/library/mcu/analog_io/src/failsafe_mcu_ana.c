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
  * @file      failsafe_mcu_ana.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for ana failsafe.
  */

#include "diagnose_mcu_ana.h"
#include "failsafe_mcu_ana.h"

/**
  * @brief Default failsafe of ana subsystem.
  * @param anaDiagnoseHandle Value of @ref ANA_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
__weak FunctionSafetyState ANA_DefaultFailSafe(void* anaFailSafeHandle, DiagnoseMoment moment)
{
    ANA_FailSafeHandle* handle = (ANA_FailSafeHandle*)anaFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    while (1) {  /* program stuck */
    }
    return state;
}

#if defined(ADC0)
/**
  * @brief failsafe of ana subsystem's ADC sampling accuracy.
  * @param anaDiagnoseHandle Value of @ref ANA_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ANA_AdcFailSafeHandler(void* anaFailSafeHandle, DiagnoseMoment moment)
{
    ANA_FailSafeHandle* handle = (ANA_FailSafeHandle*)anaFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    BASE_FUNC_UNUSED(handle);
    if (state.BIT.moudule != MODULE_ADC) {
        return state;
    }
    if (moment == MOMENT_STARTUP) {
        HAL_CRG_IpClkResetSet(handle->anaDiagnoseHandle->adcHandle, BASE_CFG_SET); /* adc module soft reset */
        HAL_ADC_Init(handle->anaDiagnoseHandle->adcHandle); /* adc init after adc soft reset */
    } else if (moment == MOMENT_RUNTIME) {
    } else if (moment == MOMENT_IRQ) {
    }
    return state;
}
#endif
/**
  * @brief failsafe of ana subsystem.
  * @param anaDiagnoseHandle Value of @ref ANA_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ANA_FailSafeHandler(void* anaFailSafeHandle, DiagnoseMoment moment)
{
    ANA_FailSafeHandle* handle = (ANA_FailSafeHandle*)anaFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    switch (state.BIT.moudule) {
#if defined(ADC0)
        case MODULE_ADC:
            state = ANA_AdcFailSafeHandler(handle, moment); /* adc fail safe handler */
            break;
#endif
#if defined(DAC0)
        case MODULE_DAC:
            break;
#endif
#if defined(PGA0)
        case MODULE_PGA:
            break;
#endif
#if defined(ACMP0)
        case MODULE_ACMP:
            break;
#endif
        default:
            state = ANA_DefaultFailSafe(handle, moment); /* default fail safe of ANA */
            break;
    }
    return state;
}