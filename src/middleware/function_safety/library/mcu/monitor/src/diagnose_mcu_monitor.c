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
  * @file      diagnose_mcu_monitor.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for monitor diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_monitor.h"
/*------------------------------- WDG or WWDG monitor program stuck -----------------------------------*/
#if defined(WDG) || defined(WWDG)
/**
  * @brief diagnose program stuck by wdg.
  * @param monitorDiagnoseHandle Value of @ref MONITOR_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_DiagnoseProgramStuckByWdg(void* monitorDiagnoseHandle, DiagnoseMoment moment)
{
    MONITOR_DiagnoseHandle* handle = (MONITOR_DiagnoseHandle*)monitorDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_MONITOR);
    STATE_SetDiagnoseMoudule(&state, MODULE_WDG); /* set diagnose subsys, module and feature to state */
    STATE_SetDiagnoseFeature(&state, FEATURE_MONITOR_PROGRAM_STUCK);
    if (DCL_SYSCTRL_GetWdgResetConut() > 1) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);  /* set fail result and fault type to state */
        STATE_SetDiagnoseFaultType(&state, FAULT_WDG_PROGRAM_STUCK_RESET);
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
/**
  * @brief diagnose wdg's reset function.
  * @param monitorDiagnoseHandle Value of @ref MONITOR_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_DiagnoseWdgResetFunction(void* monitorDiagnoseHandle, DiagnoseMoment moment)
{
    MONITOR_DiagnoseHandle* handle = (MONITOR_DiagnoseHandle*)monitorDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
#ifdef WDG
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->wdgHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->wdgHandle->baseAddress != NULL, &state); /* param check not NULL */
#elif defined(WWDG)
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->wwdgHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->wwdgHandle->baseAddress != NULL, &state); /* param check not NULL */
#endif
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_MONITOR);
    STATE_SetDiagnoseMoudule(&state, MODULE_WDG);
    STATE_SetDiagnoseFeature(&state, FEATURE_MONITOR_WDG_RESET);  /* set diagnose subsys, module and feature to state */
    if (DCL_SYSCTRL_GetWdgResetConut() == 0) {
#ifdef WDG
        DCL_WDG_EnableReset(handle->wdgHandle->baseAddress);
        HAL_WDG_Start(handle->wdgHandle);
#elif defined(WWDG)
        DCL_WWDG_EnableReset(handle->wwdgHandle->baseAddress);
        HAL_WWDG_Start(handle->wwdgHandle);
#endif
    }
    unsigned int preTime = BASE_FUNC_GetTick();
    while (DCL_SYSCTRL_GetWdgResetConut() == 0) {
        unsigned int currentTime = BASE_FUNC_GetTick();
        unsigned int durationTick = (currentTime > preTime) ? (currentTime - preTime) : \
                                    preTime + SYSTICK_MAX_VALUE -currentTime + 1;
        unsigned int durationTimeUs = durationTick / HAL_CRG_GetCoreClkFreq() / CRG_FREQ_1MHz;
        if (durationTimeUs > handle->startupResetTimeUs + 2000) {    /* 2000: over 2000us wait for reset */
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);
            STATE_SetDiagnoseFaultType(&state, FAULT_WDG_UNRESET);  /* set fail result and fault type to state */
            return state;
        }
    }
#ifdef WDG
    HAL_WDG_Stop(handle->wdgHandle);
#elif defined(WWDG)
    HAL_WWDG_Stop(handle->wwdgHandle);
#endif
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
#endif
/*------------------------------- IWDG monitor program stuck -----------------------------------*/
#ifdef IWDG
/**
  * @brief diagnose program stuck by iwdg.
  * @param monitorDiagnoseHandle Value of @ref MONITOR_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_DiagnoseProgramStuckByIwdg(void* monitorDiagnoseHandle, DiagnoseMoment moment)
{
    MONITOR_DiagnoseHandle* handle = (MONITOR_DiagnoseHandle*)monitorDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_MONITOR);
    STATE_SetDiagnoseMoudule(&state, MODULE_IWDG);
    STATE_SetDiagnoseFeature(&state, FEATURE_MONITOR_PROGRAM_STUCK);
    if (DCL_SYSCTRL_GetIWdgResetConut() > 1) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);  /* set fail result and fault type to state */
        STATE_SetDiagnoseFaultType(&state, FAULT_WDG_PROGRAM_STUCK_RESET);
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}

/**
  * @brief diagnose iwdg's reset function.
  * @param monitorDiagnoseHandle Value of @ref MONITOR_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_DiagnoseIwdgResetFunction(void* monitorDiagnoseHandle, DiagnoseMoment moment)
{
    MONITOR_DiagnoseHandle* handle = (MONITOR_DiagnoseHandle*)monitorDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->iwdgHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->iwdgHandle->baseAddress != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_MONITOR);
    STATE_SetDiagnoseMoudule(&state, MODULE_IWDG);
    STATE_SetDiagnoseFeature(&state, FEATURE_MONITOR_WDG_RESET);  /* set diagnose subsys, module and feature to state */
    if (DCL_SYSCTRL_GetIWdgResetConut() == 0) {
        DCL_IWDG_EnableReset(handle->iwdgHandle->baseAddress);
        HAL_IWDG_Start(handle->iwdgHandle);
    }
    unsigned int preTime = BASE_FUNC_GetTick();
    while (DCL_SYSCTRL_GetIWdgResetConut() == 0) {
        unsigned int currentTime = BASE_FUNC_GetTick();
        unsigned int durationTick = (currentTime > preTime) ? (currentTime - preTime) : \
                                    preTime + SYSTICK_MAX_VALUE -currentTime + 1;
        unsigned int durationTimeUs = durationTick / HAL_CRG_GetCoreClkFreq() / CRG_FREQ_1MHz;
        if (durationTimeUs > handle->startupResetTimeUs + 2000) {    /* 2000: over 2000us wait for reset */
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);
            STATE_SetDiagnoseFaultType(&state, FAULT_WDG_UNRESET);  /* set fail result and fault type to state */
            return state;
        }
    }
    HAL_IWDG_Stop(handle->iwdgHandle);
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
#endif
/*------------------------------- PMC monitor low voltage UVP -----------------------------------*/
#ifdef PMC
/**
  * @brief diagnose power low level by pmc.
  * @param monitorDiagnoseHandle Value of @ref MONITOR_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_DiagnosePowerLowLevelByPmc(void* monitorDiagnoseHandle, DiagnoseMoment moment)
{
    MONITOR_DiagnoseHandle* handle = (MONITOR_DiagnoseHandle*)monitorDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->pmcHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->pmcHandle->baseAddress != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_MONITOR);
    STATE_SetDiagnoseMoudule(&state, MODULE_PMC);  /* set diagnose subsys, module and feature to state */
    STATE_SetDiagnoseFeature(&state, FEATURE_MONITOR_POWER_LOW_LEVEL);
    if (handle->pmcIrqFlag == BASE_CFG_SET) {
        handle->pmcIrqFlag = BASE_CFG_UNSET;
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_PMC_POWER_LOW_LEVEL);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
#endif
/*------------------------------- TSENSOR monitor over heat -----------------------------------*/
#ifdef TSENSOR
/**
  * @brief diagnose temperature over heat by tsensor.
  * @param monitorDiagnoseHandle Value of @ref MONITOR_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_DiagnoseOverTemperatureByTsensor(void* monitorDiagnoseHandle, DiagnoseMoment moment)
{
    MONITOR_DiagnoseHandle* handle = (MONITOR_DiagnoseHandle*)monitorDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_MONITOR);
    STATE_SetDiagnoseMoudule(&state, MODULE_TSENSOR);  /* set diagnose subsys, module and feature to state */
    STATE_SetDiagnoseFeature(&state, FEATURE_MONITOR_CHIP_OVER_TEMPERATUR);
    unsigned int temperature = (unsigned int)HAL_TSENSOR_GetTemperature();
    if (temperature >= handle->overTemperatureValue) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);  /* set fail result and fault type to state */
        STATE_SetDiagnoseFaultType(&state, FAULT_TSENSOR_CHIP_OVER_TEMPERATUR);
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
#endif