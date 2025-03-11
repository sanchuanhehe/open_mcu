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
  * @file      failsafe_mcu_monitor.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for monitor failsafe.
  */
#include "diagnose_mcu_monitor.h"
#include "failsafe_mcu_monitor.h"

/**
  * @brief default failsafe measure of monitor.
  * @param monitorFailSafeHandle Value of @ref MONITOR_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
__weak FunctionSafetyState MONITOR_DefaultFailSafe(void* monitorFailSafeHandle, DiagnoseMoment moment)
{
    MONITOR_FailSafeHandle* handle = (MONITOR_FailSafeHandle*)monitorFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    while (1) {
    }
    return state;
}
/**
  * @brief failsafe measure of program stuck.
  * @param monitorFailSafeHandle Value of @ref MONITOR_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_wdgFailSafe(void* monitorFailSafeHandle, DiagnoseMoment moment)
{
    MONITOR_FailSafeHandle* handle = (MONITOR_FailSafeHandle*)monitorFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of power low level.
  * @param monitorFailSafeHandle Value of @ref MONITOR_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_pmcFailSafe(void* monitorFailSafeHandle, DiagnoseMoment moment)
{
    MONITOR_FailSafeHandle* handle = (MONITOR_FailSafeHandle*)monitorFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of temperature over heat.
  * @param monitorFailSafeHandle Value of @ref MONITOR_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_tsensorFailSafe(void* monitorFailSafeHandle, DiagnoseMoment moment)
{
    MONITOR_FailSafeHandle* handle = (MONITOR_FailSafeHandle*)monitorFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure handler of monitor.
  * @param monitorFailSafeHandle Value of @ref MONITOR_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState MONITOR_FailSafeHandler(void* monitorFailSafeHandle, DiagnoseMoment moment)
{
    MONITOR_FailSafeHandle* handle = (MONITOR_FailSafeHandle*)monitorFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    switch (state.BIT.faultType) {
        case FAULT_WDG_PROGRAM_STUCK_RESET:
        case FAULT_WDG_UNRESET:
            state = MONITOR_wdgFailSafe(handle, moment); /* monitor's wdg fail safe */
            break;
        case FAULT_PMC_POWER_LOW_LEVEL:
            state = MONITOR_pmcFailSafe(handle, moment); /* monitor's pmc fail safe */
            break;
        case FAULT_TSENSOR_CHIP_OVER_TEMPERATUR:
            state = MONITOR_tsensorFailSafe(handle, moment); /* monitor's tsensor fail safe */
            break;
        default:
            state = MONITOR_DefaultFailSafe(handle, moment); /* monitor's default fail safe */
            break;
    }
    return state;
}