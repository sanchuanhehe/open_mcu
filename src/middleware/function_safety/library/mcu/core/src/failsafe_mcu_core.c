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
  * @file      failsafe_mcu_core.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for core failsafe.
  */
#include "diagnose_mcu_core.h"
#include "failsafe_mcu_core.h"
/**
  * @brief default failsafe measure of core.
  * @param coreHandle Value of @ref CORE_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CORE_DefaultFailSafe(void* coreHandle, DiagnoseMoment moment)
{
    CORE_FailSafeHandle* handle = (CORE_FailSafeHandle*)coreHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    while (1) {
    }
    return state;
}
/**
  * @brief failsafe measure of core reg saf.
  * @param coreHandle Value of @ref CORE_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CORE_RegSafFailSafe(void* coreHandle, DiagnoseMoment moment)
{
    CORE_FailSafeHandle* handle = (CORE_FailSafeHandle*)coreHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure handler of core.
  * @param coreHandle Value of @ref CORE_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CORE_FailSafeHandler(void* coreHandle, DiagnoseMoment moment)
{
    CORE_FailSafeHandle* handle = (CORE_FailSafeHandle*)coreHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    switch (state.BIT.faultType) {
        case FAULT_REG_SAF:
            state = CORE_RegSafFailSafe(handle, moment); /* core reg saf fail safe */
            break;
        case FAULT_REG_AF:
            break;
        case FAULT_REG_TF:
            break;
        case FAULT_REG_CF:
            break;
        case FAULT_PC_JUMP:
            break;
        case FAULT_PC_INVALID:
            break;
        default:
            state = CORE_DefaultFailSafe(handle, moment); /* core default fail safe */
            break;
    }
    return state;
}