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
  * @file      failsafe_mcu_rom.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions for rom failsafe.
  */

#include "diagnose_mcu_rom.h"
#include "failsafe_mcu_rom.h"
/**
  * @brief default failsafe measure of rom.
  * @param romFailSafeHandle Value of @ref ROM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
__weak FunctionSafetyState ROM_DefaultFailSafe(void* romFailSafeHandle, DiagnoseMoment moment)
{
    ROM_FailSafeHandle* handle = (ROM_FailSafeHandle*)romFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    while (1) {
    }
    return state;
}
/**
  * @brief failsafe measure of rom step range diagnose.
  * @param romFailSafeHandle Value of @ref ROM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ROM_StepRangeFailSafe(void* romFailSafeHandle, DiagnoseMoment moment)
{
    ROM_FailSafeHandle* handle = (ROM_FailSafeHandle*)romFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of rom full range diagnose.
  * @param romFailSafeHandle Value of @ref ROM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ROM_FullRangeFailSafe(void* romFailSafeHandle, DiagnoseMoment moment)
{
    ROM_FailSafeHandle* handle = (ROM_FailSafeHandle*)romFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of rom integrity.
  * @param romFailSafeHandle Value of @ref ROM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ROM_IntegrityFailSafe(void* romFailSafeHandle, DiagnoseMoment moment)
{
    ROM_FailSafeHandle* handle = (ROM_FailSafeHandle*)romFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure handler of rom.
  * @param romFailSafeHandle Value of @ref ROM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ROM_FailSafeHandler(void* romFailSafeHandle, DiagnoseMoment moment)
{
    ROM_FailSafeHandle* handle = (ROM_FailSafeHandle*)romFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    switch (state.BIT.faultType) {
        case FAULT_DATA_NOT_INTEGRITY:
            state = ROM_IntegrityFailSafe(handle, moment); /* rom integrity fail safe */
            break;
        case FAULT_ROM_STEP_RANGE:
            state = ROM_StepRangeFailSafe(handle, moment); /* rom step range fail safe */
            break;
        case FAULT_ROM_FULL_RANGE:
            state = ROM_FullRangeFailSafe(handle, moment); /* rom full range fail safe */
            break;
        case FAULT_EFLASH_ECC_ERROR:
            break;
        default:
            state = ROM_DefaultFailSafe(handle, moment); /* rom default fail safe */
            break;
    }
    return state;
}