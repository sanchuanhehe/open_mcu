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
  * @file      failsafe_mcu_ram.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for ram failsafe.
  */
#include "diagnose_mcu_ram.h"
#include "failsafe_mcu_ram.h"

/**
  * @brief default failsafe measure of ram.
  * @param ramFailSafeHandle Value of @ref RAM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
__weak FunctionSafetyState RAM_DefaultFailSafe(void* ramFailSafeHandle, DiagnoseMoment moment)
{
    RAM_FailSafeHandle* handle = (RAM_FailSafeHandle*)ramFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    while (1) {
    }
    return state;
}
/**
  * @brief failsafe measure of ram step range diagnose.
  * @param ramFailSafeHandle Value of @ref RAM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_StepRangeFailSafe(void* ramFailSafeHandle, DiagnoseMoment moment)
{
    RAM_FailSafeHandle* handle = (RAM_FailSafeHandle*)ramFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of ram full range diagnose.
  * @param ramFailSafeHandle Value of @ref RAM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_FullRangeFailSafe(void* ramFailSafeHandle, DiagnoseMoment moment)
{
    RAM_FailSafeHandle* handle = (RAM_FailSafeHandle*)ramFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of ram stack overflow.
  * @param ramFailSafeHandle Value of @ref RAM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_OverflowFailSafe(void* ramFailSafeHandle, DiagnoseMoment moment)
{
    RAM_FailSafeHandle* handle = (RAM_FailSafeHandle*)ramFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of ram parity check.
  * @param ramFailSafeHandle Value of @ref RAM_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_ParityFailSafe(void* ramFailSafeHandle, DiagnoseMoment moment)
{
    RAM_FailSafeHandle* handle = (RAM_FailSafeHandle*)ramFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure handler of ram.
  * @param dioFailSafeHandle Value of @ref DIO_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_FailSafeHandler(void* ramFailSafeHandle, DiagnoseMoment moment)
{
    RAM_FailSafeHandle* handle = (RAM_FailSafeHandle*)ramFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    switch (state.BIT.faultType) {
        case FAULT_DATA_NOT_INTEGRITY:
        case FAULT_SRAM_STEP_RANGE:
            state = RAM_StepRangeFailSafe(handle, moment); /* ram step range fail safe */
            break;
        case FAULT_SRAM_FULL_RANGE:
            state = RAM_FullRangeFailSafe(handle, moment); /* ram full range fail safe */
            break;
        case FAULT_SRAM_PARITY_ERROR:
            state = RAM_ParityFailSafe(handle, moment); /* ram parity check fail safe */
            break;
        case FAULT_STACK_OVERFLOW:
            state = RAM_OverflowFailSafe(handle, moment); /* ram step range fail safe */
            break;
        default:
            state = RAM_DefaultFailSafe(handle, moment); /* ram default fail safe */
            break;
    }
    return state;
}