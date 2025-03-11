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
  * @file      failsafe_mcu_compute.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for compute failsafe.
  */

#include "diagnose_mcu_compute.h"
#include "failsafe_mcu_compute.h"

/**
  * @brief default failsafe measure of compute .
  * @param computeFailSafeHandle Value of @ref COMPUTE_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
__weak FunctionSafetyState COMPUTE_DefaultFailSafe(void* computeFailSafeHandle, DiagnoseMoment moment)
{
    COMPUTE_FailSafeHandle* handle = (COMPUTE_FailSafeHandle*)computeFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check */
    while (1) {
    }
    return state;
}
/**
  * @brief failsafe measure of crc32 algorythm correctness.
  * @param computeFailSafeHandle Value of @ref COMPUTE_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState COMPUTE_Crc32FailSafe(void* computeFailSafeHandle, DiagnoseMoment moment)
{
    COMPUTE_FailSafeHandle* handle = (COMPUTE_FailSafeHandle*)computeFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure handler of ccompute.
  * @param computeFailSafeHandle Value of @ref COMPUTE_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState COMPUTE_FailSafeHandler(void* computeFailSafeHandle, DiagnoseMoment moment)
{
    COMPUTE_FailSafeHandle* handle = (COMPUTE_FailSafeHandle*)computeFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    switch (state.BIT.faultType) {
        case FAULT_CRC32_ALGORITHM_COMPUTE_UNCORRECT:
            state = COMPUTE_Crc32FailSafe(handle, moment); /* crc32 calculate fail safe */
            break;
        default:
            state = COMPUTE_DefaultFailSafe(handle, moment); /* cpmpute default fail safe */
            break;
    }
    return state;
}