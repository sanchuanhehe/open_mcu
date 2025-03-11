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
  * @file      function_safety_process.c
  * @author    MCU Driver Team
  * @brief     function safety general process module.
  * @details   function safety general process implement.
  */
#include "function_safety_process.h"

/**
  * @brief Excute function safety diagnose process and return state.
  * @param processHandle @ref FunctionSafetyProcessHandle.
  * @param  *state @ref FunctionSafetyState.
  * @param processTime @ref ProcessTimeCycle.
  * @retval None.
  */
static void DiagnoseProcess(FunctionSafetyProcessHandle* handle, FunctionSafetyState* state, \
                            ProcessTimeCycle* processTime)
{
    if (handle->diagnoseFunc == NULL || handle->diagnoseHandle == NULL) {
        return;
    }
    if (processTime != NULL && processTime->timeCycleRecordSwitch) {
        unsigned int startCycle = BASE_FUNC_GetTick(); /* diagnose start cycle */
        *state = handle->diagnoseFunc(handle->diagnoseHandle, handle->moment); /* diagnose function */
        unsigned int endCycle = BASE_FUNC_GetTick(); /* diagnose end cycle */
        processTime->diagnoseTimeCycle = (endCycle > startCycle) ? (endCycle - startCycle) : \
                                                                (startCycle + UINT_MAX - endCycle + 1);
    } else {
        *state = handle->diagnoseFunc(handle->diagnoseHandle, handle->moment); /* diagnose function */
    }
    STATE_StoreCurrentState(state); /* store current state */
    if (state->BIT.result == RESULT_FAIL) {  /* result fail judgement */
        STATE_StoreFailEvent(state);
    }
}

/**
  * @brief Excute function safety failsafe process and return state.
  * @param processHandle @ref FunctionSafetyProcessHandle.
  * @param  *state @ref FunctionSafetyState.
  * @param processTime @ref ProcessTimeCycle.
  * @retval None.
  */
static void FailSafeProcess(FunctionSafetyProcessHandle* handle, FunctionSafetyState* state, \
                            ProcessTimeCycle* processTime)
{
    if ((state->BIT.result != RESULT_FAIL) || handle->failSafeFunc == NULL || handle->failSafeHandle == NULL) {
        return;
    }
    if (processTime != NULL && processTime->timeCycleRecordSwitch) {
        unsigned int startCycle = BASE_FUNC_GetTick(); /* get start cycle */
        *state = handle->failSafeFunc(handle->failSafeHandle, handle->moment); /* fail safe function */
        unsigned int endCycle = BASE_FUNC_GetTick(); /* get end cycle */
        /* calculate fail safe duration cycle */
        processTime->failSafeTimeCycle = (endCycle > startCycle) ? (endCycle - startCycle) : \
                                                                (startCycle + UINT_MAX - endCycle + 1);
    } else {
        *state = handle->failSafeFunc(handle->failSafeHandle, handle->moment); /* fail safe function */
    }
}

/**
  * @brief Excute function safety fault predict process and return state.
  * @param processHandle @ref FunctionSafetyProcessHandle.
  * @param  *state @ref FunctionSafetyState.
  * @param processTime @ref ProcessTimeCycle.
  * @retval None.
  */
static void FaultPredictProcess(FunctionSafetyProcessHandle* handle, FunctionSafetyState* state, \
                                ProcessTimeCycle* processTime)
{
    if ((state->BIT.result == RESULT_FAIL) || handle->faultPredictFunc == NULL || handle->faultPredictHandle == NULL) {
        return;
    }
    if (processTime != NULL && processTime->timeCycleRecordSwitch) {
        unsigned int startCycle = BASE_FUNC_GetTick(); /* get start cycle */
        *state = handle->faultPredictFunc(handle->faultPredictHandle, handle->moment); /* fault predict function */
        unsigned int endCycle = BASE_FUNC_GetTick(); /* get end cycle */
        processTime->faultPredictTimeCycle = (endCycle > startCycle) ? (endCycle - startCycle) : \
                                                                    (startCycle + UINT_MAX - endCycle + 1);
    } else {
        *state = handle->faultPredictFunc(handle->faultPredictHandle, handle->moment); /* fault predict function */
    }
}

/**
  * @brief Excute function safety process and return state.
  * @param processHandle @ref FunctionSafetyProcessHandle.
  * @param processTime @ref ProcessTimeCycle.
  * @retval state @ref FunctionSafetyState.
  */
FunctionSafetyState FunctionSafetyProcess(void* processHandle, ProcessTimeCycle* processTime)
{
    FunctionSafetyProcessHandle* handle = (FunctionSafetyProcessHandle*)processHandle;
    FunctionSafetyState state = {0}; /* init state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(processTime != NULL, &state);
    if (handle->contexBackupFunc && handle->contexHandle) {
        state = handle->contexBackupFunc(handle->contexHandle, handle->moment); /* contex backup */
    }
    if (handle->handleConfigFunc) {
        handle->handleConfigFunc(); /* handle config */
    }
    DiagnoseProcess(handle, &state, processTime);
    FailSafeProcess(handle, &state, processTime);
    FaultPredictProcess(handle, &state, processTime);
    if (handle->contexResumeFunc && handle->contexHandle) {
        handle->contexResumeFunc(handle->contexHandle, handle->moment);  /* contex resume */
    }
    return state;
}