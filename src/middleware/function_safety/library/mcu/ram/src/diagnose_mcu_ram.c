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
  * @file      diagnose_mcu_ram.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for ram diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_ram.h"

/* Reserved area for RAM buffer, include overlap for test purposes */
/* Don't change this parameter as it is related to physical technology used! */
#define RT_RAM_BLOCKSIZE          0x00000006U
/* Min overlap to cover coupling fault from one tested row to the other */
#define RT_RAM_BLOCK_OVERLAP      0x00000001U

BASE_StatusType RAM_DiagnoseStepRange(unsigned int *start, unsigned int *buffer, unsigned int pattern);
BASE_StatusType RAM_DiagnoseFullRange(unsigned int *start, unsigned int *end, unsigned int pattern);

#ifdef DIAGNOSE_RAM_SCRAMBLER
const unsigned int g_scrambleRamOrder[9] = {-8, 0, 4, 12, 8, 16, 20, 28, 24}; /* use in match c test */
#else
const unsigned int g_standRamOrder[9] = {-4, 0, 4, 8, 12, 16, 20, 24, 28}; /* use in match c test */
#endif
/* Pattern for stack overflow test in this array */
unsigned int g_ramStepCheckBufferAddr[RT_RAM_BLOCKSIZE + 2U]  __attribute__((section("RAM_DIAGNOSE_BUF")));
unsigned int g_stackChkPattern[4]                             __attribute__((section("STACK_SRAM_BOUND"))) \
             = {0xEEEEEEEE, 0xCCCCCCCCU, 0xBBBBBBBBU, 0xDDDDDDDDU};

/**
  * @brief diagnose slice sram by march algorithm.
  * @param ramDiagnoseHandle Value of @ref RAM_DiagnoseHandle.
  * @param state Value of @ref FunctionSafetyState.
  * @retval None.
  */
static void RAM_DiagnoseSliceSramAndRecovery(RAM_DiagnoseHandle* handle, FunctionSafetyState* state)
{
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, state); /* param check not NULL */
    bool status = BASE_STATUS_OK;
    if ((((unsigned int)(uintptr_t)(void *)handle->sramCurrentAddr) ^ \
        ((unsigned int)(uintptr_t)(void *)handle->sramCurrentAddrInv)) != 0xFFFFFFFFU) {
        STATE_SetDiagnoseResult(state, RESULT_FAIL);  /* set fail result and fault type to state */
        STATE_SetDiagnoseFaultType(state, FAULT_DATA_NOT_INTEGRITY);
        return;
    }
    if (handle->sramCurrentAddr > handle->sramEndAddr) {
        status = RAM_DiagnoseStepRange(g_ramStepCheckBufferAddr, g_ramStepCheckBufferAddr, handle->sramPattern);
        if (status != BASE_STATUS_OK) {
            STATE_SetDiagnoseResult(state, RESULT_FAIL);  /* set fail result and fault type to state */
            STATE_SetDiagnoseFaultType(state, FAULT_SRAM_STEP_RANGE);
            return;
        }
        STATE_SetDiagnoseResult(state, RESULT_SUCCESS);  /* set result success to state */
        handle->sramCurrentAddr = handle->sramStartAddr;
        handle->sramCurrentAddrInv = (unsigned int*)(void*)(~(unsigned int)(uintptr_t)(void*)handle->sramCurrentAddr);
    } else {
        status = RAM_DiagnoseStepRange(handle->sramCurrentAddr, g_ramStepCheckBufferAddr, handle->sramPattern);
        if (status != BASE_STATUS_OK) {
            STATE_SetDiagnoseResult(state, RESULT_FAIL);  /* set fail result and fault type to state */
            STATE_SetDiagnoseFaultType(state, FAULT_SRAM_STEP_RANGE);
            return;
        }
        if (handle->sramCurrentAddr == handle->sramStartAddr) {
            handle->sramCurrentAddr += 2 * (RT_RAM_BLOCKSIZE - (2 * RT_RAM_BLOCK_OVERLAP)); /* 2:skip bufferSize 0x20 */
        } else {
            handle->sramCurrentAddr += RT_RAM_BLOCKSIZE - (2 * RT_RAM_BLOCK_OVERLAP); /* 2: front and tail */
        }
        handle->sramCurrentAddrInv = (unsigned int*)(void*)(~(unsigned int)(uintptr_t)(void*)handle->sramCurrentAddr);
    }
    STATE_SetDiagnoseResult(state, RESULT_SUCCESS);  /* set result success to state */
    return;
}

/**
  * @brief diagnose sram by march algorithm.
  * @param ramDiagnoseHandle Value of @ref RAM_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_DiagnoseSramByMarchAlgorithm(void* ramDiagnoseHandle, DiagnoseMoment moment)
{
    RAM_DiagnoseHandle* handle = (RAM_DiagnoseHandle*)ramDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_RAM);
    STATE_SetDiagnoseMoudule(&state, MODULE_RAM_SRAM);
    STATE_SetDiagnoseFeature(&state, FEATURE_SRAM_MARCH); /* set diagnose subsys, module and feature to state */
    if (moment == MOMENT_STARTUP) {  /* startup diagnose */
        while (handle->sramCurrentAddr <= handle->sramEndAddr - 16) { /* 16 : once diagnose lenth */
            IRQ_Disable();
            RAM_DiagnoseSliceSramAndRecovery(handle, &state);
            IRQ_Enable();
            if (state.BIT.result == RESULT_FAIL) {
                STATE_SetDiagnoseResult(&state, RESULT_FAIL);  /* set fail result and fault type to state */
                STATE_SetDiagnoseFaultType(&state, FAULT_SRAM_FULL_RANGE);
                return state;
            }
        }
        handle->sramCurrentAddr = handle->sramStartAddr;
        handle->sramCurrentAddrInv = (unsigned int*)(void*)(~(unsigned int)(uintptr_t)(void*)handle->sramCurrentAddr);
    } else if (moment == MOMENT_RUNTIME) {  /* runtime diagnose */
        IRQ_Disable();
        RAM_DiagnoseSliceSramAndRecovery(handle, &state);
        IRQ_Enable();
    }
    return state;
}
/**
  * @brief diagnose sram by parity check.
  * @param ramDiagnoseHandle Value of @ref RAM_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_DiagnoseSramByParityCheck(void* ramDiagnoseHandle, DiagnoseMoment moment)
{
    RAM_DiagnoseHandle* handle = (RAM_DiagnoseHandle*)ramDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_RAM);
    STATE_SetDiagnoseMoudule(&state, MODULE_RAM_SRAM);
    STATE_SetDiagnoseFeature(&state, FEATURE_SRAM_PARITY); /* set diagnose subsys, module and feature to state */
    unsigned int status = DCL_SYSCTRL_GetSysramParityErrorStatus();
    if (status & 0x01) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_SRAM_PARITY_ERROR);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
/**
  * @brief diagnose stack overflow.
  * @param ramDiagnoseHandle Value of @ref RAM_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState RAM_DiagnoseStackOverflow(void* ramDiagnoseHandle, DiagnoseMoment moment)
{
    RAM_DiagnoseHandle* handle = (RAM_DiagnoseHandle*)ramDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_RAM);
    STATE_SetDiagnoseMoudule(&state, MODULE_RAM_STACK); /* set diagnose subsys, module and feature to state */
    STATE_SetDiagnoseFeature(&state, FEATURE_STACK_OVERFLOW_CHECK);
    if (g_stackChkPattern[0] != 0xEEEEEEEE || /* test pattern 0 */
        g_stackChkPattern[1] != 0xCCCCCCCC || /* test pattern 1 */
        g_stackChkPattern[2] != 0xBBBBBBBB || /* test pattern 2 */
        g_stackChkPattern[3] != 0xDDDDDDDD) {  /* test pattern 3 */
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_STACK_OVERFLOW);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}