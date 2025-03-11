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
  * @file      diagnose_mcu_rom.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions for rom diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_rom.h"

unsigned int g_refCrcSum                    __attribute__((section("CHECKSUM")));
#define REF_VALUE_CRC32                     (*((unsigned int *)&g_refCrcSum + 1))
#define ROM_END                             ((unsigned int *)&g_refCrcSum + 1)
/**
  * @brief endian swap function.
  * @param inputDate Value of input.
  * @retval unsigned int Value of inputDate endiam swap.
  */
static unsigned int EndianSwap(unsigned int inputDate)
{
    unsigned int outputData;
    /* big little endian swap */
    outputData = (unsigned int)(((unsigned int)(inputDate & 0xff000000) >> 24) |    /* 24: right shift 24 bit */
                                ((unsigned int)(inputDate & 0x00ff0000) >> 8) |     /* 8: right shift 8 bit */
                                ((unsigned int)(inputDate & 0x0000ff00) << 8) |     /* 8: left shift 24 bit */
                                ((unsigned int)(inputDate & 0x000000ff) << 24)) ;   /* 24: left shift 24 bit */
    return outputData;
}

/*-------------------------------- CRC diagnose rom intgrity ------------------------------------------*/
#ifdef CRC
/**
  * @brief ROM diagnose integrity at startup.
  * @param handle @ref ROM_DiagnoseHandle.
  * @param state @ref FunctionSafetyState.
  * @retval state @ref FunctionSafetyState.
  */
static FunctionSafetyState ROM_DiagnoseIntegrityStartup(ROM_DiagnoseHandle* handle, FunctionSafetyState* state)
{
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    unsigned int romFullSize = (ROM_END - handle->startAddr) * 4 / (handle->crcHandle->inputDataFormat + 1);
    bool result = HAL_CRC_CheckInputData(handle->crcHandle, handle->startAddr, \
                                         romFullSize, EndianSwap(REF_VALUE_CRC32));
    if (result == false) {
        STATE_SetDiagnoseResult(state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(state, FAULT_ROM_FULL_RANGE);  /* set fail result and fault type to state */
        return *state;
    } else {
        STATE_SetDiagnoseResult(state, RESULT_SUCCESS);  /* set result success to state */
    }
    return *state;
}

/**
  * @brief ROM diagnose integrity at run time.
  * @param handle @ref ROM_DiagnoseHandle.
  * @param state @ref FunctionSafetyState.
  * @retval state @ref FunctionSafetyState.
  */
static FunctionSafetyState ROM_DiagnoseIntegrityRuntime(ROM_DiagnoseHandle* handle, FunctionSafetyState* state)
{
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    if ((((unsigned int)(uintptr_t)(void *)handle->indexPointer) ^ \
        ((unsigned int)(uintptr_t)(void *)handle->indexPointerInv)) != 0xFFFFFFFFU) {
        STATE_SetDiagnoseResult(state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(state, FAULT_DATA_NOT_INTEGRITY);  /* set fail result and fault type to state */
        return *state;
    }
    if (handle->indexPointer <= ROM_END) {
        for (unsigned int index = 0; index < handle->flashBlockSizeInWords; ++index) {
            /* Avoid out of ROM test range */
            if ((handle->indexPointer + index) > ROM_END) {
                break;
            }
            unsigned int hostData = *((unsigned int *)handle->indexPointer + index);
            HAL_CRC_SetInputDataGetCheck(handle->crcHandle, EndianSwap(hostData));
        }
        handle->indexPointer += handle->flashBlockSizeInWords; /* Increment pointer to next block */
        handle->indexPointerInv = ((unsigned int *)(~((unsigned int)(uintptr_t)(void *)handle->indexPointer)));
        STATE_SetDiagnoseResult(state, RESULT_IS_RUNNING);
    } else {
        unsigned int crcValue = EndianSwap(DCL_CRC_GetOutputData(handle->crcHandle->baseAddress));
        if (crcValue != REF_VALUE_CRC32) {
            STATE_SetDiagnoseResult(state, RESULT_FAIL);
            STATE_SetDiagnoseFaultType(state, FAULT_ROM_STEP_RANGE);  /* set fail result and fault type to state */
            return *state;
        } else {
            STATE_SetDiagnoseResult(state, RESULT_SUCCESS);  /* set result success to state */
        }
        handle->indexPointer = handle->startAddr;
        handle->indexPointerInv = ((unsigned int *)(~((unsigned int)(uintptr_t)(void *)handle->startAddr)));
        HAL_CRC_DeInit(handle->crcHandle);
    }
    return *state;
}

/**
  * @brief diagnose rom integrith by crc32.
  * @param romDiagnoseHandle Value of @ref ROM_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ROM_DiagnoseIntegrity(void* romDiagnoseHandle, DiagnoseMoment moment)
{
    ROM_DiagnoseHandle* handle = (ROM_DiagnoseHandle*)romDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_ROM);
    STATE_SetDiagnoseMoudule(&state, MODULE_ROM_RANGE);
    STATE_SetDiagnoseFeature(&state, FEATURE_ROM_INTEGRITY); /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->crcHandle != NULL, &state); /* param check not NULL */
    if (moment == MOMENT_STARTUP) {
        state = ROM_DiagnoseIntegrityStartup(handle, &state);
    } else if (moment == MOMENT_RUNTIME) {
        state = ROM_DiagnoseIntegrityRuntime(handle, &state);
    }
    return state;
}
#endif
/*-------------------------------- CRC diagnose rom intgrity ------------------------------------------*/
#ifdef EFC
/**
  * @brief diagnose rom integrith by flash ecc.
  * @param romDiagnoseHandle Value of @ref ROM_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ROM_DiagnoseFlashEcc(void* romDiagnoseHandle, DiagnoseMoment moment)
{
    ROM_DiagnoseHandle* handle = (ROM_DiagnoseHandle*)romDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_ROM);
    STATE_SetDiagnoseMoudule(&state, MODULE_ROM_EFLASH);
    STATE_SetDiagnoseFeature(&state, FEATURE_EFLASH_ECC); /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    unsigned int status = DCL_FLASH_GetInterrupRawtStatus(handle->flashBase);
    if ((status & handle->eccIntRegVal) != 0) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_EFLASH_ECC_ERROR);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
#endif