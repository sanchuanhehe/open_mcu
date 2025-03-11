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
  * @file      function_safety_runtime.c
  * @author    MCU Driver Team
  * @brief     function safety runtime module.
  * @details   function safety runtime interface implementation.
  */
#include "main.h"
#include "function_safety_init.h"
#include "function_safety_process.h"
#include "function_safety_report.h"
#include "function_safety_runtime.h"

FunctionSafetyProcessHandle g_fuSaRuntimeProcess[] = {
    {
        .diagnoseFunc       = CORE_DiagnoseCpuStatus,  /* cpu status diagnose */
        .failSafeFunc       = CORE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCore,
        .moment             = MOMENT_RUNTIME
    },
    {
        .diagnoseFunc       = CORE_DiagnoseCpuGeneralRegister,  /* cpu general register diagnose */
        .failSafeFunc       = CORE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCore,
        .moment             = MOMENT_RUNTIME
    },
    {
        .diagnoseFunc       = CORE_DiagnosePcRegister,  /* cpu pc register diagnose */
        .failSafeFunc       = CORE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCore,
        .moment             = MOMENT_RUNTIME
    },
    {
        .handleConfigFunc   = RAM_DiagnoseRuntimeInit,  /* ram step diagnose */
        .diagnoseFunc       = RAM_DiagnoseSramByMarchAlgorithm,
        .failSafeFunc       = RAM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRam,
        .moment             = MOMENT_RUNTIME
    },
    {
        .diagnoseFunc       = RAM_DiagnoseSramByParityCheck,  /* ram parity diagnose */
        .failSafeFunc       = RAM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRam,
        .moment             = MOMENT_RUNTIME
    },
    {
        .diagnoseFunc       = RAM_DiagnoseStackOverflow,  /* ram stack overflow diagnose */
        .failSafeFunc       = RAM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRam,
        .moment             = MOMENT_RUNTIME
    },
#if defined(CRC)
    {
        .handleConfigFunc   = ROM_DiagnoseIntegrityInit,  /* rom intergrity diagnose */
        .diagnoseFunc       = ROM_DiagnoseIntegrity,
        .failSafeFunc       = ROM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRom,
        .moment             = MOMENT_RUNTIME
    },
#endif
#if defined(EFC)
    {
        .handleConfigFunc   = ROM_DiagnoseFlashEccInit,  /* flash ecc diagnose */
        .diagnoseFunc       = ROM_DiagnoseFlashEcc,
        .failSafeFunc       = ROM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRom,
        .moment             = MOMENT_RUNTIME
    },
#endif
};
/* runtime function safety iterm size */
const unsigned int RUNTIME_PROCESS_ARRAY_SIZE = sizeof(g_fuSaRuntimeProcess) / sizeof(g_fuSaRuntimeProcess[0]);

/**
  * @brief Excute run time function safety iterm.
  * @param None.
  * @retval None.
  */
void FunctionSafetyRuntimeProcess(void)
{
    FunctionSafetyState state = {0};
    ProcessTimeCycle processTime = {0};
    processTime.timeCycleRecordSwitch = true;
    /* excute runtime function safety iterm one by one */
    for (unsigned int i = 0; i < RUNTIME_PROCESS_ARRAY_SIZE; ++i) {
        state = FunctionSafetyProcess(&g_fuSaRuntimeProcess[i], &processTime);
        if (state.BIT.result == RESULT_FAIL) {
            TriggerSoftIrqSendFailEvent(); /* trigger soft interrupt to notify application */
        }
        if (processTime.timeCycleRecordSwitch) {
            FunctionSafetyProcessPrint(state, processTime);
        }
    }
}