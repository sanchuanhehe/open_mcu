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
  * @file      diagnose_mcu_core.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for core diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_core.h"

BASE_StatusType CPU_DiagnoseStartup(void);
BASE_StatusType CPU_DiagnoseRuntime(void);
/**
  * @brief diagnose the cpu status.
  * @param coreDiagnoseHandle Value of @ref CORE_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CORE_DiagnoseCpuStatus(void* coreDiagnoseHandle, DiagnoseMoment moment)
{
    CORE_DiagnoseHandle* handle = (CORE_DiagnoseHandle*)coreDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CORE);
    STATE_SetDiagnoseMoudule(&state, MODULE_CPU_REGISTER);
    STATE_SetDiagnoseFeature(&state, FEATURE_SYSCTRL_CPU_STATUS); /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    if (DCL_SYSCTRL_CheckCpuStatus(SYSCTRL_LOCKUP_BIT) != 0 ||
        DCL_SYSCTRL_CheckCpuStatus(SYSCTRL_HARD_FAULT_BIT) != 0) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_SYSCTRL_CPU_STATUS); /* set fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS); /* set success to state */
    return state;
}
/**
  * @brief diagnose soft interrupt in irq handler.
  * @param handle Value of @ref CORE_DiagnoseHandle.
  * @retval None.
  */
static void DiagnoseSoftwareInterrupt(void* handle)
{
    FunctionSafetyState* stateHandle = (FunctionSafetyState *)handle;
    stateHandle->BIT.result = RESULT_SUCCESS; /* set result success to state */
    DCL_SYSCTRL_ClearSoftInterrupt(); /* clear soft irq */
}
/**
  * @brief diagnose soft interrupt of cpu.
  * @param coreDiagnoseHandle Value of @ref CORE_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CORE_DiagnoseCpuSoftwareIrq(void* coreDiagnoseHandle, DiagnoseMoment moment)
{
    CORE_DiagnoseHandle* handle = (CORE_DiagnoseHandle*)coreDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CORE);
    STATE_SetDiagnoseMoudule(&state, MODULE_CPU_SOFT_INTTERRUPT);
    STATE_SetDiagnoseFeature(&state, FEATURE_SOFT_INTERRUPT); /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    IRQ_DisableN(IRQ_SOFTWARE);
    IRQ_Unregister(IRQ_SOFTWARE);
    IRQ_Register(IRQ_SOFTWARE, DiagnoseSoftwareInterrupt, &state);
    IRQ_SetPriority(IRQ_SOFTWARE, 7); /* 7: interrupt priority */
    IRQ_EnableN(IRQ_SOFTWARE);
    DCL_SYSCTRL_GenerateSoftInterrupt();
    BASE_FUNC_DELAY_US(100); /* 100: delay 100us */
    /* wait soft irq return and check */
    if (state.BIT.result != RESULT_SUCCESS) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_CPU_SOFT_INTTERRUPT);
        return state;
    }
    IRQ_DisableN(IRQ_SOFTWARE);
    IRQ_Unregister(IRQ_SOFTWARE);
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);
    return state;
}
/**
  * @brief diagnose general register of cpu.
  * @param coreDiagnoseHandle Value of @ref CORE_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CORE_DiagnoseCpuGeneralRegister(void* coreDiagnoseHandle, DiagnoseMoment moment)
{
    CORE_DiagnoseHandle* handle = (CORE_DiagnoseHandle*)coreDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CORE);
    STATE_SetDiagnoseMoudule(&state, MODULE_CPU_REGISTER);
    STATE_SetDiagnoseFeature(&state, FEATURE_REG_WRITE_READ);  /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    if (moment == MOMENT_STARTUP) {
        if (CPU_DiagnoseStartup() != BASE_STATUS_OK) {
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);
            STATE_SetDiagnoseFaultType(&state, FAULT_REG_SAF);  /* set fail result and fault type to state */
            return state;
        }
    } else if (moment == MOMENT_RUNTIME) {
        if (CPU_DiagnoseRuntime() != BASE_STATUS_OK) {
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);
            STATE_SetDiagnoseFaultType(&state, FAULT_REG_SAF);  /* set fail result and fault type to state */
            return state;
        }
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set success resul to state */
    return state;
}
/**
  * @brief diagnose pc jump function of cpu.
  * @param testValue Value of test pc fucntion.
  * @retval revert value of testValue.
  */
static unsigned int DiagnosePcJump(unsigned int testValue)
{
    testValue = (~testValue); /* change pc jump address */
    return testValue;
}
/**
  * @brief diagnose pc register of cpu.
  * @param coreDiagnoseHandle Value of @ref CORE_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CORE_DiagnosePcRegister(void* coreDiagnoseHandle, DiagnoseMoment moment)
{
    CORE_DiagnoseHandle* handle = (CORE_DiagnoseHandle*)coreDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CORE);
    STATE_SetDiagnoseMoudule(&state, MODULE_PC_REGISTER);
    STATE_SetDiagnoseFeature(&state, FEATURE_REG_PC_JUMP);  /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    if (DCL_SYSCTRL_CheckCpuStatus(SYSCTRL_PC_VALID_BIT) != BASE_CFG_SET) { /* check pc state */
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_PC_INVALID);  /* set fail result and fault type to state */
        return state;
    }
    unsigned int testValue = 0xAAAAAAAAU;   /* test pattern */
    if (~testValue != DiagnosePcJump(testValue)) { /* change pc jump address and return to origin address */
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_PC_JUMP);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);
    return state;
}