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
  * @file      diagnose_mcu_dio.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for dio diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_dio.h"

#define GPIO_LEVEL_HOLD_TIME_US        20

#ifdef GPIO0
/**
  * @brief diagnose gpio's direction and level.
  * @param dioDiagnoseHandle Value of @ref DIO_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState DIO_DiagnoseGpioDirectionAndLevel(void* dioDiagnoseHandle, DiagnoseMoment moment)
{
    DIO_DiagnoseHandle* handle = (DIO_DiagnoseHandle*)dioDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_DIGTAL_IO);
    STATE_SetDiagnoseMoudule(&state, MODULE_GPIO);
    STATE_SetDiagnoseFeature(&state, FEATURE_GPIO_BASIC);  /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->gpioHandleRef != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->gpioHandleTarget != NULL, &state);
    HAL_GPIO_SetDirection(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetDirection(handle->gpioHandleTarget, handle->gpioHandleTarget->pins, GPIO_INPUT_MODE);
    BASE_FUNC_DELAY_US(10);    /* 10: delay time 10us */
    unsigned int pinsValue = HAL_GPIO_GetAllValue(handle->gpioHandleTarget);
    if (pinsValue != handle->gpioHandleRef->pins) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_DIR_INPUT);  /* set fail result and fault type to state */
        return state;
    }
    HAL_GPIO_SetDirection(handle->gpioHandleTarget, handle->gpioHandleTarget->pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(handle->gpioHandleTarget, handle->gpioHandleTarget->pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetDirection(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_INPUT_MODE);
    BASE_FUNC_DELAY_US(10);     /* 10: delay time 10us */
    pinsValue = HAL_GPIO_GetAllValue(handle->gpioHandleRef);
    if (pinsValue != handle->gpioHandleTarget->pins) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_DIR_OUTPUT);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
/**
  * @brief check gpio's interrupt source.
  * @param dioDiagnoseHandle Value of @ref DIO_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState DIO_GpioIrqCheckCallback(void* dioDiagnoseHandle, DiagnoseMoment moment)
{
    DIO_DiagnoseHandle* handle = (DIO_DiagnoseHandle*)dioDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check */
    handle->irqFlag = true;
    /* Queries the masked GPIO interrupt status. */
    unsigned int mis = DCL_GPIO_GetMIS(handle->gpioHandleTarget->baseAddress);
    if (mis != handle->gpioHandleTarget->pins) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_INT_PINS_UNMATCH);  /* set fail result and fault type to state */
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
/**
  * @brief gpio's interrupt triger config.
  * @param dioDiagnoseHandle Value of @ref DIO_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
static void DIO_GpioIrqTrigerConfig(DIO_DiagnoseHandle* handle, GPIO_InterruptMode mode)
{
    HAL_GPIO_SetIrqType(handle->gpioHandleTarget, handle->gpioHandleRef->pins, mode); /* set gpio interrupt type */
    if (mode == GPIO_INT_TYPE_FALL_EDGE) { /* fall edge interrupt type testcase config */
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_HIGH_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_LOW_LEVEL);
    } else if (mode == GPIO_INT_TYPE_RISE_EDGE) { /* rise edge interrupt type testcase config */
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_LOW_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_HIGH_LEVEL);
    } else if (mode == GPIO_INT_TYPE_LOW_LEVEL) { /* low level interrupt type testcase config */
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_HIGH_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_LOW_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
    } else if (mode == GPIO_INT_TYPE_HIGH_LEVEL) { /* high level interrupt type testcase config */
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_LOW_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_HIGH_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
    } else if (mode == GPIO_INT_TYPE_BOTH_EDGE) { /* both edge interrupt type testcase config */
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_LOW_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_HIGH_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
        HAL_GPIO_SetValue(handle->gpioHandleRef, handle->gpioHandleRef->pins, GPIO_LOW_LEVEL);
        BASE_FUNC_DELAY_US(GPIO_LEVEL_HOLD_TIME_US);
    }
    DCL_GPIO_DisableIrq(handle->gpioHandleTarget->baseAddress, handle->gpioHandleTarget->pins); /* disable gpio irq */
}
/**
  * @brief diagnose gpio's interrupt.
  * @param dioDiagnoseHandle Value of @ref DIO_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState DIO_DiagnoseGpioInterrupt(void* dioDiagnoseHandle, DiagnoseMoment moment)
{
    DIO_DiagnoseHandle* handle = (DIO_DiagnoseHandle*)dioDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_DIGTAL_IO);
    STATE_SetDiagnoseMoudule(&state, MODULE_GPIO);
    STATE_SetDiagnoseFeature(&state, FEATURE_GPIO_INTERRUPT);  /* set diagnose subsys, module and feature to state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->gpioHandleRef != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->gpioHandleTarget != NULL, &state); /* param check */
    DIO_GpioIrqTrigerConfig(handle, GPIO_INT_TYPE_FALL_EDGE);  /* triggle fall edge interrupt */
    if (handle->irqFlag != true) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_INT_FALL_NACK);
        return state;
    }
    handle->irqFlag = false;
    DIO_GpioIrqTrigerConfig(handle, GPIO_INT_TYPE_RISE_EDGE);  /* triggle rise edge interrupt */
    if (handle->irqFlag != true) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_INT_RISE_NACK);
        return state;
    }
    handle->irqFlag = false;
    DIO_GpioIrqTrigerConfig(handle, GPIO_INT_TYPE_LOW_LEVEL);  /* triggle low level interrupt */
    if (handle->irqFlag != true) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_INT_LOW_LEVEL_NACK);
        return state;
    }
    handle->irqFlag = false;
    DIO_GpioIrqTrigerConfig(handle, GPIO_INT_TYPE_HIGH_LEVEL);  /* triggle high level interrupt */
    if (handle->irqFlag != true) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_INT_HIGH_LEVEL_NACK);
        return state;
    }
    handle->irqFlag = false;
    DIO_GpioIrqTrigerConfig(handle, GPIO_INT_TYPE_BOTH_EDGE);  /* triggle both edge interrupt */
    if (handle->irqFlag != true) {
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_GPIO_INT_BOTH_NACK);
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);   /* set result success to state */
    return state;
}
#endif