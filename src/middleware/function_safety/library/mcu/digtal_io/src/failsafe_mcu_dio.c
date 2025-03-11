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
  * @file      failsafe_mcu_dio.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for dio failsafe.
  */
#include "diagnose_mcu_dio.h"
#include "failsafe_mcu_dio.h"

/**
  * @brief default failsafe measure of gpio.
  * @param dioFailSafeHandle Value of @ref DIO_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
__weak FunctionSafetyState DIO_DefaultFailSafe(void* dioFailSafeHandle, DiagnoseMoment moment)
{
    DIO_FailSafeHandle* handle = (DIO_FailSafeHandle*)dioFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    while (1) {
    }
    return state;
}
/**
  * @brief failsafe measure of gpio direction and level.
  * @param dioFailSafeHandle Value of @ref DIO_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState DIO_DirectionLevelFailSafe(void* dioFailSafeHandle, DiagnoseMoment moment)
{
    DIO_FailSafeHandle* handle = (DIO_FailSafeHandle*)dioFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure of gpio interrupt.
  * @param dioFailSafeHandle Value of @ref DIO_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState DIO_InterruptFailSafe(void* dioFailSafeHandle, DiagnoseMoment moment)
{
    DIO_FailSafeHandle* handle = (DIO_FailSafeHandle*)dioFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent(); /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    BASE_FUNC_UNUSED(moment);
    BASE_FUNC_UNUSED(handle);
    return state;
}
/**
  * @brief failsafe measure handler of gpio.
  * @param dioFailSafeHandle Value of @ref DIO_FailSafeHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState DIO_FailSafeHandler(void* dioFailSafeHandle, DiagnoseMoment moment)
{
    DIO_FailSafeHandle* handle = (DIO_FailSafeHandle*)dioFailSafeHandle;
    FunctionSafetyState state = STATE_LoadFailEvent();  /* load fail event */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state); /* param check not NULL */
    switch (state.BIT.faultType) {
        case FAULT_GPIO_DIR_INPUT:
        case FAULT_GPIO_DIR_OUTPUT:
        case FAULT_GPIO_LEVEL_HIGH:
        case FAULT_GPIO_LEVEL_LOW:
            state = DIO_DirectionLevelFailSafe(handle, moment);  /* gpio direction and level fail safe */
            break;
        case FAULT_GPIO_INT_RISE_NACK:
        case FAULT_GPIO_INT_FALL_NACK:
        case FAULT_GPIO_INT_LOW_LEVEL_NACK:
        case FAULT_GPIO_INT_HIGH_LEVEL_NACK:
        case FAULT_GPIO_INT_BOTH_NACK:
        case FAULT_GPIO_INT_PINS_UNMATCH:
            state = DIO_InterruptFailSafe(handle, moment);  /* gpio interrupt fail safe */
            break;
        default:
            state = DIO_DefaultFailSafe(handle, moment);  /* gpio default fail safe */
            break;
    }
    return state;
}