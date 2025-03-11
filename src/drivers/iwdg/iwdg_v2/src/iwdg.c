/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      iwdg.c
  * @author    MCU Driver Team
  * @brief     IWDG module driver
  * @details   This file provides firmware functions to manage the following functionalities of the IWDG and IWDG.
  *             + Initialization functions.
  *             + IWDG Set And Get Functions.
  *             + Interrupt Handler Functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "iwdg.h"
/* Macro definitions ---------------------------------------------------------*/
#define IWDG_LOAD_VALUE_LIMIT     4095
#define IWDG_WINDOW_VALUE_UPPER_LIMIT  4095
static unsigned int IWDG_CalculateRegTimeout(IWDG_RegStruct *baseAddress, float timeValue, IWDG_TimeType timeType);

/**
  * @brief Initializing IWDG or IWDG register values
  * @param handle Value of @ref IWDG_handle.
  * @retval BASE_StatusType: OK, ERROR
  */
BASE_StatusType HAL_IWDG_Init(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    IWDG_PARAM_CHECK_WITH_RET(IsIwdgTimeType(handle->timeType), BASE_STATUS_ERROR);
    IWDG_PARAM_CHECK_WITH_RET(IsIwdgTimeValue(handle->baseAddress, handle->timeValue, handle->timeType),
        BASE_STATUS_ERROR);
    HAL_IWDG_Start(handle); /* Enable IWDG */

    /* IWDG frequency division value less than 1024 */
    unsigned int freqDivVal = (handle->freqDivValue > IWDG_FREQ_DIV_1024) ? IWDG_FREQ_DIV_1024 : handle->freqDivValue;
    DCL_IWDG_SetFreqDivValue(handle->baseAddress, freqDivVal);

    /* Set time value */
    HAL_IWDG_SetTimeValue(handle, handle->timeValue, handle->timeType);

    /* Enable IWDG interrupt */
    if (handle->enableIT) {
        DCL_IWDG_EnableInterrupt(handle->baseAddress);
    } else {
        DCL_IWDG_DisableInterrupt(handle->baseAddress);
    }
    DCL_IWDG_EnableReset(handle->baseAddress); /* enable reset */

    /* Wait for register to be updated */
    while (handle->baseAddress->IWDG_STATUS.reg != 0x00) {
        ;
    }
    unsigned int value = IWDG_CalculateRegTimeout(handle->baseAddress, handle->handleEx.windowValue, handle->timeType);
    unsigned int freqDiv = DCL_IWDG_GetFreqDivValue(handle->baseAddress);
    value = (value / (1 << (freqDiv + 2))); /* 1 << (freqDiv + 2) is freq div value */
    /* The upper limit of the window value is determined. */
    value = (value <= IWDG_WINDOW_VALUE_UPPER_LIMIT) ? value : IWDG_WINDOW_VALUE_UPPER_LIMIT;
    /* window value only could be set litter than load value */
    value = (value < handle->baseAddress->IWDG_LOAD.BIT.iwdg_load) ? value :
            handle->baseAddress->IWDG_LOAD.BIT.iwdg_load;

    if (handle->baseAddress->IWDG_WINDOW.BIT.iwdg_window != value) {
        DCL_IWDG_SetWindowValue(handle->baseAddress, value);
    } else {
        DCL_IWDG_Refresh(handle->baseAddress);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Calculate Reg Timeout.
  * @param timeValue Value to be load to iwdg.
  * @param timeType Value of @ref IWDG_TimeType.
  * @retval unsigned int timeout Value.
  */
static unsigned int IWDG_CalculateRegTimeout(IWDG_RegStruct *baseAddress, float timeValue, IWDG_TimeType timeType)
{
    float clockFreq = (float)HAL_CRG_GetIpFreq((void *)baseAddress);
    unsigned int timeoutValue = 0x00000000U;
    switch (timeType) {
        case IWDG_TIME_UNIT_TICK: /* If the time type is tick, calculate the timeout value. */
            timeoutValue = (unsigned int)timeValue;
            break;
        case IWDG_TIME_UNIT_S: /* If the time type is s, calculate the timeout value. */
            timeoutValue = (unsigned int)(timeValue * clockFreq);
            break;
        case IWDG_TIME_UNIT_MS: /* If the time type is ms, calculate the timeout value. */
            timeoutValue = (unsigned int)(timeValue * clockFreq / FREQ_CONVERT_MS_UNIT);
            break;
        case IWDG_TIME_UNIT_US: /* If the time type is us, calculate the timeout value. */
            timeoutValue = (unsigned int)(timeValue * clockFreq / FREQ_CONVERT_US_UNIT);
            break;
        default:
            break;
    }
    return timeoutValue;
}

/**
  * @brief Setting the load value of the IWDG counter.
  * @param handle Value of @ref IWDG_handle.
  * @param timeValue time value.
  * @param timeType IWDG time type.
  * @retval None.
  */
void HAL_IWDG_SetTimeValue(IWDG_Handle *handle, unsigned int timeValue, IWDG_TimeType timeType)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    IWDG_PARAM_CHECK_NO_RET(IsIwdgTimeType(timeType));
    IWDG_PARAM_CHECK_NO_RET(IsIwdgTimeValue(handle->baseAddress, timeValue, timeType));
    unsigned int value = IWDG_CalculateRegTimeout(handle->baseAddress, timeValue, timeType);
    unsigned int freqDiv = DCL_IWDG_GetFreqDivValue(handle->baseAddress);
    value = (value / (1 << (freqDiv + 2))); /* 1 << (freqDiv + 2) is freq div value */
    /* The upper limit of the loaded value is determined. */
    value = (value <= IWDG_LOAD_VALUE_LIMIT) ? value : IWDG_LOAD_VALUE_LIMIT;
    DCL_IWDG_SetLoadValue(handle->baseAddress, value);
}

/**
  * @brief refresh the IWDG counter.
  * @param handle Value of @ref IWDG_handle.
  * @retval None.
  */
void HAL_IWDG_Refresh(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    DCL_IWDG_Refresh(handle->baseAddress);
}

/**
  * @brief obtain the IWDG load value.
  * @param handle Value of @ref IWDG_handle.
  * @retval unsigned int Load value.
  */
unsigned int HAL_IWDG_GetLoadValue(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    return DCL_IWDG_GetLoadValue(handle->baseAddress);
}

/**
  * @brief Refresh the IWDG counter value.
  * @param handle Value of @ref IWDG_handle.
  * @retval unsigned int Counter value.
  */
unsigned int HAL_IWDG_GetCounterValue(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));

    float res = (float)handle->baseAddress->IWDG_VALUE.BIT.iwdg_value;
    if (res >= 4095) { /* 4095 is IWDG maximum current count */
        return handle->timeValue;
    }
    unsigned int freq = HAL_CRG_GetIpFreq((void *)handle->baseAddress);
    /* check clockFreq not equal zero */
    if (freq == 0) {
        return 0;
    }
    unsigned int freqDiv = DCL_IWDG_GetFreqDivValue(handle->baseAddress);
    switch (handle->timeType) {
        case IWDG_TIME_UNIT_TICK: /* Number of tick currently calculated */
            res = res * (1 << (freqDiv + 2)); /* 1 << (freqDiv + 2) is freq div value */
            break;
        case IWDG_TIME_UNIT_S:
            /* Number of seconds currently calculated */
            res = res * (1 << (freqDiv + 2)) / freq; /* 1 << (freqDiv + 2) is freq div value */
            break;
        case IWDG_TIME_UNIT_MS:
            /* 1 << (freqDiv + 2) is freq div value */
            res = res * (1 << (freqDiv + 2)) * FREQ_CONVERT_MS_UNIT / freq;
            break;
        case IWDG_TIME_UNIT_US:
            /* Number of seconds currently calculated, 1 << (freqDiv + 2) is freq div value */
            res = res * (1 << (freqDiv + 2)) * FREQ_CONVERT_US_UNIT / freq;
            break;
        default:
            break;
    }
    return (unsigned int)res; /* return current counter value */
}

/**
  * @brief Start the IWDG count.
  * @param handle Value of @ref IWDG_handle.
  * @retval None.
  */
void HAL_IWDG_Start(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    DCL_IWDG_Start(handle->baseAddress);
}

/**
  * @brief Stop the IWDG count.
  * @param handle Value of @ref IWDG_handle.
  * @retval None.
  */
void HAL_IWDG_Stop(IWDG_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief   Register IWDG interrupt callback.
  * @param   handle Value of @ref IWDG_handle.
  * @param   callBackFunc Value of @ref IWDG_CallbackType.
  * @retval  None
  */
void HAL_IWDG_RegisterCallback(IWDG_Handle *handle, IWDG_CallbackType callBackFunc)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    if (callBackFunc != NULL) {
        /* Invoke the callback function. */
        handle->userCallBack.CallbackFunc = callBackFunc;
    }
}

/**
  * @brief Interrupt handler processing function.
  * @param handle IWDG_Handle.
  * @retval None.
  */
void HAL_IWDG_IrqHandler(void *handle)
{
    IWDG_Handle *iwdgHandle = (IWDG_Handle *)handle;
    IWDG_ASSERT_PARAM(iwdgHandle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(iwdgHandle->baseAddress));

    if (iwdgHandle->baseAddress->IWDG_MIS.BIT.iwdgmis == 0x01) { /* Interrupt flag is set, fed dog in callback */
        if (iwdgHandle->userCallBack.CallbackFunc) {
            iwdgHandle->userCallBack.CallbackFunc(iwdgHandle);
        }
    }
}
