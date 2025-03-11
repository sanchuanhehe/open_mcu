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
  * @file      iwdg_ex.c
  * @author    MCU Driver Team
  * @brief     IWDG module driver
  * @details   This file provides firmware functions to manage the following functionalities of the IWDG and IWDG.
  *             + IWDG Set And Get Functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "pmc.h"
#include "iwdg_ex.h"
/* Macro definitions ---------------------------------------------------------*/
#define IWDG_WINDOW_VALUE_UPPER_LIMIT     4095

/**
  * @brief Getting the window value of the IWDG counter.
  * @param handle Value of @ref IWDG_handle.
  * @retval unsigned int the value of window reg value.
  */
unsigned int HAL_IWDG_GetWindowValueEx(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    IWDG_ASSERT_PARAM(IsIwdgTimeType(handle->timeType));
    /* handle->baseAddress only could be configed IWDG */
    return DCL_IWDG_GetWindowValue(handle->baseAddress);
}

/**
  * @brief Enable window mode.
  * @param handle Value of @ref IWDG_handle.
  * @retval None.
  */
void HAL_IWDG_EnableWindowModeEx(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    DCL_IWDG_EnableWindowsMode(handle->baseAddress);
}

/**
  * @brief Disable window mode.
  * @param handle Value of @ref IWDG_handle.
  * @retval None.
  */
void HAL_IWDG_DisableWindowModeEx(IWDG_Handle *handle)
{
    IWDG_ASSERT_PARAM(handle != NULL);
    IWDG_ASSERT_PARAM(IsIWDGInstance(handle->baseAddress));
    DCL_IWDG_DisableWindowsMode(handle->baseAddress);
}