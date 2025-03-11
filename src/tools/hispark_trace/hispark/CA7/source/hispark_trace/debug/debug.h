/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
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
  * @file    debug.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of DEBUG module.
  *          + Initialization and de-initialization functions
  *          + Format print function
  */

/* Define to prevent recursive inclusion --------------------------------------------*/
#ifndef MCU_MAGIC_TAG_DEBUG_H
#define MCU_MAGIC_TAG_DEBUG_H

/* Includes -------------------------------------------------------------------------*/
#include "uart.h"

/* Macro definitions ----------------------------------------------------------------*/
#ifdef DEBUG_PARAM_CHECK
#define DEBUG_ASSERT_PARAM BSP_ASSERT_PARAM
#else
#define DEBUG_ASSERT_PARAM(para) ((void)0U)
#endif

typedef enum {
    BSP_OK      = 0x00000000U,
    BSP_ERROR   = 0x00000001U,
    BSP_BUSY    = 0x00000002U,
    BSP_TIMEOUT = 0x00000003U
} BSP_StatusType;

/* Macro definitions for enabling the function of DEBUG_PRINT submodule */
#define DBG_USE_NO_PRINTF     1U
#define DBG_USE_CUSTOM_PRINTF 2U

#define DBG_PRINTF_USE  DBG_USE_NO_PRINTF /**< Select the format print function */
#define DBG_UART_PORT   UART4 /**< Select the UART PORT used for format print */

#if (DBG_PRINTF_USE == DBG_USE_NO_PRINTF)
static inline int DBG_Dummy(const char *format, ...)
{
    return 0;
}
#define DBG_PRINTF DBG_Dummy /**< Delete all print statement */
#endif

#if (DBG_PRINTF_USE == DBG_USE_CUSTOM_PRINTF)
#define DBG_PRINTF_UART_PORT  DBG_UART_PORT
#define DBG_PRINTF DBG_UartPrintf /**< Select the customized printf function */
#endif

/* Initialization and de-initialization functions of DEBUG module ------------------*/
BSP_StatusType DBG_UartPrintInit(unsigned int baudRate);
BSP_StatusType DBG_UartPrintDeInit(void);

/* Format print function -----------------------------------------------------------*/
int DBG_UartPrintf(const char *format, ...); /* Supported format: %c, %s, %d, %u, %x, %X, %p, %f */

#endif /* MCU_MAGIC_TAG_DEBUG_H */
