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
  * @file      diagnose_mcu_dio.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for dio diagnose.
  */

#ifndef DIAGNOSE_MCU_DIO_H
#define DIAGNOSE_MCU_DIO_H

#include "function_safety_common.h"
#ifdef GPIO0
#include "gpio.h"
#endif
#define MODULE_GPIO                    0x01

#define FEATURE_GPIO_BASIC             0x01
#define FEATURE_GPIO_INTERRUPT         0x02

#define FAULT_GPIO_DIR_INPUT           0x01
#define FAULT_GPIO_DIR_OUTPUT          0x02
#define FAULT_GPIO_LEVEL_HIGH          0x03
#define FAULT_GPIO_LEVEL_LOW           0x04
#define FAULT_GPIO_INT_RISE_NACK       0x05
#define FAULT_GPIO_INT_FALL_NACK       0x06
#define FAULT_GPIO_INT_LOW_LEVEL_NACK  0x07
#define FAULT_GPIO_INT_HIGH_LEVEL_NACK 0x08
#define FAULT_GPIO_INT_BOTH_NACK       0x09
#define FAULT_GPIO_INT_PINS_UNMATCH    0x0A

typedef struct {
#ifdef GPIO0
    GPIO_Handle*    gpioHandleRef;
    GPIO_Handle*    gpioHandleTarget;
#endif
    bool            irqFlag;
} DIO_DiagnoseHandle;

#ifdef GPIO0
FunctionSafetyState DIO_DiagnoseGpioDirectionAndLevel(void* dioDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState DIO_DiagnoseGpioInterrupt(void* dioDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState DIO_GpioIrqCheckCallback(void* dioDiagnoseHandle, DiagnoseMoment moment);
#endif

#endif