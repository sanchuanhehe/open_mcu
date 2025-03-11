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
  * @file      diagnose_mcu_core.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for core diagnose.
  */

#ifndef DIAGNOSE_MCU_CORE_H
#define DIAGNOSE_MCU_CORE_H

#include "function_safety_common.h"

#define MODULE_CPU_REGISTER             0x01
#define MODULE_PC_REGISTER              0x02
#define MODULE_CPU_SOFT_INTTERRUPT      0x03

#define FEATURE_REG_WRITE_READ          0x01
#define FEATURE_REG_PC_JUMP             0x02
#define FEATURE_SYSCTRL_CPU_STATUS      0x03
#define FEATURE_SOFT_INTERRUPT          0x04

#define FAULT_REG_SAF                   0x01
#define FAULT_REG_AF                    0x02
#define FAULT_REG_TF                    0x03
#define FAULT_REG_CF                    0x04
#define FAULT_PC_JUMP                   0x05
#define FAULT_PC_INVALID                0x06
#define FAULT_SYSCTRL_CPU_STATUS        0x07
#define FAULT_CPU_SOFT_INTTERRUPT       0x08

typedef struct {
    void* param;
} CORE_DiagnoseHandle;

FunctionSafetyState CORE_DiagnoseCpuSoftwareIrq(void* coreDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState CORE_DiagnoseCpuStatus(void* coreDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState CORE_DiagnoseCpuGeneralRegister(void* coreDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState CORE_DiagnosePcRegister(void* coreDiagnoseHandle, DiagnoseMoment moment);

#endif