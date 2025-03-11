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
  * @file      diagnose_mcu_ram.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for ram diagnose.
  */

#ifndef DIAGNOSE_MCU_RAM_H
#define DIAGNOSE_MCU_RAM_H

#include "function_safety_common.h"

#define MODULE_RAM_SRAM                 0x01
#define MODULE_RAM_STACK                0x02

#define FEATURE_SRAM_MARCH              0x01
#define FEATURE_SRAM_PARITY             0x02
#define FEATURE_STACK_OVERFLOW_CHECK    0x03

#define FAULT_DATA_NOT_INTEGRITY        0x01
#define FAULT_SRAM_STEP_RANGE           0x02
#define FAULT_SRAM_FULL_RANGE           0x03
#define FAULT_SRAM_PARITY_ERROR         0x04
#define FAULT_STACK_OVERFLOW            0x05

typedef struct {
    unsigned int*  sramStartAddr;
    unsigned int*  sramCurrentAddr;
    unsigned int*  sramCurrentAddrInv;
    unsigned int*  sramEndAddr;
    unsigned int   sramPattern;
} RAM_DiagnoseHandle;

FunctionSafetyState RAM_DiagnoseSramByMarchAlgorithm(void* ramDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState RAM_DiagnoseSramByParityCheck(void* ramDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState RAM_DiagnoseStackOverflow(void* ramDiagnoseHandle, DiagnoseMoment moment);

#endif