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
  * @file      diagnose_mcu_clock.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for clock diagnose.
  */

#ifndef DIAGNOSE_MCU_CLOCK_H
#define DIAGNOSE_MCU_CLOCK_H

#include "function_safety_common.h"
#ifdef CRG
#include "crg.h"
#endif
#ifdef CMM
#include "cmm.h"
#endif
#ifdef CFD
#include "cfd.h"
#endif

#define MODULE_CMM                            0x01
#define MODULE_CFD                            0x02

#define FEATURE_CLOCK_ACCURACY_SINGLE_CHECK   0x01
#define FEATURE_CLOCK_ACCURACY_CONTINUE_CHECK 0x02
#define FEATURE_PLLREF_CLOCK_STOP_CHECK       0x03

#define FAULT_OVER_ACCURACY                   0x01
#define FAULT_PLLREF_CLOCK_STOP               0x02

typedef struct {
#ifdef CMM
    CMM_Handle*         cmmHandle;
    unsigned int        cmmTargetVal;
    unsigned int        cmmClockCount;
    bool                cmmIrqFlag;
#endif
#ifdef CFD
    CFD_Handle*         cfdHandle;
    unsigned int        cfdTargetVal;
    unsigned int        cfdClockCount;
    bool                cfdIrqFlag;
#endif
    unsigned int        checkEndDelayTimeUs;
    unsigned int        errRangePercent;
} CLOCK_DiagnoseHandle;

#ifdef CMM
FunctionSafetyState CLOCK_CmmWindowsBoundCalculate(void* clockDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState CLOCK_CmmDiagnoseAccuracy(void* clockDiagnoseHandle, DiagnoseMoment moment);
#endif
#ifdef CFD
FunctionSafetyState CLOCK_CfdWindowsBoundCalculate(void* clockDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState CLOCK_CfdDiagnosePllRefClockStop(void* clockDiagnoseHandle, DiagnoseMoment moment);
#endif

#endif