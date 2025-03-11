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
  * @file      diagnose_mcu_monitor.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for monitor diagnose.
  */

#ifndef DIAGNOSE_MCU_MONITOR_H
#define DIAGNOSE_MCU_MONITOR_H

#include "function_safety_common.h"
#ifdef CRG
#include "crg.h"
#endif
#ifdef WDG
#include "wdg.h"
#endif
#ifdef WWDG
#include "wwdg.h"
#endif
#ifdef IWDG
#include "iwdg.h"
#endif
#ifdef PMC
#include "pmc.h"
#endif
#ifdef TSENSOR
#include "tsensor.h"
#endif
#ifdef SYSCTRL0
#include "sysctrl.h"
#endif

#define MODULE_WDG                0x01
#define MODULE_IWDG               0x02
#define MODULE_WWDG               0x03
#define MODULE_PMC                0x04
#define MODULE_TSENSOR            0x05

#define FEATURE_MONITOR_PROGRAM_STUCK        0x01
#define FEATURE_MONITOR_WDG_RESET            0x02
#define FEATURE_MONITOR_POWER_LOW_LEVEL      0x03
#define FEATURE_MONITOR_CHIP_OVER_TEMPERATUR 0x04

#define FAULT_WDG_PROGRAM_STUCK_RESET        0x01
#define FAULT_WDG_UNRESET                    0x02
#define FAULT_PMC_POWER_LOW_LEVEL            0x03
#define FAULT_TSENSOR_CHIP_OVER_TEMPERATUR   0x04

typedef struct {
#ifdef WDG
    WDG_Handle*     wdgHandle;
#endif
#ifdef WWDG
    WWDG_Handle*    wwdgHandle;
#endif
#ifdef IWDG
    IWDG_Handle*    iwdgHandle;
#endif
#ifdef PMC
    PMC_Handle*     pmcHandle;
#endif
    unsigned int    startupResetTimeUs;
    unsigned int    runTimeResetTimeUs;
    unsigned int    overTemperatureValue;
    bool            pmcIrqFlag;
} MONITOR_DiagnoseHandle;

#if defined(WDG) || defined(WWDG)
FunctionSafetyState MONITOR_DiagnoseProgramStuckByWdg(void* monitorDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState MONITOR_DiagnoseWdgResetFunction(void* monitorDiagnoseHandle, DiagnoseMoment moment);
#endif
#if defined(IWDG)
FunctionSafetyState MONITOR_DiagnoseProgramStuckByIwdg(void* monitorDiagnoseHandle, DiagnoseMoment moment);
FunctionSafetyState MONITOR_DiagnoseIwdgResetFunction(void* monitorDiagnoseHandle, DiagnoseMoment moment);
#endif
#if defined(PMC)
FunctionSafetyState MONITOR_DiagnosePowerLowLevelByPmc(void* monitorDiagnoseHandle, DiagnoseMoment moment);
#endif
#if defined(TSENSOR)
FunctionSafetyState MONITOR_DiagnoseOverTemperatureByTsensor(void* monitorDiagnoseHandle, DiagnoseMoment moment);
#endif

#endif