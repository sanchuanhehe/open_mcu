/**
  * @copyright Copyright (c) 2024, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      function_safety_init.h
  * @author    MCU Driver Team
  * @brief    function safety library adapter layer init.
  * @details   function safety library adapter layer init
  */

#ifndef FUNCTION_SAFETY_INIT_H
#define FUNCTION_SAFETY_INIT_H

#include "function_safety_mcu.h"
#include "function_safety_config.h"

extern CORE_DiagnoseHandle      g_diagnoseCore;
extern ANA_DiagnoseHandle       g_diagnoseAna;
extern CLOCK_DiagnoseHandle     g_diagnoseClock;
extern COMPUTE_DiagnoseHandle   g_diagnoseCompute;
extern DIO_DiagnoseHandle       g_diagnoseDio;
extern MONITOR_DiagnoseHandle   g_diagnoseMonitor;
extern RAM_DiagnoseHandle       g_diagnoseRam;
extern ROM_DiagnoseHandle       g_diagnoseRom;
extern TIMERS_DiagnoseHandle    g_diagnoseTimers;
extern CONNECT_DiagnoseHandle   g_diagnoseConnect;

void ANA_DiagnoseAdc0AccuracyInit(void);
void ANA_DiagnoseAdc1AccuracyInit(void);
void ANA_DiagnoseAdc2AccuracyInit(void);
void CLOCK_SingleDiagnoseSysClockAccuracyInit(void);
void CLOCK_SingleDiagnoseHoscCLockAccuracyInit(void);
void CLOCK_SingleDiagnoseLoscCLockAccuracyInit(void);
void CLOCK_SingleDiagnoseTcxoCLockAccuracyInit(void);
void CLOCK_ContinueDiagnoseSysClockAccuracyInit(void);
void CLOCK_ContinueDiagnoseHoscCLockAccuracyInit(void);
void CLOCK_ContinueDiagnoseLoscCLockAccuracyInit(void);
void CLOCK_ContinueDiagnoseTcxoCLockAccuracyInit(void);
void CLOCK_SingleDiagnosePllRefCLockStopInit(void);
void CLOCK_ContinueDiagnosePllRefCLockStopInit(void);
void COMPUTE_DiagnoseCrcInit(void);
void MONITOR_DiagnoseWdgResetInit(void);
void MONITOR_DiagnoseWdgProgramStuckInit(void);
void MONITOR_DiagnoseIwdgResetInit(void);
void MONITOR_DiagnoseIwdgProgramStuckInit(void);
void MONITOR_DiagnosePmcLowPowerInit(void);
void MONITOR_DiagnoseTsensorOverTemperatureInit(void);
void RAM_DiagnoseStartupInit(void);
void RAM_DiagnoseRuntimeInit(void);
void ROM_DiagnoseIntegrityInit(void);
void ROM_DiagnoseFlashEccInit(void);
void TIMERS_DiagnoseTimer0InterruptIntervalInit(void);
void TIMERS_DiagnoseTimer1InterruptIntervalInit(void);
void TIMERS_DiagnoseTimer2InterruptIntervalInit(void);

#endif