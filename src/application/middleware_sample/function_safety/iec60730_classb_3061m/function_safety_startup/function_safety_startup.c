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
  * @file      function_safety_startup.c
  * @author    MCU Driver Team
  * @brief     function safety startup module.
  * @details   function safety startup interface implementation.
  */
#include "main.h"
#include "function_safety_init.h"
#include "function_safety_process.h"
#include "function_safety_report.h"
#include "function_safety_startup.h"

FunctionSafetyProcessHandle g_fuSaStartupProcess[] = {
    {
        .diagnoseFunc       = CORE_DiagnoseCpuStatus, /* cpu status diagnose */
        .failSafeFunc       = CORE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCore,
        .moment             = MOMENT_STARTUP
    },
    {
        .diagnoseFunc       = CORE_DiagnoseCpuGeneralRegister, /* cpu general register diagnose */
        .failSafeFunc       = CORE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCore,
        .moment             = MOMENT_STARTUP
    },
    {
        .diagnoseFunc       = CORE_DiagnoseCpuSoftwareIrq, /* cpu soft irq diagnose */
        .failSafeFunc       = CORE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCore,
        .moment             = MOMENT_STARTUP
    },
    {
        .diagnoseFunc       = CORE_DiagnosePcRegister, /* cpu pc register diagnose */
        .failSafeFunc       = CORE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCore,
        .moment             = MOMENT_STARTUP
    },
    {
        .handleConfigFunc   = RAM_DiagnoseStartupInit,  /* ram marchx diagnose */
        .diagnoseFunc       = RAM_DiagnoseSramByMarchAlgorithm,
        .failSafeFunc       = RAM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRam,
        .moment             = MOMENT_STARTUP
    },
    {
        .diagnoseFunc       = RAM_DiagnoseSramByParityCheck,  /* ram parity diagnose */
        .failSafeFunc       = RAM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRam,
        .moment             = MOMENT_STARTUP
    },
    {
        .diagnoseFunc       = RAM_DiagnoseStackOverflow,  /* ram stack overflow diagnose */
        .failSafeFunc       = RAM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRam,
        .moment             = MOMENT_STARTUP
    },
    {
        .handleConfigFunc   = CLOCK_SingleDiagnoseSysClockAccuracyInit, /* systerm clock single diagnose */
        .diagnoseFunc       = CLOCK_CmmDiagnoseAccuracy,
        .failSafeFunc       = CLOCK_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseClock,
        .moment             = MOMENT_IRQ
    },
    {
        .handleConfigFunc   = CLOCK_SingleDiagnoseHoscCLockAccuracyInit, /* hosc clock single diagnose */
        .diagnoseFunc       = CLOCK_CmmDiagnoseAccuracy,
        .failSafeFunc       = CLOCK_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseClock,
        .moment             = MOMENT_IRQ
    },
    {
        .handleConfigFunc   = CLOCK_SingleDiagnoseLoscCLockAccuracyInit, /* losc clock single diagnose */
        .diagnoseFunc       = CLOCK_CmmDiagnoseAccuracy,
        .failSafeFunc       = CLOCK_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseClock,
        .moment             = MOMENT_IRQ
    },
    {
        .handleConfigFunc   = CLOCK_ContinueDiagnoseSysClockAccuracyInit,  /* systerm clock continue diagnose */
        .diagnoseFunc       = CLOCK_CmmDiagnoseAccuracy,
        .failSafeFunc       = CLOCK_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseClock,
        .moment             = MOMENT_IRQ
    },
    {
        .handleConfigFunc   = COMPUTE_DiagnoseCrcInit, /* crc calculate correct diagnose */
        .diagnoseFunc       = COMPUTE_DiagnoseCrcAlgorithmCorrect,
        .failSafeFunc       = COMPUTE_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseCompute,
        .moment             = MOMENT_STARTUP
    },
    {
        .handleConfigFunc   = ROM_DiagnoseIntegrityInit, /* rom integrity diagnose */
        .diagnoseFunc       = ROM_DiagnoseIntegrity,
        .failSafeFunc       = ROM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRom,
        .moment             = MOMENT_STARTUP
    },
    {
        .handleConfigFunc   = ROM_DiagnoseFlashEccInit, /* rom flash ecc diagnose */
        .diagnoseFunc       = ROM_DiagnoseFlashEcc,
        .failSafeFunc       = ROM_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseRom,
        .moment             = MOMENT_STARTUP
    },
    {
        .handleConfigFunc   = TIMERS_DiagnoseTimer0InterruptIntervalInit,  /* timer0 interrupt interval diagnose */
        .diagnoseFunc       = TIMERS_DiagnoseInterruptIntervalAccuracy,
        .failSafeFunc       = TIMERS_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseTimers,
        .moment             = MOMENT_IRQ
    },
    {
        .handleConfigFunc   = TIMERS_DiagnoseTimer1InterruptIntervalInit,  /* timer1 interrupt interval diagnose */
        .diagnoseFunc       = TIMERS_DiagnoseInterruptIntervalAccuracy,
        .failSafeFunc       = TIMERS_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseTimers,
        .moment             = MOMENT_IRQ
    },
    {
        .handleConfigFunc   = TIMERS_DiagnoseTimer2InterruptIntervalInit,  /* timer2 interrupt interval diagnose */
        .diagnoseFunc       = TIMERS_DiagnoseInterruptIntervalAccuracy,
        .failSafeFunc       = TIMERS_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseTimers,
        .moment             = MOMENT_IRQ
    },
    {
        .handleConfigFunc   = ANA_DiagnoseAdc0AccuracyInit,  /* adc0 accuracy diagnose */
        .diagnoseFunc       = ANA_DiagnoseAdcSampleAccuracy,
        .failSafeFunc       = ANA_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseAna,
        .moment             = MOMENT_STARTUP
    },
    {
        .handleConfigFunc   = MONITOR_DiagnosePmcLowPowerInit, /* pmc low power diagnose */
        .diagnoseFunc       = MONITOR_DiagnosePowerLowLevelByPmc,
        .failSafeFunc       = MONITOR_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseMonitor,
        .moment             = MOMENT_ALLTIME
    },
    {
        .handleConfigFunc   = MONITOR_DiagnoseTsensorOverTemperatureInit, /* tsensor over temperature diagnose */
        .diagnoseFunc       = MONITOR_DiagnoseOverTemperatureByTsensor,
        .failSafeFunc       = MONITOR_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseMonitor,
        .moment             = MOMENT_ALLTIME
    },
    {
        .handleConfigFunc   = MONITOR_DiagnoseWdgProgramStuckInit, /* wdg program stuck diagnose */
        .diagnoseFunc       = MONITOR_DiagnoseProgramStuckByWdg,
        .failSafeFunc       = MONITOR_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseMonitor,
        .moment             = MOMENT_ALLTIME
    },
    {
        .handleConfigFunc   = MONITOR_DiagnoseIwdgProgramStuckInit, /* iwdg program stuck diagnose */
        .diagnoseFunc       = MONITOR_DiagnoseProgramStuckByIwdg,
        .failSafeFunc       = MONITOR_FailSafeHandler,
        .diagnoseHandle     = &g_diagnoseMonitor,
        .moment             = MOMENT_ALLTIME
    }
};
/* startup function safety iterm size */
const unsigned int STARTUP_PROCESS_ARRAY_SIZE = sizeof(g_fuSaStartupProcess) / sizeof(g_fuSaStartupProcess[0]);

/**
  * @brief Excute startup function safety iterm.
  * @param None.
  * @retval None.
  */
void FunctionSafetyStartupProcess(void)
{
    FunctionSafetyState state = {0};
    ProcessTimeCycle processTime = {0};
    processTime.timeCycleRecordSwitch = true;
    /* excute startup function safety iterm one by one */
    for (unsigned int i = 0; i < STARTUP_PROCESS_ARRAY_SIZE; ++i) {
        state = FunctionSafetyProcess(&g_fuSaStartupProcess[i], &processTime);
        if (state.BIT.moudule == MODULE_CPU_SOFT_INTTERRUPT && state.BIT.subsysterm == SUBSYS_CORE) {
            RegisterReportFailEventHandler(ReportFailEventLogBySoftIrq, 3, NULL); /* 3: interrupt priority */
        }
        if (i >= 3 && state.BIT.result == RESULT_FAIL) { /* 3 : wait soft interrupt enable and register callback */
            TriggerSoftIrqSendFailEvent(); /* trigger soft interrupt to notify application */
        }
        if (processTime.timeCycleRecordSwitch) {
            FunctionSafetyProcessPrint(state, processTime);
        }
    }
}