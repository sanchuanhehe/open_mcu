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
  * @file      diagnose_mcu_timers.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for timers diagnose.
  */

#ifndef DIAGNOSE_MCU_TIMERS_H
#define DIAGNOSE_MCU_TIMERS_H

#include "function_safety_common.h"
#ifdef CRG
#include "crg.h"
#endif
#ifdef TIMER0
#include "timer.h"
#endif
#ifdef APT0
#include "apt.h"
#endif
#ifdef QDM0
#include "qdm.h"
#endif
#ifdef CAPM0
#include "capm.h"
#endif
#ifdef GPT0
#include "gpt.h"
#endif

#define MODULE_TIMER          0x01
#define MODULE_APT            0x02
#define MODULE_QDM            0x03
#define MODULE_CAPM           0x04
#define MODULE_GPT            0x05

#define FEATURE_TIMER_INTERRUPT_ACCURACY          0x01
#define FEATURE_APT_INTERRUPT_ACCURACY            0x02
#define FEATURE_QDM_INTERRUPT_ACCURACY            0x03
#define FEATURE_CAPM_INTERRUPT_ACCURACY           0x04
#define FEATURE_GPT_INTERRUPT_ACCURACY            0x05

#define FAULT_INTERRUPT_ACCURACY                  0x01

typedef struct {
#ifdef TIMER0
    TIMER_Handle*    timerHandle;
#endif
#ifdef APT0
    APT_Handle*      aptHandle;
#endif
#ifdef QDM0
    QDM_Handle*      qdmHandle;
#endif
#ifdef CAPM0
    CAPM_Handle*     capmHandle;
#endif
#ifdef GPT0
    GPT_Handle*      gptHandle;
#endif
    unsigned int     setTimeUs;
    unsigned int     pretimeCycle;
    unsigned int     currenttimeCycle;
    unsigned int     durationUs;
    unsigned int     errRangeUs;
    bool             timerIrqFlag;
} TIMERS_DiagnoseHandle;

#ifdef TIMER0
FunctionSafetyState TIMERS_DiagnoseInterruptIntervalAccuracy(void* timersDiagnoseHandle, DiagnoseMoment moment);
#endif

#endif