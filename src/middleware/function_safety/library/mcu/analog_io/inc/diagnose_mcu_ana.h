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
  * @file      diagnose_mcu_ana.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for ana diagnose.
  */

#ifndef DIAGNOSE_MCU_ANA_H
#define DIAGNOSE_MCU_ANA_H

#include "function_safety_common.h"
#ifdef ADC0  /* ADC0 represent ip support adc controler, not means only adc0 */
#include "adc.h"
#endif
#ifdef DAC0  /* DAC0 represent ip support dac controler, not means only dac0 */
#include "dac.h"
#endif
#ifdef ACMP0  /* ACMP0 represent ip support acmp controler, not means only acmp0 */
#include "acmp.h"
#endif
#ifdef PGA0  /* PGA0 represent ip support pga controler, not means only pga0 */
#include "pga.h"
#endif

#define MODULE_ADC                 0x01
#define MODULE_DAC                 0x02
#define MODULE_PGA                 0x04
#define MODULE_ACMP                0x08

#define FEATURE_ACCURACY           0x01

#define FAULT_1ST_OVER_ACCURACY    0x01
#define FAULT_2ST_OVER_ACCURACY    0x02
#define FAULT_3ST_OVER_ACCURACY    0x03

typedef struct {
#ifdef ADC0
    ADC_Handle*     adcHandle;
    ADC_SOCNumber   socx;
#endif
#ifdef DAC0
    DAC_Handle*     dacHandleRef;
    DAC_Handle*     dacHandleTarget;
#endif
#ifdef PGA0
    PGA_Handle*     pgaHandle;
#endif
#ifdef ACMP0
    ACMP_Handle*    acmpHandle;
#endif
    unsigned int    adcMaxRange;
    unsigned int    dacMaxRange;
    unsigned int    errRangePercent;
} ANA_DiagnoseHandle;

#if defined(ADC0) && defined(DAC0)
FunctionSafetyState ANA_DiagnoseAdcSampleAccuracy(void* anaDiagnoseHandle, DiagnoseMoment moment);
#endif
#if defined(ADC0) && defined(DAC0) && defined(PGA0)
FunctionSafetyState ANA_DiagnosePgaInnerGainAccuracy(void* anaDiagnoseHandle, DiagnoseMoment moment);
#endif
#if defined(DAC0) && defined(ACMP0)
FunctionSafetyState ANA_DiagnoseAcmpThresholdAccuracy(void* anaDiagnoseHandle, DiagnoseMoment moment);
#endif

#endif