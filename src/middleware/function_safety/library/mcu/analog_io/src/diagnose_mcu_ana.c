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
  * @file      diagnose_mcu_ana.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for ana diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_ana.h"
/*----------------------------- ADC sample DAC output accuracy diagnose ------------------------------------*/
#if defined (ADC0) && defined(DAC0)
/**
  * @brief Diagnose ana subsystem's ADC sampling dac accuracy.
  * @param handle Value of @ref ANA_DiagnoseHandle.
  * @param state point of @ref FunctionSafetyState.
  * @param testDacVal Value of dac config.
  * @param refAdcVal ref adc value.
  * @retval state Value of @ref FunctionSafetyState.
  */
static FunctionSafetyState AdcSampleDacAccuracyDiagnose(ANA_DiagnoseHandle* handle,  FunctionSafetyState* state, \
                                                        unsigned int testDacVal, unsigned int refAdcVal)
{
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->adcMaxRange != 0, state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(testDacVal <= handle->adcMaxRange, state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(refAdcVal <= handle->adcMaxRange, state);
    HAL_DAC_SetValue(handle->dacHandleRef, testDacVal);
    BASE_FUNC_DELAY_US(10); /* 10 : delay time 10us */
    HAL_ADC_SoftTrigSample(handle->adcHandle, handle->socx);
    while (HAL_ADC_CheckSocFinish(handle->adcHandle, handle->socx) != BASE_STATUS_OK) {
    }
    unsigned int value = HAL_ADC_GetConvResult(handle->adcHandle, handle->socx);
    unsigned int errRate = 0;
    bool unsginFlag = 0;
    if (handle->errRangePercent != 0) {
        if (value > refAdcVal) {
            errRate = (value - refAdcVal) * 100 / handle->adcMaxRange; /* 100: percent rate, 4096 is full range */
            unsginFlag = true;
        } else {
            errRate = (refAdcVal - value) * 100 / handle->adcMaxRange; /* 100: percent rate, 4096 is full range */
            unsginFlag = false;
        }
        STATE_SetDiagnoseErrorRate(state, errRate, unsginFlag); /* set error rate into state */
    }
    if (errRate > handle->errRangePercent) { /* error judgment */
        STATE_SetDiagnoseResult(state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(state, FAULT_1ST_OVER_ACCURACY); /* set fault type  and fail result into state */
        return *state;
    }
    return *state;
}

/**
  * @brief Diagnose ana subsystem's ADC sampling accuracy.
  * @param anaDiagnoseHandle Value of @ref ANA_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ANA_DiagnoseAdcSampleAccuracy(void* anaDiagnoseHandle, DiagnoseMoment moment)
{
    ANA_DiagnoseHandle* handle = (ANA_DiagnoseHandle*)anaDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_ANALOG_IO);
    STATE_SetDiagnoseMoudule(&state, MODULE_ADC);
    STATE_SetDiagnoseFeature(&state, FEATURE_ACCURACY); /* set subsysterm and module and feature */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->adcHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->dacHandleRef != NULL, &state); /* param check */
    /* test 1 : set DAC output 1/4 max range , adc sample dac should be about 1/4 max range */
    state = AdcSampleDacAccuracyDiagnose(handle, &state, (handle->dacMaxRange / 4),    /* 1/4 dac max range */
                                                         (handle->adcMaxRange / 4));   /* 1/4 adc max range */
    if (state.BIT.result == RESULT_FAIL) {
        return state;
    }
    /* test 2 : set DAC output 2/4 max range , adc sample dac should be about 2/4 max range */
    state = AdcSampleDacAccuracyDiagnose(handle, &state, (handle->dacMaxRange / 2),    /* 1/2 dac max range */
                                                         (handle->adcMaxRange / 2));   /* 1/2 adc max range */
    if (state.BIT.result == RESULT_FAIL) {
        return state;
    }
    /* test 3 : set DAC output 3/4 max range , adc sample dac should be about 3/4 max range */
    state = AdcSampleDacAccuracyDiagnose(handle, &state, ((handle->dacMaxRange * 3) / 4),    /* 3/4 dac max range */
                                                         ((handle->adcMaxRange * 3) / 4));   /* 3/4 adc max range */
    if (state.BIT.result == RESULT_FAIL) {
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS); /* set success into state */
    return state;
}
#endif
/*----------------------------- PGA Inner Gain accuracy diagnose ------------------------------------*/
#if defined(ADC0) && defined(DAC0) && defined(PGA0)
/**
  * @brief Diagnose ana subsystem's ADC sampling dac accuracy.
  * @param handle Value of @ref ANA_DiagnoseHandle.
  * @param state point of @ref FunctionSafetyState.
  * @param testDacVal Value of dac config.
  * @param refAdcVal ref adc value.
  * @retval state Value of @ref FunctionSafetyState.
  */
static FunctionSafetyState PgaInnerGainAccuracyDiagnose(ANA_DiagnoseHandle* handle,  FunctionSafetyState* state, \
    unsigned int refDacVal, unsigned int targetDacVal, PGA_GainValue pgaGain)
{
    DIAGNOSE_PARAM_CHECK_WITH_STATE(refDacVal <= handle->dacMaxRange, state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(targetDacVal <= handle->dacMaxRange, state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(pgaGain <= PGA_GAIN_16X, state);
    HAL_DAC_SetValue(handle->dacHandleRef, refDacVal);
    HAL_DAC_SetValue(handle->dacHandleTarget, targetDacVal); /* set ref and target dac value */
    HAL_PGA_SetGain(handle->pgaHandle, pgaGain);
    HAL_ADC_SoftTrigSample(handle->adcHandle, handle->socx);
    while (HAL_ADC_CheckSocFinish(handle->adcHandle, handle->socx) != BASE_STATUS_OK) {
    }
    signed int value = (signed int)HAL_ADC_GetConvResult(handle->adcHandle, handle->socx);
    signed int errRate = value - (signed int)(handle->dacMaxRange / 2);  /* ref adc value */
    errRate = (errRate < 0) ? -errRate : errRate;
    signed int adcValErrRang = (signed int)handle->errRangePercent;
    if (errRate >= adcValErrRang) {
        STATE_SetDiagnoseResult(state, RESULT_FAIL); /* set fail result into state */
        STATE_SetDiagnoseFaultType(state, FAULT_1ST_OVER_ACCURACY); /* set fault type into state */
        return *state;
    }
    return *state;
}

/**
  * @brief Diagnose ana subsystem's PGA inner gain accuracy.
  * @param anaDiagnoseHandle Value of @ref ANA_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ANA_DiagnosePgaInnerGainAccuracy(void* anaDiagnoseHandle, DiagnoseMoment moment)
{
    ANA_DiagnoseHandle* handle = (ANA_DiagnoseHandle*)anaDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_ANALOG_IO);
    STATE_SetDiagnoseMoudule(&state, MODULE_PGA);
    STATE_SetDiagnoseFeature(&state, FEATURE_ACCURACY); /* set function state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->adcHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->pgaHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->dacHandleRef != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->dacHandleTarget != NULL, &state);
    /* test 1 : PGA Differential 1/4 voltage, adc sample PGA out should be about 1/2 max range in gain equal 2x */
    state = PgaInnerGainAccuracyDiagnose(handle, &state, 0, (handle->dacMaxRange / 4), PGA_GAIN_2X); /* 1/4 range */
    if (state.BIT.result == RESULT_FAIL) {
        return state;
    }
    /* test 2 : set PGA Differential 1/16 voltage, adc sample PGA out should be about 1/2 max range in gain equal 8x */
    state = PgaInnerGainAccuracyDiagnose(handle, &state, 0, (handle->dacMaxRange / 16), PGA_GAIN_8X); /* 1/16 range */
    if (state.BIT.result == RESULT_FAIL) {
        return state;
    }
    /* test 3 : set PGA Differential 1/8 voltage, adc sample PGA out should be about 1/2 max range in gain equal 4x */
    state = PgaInnerGainAccuracyDiagnose(handle, &state, 0, (handle->dacMaxRange / 8), PGA_GAIN_4X); /* 1/8 range */
    if (state.BIT.result == RESULT_FAIL) {
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);
    return state;
}
#endif
/*----------------------------- ACMP Threshold accuracy diagnose ------------------------------------*/
#if defined(DAC0) && defined(ACMP0)
/**
  * @brief Diagnose ana subsystem's ACMP compare output.
  * @param anaDiagnoseHandle Value of @ref ANA_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState ANA_DiagnoseAcmpThresholdAccuracy(void* anaDiagnoseHandle, DiagnoseMoment moment)
{
    ANA_DiagnoseHandle* handle = (ANA_DiagnoseHandle*)anaDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_ANALOG_IO);
    STATE_SetDiagnoseMoudule(&state, MODULE_ACMP);
    STATE_SetDiagnoseFeature(&state, FEATURE_ACCURACY); /* set function state */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->acmpHandle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->dacHandleRef != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->dacHandleTarget != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->errRangePercent < 100, &state); /* 100: upbound */
    unsigned int dacValErrRang = (handle->dacMaxRange / 2) * handle->errRangePercent / 100; /* 100: percent, 2: half */
    /* test 1 : set ACMP Differential voltage < 0, Not invert, ACMP out should be equal 0 */
    HAL_DAC_SetValue(handle->dacHandleRef, (handle->dacMaxRange / 2) + dacValErrRang); /* 2: half dac max range */
    HAL_DAC_SetValue(handle->dacHandleTarget, (handle->dacMaxRange / 2)); /*  2: half dac max range */
    unsigned int value = DCL_ACMP_GetCmpOutValueOriginal(handle->acmpHandle->baseAddress);
    if (value == 1) { /* acmp output high level */
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_1ST_OVER_ACCURACY);
        return state;
    }
    /* test 2 : set ACMP Differential voltage > 0, Not invert, ACMP out should be equal 1 */
    HAL_DAC_SetValue(handle->dacHandleRef, (handle->dacMaxRange / 2) - dacValErrRang); /* N channel, 2:half dac range */
    HAL_DAC_SetValue(handle->dacHandleTarget, (handle->dacMaxRange / 2));            /* P channel, 2:half dac range */
    value = DCL_ACMP_GetCmpOutValueOriginal(handle->acmpHandle->baseAddress);
    if (value == 0) { /* acmp output low level */
        STATE_SetDiagnoseResult(&state, RESULT_FAIL);
        STATE_SetDiagnoseFaultType(&state, FAULT_2ST_OVER_ACCURACY);
        return state;
    }
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);
    return state;
}
#endif