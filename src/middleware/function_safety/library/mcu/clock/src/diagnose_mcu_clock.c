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
  * @file      diagnose_mcu_clock.c
  * @author    MCU Driver Team
  * @brief     This file contains the functions definition for clock diagnose.
  */

/* Includes ------------------------------------------------------------------*/
#include "diagnose_mcu_clock.h"

#define FREQ_ERR_CONVERT_CNT_ERR_RANGE     0U
#define CNT_ERR_CONVERT_FREQ_ERR_RANGE     1U

/**
  * @brief clock frequency or count error swap.
  * @param clockFreq Value of clock frequency.
  * @param targetValue Value of target count value.
  * @param percent Value of error range.
  * @param convertType Value of FREQ_ERR_CONVERT_CNT_ERR_RANGE or CNT_ERR_CONVERT_FREQ_ERR_RANGE.
  * @retval unsigned int frequency or count value.
  */
static unsigned int CLOCK_FreqAndCntErrSwap(unsigned int clockFreq, float targetValue, \
                                            unsigned int percent, bool convertType)
{
    float error = 0.0;
    error = ((float)percent / 100.0f); /* 100: percent unit */
    error = (error < 0) ? (-error) : error;
    if (convertType == FREQ_ERR_CONVERT_CNT_ERR_RANGE) {
        return (unsigned int)(error * targetValue + 0.5f);    /* 0.5: round value */
    } else {
        BASE_FUNC_ASSERT_PARAM(clockFreq != 0);
        return (unsigned int)(error * clockFreq + 0.5f);    /* 0.5: round value */
    }
}
/* -------------------------------- CMM Diagnose Clock frequency ------------------- */
#ifdef CMM
/**
  * @brief Get the Frequency of a Clock Source.
  * @param clockSource Clock source corresponding to the register.
  * @param clockType zero means target clock, one means ref clock.
  * @retval clock frequency.
  */
static unsigned int CLOCK_GetTargetClockSourceFreq(CMM_Target_Clock_Source clockSource)
{
    BASE_FUNC_ASSERT_PARAM(clockSource <= 4); /* 4: reg value limit */
    unsigned int clockFreq = 0;
    switch (clockSource) {
        case CMM_TARGET_CLK_HS_SYS:
            clockFreq = HAL_CRG_GetCoreClkFreq(); /* get core clock frequency */
            break;
        case CMM_TARGET_CLK_HOSC:
            clockFreq = HOSC_FREQ;  /* HOSC clock frequency */
            break;
        case CMM_TARGET_CLK_TCXO:
            clockFreq = XTRAIL_FREQ;  /* XTAL clock frequency */
            break;
        case CMM_TARGET_CLK_LOSC:
            clockFreq = LOSC_FREQ;  /* LOSC clock frequency */
            break;
        case 4:  /* 4: reg value means LS clock frequency */
            clockFreq = HAL_CRG_GetCoreClkFreq() / 2;  /* 2: half hs clock */
            break;
        default:
            break;
    }
    return clockFreq;
}
/**
  * @brief Get the Frequency of a Clock Source.
  * @param clockSource Clock source corresponding to the register.
  * @param clockType zero means target clock, one means ref clock.
  * @retval clock frequency.
  */
static unsigned int CLOCK_GetRefClockSourceFreq(CMM_Ref_Clock_Source clockSource)
{
    BASE_FUNC_ASSERT_PARAM(clockSource <= 4); /* 4: reg value limit */
    unsigned int clockFreq = 0;
    switch (clockSource) {
        case CMM_REF_CLK_HS_SYS:
            clockFreq = HAL_CRG_GetCoreClkFreq(); /* get core clock frequency */
            break;
        case CMM_REF_CLK_HOSC:
            clockFreq = HOSC_FREQ;  /* HOSC clock frequency */
            break;
        case CMM_REF_CLK_TCXO:
            clockFreq = XTRAIL_FREQ;  /* XTAL clock frequency */
            break;
        case CMM_REF_CLK_LOSC:
            clockFreq = LOSC_FREQ;  /* LOSC clock frequency */
            break;
        default:
            break;
    }
    return clockFreq;
}

/**
  * @brief Get the Frequency division of a Clock Source.
  * @param clockSourceDiv Clock division corresponding to the register.
  * @param clockType zero means target clock, one means ref clock.
  * @retval clock frequency division.
  */
static unsigned int CLOCK_GetTargetClockSourceDiv(CMM_Target_Freq_Div_Value clockSourceDiv)
{
    BASE_FUNC_ASSERT_PARAM(clockSourceDiv <= 4); /* 4: reg value limit */
    unsigned int clockDiv = 0;
    switch (clockSourceDiv) {
        case CMM_TARGET_FREQ_DIV_0:
            clockDiv = 1;    /* 1: div value is 1 */
            break;
        case CMM_TARGET_FREQ_DIV_32:
            clockDiv = 32;    /* 32: div value is 32 */
            break;
        case CMM_TARGET_FREQ_DIV_128:
            clockDiv = 128;    /* 128: div value is 128 */
            break;
        case CMM_TARGET_FREQ_DIV_1024:
            clockDiv = 1024;    /* 1024: div value is 1024 */
            break;
        case CMM_TARGET_FREQ_DIV_8192:
            clockDiv = 8192;    /* 8192: div value is 8192 */
            break;
        default:
            break;
    }
    return clockDiv;
}

/**
  * @brief Get the Frequency division of a Clock Source.
  * @param clockSourceDiv Clock division corresponding to the register.
  * @param clockType zero means target clock, one means ref clock.
  * @retval clock frequency division.
  */
static unsigned int CLOCK_GetRefClockSourceDiv(CMM_Ref_Freq_Div_Value clockSourceDiv)
{
    BASE_FUNC_ASSERT_PARAM(clockSourceDiv <= 4); /* 4: reg value up bound */
    unsigned int clockDiv = 0;
    switch (clockSourceDiv) {
        case CMM_REF_FREQ_DIV_0:
            clockDiv = 1;    /* 1: div value is 1 */
            break;
        case CMM_REF_FREQ_DIV_4:
            clockDiv = 4;    /* 4: div value is 4 */
            break;
        case CMM_REF_FREQ_DIV_8:
            clockDiv = 8;    /* 8: div value is 8 */
            break;
        case CMM_REF_FREQ_DIV_32:
            clockDiv = 32;    /* 32: div value is 32 */
            break;
        default:
            break;
    }
    return clockDiv;
}
/**
  * @brief Get the cmm count tagert value.
  * @param cmmHandle Value of @ref CMM_Handle.
  * @retval unsigned int cmm count tagert value.
  */
static unsigned int CLOCK_GetCmmTargetValue(CMM_Handle* cmmHandle)
{
    BASE_FUNC_ASSERT_PARAM(cmmHandle != NULL);
    unsigned int ret = 0;
    /* get target clock and ref clock's frequency and division */
    unsigned int targetClockSourceFreq = CLOCK_GetTargetClockSourceFreq(cmmHandle->targetClockSource);
    unsigned int refClockSourceFreq = CLOCK_GetRefClockSourceFreq(cmmHandle->refClockSource);
    unsigned int targetClockDivVal = CLOCK_GetTargetClockSourceDiv(cmmHandle->targetFreqDivision);
    unsigned int refClockDivVal = CLOCK_GetRefClockSourceDiv(cmmHandle->refFreqDivision);
    if (targetClockSourceFreq == 0 || refClockSourceFreq == 0 || targetClockDivVal == 0 || refClockDivVal == 0) {
        return ret;
    }
    /* prevent value is float conver to int drop data, up to unsigned int to set windows value, error control is 0.01 */
    ret = (unsigned int)(((float)targetClockDivVal / refClockDivVal) * \
          ((float)refClockSourceFreq / targetClockSourceFreq) + 0.5f);
    return ret;
}

/**
  * @brief Cmm count value windows bound config.
  * @param clockDiagnoseHandle Value of @ref CLOCK_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CLOCK_CmmWindowsBoundCalculate(void* clockDiagnoseHandle, DiagnoseMoment moment)
{
    CLOCK_DiagnoseHandle* handle = (CLOCK_DiagnoseHandle*)clockDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CLOCK);
    STATE_SetDiagnoseMoudule(&state, MODULE_CMM);  /* set subsysterm and module */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->cmmHandle != NULL, &state);
    handle->cmmTargetVal = CLOCK_GetCmmTargetValue(handle->cmmHandle);  /* get target cmm count value */
    if (handle->cmmTargetVal == 0) {
        STATE_SetDiagnoseFaultType(&state, FAULT_SOFTWARE_PARAM_INCORRECT);
        return state;
    }
    unsigned int clockFreq = CLOCK_GetTargetClockSourceFreq(handle->cmmHandle->targetClockSource);
    /* error unit swap from frequency and count */
    unsigned int cntError = CLOCK_FreqAndCntErrSwap(clockFreq, handle->cmmTargetVal, \
                                                    handle->errRangePercent, FREQ_ERR_CONVERT_CNT_ERR_RANGE);
    handle->cmmHandle->lowerBound = ((signed int)(handle->cmmTargetVal - cntError) < 0) ? \
                                    0 : handle->cmmTargetVal - cntError;
    handle->cmmHandle->upperBound = handle->cmmTargetVal + cntError;
    return state;
}

/**
  * @brief Cmm diagnose clock accuracy.
  * @param clockDiagnoseHandle Value of @ref CLOCK_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CLOCK_CmmDiagnoseAccuracy(void* clockDiagnoseHandle, DiagnoseMoment moment)
{
    CLOCK_DiagnoseHandle* handle = (CLOCK_DiagnoseHandle*)clockDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CLOCK);
    STATE_SetDiagnoseMoudule(&state, MODULE_CMM);
    STATE_SetDiagnoseFeature(&state, FEATURE_CLOCK_ACCURACY_SINGLE_CHECK); /* set subsysterm and module and feature */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->cmmHandle != NULL, &state);
    if (handle->cmmHandle->interruptType == CMM_INT_CHECK_END_MASK) { /* interrupt check end single times */
        BASE_FUNC_DELAY_US(handle->checkEndDelayTimeUs);
        if (!handle->cmmIrqFlag) {
            STATE_SetDiagnoseResult(&state, RESULT_IS_RUNNING);
            return state;
        }
        handle->cmmIrqFlag = BASE_CFG_UNSET;
        HAL_CMM_Stop(handle->cmmHandle); /* single check and stop */
        if (handle->cmmClockCount > handle->cmmHandle->upperBound || \
            handle->cmmClockCount < handle->cmmHandle->lowerBound) {
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);
            STATE_SetDiagnoseFaultType(&state, FAULT_OVER_ACCURACY);  /* set fault type */
            return state;
        }
    } else {
        if (handle->cmmIrqFlag) {
            handle->cmmIrqFlag = BASE_CFG_UNSET;
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);   /* IN CMM CHECK ERR IRQ */
            STATE_SetDiagnoseFaultType(&state, FAULT_OVER_ACCURACY);
            return state;
        }
    }
    unsigned int errorRate = 0;
    bool unsignFlag = false;
    if (handle->cmmClockCount > handle->cmmTargetVal) {
        errorRate = (handle->cmmClockCount - handle->cmmTargetVal);
        unsignFlag = true;
    } else {
        errorRate = (handle->cmmTargetVal - handle->cmmClockCount);
        unsignFlag = false;
    }
    errorRate = errorRate * 100 / (handle->cmmHandle->upperBound - handle->cmmTargetVal); /* 100 : percent */
    STATE_SetDiagnoseErrorRate(&state, errorRate, unsignFlag);  /* set error rate to state */
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
#endif
/* -------------------------------- CFD Diagnose Pll ref Clock stop ------------------- */
#ifdef CFD
/**
  * @brief Get the cfd count tagert value.
  * @param cfdHandle Value of @ref CFD_Handle.
  * @retval unsigned int cfd count tagert value.
  */
static unsigned int CLOCK_GetCfdTargetValue(CFD_Handle* cfdHandle)
{
    BASE_FUNC_ASSERT_PARAM(cfdHandle != NULL);
    unsigned int targetClockSource = DCL_CRG_GetPllRefClkSel(CRG);
    unsigned int targetClockSourceFreq = (targetClockSource == (unsigned int)CRG_PLL_REF_CLK_SELECT_HOSC) ? \
                                         HOSC_FREQ : XTRAIL_FREQ;
    unsigned int refClockSourceFreq = LOSC_FREQ;
    unsigned int targetClockDivVal = 2048;  /* 2048:chip has fix the value */
    unsigned int refClockDivVal = 1;  /* chip has fix the value */
    /* prevent value is float conver to int drop data, up to unsigned int to set windows value, error control is 0.01 */
    return ((unsigned int)(((float)targetClockDivVal * (float)refClockSourceFreq) / \
            ((float)refClockDivVal * (float)targetClockSourceFreq) + 0.5f));
}

/**
  * @brief Cfd count value windows bound config.
  * @param clockDiagnoseHandle Value of @ref CLOCK_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CLOCK_CfdWindowsBoundCalculate(void* clockDiagnoseHandle, DiagnoseMoment moment)
{
    CLOCK_DiagnoseHandle* handle = (CLOCK_DiagnoseHandle*)clockDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CLOCK);
    STATE_SetDiagnoseMoudule(&state, MODULE_CFD); /* set subsysterm and module */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->cfdHandle != NULL, &state);
    /* target cfd count value, 0.5 for mod */
    handle->cfdTargetVal = (unsigned int)((float)CLOCK_GetCfdTargetValue(handle->cfdHandle) + 0.5f); /* 0.5 for mod */
    unsigned int clockFreq = (DCL_CRG_GetPllRefClkSel(CRG) == (unsigned int)CRG_PLL_REF_CLK_SELECT_HOSC) ? \
                             HOSC_FREQ : XTRAIL_FREQ;
    /* error unit swap from frequency and count */
    unsigned int cntError = CLOCK_FreqAndCntErrSwap(clockFreq, handle->cfdTargetVal, \
                                                    handle->errRangePercent, FREQ_ERR_CONVERT_CNT_ERR_RANGE);
    handle->cfdHandle->upperBound = handle->cfdTargetVal + cntError;
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);
    return state;
}

/**
  * @brief Cfd diagnose pll ref clock accuracy.
  * @param clockDiagnoseHandle Value of @ref CLOCK_DiagnoseHandle.
  * @param moment Value of @ref DiagnoseMoment.
  * @retval state Value of @ref FunctionSafetyState.
  */
FunctionSafetyState CLOCK_CfdDiagnosePllRefClockStop(void* clockDiagnoseHandle, DiagnoseMoment moment)
{
    CLOCK_DiagnoseHandle* handle = (CLOCK_DiagnoseHandle*)clockDiagnoseHandle;
    FunctionSafetyState state = {0};
    BASE_FUNC_UNUSED(moment);
    STATE_SetDiagnoseSubsysterm(&state, SUBSYS_CLOCK);
    STATE_SetDiagnoseMoudule(&state, MODULE_CFD);
    STATE_SetDiagnoseFeature(&state, FEATURE_PLLREF_CLOCK_STOP_CHECK); /* set subsysterm and module and feature */
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle != NULL, &state);
    DIAGNOSE_PARAM_CHECK_WITH_STATE(handle->cfdHandle != NULL, &state);
    if (handle->cfdHandle->interruptType == CFD_INT_CHECK_END_MASK) { /* interrupt check end single times */
        BASE_FUNC_DELAY_US(handle->checkEndDelayTimeUs);
        if (!handle->cfdIrqFlag) {
            STATE_SetDiagnoseResult(&state, RESULT_IS_RUNNING);
            return state;
        }
        handle->cfdIrqFlag = BASE_CFG_UNSET;
        HAL_CFD_Stop(handle->cfdHandle); /* single check and stop */
        if (handle->cfdClockCount > handle->cfdHandle->upperBound) {
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);
            STATE_SetDiagnoseFaultType(&state, FAULT_PLLREF_CLOCK_STOP);  /* set fault type */
            return state;
        }
    } else {
        if (handle->cfdIrqFlag) {
            handle->cfdIrqFlag = BASE_CFG_UNSET;
            STATE_SetDiagnoseResult(&state, RESULT_FAIL);   /* IN CMM CHECK ERR IRQ */
            STATE_SetDiagnoseFaultType(&state, FAULT_PLLREF_CLOCK_STOP);
            return state;
        }
    }
    unsigned int errorRate = 0;
    bool unsignFlag = false;
    if (handle->cfdClockCount > handle->cfdTargetVal) {
        errorRate = (handle->cfdClockCount - handle->cfdTargetVal);
        unsignFlag = true;
    } else {
        errorRate = (handle->cfdTargetVal - handle->cfdClockCount);
        unsignFlag = false;
    }
    errorRate = errorRate * 100 / (handle->cmmHandle->upperBound - handle->cmmTargetVal); /* 100 : conver to percent */
    STATE_SetDiagnoseErrorRate(&state, errorRate, unsignFlag);  /* set error rate to state */
    STATE_SetDiagnoseResult(&state, RESULT_SUCCESS);  /* set result success to state */
    return state;
}
#endif