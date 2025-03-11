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
  * @file      function_safety_battery.h
  * @author    MCU Driver Team
  * @brief     This file contains the function safety head file of battery.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FUNCTION_SAFETYE_COMMON_H
#define FUNCTION_SAFETYE_COMMON_H

#include "baseinc.h"
#include "sysctrl.h"
#include "crg.h"
#include "gpio.h"
/*--- Diagnose param check ----------------------------------------------------*/
#ifdef DIAGNOSE_PARAM_CHECK
#define DIAGNOSE_PARAM_CHECK_WITH_STATE(param, state) \
    do { \
        if (!(param)) { \
            STATE_SetDiagnoseResult((FunctionSafetyState *)state, RESULT_FAIL); \
            STATE_SetDiagnoseFaultType((FunctionSafetyState *)state, FAULT_SOFTWARE_PARAM_INCORRECT); \
            return *(FunctionSafetyState *)state; \
        } \
    } while (0)
#else
#define DIAGNOSE_PARAM_CHECK_WITH_STATE(param, ret)   ((void)0U)
#endif
/*--- Result definition ------------------------------------------------------*/
#define RESULT_IS_RUNNING   0x00
#define RESULT_WARNNING     0x01
#define RESULT_FAIL         0x02
#define RESULT_SUCCESS      0x03

/*--- Subsysterm definition --------------------------------------------------*/
#define SUBSYS_CORE         0x01
#define SUBSYS_CLOCK        0x02
#define SUBSYS_COMPUTE      0x03
#define SUBSYS_ROM          0x04
#define SUBSYS_RAM          0x05
#define SUBSYS_ANALOG_IO    0x06
#define SUBSYS_DIGTAL_IO    0x07
#define SUBSYS_TIMERS       0x08
#define SUBSYS_CONNECT      0x09
#define SUBSYS_MONITOR      0x0A

/*--- Fault type definition --------------------------------------------------*/
/* common fault type define in here, subsysterm fault type define in subsysterm file */
#define FAULT_SOFTWARE_PARAM_INCORRECT      0x0F
#define FAULT_REPORT_TIMEOUT                0x0E
/*--- Error rate definition --------------------------------------------------*/
#define ERROR_RATE_OVER_RANGE               0x7F
/*--- Fail safe measure definition -------------------------------------------*/
/* common Fail safe measure define in here, subsysterm Fail safe measure define in subsysterm file */
#define FAIL_SAFE_DEFAULT_MEASURE           0x0F

typedef enum {
    MOMENT_ALLTIME = 0U,
    MOMENT_STARTUP,
    MOMENT_RUNTIME,
    MOMENT_IRQ,
} DiagnoseMoment;

typedef union {
    unsigned int code;
    struct {
        unsigned int result           : 2;
        unsigned int moment           : 2;
        unsigned int errorRate        : 8;
        unsigned int failSafeMeasure  : 4;
        unsigned int faultType        : 4;
        unsigned int feature          : 4;
        unsigned int moudule          : 4;
        unsigned int subsysterm       : 4;
    } BIT;
} FunctionSafetyState;
/**
  * @brief set diagnose feature to state.
  * @param state @ref FunctionSafetyState.
  * @param moment moment.
  * @retval None.
  */
static inline void STATE_SetDiagnoseMoment(FunctionSafetyState* state, unsigned int moment)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(moment < 4); /* 4: moment state value limit */
    state->BIT.moment = moment;
}
/**
  * @brief set diagnose feature to state.
  * @param state @ref FunctionSafetyState.
  * @param result result.
  * @retval None.
  */
static inline void STATE_SetDiagnoseResult(FunctionSafetyState* state, unsigned int result)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(result < 4); /* 4: result state value limit */
    state->BIT.result = result;
}
/**
  * @brief set diagnose feature to state.
  * @param state @ref FunctionSafetyState.
  * @param subsysterm subsysterm.
  * @retval None.
  */
static inline void STATE_SetDiagnoseSubsysterm(FunctionSafetyState* state, unsigned int subsysterm)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(subsysterm < 16); /* 16: subsysterm type value limit */
    state->BIT.subsysterm = subsysterm;
}
/**
  * @brief set diagnose feature to state.
  * @param state @ref FunctionSafetyState.
  * @param moudule module.
  * @retval None.
  */
static inline void STATE_SetDiagnoseMoudule(FunctionSafetyState* state, unsigned int moudule)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(moudule < 16); /* 16: moudule type value limit */
    state->BIT.moudule = moudule;
}
/**
  * @brief set diagnose feature to state.
  * @param state @ref FunctionSafetyState.
  * @param feature feature.
  * @retval None.
  */
static inline void STATE_SetDiagnoseFeature(FunctionSafetyState* state, unsigned int feature)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(feature < 16); /* 16: feature type value limit */
    state->BIT.feature = feature;
}
/**
  * @brief set diagnose fault type to state.
  * @param state @ref FunctionSafetyState.
  * @param faultType fault type.
  * @retval None.
  */
static inline void STATE_SetDiagnoseFaultType(FunctionSafetyState* state, unsigned int faultType)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(faultType < 16); /* 16: faultType value limit */
    state->BIT.faultType = faultType;
}
/**
  * @brief set diagnose fail safe measure to state.
  * @param state @ref FunctionSafetyState.
  * @param failSafeMeasure fail safe measure.
  * @retval None.
  */
static inline void STATE_SetDiagnoseFailSafeMeasure(FunctionSafetyState* state, unsigned int failSafeMeasure)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(failSafeMeasure < 16); /* 16: failSafeMeasure type value limit */
    state->BIT.failSafeMeasure = failSafeMeasure;
}
/**
  * @brief set diagnose error rate to state.
  * @param state @ref FunctionSafetyState.
  * @param errorRate diagnose error rate.
  * @retval None.
  */
static inline void STATE_SetDiagnoseErrorRate(FunctionSafetyState* state, unsigned int errorRate, bool unsignFlag)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    BASE_FUNC_PARAMCHECK_NO_RET(errorRate < 128); /* 128: errorRate type value limit */
    state->BIT.errorRate = unsignFlag == true ? errorRate : (0x80 | errorRate);
}
/**
  * @brief store fail event state.
  * @param state @ref FunctionSafetyState.
  * @retval None.
  */
static inline void STATE_StoreFailEvent(FunctionSafetyState* state)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    unsigned int softIrqEventId = *(unsigned int*)(void*)state;
    DCL_SYSCTRL_SetSoftInterruptEventId(softIrqEventId); /* set fail event */
}
/**
  * @brief Load fail event state.
  * @param None.
  * @retval state @ref FunctionSafetyState.
  */
static inline FunctionSafetyState STATE_LoadFailEvent(void)
{
    return (FunctionSafetyState)DCL_SYSCTRL_GetSoftInterruptEventId(); /* get fail event */
}
/**
  * @brief store current state.
  * @param state @ref FunctionSafetyState.
  * @retval None.
  */
static inline void STATE_StoreCurrentState(FunctionSafetyState* state)
{
    BASE_FUNC_ASSERT_PARAM(state != NULL);
    SYSCTRL0->USER_REG0.reg = *(unsigned int*)(void*)state; /* store current state into USER_REG0 */
}
/**
  * @brief Load current state.
  * @param None.
  * @retval state @ref FunctionSafetyState.
  */
static inline FunctionSafetyState STATE_LoadCurrentState(void)
{
    return (FunctionSafetyState)SYSCTRL0->USER_REG0.reg; /* load current state from USER_REG0 */
}
/**
  * @brief Get cpu cycle.
  * @param None.
  * @retval current cpu cycle.
  */
static inline unsigned int BASE_GetCpuCycle(void)
{
    /* Get the Cpu Cycle Register(CSR) */
    unsigned int cycle;
    asm volatile("csrr %0, cycle" : "=r"(cycle));
    return cycle;
}

typedef void (* ConfigFunc)(void);
typedef FunctionSafetyState (* BackupFunc)(void* handle, DiagnoseMoment moment);
typedef FunctionSafetyState (* DiagnoseFunc)(void* handle, DiagnoseMoment moment);
typedef FunctionSafetyState (* FailSafeFunc)(void* handle, DiagnoseMoment moment);
typedef FunctionSafetyState (* FaultPredictFunc)(void* handle, DiagnoseMoment moment);
typedef FunctionSafetyState (* ResumeFunc)(void* handle, DiagnoseMoment moment);

#endif