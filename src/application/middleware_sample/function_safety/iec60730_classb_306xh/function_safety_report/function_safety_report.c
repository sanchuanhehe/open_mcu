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
  * @file      function_safety_report.c
  * @author    MCU Driver Team
  * @brief     function safety report module.
  * @details   function safety report interface implementation.
  */

#include "function_safety_report.h"
#include "debug.h"
#include "iocmg.h"
/**
  * @brief rigister soft irq callback and prioerity , callback param.
  * @param handlerFunc irq callback function.
  * @param priority irq priority.
  * @param param irq callback param.
  * @retval None.
  */
void RegisterReportFailEventHandler(IRQ_PROC_FUNC handlerFunc, unsigned int priority, void *param)
{
    IRQ_DisableN(IRQ_SOFTWARE);
    IRQ_Unregister(IRQ_SOFTWARE); /* unregister soft irq */
    IRQ_Register(IRQ_SOFTWARE, handlerFunc, param); /* register soft irq callback and param */
    IRQ_SetPriority(IRQ_SOFTWARE, priority); /* set soft irq priority */
    IRQ_EnableN(IRQ_SOFTWARE);
}
/**
  * @brief soft irq callback of report fail event.
  * @param param irq callback param.
  * @retval None.
  */
void ReportFailEventLogBySoftIrq(void *param)
{
    BASE_FUNC_UNUSED(param);
    DCL_SYSCTRL_ClearSoftInterrupt(); /* clear soft irq */
    IRQ_ClearN(IRQ_SOFTWARE);
    IRQ_Disable();
    FunctionSafetyState failEvent = STATE_LoadFailEvent(); /* load fail event */
    DBG_PRINTF("Fail Event Info : FailEvent[0x%x], Subsysterm [%d], Module [%d], Feature [%d], Failtype [%d]\r\n", \
        failEvent, failEvent.BIT.subsysterm, failEvent.BIT.moudule, failEvent.BIT.feature, failEvent.BIT.faultType);
}

/**
  * @brief Excute function safety process and return state.
  * @param state @ref FunctionSafetyState.
  * @param processTime @ref ProcessTimeCycle.
  * @retval None.
  */
void FunctionSafetyProcessPrint(FunctionSafetyState state, ProcessTimeCycle processTime)
{
    if (!processTime.timeCycleRecordSwitch) {
        return;
    }
    if (state.BIT.result == RESULT_FAIL) {  /* result fail judgement */
        DBG_PRINTF("\r\nFAIL info : State[0x%x], Subsysterm[%d], Module[%d], Feature[%d], Failtype[%d] \
            \r\n            DiagnoseCycle[%d], FailSafeCycle[%d], FaultPredictCycle[%d]\r\n", \
            state, state.BIT.subsysterm, state.BIT.moudule, state.BIT.feature, state.BIT.faultType, \
            processTime.diagnoseTimeCycle, processTime.failSafeTimeCycle, processTime.faultPredictTimeCycle);
    } else {
        if (state.BIT.result == RESULT_SUCCESS) { /* result success judgement */
            DBG_PRINTF("\r\nSUCCESS info : State [0x%x], Subsysterm [%d], Module [%d], Feature [%d] \
                \r\n               DiagnoseCycle[%d], FailSafeCycle[%d], FaultPredictCycle[%d]\r\n", \
                state, state.BIT.subsysterm, state.BIT.moudule, state.BIT.feature,\
                processTime.diagnoseTimeCycle, processTime.failSafeTimeCycle, processTime.faultPredictTimeCycle);
        }
        if (state.BIT.result == RESULT_IS_RUNNING) { /* result ongoing judgement */
            DBG_PRINTF("\r\nRUNNING info : State [0x%x], Subsysterm [%d], Module [%d], Feature [%d] \
                \r\n               DiagnoseCycle[%d], FailSafeCycle[%d], FaultPredictCycle[%d]\r\n", \
                state, state.BIT.subsysterm, state.BIT.moudule, state.BIT.feature,\
                processTime.diagnoseTimeCycle, processTime.failSafeTimeCycle, processTime.faultPredictTimeCycle);
        }
    }
}