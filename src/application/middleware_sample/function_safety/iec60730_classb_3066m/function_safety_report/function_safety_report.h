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
  * @file      function_safety_report.h
  * @author    MCU Driver Team
  * @brief     function safety report module.
  * @details   function safety report interface declaration.
  */

#ifndef FUNCTION_SAFETY_REPORT_H
#define FUNCTION_SAFETY_REPORT_H

#include "function_safety_mcu.h"

/**
  * @brief Triggle soft irq send fail event.
  * @param None.
  * @retval None.
  */
static inline void TriggerSoftIrqSendFailEvent(void)
{
    return DCL_SYSCTRL_GenerateSoftInterrupt();
}

void RegisterReportFailEventHandler(IRQ_PROC_FUNC handlerFunc, unsigned int priority, void *param);
void ReportFailEventLogBySoftIrq(void *param);
void FunctionSafetyProcessPrint(FunctionSafetyState state, ProcessTimeCycle processTime);

#endif