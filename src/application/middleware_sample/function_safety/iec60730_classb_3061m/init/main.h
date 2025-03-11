/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      main.h
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_SYSTEM_INIT_H
#define McuMagicTag_SYSTEM_INIT_H

#include "adc.h"
#include "adc_ex.h"
#include "uart.h"
#include "uart_ex.h"
#include "crc.h"
#include "cfd.h"
#include "cmm.h"
#include "dac.h"
#include "timer.h"
#include "timer_ex.h"
#include "pmc.h"
#include "crg.h"
#include "iwdg.h"
#include "iwdg_ex.h"
#include "wwdg.h"
#include "wwdg_ex.h"
#include "iocmg.h"

#define    IO_SPEED_FAST     0x00U
#define    IO_SPEED_SLOW     0x01U

#define    IO_DRV_LEVEL4     0x00U
#define    IO_DRV_LEVEL3     0x01U
#define    IO_DRV_LEVEL2     0x02U
#define    IO_DRV_LEVEL1     0x03U

#define    XTAL_DRV_LEVEL4   0x03U
#define    XTAL_DRV_LEVEL3   0x02U
#define    XTAL_DRV_LEVEL2   0x01U
#define    XTAL_DRV_LEVEL1   0x00U

#define DIAGNOSE_ADC0_HANDLE g_adc0
#define DIAGNOSE_ADC0_SOC ADC_SOC_NUM0
#define DIAGNOSE_ADC0_CHANNEL ADC_CH_ADCINA17

extern CFD_Handle g_cfd;
extern CMM_Handle g_cmm;
extern CRC_Handle g_crc;
extern DAC_Handle g_dac0;
extern PMC_Handle g_pmc;
extern WWDG_Handle g_wwdg;
extern IWDG_Handle g_iwdg;
extern TIMER_Handle g_timer0;
extern TIMER_Handle g_timer1;
extern TIMER_Handle g_timer2;
extern UART_Handle g_uart0;
extern ADC_Handle g_adc0;

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect);
void SystemInit(void);

void PMCPVDInterruptCallback(void *handle);
void CFDCheckEndCallback(CFD_Handle *handle);
void CFDClockStopCallback(CFD_Handle *handle);
void CMMCounterOverFlowCallback(CMM_Handle *handle);
void CMMCheckEndCallback(CMM_Handle *handle);
void CMMFreqErrorCallback(CMM_Handle *handle);
void TIMER0CallbackFunction(void *handle);
void TIMER0_DMAOverFlow_InterruptProcess(void *handle);
void TIMER1CallbackFunction(void *handle);
void TIMER1_DMAOverFlow_InterruptProcess(void *handle);
void TIMER2CallbackFunction(void *handle);
void TIMER2_DMAOverFlow_InterruptProcess(void *handle);

/* USER CODE BEGIN 0 */
/* USER CODE 区域内代码不会被覆盖，区域外会被生成的默认代码覆盖（其余USER CODE 区域同理） */
/* USER CODE END 0 */

#endif /* McuMagicTag_SYSTEM_INIT_H */