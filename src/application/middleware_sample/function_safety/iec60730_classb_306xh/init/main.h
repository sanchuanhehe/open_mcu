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
#include "crc.h"
#include "cfd.h"
#include "cmm.h"
#include "dac.h"
#include "dac_ex.h"
#include "timer.h"
#include "pmc.h"
#include "crg.h"
#include "wdg.h"
#include "iwdg.h"

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

#define DIAGNOSE_ADC0_HANDLE    g_adc0
#define DIAGNOSE_ADC0_SOC       ADC_SOC_NUM0
#define DIAGNOSE_ADC0_CHANNEL   ADC_CH_ADCINB7

#define DIAGNOSE_ADC1_HANDLE    g_adc1
#define DIAGNOSE_ADC1_SOC       ADC_SOC_NUM0
#define DIAGNOSE_ADC1_CHANNEL   ADC_CH_ADCINB7

#define DIAGNOSE_ADC2_HANDLE    g_adc2
#define DIAGNOSE_ADC2_SOC       ADC_SOC_NUM0
#define DIAGNOSE_ADC2_CHANNEL   ADC_CH_ADCINB7

extern CFD_Handle g_cfd;
extern CMM_Handle g_cmm;
extern CRC_Handle g_crc;
extern DAC_Handle g_dac0;
extern DAC_Handle g_dac1;
extern DAC_Handle g_dac2;
extern PMC_Handle g_pmc;
extern IWDG_Handle g_iwdg;
extern WDG_Handle g_wdg;
extern TIMER_Handle g_timer0;
extern TIMER_Handle g_timer1;
extern TIMER_Handle g_timer2;
extern UART_Handle g_uart0;
extern ADC_Handle g_adc0;
extern ADC_Handle g_adc1;
extern ADC_Handle g_adc2;

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect);
void SystemInit(void);

void PMCPVDInterruptCallback(void *handle);
void CFDCheckEndCallback(CFD_Handle *handle);
void CFDClockStopCallback(CFD_Handle *handle);
void CMMCounterOverFlowCallback(CMM_Handle *handle);
void CMMCheckEndCallback(CMM_Handle *handle);
void CMMFreqErrorCallback(CMM_Handle *handle);
void TIMER0CallbackFunction(void *handle);
void TIMER1CallbackFunction(void *handle);
void TIMER2CallbackFunction(void *handle);

#endif /* McuMagicTag_SYSTEM_INIT_H */