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
  * @file      function_safety_init.c
  * @author    MCU Driver Team
  * @brief     function safety library adapter layer init.
  * @details   function safety library adapter layer init interface.
  */
#include "main.h"
#include "function_safety_config.h"
#include "function_safety_init.h"

CORE_DiagnoseHandle     g_diagnoseCore      = {0};
ANA_DiagnoseHandle      g_diagnoseAna       = {0};
CLOCK_DiagnoseHandle    g_diagnoseClock     = {0};
COMPUTE_DiagnoseHandle  g_diagnoseCompute   = {0};
DIO_DiagnoseHandle      g_diagnoseDio       = {0};
MONITOR_DiagnoseHandle  g_diagnoseMonitor   = {0};
RAM_DiagnoseHandle      g_diagnoseRam       = {0};
ROM_DiagnoseHandle      g_diagnoseRom       = {0};
TIMERS_DiagnoseHandle   g_diagnoseTimers    = {0};
CONNECT_DiagnoseHandle  g_diagnoseConnect   = {0};
/*--------------------------------- analog diagnose enviroment init ----------------------------------------*/
/**
  * @brief ANA subsysterm's adc1 accuracy diagnose init.
  * @param None.
  * @retval None.
  */
void ANA_DiagnoseAdc1AccuracyInit(void)
{
    g_diagnoseAna.adcHandle       = &DIAGNOSE_ADC1_HANDLE;
    g_diagnoseAna.dacHandleRef    = &g_dac1; /* dac output to adc */
    g_diagnoseAna.socx            = DIAGNOSE_ADC1_SOC;
    g_diagnoseAna.adcMaxRange     = 4096;   /* 4096 : max range of adc */
    g_diagnoseAna.dacMaxRange     = 256;    /* 256 : max range of dac */
    g_diagnoseAna.errRangePercent = 100; /* 100: adc value error range */
    DCL_ADC_SOCxSelectChannel(g_diagnoseAna.adcHandle->baseAddress, g_diagnoseAna.socx, DIAGNOSE_ADC1_CHANNEL);
}
/**
  * @brief ANA subsysterm's adc2 accuracy diagnose init.
  * @param None.
  * @retval None.
  */
void ANA_DiagnoseAdc2AccuracyInit(void)
{
    g_diagnoseAna.adcHandle       = &DIAGNOSE_ADC2_HANDLE;
    g_diagnoseAna.dacHandleRef    = &g_dac2; /* dac output to adc */
    g_diagnoseAna.socx            = DIAGNOSE_ADC2_SOC;
    g_diagnoseAna.adcMaxRange     = 4096;   /* 4096 : max range of adc */
    g_diagnoseAna.dacMaxRange     = 256;    /* 256 : max range of dac */
    g_diagnoseAna.errRangePercent = 100; /* 100: adc value error range */
    DCL_ADC_SOCxSelectChannel(g_diagnoseAna.adcHandle->baseAddress, g_diagnoseAna.socx, DIAGNOSE_ADC2_CHANNEL);
}
/*--------------------------------- clock diagnose enviroment init ----------------------------------------*/
/**
  * @brief CLock subsysterm's systerm clock single accuracy diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_SingleDiagnoseSysClockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_HS_SYS;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_8192;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HOSC;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_CHECK_END_MASK;
    g_diagnoseClock.checkEndDelayTimeUs           = 2000; /* 2000: wait check end interrupt */
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_STARTUP); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's hosc clock single accuracy diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_SingleDiagnoseHoscCLockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_HOSC;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_8192;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HS_SYS;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_CHECK_END_MASK;
    g_diagnoseClock.checkEndDelayTimeUs           = 2000; /* 2000: wait check end interrupt */
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_STARTUP); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's losc clock single accuracy diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_SingleDiagnoseLoscCLockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_LOSC;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_0;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HOSC;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_CHECK_END_MASK;
    g_diagnoseClock.checkEndDelayTimeUs           = 2000; /* 2000: wait check end interrupt */
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_STARTUP); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's tcxo clock accuracy single diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_SingleDiagnoseTcxoCLockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_TCXO;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_8192;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HOSC;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_CHECK_END_MASK;
    g_diagnoseClock.checkEndDelayTimeUs           = 2000; /* 2000: wait check end interrupt */
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_STARTUP); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's systerm clock accuracy continue diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_ContinueDiagnoseSysClockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_HS_SYS;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_8192;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HOSC;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_FREQ_ERR_MASK;
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_RUNTIME); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's hosc clock accuracy continue diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_ContinueDiagnoseHoscCLockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_HOSC;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_8192;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HS_SYS;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_FREQ_ERR_MASK;
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_RUNTIME); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's losc clock accuracy continue diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_ContinueDiagnoseLoscCLockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_LOSC;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_0;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HOSC;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_FREQ_ERR_MASK;
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_RUNTIME); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's tcxo clock accuracy continue diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_ContinueDiagnoseTcxoCLockAccuracyInit(void)
{
    g_diagnoseClock.cmmHandle                     = &g_cmm;
    g_diagnoseClock.cmmHandle->targetClockSource  = CMM_TARGET_CLK_TCXO;
    g_diagnoseClock.cmmHandle->targetFreqDivision = CMM_TARGET_FREQ_DIV_8192;
    g_diagnoseClock.cmmHandle->refClockSource     = CMM_REF_CLK_HOSC;
    g_diagnoseClock.cmmHandle->refFreqDivision    = CMM_REF_FREQ_DIV_32;
    g_diagnoseClock.cmmHandle->interruptType      = CMM_INT_FREQ_ERR_MASK;
    g_diagnoseClock.errRangePercent               = 10; /* 10: cmm windows value error range */
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle); /* stop cmm monitor clock frequency */
    BASE_FUNC_DELAY_US(200); /* 200: wait cmm stable */
    CLOCK_CmmWindowsBoundCalculate(&g_diagnoseClock, MOMENT_RUNTIME); /* calculate windows bound with error range */
    HAL_CMM_Init(g_diagnoseClock.cmmHandle);
    HAL_CMM_Start(g_diagnoseClock.cmmHandle); /* start cmm monitor clock frequency */
}
/**
  * @brief CLock subsysterm's pll ref clock accuracy single diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_SingleDiagnosePllRefCLockStopInit(void)
{
    g_diagnoseClock.cfdHandle                    = &g_cfd;
    g_diagnoseClock.cfdHandle->interruptType     = CFD_INT_CHECK_END_MASK;
    g_diagnoseClock.checkEndDelayTimeUs          = 1000; /* 1000: wait check end interrupt */
    g_diagnoseClock.errRangePercent              = 10; /* 10: cfd windows value error range */
    HAL_CFD_Stop(g_diagnoseClock.cfdHandle); /* stop cfd monitor clock stop */
    BASE_FUNC_DELAY_US(200); /* 200: wait cfd stable */
    CLOCK_CfdWindowsBoundCalculate(&g_diagnoseClock, MOMENT_STARTUP); /* calculate windows bound with error range */
    HAL_CFD_Init(g_diagnoseClock.cfdHandle);
    HAL_CFD_Start(g_diagnoseClock.cfdHandle); /* start cfd monitor clock stop */
}
/**
  * @brief CLock subsysterm's pll ref clock accuracy continue diagnose init.
  * @param None.
  * @retval None.
  */
void CLOCK_ContinueDiagnosePllRefCLockStopInit(void)
{
    g_diagnoseClock.cfdHandle                    = &g_cfd;
    g_diagnoseClock.cfdHandle->interruptType     = CFD_INT_PLL_REF_CLOCK_STOP_MASK;
    g_diagnoseClock.errRangePercent              = 50; /* 50: cfd windows value error range */
    HAL_CFD_Stop(g_diagnoseClock.cfdHandle); /* stop cfd monitor clock stop */
    BASE_FUNC_DELAY_US(200); /* 200: wait cfd stable */
    CLOCK_CfdWindowsBoundCalculate(&g_diagnoseClock, MOMENT_RUNTIME); /* calculate windows bound with error range */
    HAL_CFD_Init(g_diagnoseClock.cfdHandle);
    HAL_CFD_Start(g_diagnoseClock.cfdHandle); /* start cfd monitor clock stop */
    BASE_FUNC_DELAY_US(200); /* 200 : wait clock stabilization */
}
/*--------------------------------- compute diagnose enviroment init ----------------------------------------*/
/**
  * @brief Compute subsysterm's crc32 calculate correct diagnose init.
  * @param None.
  * @retval None.
  */
void COMPUTE_DiagnoseCrcInit(void)
{
    g_diagnoseCompute.crcHandle                    = &g_crc;
    g_diagnoseCompute.inputTestData                = 0xADDAA55A; /* 0xADDAA55A: input test data */
    g_diagnoseCompute.inputDataSize                = 4; /* 4: 32bit width */
    g_diagnoseCompute.outputRefData                = 0x89F75D91; /* 0x89F75D91: output ref data */
}
/*--------------------------------- monitor diagnose enviroment init ----------------------------------------*/
/**
  * @brief Monitor subsysterm's wdg reset function diagnose init.
  * @param None.
  * @retval None.
  */
void MONITOR_DiagnoseWdgResetInit(void)
{
    g_diagnoseMonitor.wdgHandle                  = &g_wdg;
    g_diagnoseMonitor.startupResetTimeUs         = 100; /* 100: wdg startup reset time interval */
    HAL_WDG_Stop(g_diagnoseMonitor.wdgHandle); /* stop wdg */
    HAL_WDG_SetTimeValue(g_diagnoseMonitor.wdgHandle, g_diagnoseMonitor.startupResetTimeUs, WDG_TIME_UNIT_US);
}

/**
  * @brief Monitor subsysterm's wdg program stuck diagnose init.
  * @param None.
  * @retval None.
  */
void MONITOR_DiagnoseWdgProgramStuckInit(void)
{
    g_diagnoseMonitor.wdgHandle                  = &g_wdg;
    g_diagnoseMonitor.startupResetTimeUs         = 1000000; /* 1000000: wdg runtime reset time interval */
    HAL_WDG_Stop(g_diagnoseMonitor.wdgHandle); /* stop wdg */
    HAL_WDG_SetTimeValue(g_diagnoseMonitor.wdgHandle, g_diagnoseMonitor.startupResetTimeUs, WDG_TIME_UNIT_US);
    HAL_WDG_Start(g_diagnoseMonitor.wdgHandle);
}
/**
  * @brief Monitor subsysterm's iwdg reset function diagnose init.
  * @param None.
  * @retval None.
  */
void MONITOR_DiagnoseIwdgResetInit(void)
{
    g_diagnoseMonitor.iwdgHandle                 = &g_iwdg;
    g_diagnoseMonitor.startupResetTimeUs         = 100; /* 100: iwdg startup reset time interval */
    HAL_IWDG_Stop(g_diagnoseMonitor.iwdgHandle); /* stop iwdg */
    HAL_IWDG_SetTimeValue(g_diagnoseMonitor.iwdgHandle, g_diagnoseMonitor.startupResetTimeUs, IWDG_TIME_UNIT_US);
}

/**
  * @brief Monitor subsysterm's iwdg program stuck diagnose init.
  * @param None.
  * @retval None.
  */
void MONITOR_DiagnoseIwdgProgramStuckInit(void)
{
    g_diagnoseMonitor.iwdgHandle                 = &g_iwdg;
    g_diagnoseMonitor.startupResetTimeUs         = 1000000; /* 1000000: iwdg runtime reset time interval */
    HAL_IWDG_Stop(g_diagnoseMonitor.iwdgHandle); /* stop iwdg */
    HAL_IWDG_SetTimeValue(g_diagnoseMonitor.iwdgHandle, g_diagnoseMonitor.startupResetTimeUs, IWDG_TIME_UNIT_US);
    HAL_IWDG_Start(g_diagnoseMonitor.iwdgHandle);
}
/**
  * @brief Monitor subsysterm's pmc voltage low power diagnose init.
  * @param None.
  * @retval None.
  */
void MONITOR_DiagnosePmcLowPowerInit(void)
{
    g_diagnoseMonitor.pmcHandle                  = &g_pmc; /* monitor of pmc */
}
/**
  * @brief Monitor subsysterm's tsensor over temperature diagnose init.
  * @param None.
  * @retval None.
  */
void MONITOR_DiagnoseTsensorOverTemperatureInit(void)
{
    g_diagnoseMonitor.overTemperatureValue       = 90; /* 90: over temperature value */
    HAL_TSENSOR_Init();
}
/*--------------------------------- ram diagnose enviroment init ----------------------------------------*/
/**
  * @brief Ram subsysterm's startup diagnose init.
  * @param None.
  * @retval None.
  */
void RAM_DiagnoseStartupInit(void)
{
    g_diagnoseRam.sramStartAddr                  = STARTUP_RAM_START;  /* ram start address */
    g_diagnoseRam.sramCurrentAddr                = STARTUP_RAM_START;  /* rom current address */
    g_diagnoseRam.sramCurrentAddrInv = (unsigned int *)(~(unsigned int)(void*)g_diagnoseRam.sramCurrentAddr);
    g_diagnoseRam.sramEndAddr                    = STARTUP_RAM_END;
    g_diagnoseRam.sramPattern                    = BCKGRND_PATTERN;
}
/**
  * @brief Ram subsysterm's runtime diagnose init.
  * @param None.
  * @retval None.
  */
void RAM_DiagnoseRuntimeInit(void)
{
    g_diagnoseRam.sramStartAddr                  = RUNTIME_RAM_START;  /* ram start address */
    g_diagnoseRam.sramCurrentAddr                = RUNTIME_RAM_START;  /* rom current address */
    g_diagnoseRam.sramCurrentAddrInv = (unsigned int *)(~(unsigned int)(void*)g_diagnoseRam.sramCurrentAddr);
    g_diagnoseRam.sramEndAddr                    = RUNTIME_RAM_END;
    g_diagnoseRam.sramPattern                    = BCKGRND_PATTERN;
}
/*--------------------------------- rom diagnose enviroment init ----------------------------------------*/
/**
  * @brief Rom subsysterm's intgrity diagnose init.
  * @param None.
  * @retval None.
  */
void ROM_DiagnoseIntegrityInit(void)
{
    g_diagnoseRom.crcHandle                      = &g_crc;
    g_diagnoseRom.startAddr                      = ROM_START;  /* rom start address */
    g_diagnoseRom.indexPointer                   = ROM_START;  /* rom end address */
    g_diagnoseRom.indexPointerInv = (unsigned int *)(~(unsigned int)(void*)ROM_START);
    g_diagnoseRom.flashBlockSizeInWords          = FLASH_BLOCK_SIZE_IN_WORD;
}
/**
  * @brief Rom subsysterm's flash ecc diagnose init.
  * @param None.
  * @retval None.
  */
void ROM_DiagnoseFlashEccInit(void)
{
    g_diagnoseRom.flashBase                      = EFC;   /* ecc function in eflash reg */
    g_diagnoseRom.eccIntRegVal                   = EFLASH_ECC_INT_ENABLE_VALUE;
}
/*--------------------------------- timers diagnose enviroment init ----------------------------------------*/
/**
  * @brief Timers subsysterm's timer0 interrupt interval diagnose init.
  * @param None.
  * @retval None.
  */
void TIMERS_DiagnoseTimer0InterruptIntervalInit(void)
{
    g_diagnoseTimers.timerHandle                 = &g_timer0;
    g_diagnoseTimers.setTimeUs                   = 1000;  /* 1000: timer interval */
    g_diagnoseTimers.errRangeUs                  = 1; /* 1: 1us error range */
    g_diagnoseTimers.timerHandle->load = (HAL_CRG_GetIpFreq((void *)TIMER0) / 1000000u) * g_diagnoseTimers.setTimeUs;
    g_diagnoseTimers.timerHandle->bgLoad         = g_diagnoseTimers.timerHandle->load;
    HAL_TIMER_Config(g_diagnoseTimers.timerHandle, TIMER_CFG_LOAD);  /* config load time */
    HAL_TIMER_Config(g_diagnoseTimers.timerHandle, TIMER_CFG_BGLOAD);  /* config bgload time */
    g_diagnoseTimers.pretimeCycle                = BASE_FUNC_GetTick();
    HAL_TIMER_Start(g_diagnoseTimers.timerHandle);
    BASE_FUNC_DELAY_US(1500); /* 1500 : wait timer interrupt */
}
/**
  * @brief Timers subsysterm's timer1 interrupt interval diagnose init.
  * @param None.
  * @retval None.
  */
void TIMERS_DiagnoseTimer1InterruptIntervalInit(void)
{
    g_diagnoseTimers.timerHandle                 = &g_timer1;
    g_diagnoseTimers.setTimeUs                   = 1000;  /* 1000: timer interval */
    g_diagnoseTimers.errRangeUs                  = 1; /* 1: 1us error range */
    g_diagnoseTimers.timerHandle->load  = (HAL_CRG_GetIpFreq((void *)TIMER1) / 1000000u) * g_diagnoseTimers.setTimeUs;
    g_diagnoseTimers.timerHandle->bgLoad         = g_diagnoseTimers.timerHandle->load;
    HAL_TIMER_Config(g_diagnoseTimers.timerHandle, TIMER_CFG_LOAD);  /* config load time */
    HAL_TIMER_Config(g_diagnoseTimers.timerHandle, TIMER_CFG_BGLOAD);  /* config bgload time */
    g_diagnoseTimers.pretimeCycle                = BASE_FUNC_GetTick();
    HAL_TIMER_Start(g_diagnoseTimers.timerHandle);
    BASE_FUNC_DELAY_US(1500); /* 1500 : wait timer interrupt */
}
/**
  * @brief Timers subsysterm's timer2 interrupt interval diagnose init.
  * @param None.
  * @retval None.
  */
void TIMERS_DiagnoseTimer2InterruptIntervalInit(void)
{
    g_diagnoseTimers.timerHandle                 = &g_timer2;
    g_diagnoseTimers.setTimeUs                   = 1000;  /* 1000: timer interval */
    g_diagnoseTimers.errRangeUs                  = 1; /* 1: 1us error range */
    g_diagnoseTimers.timerHandle->load = (HAL_CRG_GetIpFreq((void *)TIMER2) / 1000000u) * g_diagnoseTimers.setTimeUs;
    g_diagnoseTimers.timerHandle->bgLoad         = g_diagnoseTimers.timerHandle->load;
    HAL_TIMER_Config(g_diagnoseTimers.timerHandle, TIMER_CFG_LOAD);  /* config load time */
    HAL_TIMER_Config(g_diagnoseTimers.timerHandle, TIMER_CFG_BGLOAD);  /* config bgload time */
    g_diagnoseTimers.pretimeCycle                = BASE_FUNC_GetTick();
    HAL_TIMER_Start(g_diagnoseTimers.timerHandle);
    BASE_FUNC_DELAY_US(1500); /* 1500 : wait timer interrupt */
}
/*--------------------------------- irq diagnose enviroment init ----------------------------------------*/
/**
  * @brief CFD check end callback function.
  * @param handle @ref CFD_Handle.
  * @retval None.
  */
void CFDCheckEndCallback(CFD_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseClock.cfdIrqFlag = true;  /* set irq flag */
    g_diagnoseClock.cfdClockCount = DCL_CFD_GetCntValue(g_diagnoseClock.cfdHandle->baseAddress);
    HAL_CFD_Stop(g_diagnoseClock.cfdHandle);
}
/**
  * @brief CFD clock stop callback function.
  * @param handle @ref CFD_Handle.
  * @retval None.
  */
void CFDClockStopCallback(CFD_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseClock.cfdIrqFlag = true;  /* set irq flag */
    g_diagnoseClock.cfdClockCount = DCL_CFD_GetCntValue(g_diagnoseClock.cfdHandle->baseAddress);
}
/**
  * @brief CMM check end callback function.
  * @param handle @ref CMM_Handle.
  * @retval None.
  */
void CMMCheckEndCallback(CMM_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseClock.cmmIrqFlag = true;  /* set irq flag */
    g_diagnoseClock.cmmClockCount = DCL_CMM_GetCntValue(g_diagnoseClock.cmmHandle->baseAddress);
    HAL_CMM_Stop(g_diagnoseClock.cmmHandle);
}
/**
  * @brief CMM frequency error callback function.
  * @param handle @ref CMM_Handle.
  * @retval None.
  */
void CMMFreqErrorCallback(CMM_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseClock.cmmIrqFlag = true;  /* set irq flag */
    g_diagnoseClock.cmmClockCount = DCL_CMM_GetCntValue(g_diagnoseClock.cmmHandle->baseAddress);
}
/**
  * @brief PMC low poer callback function.
  * @param handle @ref PMC_Handle.
  * @retval None.
  */
void PMCPVDInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseMonitor.pmcIrqFlag = true;  /* set irq flag */
}
/**
  * @brief Timer0 interrupt callback function.
  * @param handle @ref Timer_Handle.
  * @retval None.
  */
void TIMER0CallbackFunction(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseTimers.timerIrqFlag =true;  /* set irq flag */
    g_diagnoseTimers.currenttimeCycle = BASE_FUNC_GetTick();
    /* duration cycle calculate */
    unsigned int durationCycle = (g_diagnoseTimers.currenttimeCycle > g_diagnoseTimers.pretimeCycle) ? \
        (g_diagnoseTimers.currenttimeCycle - g_diagnoseTimers.pretimeCycle) : \
        (SYSTICK_MAX_VALUE + g_diagnoseTimers.currenttimeCycle - g_diagnoseTimers.pretimeCycle + 1);
    g_diagnoseTimers.durationUs = durationCycle / HAL_CRG_GetCoreClkFreq() / CRG_FREQ_1MHz;
    g_diagnoseTimers.pretimeCycle = g_diagnoseTimers.currenttimeCycle;  /* prepare for next calculate */
}
/**
  * @brief Timer1 interrupt callback function.
  * @param handle @ref Timer_Handle.
  * @retval None.
  */
void TIMER1CallbackFunction(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseTimers.timerIrqFlag =true;  /* set irq flag */
    g_diagnoseTimers.currenttimeCycle = BASE_FUNC_GetTick();
    /* duration cycle calculate */
    unsigned int durationCycle = (g_diagnoseTimers.currenttimeCycle > g_diagnoseTimers.pretimeCycle) ? \
        (g_diagnoseTimers.currenttimeCycle - g_diagnoseTimers.pretimeCycle) : \
        (SYSTICK_MAX_VALUE + g_diagnoseTimers.currenttimeCycle - g_diagnoseTimers.pretimeCycle + 1);
    g_diagnoseTimers.durationUs = durationCycle / HAL_CRG_GetCoreClkFreq() / CRG_FREQ_1MHz;
    g_diagnoseTimers.pretimeCycle = g_diagnoseTimers.currenttimeCycle;  /* prepare for next calculate */
}
/**
  * @brief Timer2 interrupt callback function.
  * @param handle @ref Timer_Handle.
  * @retval None.
  */
void TIMER2CallbackFunction(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    g_diagnoseTimers.timerIrqFlag =true;  /* set irq flag */
    g_diagnoseTimers.currenttimeCycle = BASE_FUNC_GetTick();
    /* duration cycle calculate */
    unsigned int durationCycle = (g_diagnoseTimers.currenttimeCycle > g_diagnoseTimers.pretimeCycle) ? \
        (g_diagnoseTimers.currenttimeCycle - g_diagnoseTimers.pretimeCycle) : \
        (SYSTICK_MAX_VALUE + g_diagnoseTimers.currenttimeCycle - g_diagnoseTimers.pretimeCycle + 1);
    g_diagnoseTimers.durationUs = durationCycle / HAL_CRG_GetCoreClkFreq() / CRG_FREQ_1MHz;
    g_diagnoseTimers.pretimeCycle = g_diagnoseTimers.currenttimeCycle;  /* prepare for next calculate */
}