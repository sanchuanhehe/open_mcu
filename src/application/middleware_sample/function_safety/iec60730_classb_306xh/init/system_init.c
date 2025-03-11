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
  * @file      system_init.c
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

#include "main.h"
#include "ioconfig.h"
#include "iocmg.h"

#define UART0_BAND_RATE 115200

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) { /* crg init */
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void ADC1_Init(void)
{
    HAL_CRG_IpEnableSet(ADC1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC1_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC1_BASE, CRG_ADC_DIV_5);

    g_adc1.baseAddress = ADC1;
    g_adc1.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc1.handleEx.vrefBuf = ADC_VREF_2P5V;
    HAL_ADC_Init(&g_adc1);
    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINB7; /* DAC1_OUT(ADC INB7) */
    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 127; /* adc sample total time 127 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT; /* soft trigger adc sample */
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM0, &socParam); /* config soc */
}

static void ADC2_Init(void)
{
    HAL_CRG_IpEnableSet(ADC2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC2_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC2_BASE, CRG_ADC_DIV_5);

    g_adc2.baseAddress = ADC2;
    g_adc2.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc2.handleEx.vrefBuf = ADC_VREF_2P5V;
    HAL_ADC_Init(&g_adc2);
    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINB7; /* DAC2_OUT(ADC INB7) */
    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 127; /* adc sample total time 127 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT; /* soft trigger adc sample */
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM0, &socParam); /* config soc */
}

__weak void CFDCheckEndCallback(CFD_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN CFD_INT_CHECK_END_MASK */
    /* USER CODE END CFD_INT_CHECK_END_MASK */
}

__weak void CFDClockStopCallback(CFD_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN CFD_INT_PLL_REF_CLOCK_STOP_MASK */
    /* USER CODE END CFD_INT_PLL_REF_CLOCK_STOP_MASK */
}

static BASE_StatusType CFD_Init(void)
{
    g_cfd.baseAddress = CFD;
    g_cfd.upperBound = 2;  /* 2: windows upbound */
    g_cfd.interruptType = CFD_INT_PLL_REF_CLOCK_STOP_MASK; /* pll ref clock stop error interrupt */

    HAL_CFD_RegisterCallback(&g_cfd, CFD_INT_CHECK_END_MASK, (CFD_CallBackFuncType)CFDCheckEndCallback);
    HAL_CFD_RegisterCallback(&g_cfd, CFD_INT_PLL_REF_CLOCK_STOP_MASK, (CFD_CallBackFuncType)CFDClockStopCallback);

    IRQ_Register(IRQ_CFD, HAL_CFD_IrqHandler, &g_cfd); /* register cfd irq callback */
    IRQ_SetPriority(IRQ_CFD, 1); /* interrupt priority */
    IRQ_EnableN(IRQ_CFD);
    return HAL_CFD_Init(&g_cfd);
}

__weak void CMMCounterOverFlowCallback(CMM_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN CMM_INT_COUNTER_OVERFLOW_MASK */
    /* USER CODE END CMM_INT_COUNTER_OVERFLOW_MASK */
}

__weak void CMMCheckEndCallback(CMM_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN CMM_INT_CHECK_END_MASK */
    /* USER CODE END CMM_INT_CHECK_END_MASK */
}

__weak void CMMFreqErrorCallback(CMM_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN CMM_INT_FREQ_ERR_MASK */
    /* USER CODE END CMM_INT_FREQ_ERR_MASK */
}

static BASE_StatusType CMM_Init(void)
{
    g_cmm.baseAddress = CMM;
    g_cmm.mode = CMM_TRIGGER_RISE;
    g_cmm.targetFreqDivision = CMM_TARGET_FREQ_DIV_8192; /* target clock 8192 division */
    g_cmm.refFreqDivision = CMM_REF_FREQ_DIV_32; /* ref clock 32 division */
    g_cmm.targetClockSource = CMM_TARGET_CLK_HOSC;
    g_cmm.refClockSource = CMM_REF_CLK_LOSC;
    g_cmm.upperBound = 65535; /* 65535: upbound limit */
    g_cmm.lowerBound = 0;
    g_cmm.interruptType = CMM_INT_FREQ_ERR_MASK;  /* clock frequency error interrupt */

    HAL_CMM_RegisterCallback(&g_cmm, CMM_INT_COUNTER_OVERFLOW_MASK, (CMM_CallBackFuncType)CMMCounterOverFlowCallback);
    HAL_CMM_RegisterCallback(&g_cmm, CMM_INT_CHECK_END_MASK, (CMM_CallBackFuncType)CMMCheckEndCallback);
    HAL_CMM_RegisterCallback(&g_cmm, CMM_INT_FREQ_ERR_MASK, (CMM_CallBackFuncType)CMMFreqErrorCallback);
    IRQ_Register(IRQ_CMM, HAL_CMM_IrqHandler, &g_cmm);
    IRQ_SetPriority(IRQ_CMM, 1); /* interrupt priority */
    IRQ_EnableN(IRQ_CMM);
    return HAL_CMM_Init(&g_cmm);
}

static void CRC_Init(void)
{
    HAL_CRG_IpEnableSet(CRC_BASE, IP_CLK_ENABLE);

    g_crc.baseAddress = CRC;
    g_crc.handleEx.algoMode = CRC32;  /* crc32 algorythm */
    g_crc.handleEx.timeout = 0;
    g_crc.handleEx.enableIT = BASE_CFG_DISABLE;
    g_crc.handleEx.enableErrInject = BASE_CFG_DISABLE;
    g_crc.inputDataFormat = CRC_MODE_BIT8; /* input data bitwidth */
    HAL_CRC_Init(&g_crc);
}

static void DAC1_Init(void)
{
    HAL_CRG_IpEnableSet(DAC1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkDivSet(DAC1_BASE, CRG_DAC_DIV_4);

    g_dac1.baseAddress = DAC1_BASE;
    g_dac1.dacValue = 128; /* 128: DAC Value */
    g_dac1.handleEx.sineMode = BASE_CFG_UNSET;
    HAL_DAC_Init(&g_dac1); /* dac init */
}

static void DAC2_Init(void)
{
    HAL_CRG_IpEnableSet(DAC2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkDivSet(DAC2_BASE, CRG_DAC_DIV_4);

    g_dac2.baseAddress = DAC2_BASE;
    g_dac2.dacValue = 128; /* 128: DAC Value */
    g_dac2.handleEx.sineMode = BASE_CFG_UNSET;
    HAL_DAC_Init(&g_dac2); /* dac init */
}

__weak void PMCPVDInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN PMC Callback */
    /* USER CODE END PMC Callback */
}

static void PMC_Init(void)
{
    HAL_CRG_IpEnableSet(PMC_BASE, IP_CLK_ENABLE);
    g_pmc.baseAddress = PMC_BASE;
    g_pmc.wakeupSrc = PMC_WAKEUP_0;  /* wakeup source from low power mode */
    g_pmc.wakeupActMode = PMC_WAKEUP_ACT_UP_EDGE;
    g_pmc.pvdEnable = BASE_CFG_ENABLE;  /* enable volatege monitor */
    g_pmc.pvdThreshold = PMC_PVD_THRED_LEVEL7; /* pvd level 7 */
    HAL_PMC_Init(&g_pmc);
    HAL_PMC_RegisterCallback(&g_pmc, PMC_PVD_INT_ID, PMCPVDInterruptCallback);
    IRQ_Register(IRQ_PVD, HAL_PMC_IrqHandler, &g_pmc);
    IRQ_SetPriority(IRQ_PVD, 1); /* interrupt priority */
    IRQ_EnableN(IRQ_PVD);
}

__weak void TIMER0CallbackFunction(void *handle)
{
    DCL_TIMER_IrqClear((TIMER_RegStruct *)handle);
    /* USER CODE BEGIN TIMER0 ITCallBackFunc */
    /* USER CODE END TIMER0 ITCallBackFunc */
}

static void TIMER0_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER0) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 100;
    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER0_BASE, CRG_PLL_NO_PREDV);
    g_timer0.baseAddress = TIMER0;
    g_timer0.load        = load - 1; /* Set timer value immediately */
    g_timer0.bgLoad      = load - 1; /* Set timer value */
    g_timer0.mode        = TIMER_MODE_RUN_ONTSHOT; /* Run in period mode */
    g_timer0.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer0.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer0.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer0.dmaReqEnable = BASE_CFG_DISABLE;
    g_timer0.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer0);
    IRQ_Register(IRQ_TIMER0, HAL_TIMER_IrqHandler, &g_timer0);
    HAL_TIMER_RegisterCallback(&g_timer0, TIMER_PERIOD_FIN, TIMER0CallbackFunction);
    IRQ_SetPriority(IRQ_TIMER0, 1);  /* interrupt priority */
    IRQ_EnableN(IRQ_TIMER0);
}

__weak void TIMER1CallbackFunction(void *handle)
{
    DCL_TIMER_IrqClear((TIMER_RegStruct *)handle);
    /* USER CODE BEGIN TIMER1 ITCallBackFunc */
    /* USER CODE END TIMER1 ITCallBackFunc */
}

static void TIMER1_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER1) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 100;
    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER1_BASE, CRG_PLL_NO_PREDV);
    g_timer1.baseAddress = TIMER1;
    g_timer1.load        = load - 1; /* Set timer value immediately */
    g_timer1.bgLoad      = load - 1; /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_ONTSHOT; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaReqEnable = BASE_CFG_DISABLE;
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer1);
    IRQ_Register(IRQ_TIMER1, HAL_TIMER_IrqHandler, &g_timer1);
    HAL_TIMER_RegisterCallback(&g_timer1, TIMER_PERIOD_FIN, TIMER1CallbackFunction);
    IRQ_SetPriority(IRQ_TIMER1, 1); /* interrupt priority */
    IRQ_EnableN(IRQ_TIMER1);
}

__weak void TIMER2CallbackFunction(void *handle)
{
    DCL_TIMER_IrqClear((TIMER_RegStruct *)handle);
    /* USER CODE BEGIN TIMER2 ITCallBackFunc */
    /* USER CODE END TIMER2 ITCallBackFunc */
}

static void TIMER2_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER2) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 100;

    HAL_CRG_IpEnableSet(TIMER2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER2_BASE, CRG_PLL_NO_PREDV);
    g_timer2.baseAddress = TIMER2;
    g_timer2.load        = load - 1; /* Set timer value immediately */
    g_timer2.bgLoad      = load - 1; /* Set timer value */
    g_timer2.mode        = TIMER_MODE_RUN_ONTSHOT; /* Run in period mode */
    g_timer2.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer2.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer2.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer2.dmaReqEnable = BASE_CFG_DISABLE;
    g_timer2.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer2);
    IRQ_Register(IRQ_TIMER2, HAL_TIMER_IrqHandler, &g_timer2);
    HAL_TIMER_RegisterCallback(&g_timer2, TIMER_PERIOD_FIN, TIMER2CallbackFunction);
    IRQ_SetPriority(IRQ_TIMER2, 1); /* interrupt priority */
    IRQ_EnableN(IRQ_TIMER2);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);
    g_uart0.baseAddress = UART0;
    g_uart0.baudRate = UART0_BAND_RATE; /* baud rate */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;  /* no parity */
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;  /* fifo enable */
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void WDG_Init(void)
{
    HAL_CRG_IpEnableSet(WDG_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(WDG_BASE, CRG_PLL_NO_PREDV);
    g_wdg.baseAddress = WDG;
    g_wdg.timeValue = 1000000;  /* 1000000 : wdg reset interval 1000000us */
    g_wdg.timeType = WDG_TIME_UNIT_US;
    g_wdg.enableIT = BASE_CFG_DISABLE;
    HAL_WDG_Init(&g_wdg); /* wdg init */
}

static void IWDG_Init(void)
{
    HAL_CRG_IpEnableSet(IWDG_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(IWDG_BASE, CRG_PLL_NO_PREDV);
    g_iwdg.baseAddress = IWDG;
    g_iwdg.timeValue = 1000000;  /* 1000000 : iwdg reset interval 1000000us */
    g_iwdg.timeType = IWDG_TIME_UNIT_US;
    g_iwdg.enableIT = BASE_CFG_DISABLE;
    HAL_IWDG_Init(&g_iwdg); /* iwdg init */
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;

    HAL_IOCMG_SetPinAltFuncMode(IO52_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO52_AS_UART0_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO52_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO52_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO52_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

 /* UART RX recommend PULL_UP */
    HAL_IOCMG_SetPinAltFuncMode(IO53_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO53_AS_UART0_RXD, PULL_UP);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO53_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO53_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO53_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    /* system init */
    IOConfig();  /* ioconfig init */
    UART0_Init();
    ADC1_Init();
    ADC2_Init();
    CRC_Init();
    CMM_Init();
    CFD_Init();
    DAC1_Init();
    DAC2_Init();
    PMC_Init();
    TIMER0_Init();
    TIMER1_Init();
    TIMER2_Init();
    IWDG_Init();  /* iwdg init */
    WDG_Init();  /* wdg init */
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}