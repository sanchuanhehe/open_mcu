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
#include "iocmg_ip.h"

#define UART0_BAND_RATE 115200

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG; /* crg baseaddr */
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 48; /* PLL Multiplier 48 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_3;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    /* The 1 MHz freq is equal to the input clock frequency / (clk_1m_div + 1). 25 is the div of the clk_1m in CLOCK. */
    crg.handleEx.clk1MDiv = (25 - 1);

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE); /* ADC IP Enable */
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_ASYN_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_1);

    g_adc0.baseAddress = ADC0; /* adc0 */
    g_adc0.socPriority = ADC_PRIMODE_ALL_ROUND;

    HAL_ADC_Init(&g_adc0);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA17; /* DAC_OUT(ADC AIN17) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_500CLK; /* adc sample total time 500 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM0, &socParam); /* config soc */
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
    HAL_CRG_IpEnableSet(CFD_BASE, IP_CLK_ENABLE);

    g_cfd.baseAddress = CFD;
    g_cfd.upperBound = 20;  /* 20: windows upbound */
    g_cfd.interruptType = CFD_INT_PLL_REF_CLOCK_STOP_MASK; /* pll ref clock stop error interrupt */

    HAL_CFD_RegisterCallback(&g_cfd, CFD_INT_CHECK_END_MASK, (CFD_CallBackFuncType)CFDCheckEndCallback);
    HAL_CFD_RegisterCallback(&g_cfd, CFD_INT_PLL_REF_CLOCK_STOP_MASK, (CFD_CallBackFuncType)CFDClockStopCallback);

    IRQ_Register(IRQ_CFD, HAL_CFD_IrqHandler, &g_cfd);
    IRQ_SetPriority(IRQ_CFD, 1); /* 1 is priority value */
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
    HAL_CRG_IpEnableSet(CMM_BASE, IP_CLK_ENABLE);

    g_cmm.baseAddress = CMM;
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
    IRQ_SetPriority(IRQ_CMM, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_CMM);
    return HAL_CMM_Init(&g_cmm);
}

static void CRC_Init(void)
{
    HAL_CRG_IpEnableSet(CRC_BASE, IP_CLK_ENABLE);

    g_crc.baseAddress = CRC;   /* crc baseAddr */

    g_crc.inputDataFormat = CRC_MODE_BIT8;
    g_crc.handleEx.algoMode = CRC32;  /* crc32 algorythm */
    HAL_CRC_Init(&g_crc);
}

static void DAC0_Init(void)
{
    HAL_CRG_IpEnableSet(DAC0_BASE, IP_CLK_ENABLE);  /* DAC0 clock enable. */

    g_dac0.baseAddress = DAC0;
    g_dac0.dacValue = 512;  /* 512 : half vdda */
    HAL_DAC_Init(&g_dac0);
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
    g_pmc.wakeupSrc = PMC_WAKEUP_NONE;
    g_pmc.pvdEnable = BASE_CFG_ENABLE;  /* enable volatege monitor */
    g_pmc.pvdThreshold = PMC_PVD_THRED_LEVEL7; /* pvd level 7 */
    HAL_PMC_Init(&g_pmc);
    HAL_PMC_RegisterCallback(&g_pmc, PMC_PVD_INT_ID, PMCPVDInterruptCallback);
    IRQ_Register(IRQ_PVD, HAL_PMC_IrqHandler, &g_pmc);
    IRQ_SetPriority(IRQ_PVD, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_PVD);
}

__weak void TIMER0CallbackFunction(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN TIMER0CallbackFunction */
    /* USER CODE END TIMER0CallbackFunction */
}

static void TIMER0_Init(void)
{
    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE);  /* TIMER0 clock enable. */
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER0) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 100;

    g_timer0.baseAddress = TIMER0;
    g_timer0.load        = load - 1; /* Set timer value immediately */
    g_timer0.bgLoad      = load - 1; /* Set timer value */
    g_timer0.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer0.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer0.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer0.interruptEn = BASE_CFG_ENABLE;
    g_timer0.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer0.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer0);
    IRQ_Register(IRQ_TIMER0, HAL_TIMER_IrqHandler, &g_timer0);

    HAL_TIMER_RegisterCallback(&g_timer0, TIMER_PERIOD_FIN, TIMER0CallbackFunction);
    IRQ_SetPriority(IRQ_TIMER0, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER0);
}

__weak void TIMER1CallbackFunction(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN TIMER1CallbackFunction */
    /* USER CODE END TIMER1CallbackFunction */
}

static void TIMER1_Init(void)
{
    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);  /* TIMER1 clock enable. */
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER1) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 100;

    g_timer1.baseAddress = TIMER1;
    g_timer1.load        = load - 1; /* Set timer value immediately */
    g_timer1.bgLoad      = load - 1; /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    g_timer1.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer1);
    IRQ_Register(IRQ_TIMER1, HAL_TIMER_IrqHandler, &g_timer1);

    HAL_TIMER_RegisterCallback(&g_timer1, TIMER_PERIOD_FIN, TIMER1CallbackFunction);
    IRQ_SetPriority(IRQ_TIMER1, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER1);
}

__weak void TIMER2CallbackFunction(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN TIMER2CallbackFunction */
    /* USER CODE END TIMER2CallbackFunction */
}

static void TIMER2_Init(void)
{
    HAL_CRG_IpEnableSet(TIMER2_BASE, IP_CLK_ENABLE);  /* TIMER2 clock enable. */
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER2) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 100;

    g_timer2.baseAddress = TIMER2;
    g_timer2.load        = load - 1; /* Set timer value immediately */
    g_timer2.bgLoad      = load - 1; /* Set timer value */
    g_timer2.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer2.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer2.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer2.interruptEn = BASE_CFG_ENABLE;
    g_timer2.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer2.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer2);
    IRQ_Register(IRQ_TIMER2, HAL_TIMER_IrqHandler, &g_timer2);

    HAL_TIMER_RegisterCallback(&g_timer2, TIMER_PERIOD_FIN, TIMER2CallbackFunction);
    IRQ_SetPriority(IRQ_TIMER2, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER2);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);  /* UART0 clock enable. */
    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAND_RATE; /* baud rate */
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;  /* no parity */
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;  /* fifo enable */
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void WWDG_Init(void)
{
    HAL_CRG_IpEnableSet(WWDG_BASE, IP_CLK_ENABLE);  /* WWDG clock enable. */
    g_wwdg.baseAddress = WWDG;

    g_wwdg.timeValue = 1000000; /* 1000000 is time value */
    g_wwdg.timeType = WWDG_TIME_UNIT_US;
    g_wwdg.enableIT = BASE_CFG_DISABLE;
    g_wwdg.freqDivValue = WWDG_FREQ_DIV_NONE;
    HAL_WWDG_Init(&g_wwdg);
}

static void IWDG_Init(void)
{
    HAL_CRG_IpEnableSet(IWDG_BASE, IP_CLK_ENABLE);  /* IWDG clock enable. */
    g_iwdg.baseAddress = IWDG;

    g_iwdg.timeValue = 100000; /* 100000 is time value */
    g_iwdg.timeType = IWDG_TIME_UNIT_US;
    g_iwdg.freqDivValue = IWDG_FREQ_DIV_256;
    HAL_IWDG_Init(&g_iwdg);
}

static void IOConfig(void)
{
    /* Config PIN39 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_3_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_3_AS_UART0_TXD, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_3_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_3_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_3_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN40 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_4_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_4_AS_UART0_RXD, PULL_UP);  /* Pull-up and Pull-down, UART RX recommend PULL_UP */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_4_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_4_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_4_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();  /* ioconfig init */
    UART0_Init();
    ADC0_Init();
    CRC_Init();
    CMM_Init();
    CFD_Init();
    DAC0_Init();
    PMC_Init();
    TIMER0_Init();
    TIMER1_Init();
    TIMER2_Init();
    IWDG_Init();  /* iwdg init */
    WWDG_Init();  /* wwdg init */
    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}