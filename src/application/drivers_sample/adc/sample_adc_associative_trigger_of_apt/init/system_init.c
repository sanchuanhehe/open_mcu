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

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

__weak void ADC_Init2FromApt(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ADC2_CALLBACK_INT2 */
    /* USER CODE END ADC2_CALLBACK_INT2 */
}

static void ADC2_Init(void)
{
    HAL_CRG_IpEnableSet(ADC2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC2_BASE, CRG_ADC_CLK_SELECT_PLL_DIV); /* select PLL as clock. */
    HAL_CRG_IpClkDivSet(ADC2_BASE, CRG_ADC_DIV_5);

    g_adc.baseAddress = ADC2;
    g_adc.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc.handleEx.vrefBuf = ADC_VREF_2P5V;

    HAL_ADC_Init(&g_adc);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA7; /* PIN36(ADC INA7) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 127; /* adc sample total time 127 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;  /* select apt0 as TrigSource */
    socParam.finishMode = ADC_SOCFINISH_INT2;
    HAL_ADC_ConfigureSoc(&g_adc, ADC_SOC_NUM1, &socParam);
    HAL_ADC_RegisterCallBack(&g_adc, ADC_CALLBACK_INT2, (ADC_CallbackType)ADC_Init2FromApt);

    IRQ_Register(IRQ_ADC2_INT2, HAL_ADC_IrqHandlerInt2, &g_adc);
    IRQ_SetPriority(IRQ_ADC2_INT2, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_ADC2_INT2);
}

__weak void APT0EventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_EVENT_INTERRUPT */
    /* USER CODE END APT0_EVENT_INTERRUPT */
}

__weak void APT0TimerCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_TIMER_INTERRUPT */
    /* USER CODE END APT0_TIMER_INTERRUPT */
}

static void APT0_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;    /* APT protection action. */
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;    /* Enable APT even interrupt. */
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_hAptU, &protectApt);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_hAptU.baseAddress = APT0;

    /* Clock Settings */
    g_hAptU.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptU.waveform.timerPeriod = 20000; /* 20000 is count period of APT time-base timer */
    g_hAptU.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_hAptU.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptU.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.divInitVal = 0;
    g_hAptU.waveform.cntInitVal = 0;
    g_hAptU.waveform.cntCmpLeftEdge = 1; /* 1 is count compare point of the left edge of PWM waveform */
    g_hAptU.waveform.cntCmpRightEdge = 19999; /* 19999 is count compare point of the right edge of PWM waveform */
    g_hAptU.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptU.waveform.deadBandCnt = 60; /* 60 is dead-band value */

    /* ADC Trigger SOCA */
    g_hAptU.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCA = 1; /* 1 is count compare point of ADC trigger source SOCA when using CMPA */
    g_hAptU.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_hAptU.adcTrg.trgScaleSOCA = 1;

    /* ADC Trigger SOCB */
    g_hAptU.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCB =  1;
    g_hAptU.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_UP;
    g_hAptU.adcTrg.trgScaleSOCB = 1;

    g_hAptU.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    /* Timer Trigger */
    g_hAptU.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_hAptU.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO_PERIOD;
    g_hAptU.tmrInterrupt.tmrInterruptScale = 1;

    APT0_ProtectInit();

    HAL_APT_PWMInit(&g_hAptU);
    IRQ_Register(IRQ_APT0_EVT, HAL_APT_EventIrqHandler, &g_hAptU);
    HAL_APT_RegisterCallBack(&g_hAptU, APT_EVENT_INTERRUPT, APT0EventCallback);
    IRQ_SetPriority(IRQ_APT0_EVT, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_APT0_EVT);

    IRQ_Register(IRQ_APT0_TMR, HAL_APT_TimerIrqHandler, &g_hAptU);
    HAL_APT_RegisterCallBack(&g_hAptU, APT_TIMER_INTERRUPT, APT0TimerCallback);
    IRQ_SetPriority(IRQ_APT0_TMR, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_APT0_TMR);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    /* Config PIN15 */
    HAL_IOCMG_SetPinAltFuncMode(IO15_AS_APT0_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO15_AS_APT0_PWMA, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO15_AS_APT0_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO15_AS_APT0_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO15_AS_APT0_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN18 */
    HAL_IOCMG_SetPinAltFuncMode(IO18_AS_APT0_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO18_AS_APT0_PWMB, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO18_AS_APT0_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO18_AS_APT0_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO18_AS_APT0_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN52 */
    HAL_IOCMG_SetPinAltFuncMode(IO52_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO52_AS_UART0_TXD, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO52_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO52_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO52_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN53 */
    HAL_IOCMG_SetPinAltFuncMode(IO53_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO53_AS_UART0_RXD, PULL_UP);  /* Pull-up and Pull-down, UART RX recommend PULL_UP */
    HAL_IOCMG_SetPinSchmidtMode(IO53_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO53_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO53_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN36 */
    HAL_IOCMG_SetPinAltFuncMode(IO36_AS_ADC2_ANA_A7);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO36_AS_ADC2_ANA_A7, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO36_AS_ADC2_ANA_A7, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO36_AS_ADC2_ANA_A7, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO36_AS_ADC2_ANA_A7, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    APT0_Init();
    ADC2_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}