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
  * @date      2025-04-17 14:22:06
  */

#include "main.h"
#include "ioconfig.h"
#include "iocmg_ip.h"

#define UART0_BAUD_RATE 1843200

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 48; /* PLL Multiplier 48 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_2;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    crg.handleEx.pllPostDiv2   = CRG_PLL_POSTDIV2_3;
    crg.handleEx.clk1MSelect   = CRG_1M_CLK_SELECT_HOSC;
    crg.handleEx.clk1MDiv = (25 - 1); /* The 1 MHz freq is equal to the input clock frequency / (clk_1m_div + 1). 25 is the div of the clk_1m in CLOCK. */

    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static void DMA_Channel0Init(void *handle)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_MEMORY_TO_PERIPH_BY_DMAC;
    dma_param.srcAddrInc = DMA_ADDR_INCREASE;
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_MEM;
    dma_param.destPeriph = DMA_REQUEST_UART0_TX;
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_ZERO);
}

static void DMA_Init(void)
{
    HAL_CRG_IpEnableSet(DMA_BASE, IP_CLK_ENABLE);
    g_dmac.baseAddress = DMA;
    IRQ_Register(IRQ_DMA_TC, HAL_DMA_IrqHandlerTc, &g_dmac);
    IRQ_Register(IRQ_DMA_ERR, HAL_DMA_IrqHandlerError, &g_dmac);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
    HAL_DMA_Init(&g_dmac);
    DMA_Channel0Init((void *)(&g_uart0));
    HAL_DMA_SetChannelPriorityEx(&g_dmac, DMA_CHANNEL_ZERO, DMA_PRIORITY_HIGHEST);
}

static void ACMP0_Init(void)
{
    HAL_CRG_IpEnableSet(ACMP0_BASE, IP_CLK_ENABLE);                /* ACMP clock bit reset. */
    g_acmp0.baseAddress =  ACMP0_BASE;
    g_acmp0.inOutConfig.inputNNum = ACMP_INPUT_N_SELECT2;
    g_acmp0.inOutConfig.inputPNum = ACMP_INPUT_P_SELECT2;
    g_acmp0.inOutConfig.polarity = ACMP_OUT_NOT_INVERT;
    g_acmp0.filterCtrl.filterMode = ACMP_FILTER_NONE;
    g_acmp0.hysteresisVol = ACMP_HYS_VOL_30MV;
    g_acmp0.interruptEn = BASE_CFG_UNSET;
    HAL_ACMP_Init(&g_acmp0);
}

static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_ASYN_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_1);

    g_adc0.baseAddress = ADC0;
    g_adc0.socPriority = ADC_PRIMODE_ALL_ROUND;

    HAL_ADC_Init(&g_adc0);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0; /* PGA0_OUT(ADC AIN0) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_22CLK; /* adc sample total time 22 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM1, &socParam);

    socParam.adcInput = ADC_CH_ADCINA1; /* PGA1_OUT(ADC AIN1) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_22CLK; /* adc sample total time 22 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM6, &socParam);

    socParam.adcInput = ADC_CH_ADCINA9; /* PIN7(ADC AIN9) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_22CLK; /* adc sample total time 22 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM0, &socParam);

    socParam.adcInput = ADC_CH_ADCINA5; /* PIN2(ADC AIN5) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_22CLK; /* adc sample total time 22 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM3, &socParam);

    socParam.adcInput = ADC_CH_ADCINA15; /* PIN14(ADC AIN15) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_22CLK; /* adc sample total time 22 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_SOFT;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM4, &socParam);
}

__weak void MotorSysErrCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_EVENT_INTERRUPT */
    /* USER CODE END APT0_EVENT_INTERRUPT */
}

__weak void MotorCarrierProcessCallback(void *aptHandle)
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
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP0;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP0_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt0, &protectApt);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_apt0.baseAddress = APT0;

    /* Clock Settings */
    g_apt0.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt0.waveform.timerPeriod = 7500; /* 7500 is count period of APT time-base timer */
    g_apt0.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt0.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt0.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.divInitVal = 0;
    g_apt0.waveform.cntInitVal = 0;
    g_apt0.waveform.cntCmpLeftEdge = 500; /* 500 is count compare point of the left edge of PWM waveform */
    g_apt0.waveform.cntCmpRightEdge = 4000; /* 4000 is count compare point of the right edge of PWM waveform */
    g_apt0.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt0.waveform.deadBandCnt = 225; /* 225 is dead-band value */

    /* ADC Trigger SOCA */
    g_apt0.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCA = 375; /* 375 is count compare point of ADC trigger source SOCA when using CMPA */
    g_apt0.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_DOWN;
    g_apt0.adcTrg.trgScaleSOCA = 1;

    /* ADC Trigger SOCB */
    g_apt0.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCB =  1;
    g_apt0.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_DOWN;
    g_apt0.adcTrg.trgScaleSOCB = 1;

    g_apt0.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    /* Timer Trigger */
    g_apt0.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_apt0.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO;
    g_apt0.tmrInterrupt.tmrInterruptScale = 1;

    APT0_ProtectInit();

    HAL_APT_PWMInit(&g_apt0);
    HAL_APT_RegisterCallBack(&g_apt0, APT_EVENT_INTERRUPT, MotorSysErrCallback);
    IRQ_SetPriority(IRQ_APT0_EVT, 7); /* 7 is priority value */
    IRQ_Register(IRQ_APT0_EVT, HAL_APT_EventIrqHandler, &g_apt0);
    IRQ_EnableN(IRQ_APT0_EVT);
    HAL_APT_RegisterCallBack(&g_apt0, APT_TIMER_INTERRUPT, MotorCarrierProcessCallback);
    IRQ_SetPriority(IRQ_APT0_TMR, 5); /* 5 is priority value */
    IRQ_Register(IRQ_APT0_TMR, HAL_APT_TimerIrqHandler, &g_apt0);
    IRQ_EnableN(IRQ_APT0_TMR);
}

static void APT1_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP0;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP0_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt1, &protectApt);
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_apt1.baseAddress = APT1;

    /* Clock Settings */
    g_apt1.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt1.waveform.timerPeriod = 7500; /* 7500 is count period of APT time-base timer */
    g_apt1.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt1.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt1.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.divInitVal = 0;
    g_apt1.waveform.cntInitVal = 0;
    g_apt1.waveform.cntCmpLeftEdge = 500; /* 500 is count compare point of the left edge of PWM waveform */
    g_apt1.waveform.cntCmpRightEdge = 4000; /* 4000 is count compare point of the right edge of PWM waveform */
    g_apt1.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt1.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt1.waveform.deadBandCnt = 225; /* 225 is dead-band value */

    APT1_ProtectInit();

    HAL_APT_PWMInit(&g_apt1);
}

static void APT2_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP0;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP0_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt2, &protectApt);
}

static void APT2_Init(void)
{
    HAL_CRG_IpEnableSet(APT2_BASE, IP_CLK_ENABLE);

    g_apt2.baseAddress = APT2;

    /* Clock Settings */
    g_apt2.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt2.waveform.timerPeriod = 7500; /* 7500 is count period of APT time-base timer */
    g_apt2.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt2.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt2.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.divInitVal = 0;
    g_apt2.waveform.cntInitVal = 0;
    g_apt2.waveform.cntCmpLeftEdge = 500; /* 500 is count compare point of the left edge of PWM waveform */
    g_apt2.waveform.cntCmpRightEdge = 4000; /* 4000 is count compare point of the right edge of PWM waveform */
    g_apt2.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt2.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt2.waveform.deadBandCnt = 225; /* 225 is dead-band value */

    APT2_ProtectInit();

    HAL_APT_PWMInit(&g_apt2);
}

__weak void MotorStartStopKeyCallback(void *param)
{
    GPIO_Handle *handle = (GPIO_Handle *)param;
    BASE_FUNC_UNUSED(handle);
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO2_BASE, IP_CLK_ENABLE);
    g_gpio2.baseAddress = GPIO2;

    g_gpio2.pins = GPIO_PIN_3;
    HAL_GPIO_Init(&g_gpio2);
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_NONE);

    g_gpio2.pins = GPIO_PIN_4;
    HAL_GPIO_Init(&g_gpio2);
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_INPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_LOW_LEVEL);

    HAL_CRG_IpEnableSet(GPIO1_BASE, IP_CLK_ENABLE);
    g_gpio1.baseAddress = GPIO1;

    g_gpio1.pins = GPIO_PIN_0;
    HAL_GPIO_Init(&g_gpio1);
    HAL_GPIO_SetDirection(&g_gpio1, g_gpio1.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio1, g_gpio1.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio1, g_gpio1.pins, GPIO_INT_TYPE_NONE);

    HAL_GPIO_RegisterCallBack(&g_gpio2, GPIO_PIN_4, MotorStartStopKeyCallback);
    IRQ_Register(IRQ_GPIO2, HAL_GPIO_IrqHandler, &g_gpio2);
    IRQ_SetPriority(IRQ_GPIO2, 1); /* set gpio1 interrupt priority to 1, 1~15. 1 is priority value */
    IRQ_EnableN(IRQ_GPIO2); /* gpio interrupt enable */

    return;
}

static void PGA0_Init(void)
{
    HAL_CRG_IpEnableSet(PGA0_BASE, IP_CLK_ENABLE);

    g_pga0.baseAddress = PGA0_BASE;
    g_pga0.externalResistorMode = BASE_CFG_ENABLE;
    g_pga0.handleEx.extCapCompensation = PGA_EXT_COMPENSATION_2X;
    HAL_PGA_Init(&g_pga0);
}

static void PGA1_Init(void)
{
    HAL_CRG_IpEnableSet(PGA1_BASE, IP_CLK_ENABLE);

    g_pga1.baseAddress = PGA1_BASE;
    g_pga1.externalResistorMode = BASE_CFG_ENABLE;
    g_pga1.handleEx.extCapCompensation = PGA_EXT_COMPENSATION_2X;
    HAL_PGA_Init(&g_pga1);
}

__weak void CheckPotentiometerValueCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN CheckPotentiometerValueCallback */
    /* USER CODE END CheckPotentiometerValueCallback */
}

static void TIMER0_Init(void)
{
    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE);  /* TIMER0 clock enable. */
    unsigned int load = ((float)HAL_CRG_GetIpFreq((void *)TIMER0) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 1000000;

    g_timer0.baseAddress = TIMER0;
    g_timer0.load        = (load < 1) ? 0 : (load - 1); /* Set timer value immediately */
    g_timer0.bgLoad      = (load < 1) ? 0 : (load - 1); /* Set timer value */
    g_timer0.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer0.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer0.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer0.interruptEn = BASE_CFG_ENABLE;
    g_timer0.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer0.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer0);
    IRQ_Register(IRQ_TIMER0, HAL_TIMER_IrqHandler, &g_timer0);

    HAL_TIMER_RegisterCallback(&g_timer0, TIMER_PERIOD_FIN, CheckPotentiometerValueCallback);
    IRQ_SetPriority(IRQ_TIMER0, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER0);
}

__weak void MotorStatemachineCallBack(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN MotorStatemachineCallBack */
    /* USER CODE END MotorStatemachineCallBack */
}

static void TIMER1_Init(void)
{
    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);  /* TIMER1 clock enable. */
    unsigned int load = ((float)HAL_CRG_GetIpFreq((void *)TIMER1) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 500;

    g_timer1.baseAddress = TIMER1;
    g_timer1.load        = (load < 1) ? 0 : (load - 1); /* Set timer value immediately */
    g_timer1.bgLoad      = (load < 1) ? 0 : (load - 1); /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    g_timer1.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaReqEnable = BASE_CFG_DISABLE;
    HAL_TIMER_Init(&g_timer1);
    IRQ_Register(IRQ_TIMER1, HAL_TIMER_IrqHandler, &g_timer1);

    HAL_TIMER_RegisterCallback(&g_timer1, TIMER_PERIOD_FIN, MotorStatemachineCallBack);
    IRQ_SetPriority(IRQ_TIMER1, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER1);
}

__weak void UART0InterruptErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_TRNS_IT_ERROR */
    /* USER CODE END UART0_TRNS_IT_ERROR */
}

__weak void UART0_TXDMACallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_WRITE_DMA_FINISH */
    /* USER CODE END UART0_WRITE_DMA_FINISH */
}

__weak void UART0WriteInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_WRITE_IT_FINISH */
    /* USER CODE END UART0_WRITE_IT_FINISH */
}

__weak void UART0ReadInterruptCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0_READ_IT_FINISH */
    /* USER CODE END UART0_READ_IT_FINISH */
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);  /* UART0 clock enable. */
    g_uart0.baseAddress = UART0;

    g_uart0.baudRate = UART0_BAUD_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_DMA;
    g_uart0.rxMode = UART_MODE_INTERRUPT;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
    HAL_UART_RegisterCallBack(&g_uart0, UART_TRNS_IT_ERROR, (UART_CallbackType)UART0InterruptErrorCallback);
    HAL_UART_RegisterCallBack(&g_uart0, UART_READ_IT_FINISH, (UART_CallbackType)UART0ReadInterruptCallback);
    IRQ_Register(IRQ_UART0, HAL_UART_IrqHandler, &g_uart0);
    IRQ_SetPriority(IRQ_UART0, 6); /* 6 is priority value */
    IRQ_EnableN(IRQ_UART0);
    g_uart0.dmaHandle = &g_dmac;
    g_uart0.uartDmaTxChn = 0;
    HAL_UART_RegisterCallBack(&g_uart0, UART_WRITE_DMA_FINISH, (UART_CallbackType)UART0_TXDMACallback);
    HAL_UART_RegisterCallBack(&g_uart0, UART_WRITE_IT_FINISH, (UART_CallbackType)UART0WriteInterruptCallback);
}

static void IOConfig(void)
{
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;
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
    /* Config PIN1 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_7_AS_ACMP0_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_7_AS_ACMP0_OUT, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_7_AS_ACMP0_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_7_AS_ACMP0_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_7_AS_ACMP0_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN48 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_6_AS_ACMP_N2);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_6_AS_ACMP_N2, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_6_AS_ACMP_N2, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_6_AS_ACMP_N2, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_6_AS_ACMP_N2, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN47 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_5_AS_ACMP_P2);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_5_AS_ACMP_P2, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_5_AS_ACMP_P2, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_5_AS_ACMP_P2, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_5_AS_ACMP_P2, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN4 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_6_AS_PGA0_N0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_6_AS_PGA0_N0, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_6_AS_PGA0_N0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_6_AS_PGA0_N0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_6_AS_PGA0_N0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN5 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_5_AS_PGA0_P0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_5_AS_PGA0_P0, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_5_AS_PGA0_P0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_5_AS_PGA0_P0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_5_AS_PGA0_P0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN3 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_7_AS_PGA0_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_7_AS_PGA0_OUT, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_7_AS_PGA0_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_7_AS_PGA0_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_7_AS_PGA0_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN7 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_7_AS_ADC_AIN9);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_7_AS_ADC_AIN9, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_7_AS_ADC_AIN9, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_7_AS_ADC_AIN9, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_7_AS_ADC_AIN9, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN2 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO5_1_AS_ADC_AIN5);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO5_1_AS_ADC_AIN5, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO5_1_AS_ADC_AIN5, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO5_1_AS_ADC_AIN5, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO5_1_AS_ADC_AIN5, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN14 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_7_AS_ADC_AIN15);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_7_AS_ADC_AIN15, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_7_AS_ADC_AIN15, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_7_AS_ADC_AIN15, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_7_AS_ADC_AIN15, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN11 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_5_AS_PGA1_P0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_5_AS_PGA1_P0, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_5_AS_PGA1_P0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_5_AS_PGA1_P0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_5_AS_PGA1_P0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN12 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_6_AS_PGA1_N0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_6_AS_PGA1_N0, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_6_AS_PGA1_N0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_6_AS_PGA1_N0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_6_AS_PGA1_N0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN13 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_7_AS_PGA1_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_7_AS_PGA1_OUT, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_7_AS_PGA1_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_7_AS_PGA1_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_7_AS_PGA1_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN35 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_3_AS_GPIO2_3);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_3_AS_GPIO2_3, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_3_AS_GPIO2_3, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_3_AS_GPIO2_3, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_3_AS_GPIO2_3, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN27 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO1_0_AS_GPIO1_0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO1_0_AS_GPIO1_0, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO1_0_AS_GPIO1_0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO1_0_AS_GPIO1_0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO1_0_AS_GPIO1_0, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN41 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_4_AS_GPIO2_4);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_4_AS_GPIO2_4, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_4_AS_GPIO2_4, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_4_AS_GPIO2_4, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_4_AS_GPIO2_4, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN19 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_0_AS_APT0_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_0_AS_APT0_PWMA, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_0_AS_APT0_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_0_AS_APT0_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_0_AS_APT0_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN23 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_0_AS_APT0_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_0_AS_APT0_PWMB, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_0_AS_APT0_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_0_AS_APT0_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_0_AS_APT0_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN20 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_1_AS_APT1_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_1_AS_APT1_PWMA, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_1_AS_APT1_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_1_AS_APT1_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_1_AS_APT1_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN24 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_1_AS_APT1_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_1_AS_APT1_PWMB, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_1_AS_APT1_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_1_AS_APT1_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_1_AS_APT1_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN21 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO3_2_AS_APT2_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO3_2_AS_APT2_PWMA, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO3_2_AS_APT2_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO3_2_AS_APT2_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO3_2_AS_APT2_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */
    /* Config PIN25 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO4_2_AS_APT2_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_2_AS_APT2_PWMB, PULL_NONE);  /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_2_AS_APT2_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_2_AS_APT2_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_2_AS_APT2_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

static void APT_SyncMasterInit(void)
{
    HAL_APT_MasterSyncInit(&g_apt0, APT_SYNC_OUT_ON_CNTR_ZERO);
}

static void APT_SyncSlaveInit(void)
{
    APT_SlaveSyncIn aptSlave;

    aptSlave.cntPhase = 0; /* counter phase value  */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT0_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt1, &aptSlave);

    aptSlave.cntPhase = 0; /* counter phase value  */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT0_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt2, &aptSlave);

}

void SystemInit(void)
{
    IOConfig();
    DMA_Init();
    UART0_Init();
    ACMP0_Init();
    APT0_Init();
    APT1_Init();
    APT2_Init();
    ADC0_Init();
    PGA0_Init();
    PGA1_Init();
    TIMER0_Init();
    TIMER1_Init();
    GPIO_Init();

    APT_SyncMasterInit();
    APT_SyncSlaveInit();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}