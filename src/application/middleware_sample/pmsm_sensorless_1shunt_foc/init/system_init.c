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

#define UART0_BAUD_RATE 921600

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;  /* pll ref clock hosc */
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

static void DMA_Channel0Init(void *handle)
{
    DMA_ChannelParam dma_param;
    dma_param.direction = DMA_MEMORY_TO_PERIPH_BY_DMAC; /* memory to periph */
    dma_param.srcAddrInc = DMA_ADDR_INCREASE; /* addr increase */
    dma_param.destAddrInc = DMA_ADDR_UNALTERED;
    dma_param.srcPeriph = DMA_REQUEST_MEM; /* source memory */
    dma_param.destPeriph = DMA_REQUEST_UART0_TX; /* destination uart0 tx */
    dma_param.srcWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.destWidth = DMA_TRANSWIDTH_BYTE;
    dma_param.srcBurst = DMA_BURST_LENGTH_1;
    dma_param.destBurst = DMA_BURST_LENGTH_1;
    dma_param.pHandle = handle;
    HAL_DMA_InitChannel(&g_dmac, &dma_param, DMA_CHANNEL_ZERO); /* init dma channel */
}

static void DMA_Init(void)
{
    HAL_CRG_IpEnableSet(DMA_BASE, IP_CLK_ENABLE);
    g_dmac.baseAddress = DMA;
    g_dmac.handleEx.srcByteOrder = DMA_BYTEORDER_SMALLENDIAN;   /* small endian */
    g_dmac.handleEx.destByteOrder = DMA_BYTEORDER_SMALLENDIAN;
    IRQ_Register(IRQ_DMA_TC, HAL_DMA_IrqHandlerTc, &g_dmac);  /* register intterupt */
    IRQ_Register(IRQ_DMA_ERR, HAL_DMA_IrqHandlerError, &g_dmac);
    IRQ_EnableN(IRQ_DMA_TC);
    IRQ_EnableN(IRQ_DMA_ERR);
    HAL_DMA_Init(&g_dmac); /* dma init */

    DMA_Channel0Init((void *)(&g_uart0));
}

static void ACMP1_Init(void)
{
    HAL_CRG_IpEnableSet(ACMP1_BASE, BASE_CFG_ENABLE);  /* acmp clock enable */

    g_acmp1.baseAddress =  ACMP1_BASE;
    g_acmp1.inOutConfig.vinNNum = ACMP_VIN_MUX3;
    g_acmp1.inOutConfig.vinPNum = ACMP_VIN_MUX3;
    g_acmp1.inOutConfig.swVinPNum = ACMP_SW_VIN3;
    g_acmp1.inOutConfig.swVinNNum = ACMP_SW_VIN3;
    g_acmp1.inOutConfig.polarity = ACMP_OUT_NOT_INVERT; /* none invert */
    g_acmp1.filterCtrl.filterMode = ACMP_FILTER_NONE; /* none filter */
    g_acmp1.hysteresisVol = ACMP_HYS_VOL_30MV; /* 30mv: without hysteresis */
    HAL_ACMP_Init(&g_acmp1);
}

static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE); /* adc clock enable */
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_5); /* adc clock div 5 */

    g_adc0.baseAddress = ADC0;
    g_adc0.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc0.handleEx.vrefBuf = ADC_VREF_2P5V;
    HAL_ADC_Init(&g_adc0);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0; /* PGA0_OUT(ADC INA0) */
    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;  /* sample triggle by apt0 soca */
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM8, &socParam);  /* adc soc config */
}

static void ADC1_Init(void)
{
    HAL_CRG_IpEnableSet(ADC1_BASE, IP_CLK_ENABLE); /* adc clock enable */
    HAL_CRG_IpClkSelectSet(ADC1_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC1_BASE, CRG_ADC_DIV_5); /* adc clock div 5 */

    g_adc1.baseAddress = ADC1;
    g_adc1.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc1.handleEx.vrefBuf = ADC_VREF_2P5V;

    HAL_ADC_Init(&g_adc1);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINB0; /* PGA0_OUT(ADC INB0) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCB;  /* sample triggle by apt0 socb */
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM8, &socParam);
}

static void ADC2_Init(void)
{
    HAL_CRG_IpEnableSet(ADC2_BASE, IP_CLK_ENABLE); /* adc clock enable */
    HAL_CRG_IpClkSelectSet(ADC2_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC2_BASE, CRG_ADC_DIV_5); /* adc clock div 5 */

    g_adc2.baseAddress = ADC2;
    g_adc2.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc2.handleEx.vrefBuf = ADC_VREF_2P5V;
    HAL_ADC_Init(&g_adc2);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINB1; /* PIN31(ADC INB1) */
    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM0, &socParam);

    socParam.adcInput = ADC_CH_ADCINA7; /* PIN36(ADC INA7) */
    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM14, &socParam);

    socParam.adcInput = ADC_CH_ADCINB2; /* PIN40(ADC INB2) */
    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM1, &socParam);
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
    protectApt.ocEventEnEx = BASE_CFG_ENABLE; /* enable event protect */
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT; /* protect mode one shot */
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;
    /* config systerm error event protect */
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1; /* extern event is from acmp1 */
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt0, &protectApt); /* apt protect init */
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_apt0.baseAddress = APT0;
    /* Clock Settings */
    g_apt0.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt0.waveform.timerPeriod = 10000;  /* apt timer count period is 10000 */
    g_apt0.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;
    /* Wave Form */
    g_apt0.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt0.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.divInitVal = 0;
    g_apt0.waveform.cntInitVal = 0;
    g_apt0.waveform.cntCmpLeftEdge = 5000; /* apt signal left edge moment is 5000 */
    g_apt0.waveform.cntCmpRightEdge = 4000; /* 4000 is count compare point of the right edge of PWM waveform */
    g_apt0.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt0.waveform.deadBandCnt = 300; /* apt signal dead duration is 300 */
    /* ADC Trigger SOCA */
    g_apt0.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCA = 1;
    g_apt0.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_DOWN;
    g_apt0.adcTrg.trgScaleSOCA = 1;
    /* ADC Trigger SOCB */
    g_apt0.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCB =  1;
    g_apt0.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_DOWN;
    g_apt0.adcTrg.trgScaleSOCB = 1;
    g_apt0.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    /* Timer Triggle */
    g_apt0.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_apt0.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO;
    g_apt0.tmrInterrupt.tmrInterruptScale = 1;
    APT0_ProtectInit();

    HAL_APT_PWMInit(&g_apt0);
    IRQ_Register(IRQ_APT0_EVT, HAL_APT_EventIrqHandler, &g_apt0);
    HAL_APT_RegisterCallBack(&g_apt0, APT_EVENT_INTERRUPT, MotorSysErrCallback);
    IRQ_SetPriority(IRQ_APT0_EVT, 7); /* 7 is priority value */
    IRQ_EnableN(IRQ_APT0_EVT);

    IRQ_Register(IRQ_APT0_TMR, HAL_APT_TimerIrqHandler, &g_apt0);
    HAL_APT_RegisterCallBack(&g_apt0, APT_TIMER_INTERRUPT, MotorCarrierProcessCallback);
    IRQ_SetPriority(IRQ_APT0_TMR, 6); /* 6 is priority value */
    IRQ_EnableN(IRQ_APT0_TMR);
}

static void APT1_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT; /* protect mode one shot */
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1; /* extern event is from acmp1 */
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt1, &protectApt); /* apt protect init */
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_apt1.baseAddress = APT1;
    /* Clock Settings */
    g_apt1.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt1.waveform.timerPeriod = 10000; /* 10000 is count period of APT time-base timer */
    g_apt1.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;
    /* Wave Form */
    g_apt1.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt1.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.divInitVal = 0;
    g_apt1.waveform.cntInitVal = 0;
    g_apt1.waveform.cntCmpLeftEdge = 5000; /* 5000 is count compare point of the left edge of PWM waveform */
    g_apt1.waveform.cntCmpRightEdge = 4000; /* 4000 is count compare point of the right edge of PWM waveform */
    g_apt1.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt1.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt1.waveform.deadBandCnt = 300; /* 300 is dead-band value */
    APT1_ProtectInit();

    HAL_APT_PWMInit(&g_apt1);
}

static void APT2_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT; /* protect mode one shot */
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocActionBEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1; /* extern event is from acmp1 */
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt2, &protectApt); /* apt protect init */
}

static void APT2_Init(void)
{
    HAL_CRG_IpEnableSet(APT2_BASE, IP_CLK_ENABLE);

    g_apt2.baseAddress = APT2;
    /* Clock Settings */
    g_apt2.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt2.waveform.timerPeriod = 10000; /* 10000 is count period of APT time-base timer */
    g_apt2.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;
    /* Wave Form */
    g_apt2.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt2.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.divInitVal = 0;
    g_apt2.waveform.cntInitVal = 0;
    g_apt2.waveform.cntCmpLeftEdge = 5000; /* 5000 is count compare point of the left edge of PWM waveform */
    g_apt2.waveform.cntCmpRightEdge = 4000; /* 4000 is count compare point of the right edge of PWM waveform */
    g_apt2.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt2.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt2.waveform.deadBandCnt = 300; /* 300 is dead-band value */
    APT2_ProtectInit();

    HAL_APT_PWMInit(&g_apt2);
}

__weak void MotorStartStopKeyCallback(void *param)
{
    GPIO_Handle *handle = (GPIO_Handle *)param;
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN GPIO_INTERRUPT */
    /* USER CODE END GPIO_INTERRUPT */
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO2_BASE, IP_CLK_ENABLE); /* gpio clock enable */
    g_gpio2.baseAddress = GPIO2;
    g_gpio2.pins = GPIO_PIN_2;
    HAL_GPIO_Init(&g_gpio2); /* init gpio2_2 */
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_INPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_LOW_LEVEL);

    g_gpio2.pins = GPIO_PIN_6;
    HAL_GPIO_Init(&g_gpio2); /* init gpio2_6 */
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_NONE);

    HAL_CRG_IpEnableSet(GPIO0_BASE, IP_CLK_ENABLE);
    g_gpio0.baseAddress = GPIO0;
    g_gpio0.pins = GPIO_PIN_7 | GPIO_PIN_6;
    HAL_GPIO_Init(&g_gpio0); /* init gpio0_6 and gpio0_7 */
    HAL_GPIO_SetDirection(&g_gpio0, g_gpio0.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio0, g_gpio0.pins, GPIO_HIGH_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio0, g_gpio0.pins, GPIO_INT_TYPE_NONE);

    HAL_GPIO_RegisterCallBack(&g_gpio2, GPIO_PIN_2, MotorStartStopKeyCallback);
    IRQ_Register(IRQ_GPIO2, HAL_GPIO_IrqHandler, &g_gpio2);
    IRQ_SetPriority(IRQ_GPIO2, 1); /* set gpio1 interrupt priority to 1, 1~7. 1 */
    IRQ_EnableN(IRQ_GPIO2); /* gpio interrupt enable */
    return;
}

static void PGA0_Init(void)
{
    HAL_CRG_IpEnableSet(PGA0_BASE, IP_CLK_ENABLE); /* pga clock enable */

    g_pga0.baseAddress = PGA0_BASE;
    g_pga0.externalResistorMode = BASE_CFG_ENABLE; /* pga external resistor mode */
    g_pga0.handleEx.pgaMux = PGA_EXT_RES_VI0;
    g_pga0.gain = PGA_GAIN_1X;

    HAL_PGA_Init(&g_pga0);
    DCL_PGA_EnableOut(g_pga0.baseAddress);
    DCL_PGA_EnableExtOut(g_pga0.baseAddress);
}

__weak void CheckPotentiometerValueCallback(void *handle)
{
    DCL_TIMER_IrqClear((TIMER_RegStruct *)handle);
    /* USER CODE BEGIN TIMER0 ITCallBackFunc */
    /* USER CODE END TIMER0 ITCallBackFunc */
}

static void TIMER0_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER0) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 1000000;

    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE); /* timer clock enable */
    HAL_CRG_IpClkSelectSet(TIMER0_BASE, CRG_PLL_NO_PREDV);

    g_timer0.baseAddress = TIMER0;
    g_timer0.load        = load - 1; /* Set timer value immediately */
    g_timer0.bgLoad      = load - 1; /* Set timer value */
    g_timer0.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer0.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer0.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer0.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer0.dmaReqEnable = BASE_CFG_DISABLE;
    g_timer0.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer0);
    IRQ_Register(IRQ_TIMER0, HAL_TIMER_IrqHandler, &g_timer0);
    HAL_TIMER_RegisterCallback(&g_timer0, TIMER_PERIOD_FIN, CheckPotentiometerValueCallback);
    IRQ_SetPriority(IRQ_TIMER0, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER0);
}

__weak void MotorStatemachineCallBack(void *handle)
{
    DCL_TIMER_IrqClear((TIMER_RegStruct *)handle);
    /* USER CODE BEGIN TIMER1 ITCallBackFunc */
    /* USER CODE END TIMER1 ITCallBackFunc */
}

static void TIMER1_Init(void)
{
    unsigned int load = (HAL_CRG_GetIpFreq((void *)TIMER1) / (1u << (TIMERPRESCALER_NO_DIV * 4)) / 1000000u) * 500;

    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER1_BASE, CRG_PLL_NO_PREDV);

    g_timer1.baseAddress = TIMER1;
    g_timer1.load        = load - 1; /* Set timer value immediately */
    g_timer1.bgLoad      = load - 1; /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaReqEnable = BASE_CFG_DISABLE;
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer1);
    IRQ_Register(IRQ_TIMER1, HAL_TIMER_IrqHandler, &g_timer1);
    HAL_TIMER_RegisterCallback(&g_timer1, TIMER_PERIOD_FIN, MotorStatemachineCallBack);
    IRQ_SetPriority(IRQ_TIMER1, 1); /* 1 is priority value */
    IRQ_EnableN(IRQ_TIMER1);
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

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE); /* uart clock enable */
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0.baseAddress = UART0;
    g_uart0.baudRate = UART0_BAUD_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_DMA; /* dma mode */
    g_uart0.rxMode = UART_MODE_INTERRUPT;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
    HAL_UART_RegisterCallBack(&g_uart0, UART_WRITE_IT_FINISH, UART0WriteInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart0, UART_READ_IT_FINISH, UART0ReadInterruptCallback);
    HAL_UART_RegisterCallBack(&g_uart0, UART_TRNS_IT_ERROR, UART0InterruptErrorCallback);
    IRQ_Register(IRQ_UART0, HAL_UART_IrqHandler, &g_uart0);
    IRQ_SetPriority(IRQ_UART0, 2); /* 2 is priority value */
    IRQ_EnableN(IRQ_UART0);
    g_uart0.dmaHandle = &g_dmac;
    g_uart0.uartDmaTxChn = 0;
    HAL_UART_RegisterCallBack(&g_uart0, UART_WRITE_DMA_FINISH, UART0_TXDMACallback);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(IO15_AS_APT0_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO15_AS_APT0_PWMA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO15_AS_APT0_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO15_AS_APT0_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO15_AS_APT0_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO18_AS_APT0_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO18_AS_APT0_PWMB, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO18_AS_APT0_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO18_AS_APT0_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO18_AS_APT0_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO16_AS_APT1_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO16_AS_APT1_PWMA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO16_AS_APT1_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO16_AS_APT1_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO16_AS_APT1_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO19_AS_APT1_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO19_AS_APT1_PWMB, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO19_AS_APT1_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO19_AS_APT1_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO19_AS_APT1_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO17_AS_APT2_PWMA);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO17_AS_APT2_PWMA, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO17_AS_APT2_PWMA, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO17_AS_APT2_PWMA, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO17_AS_APT2_PWMA, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO20_AS_APT2_PWMB);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO20_AS_APT2_PWMB, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO20_AS_APT2_PWMB, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO20_AS_APT2_PWMB, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO20_AS_APT2_PWMB, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO10_AS_ACMP1_OUT);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO10_AS_ACMP1_OUT, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO10_AS_ACMP1_OUT, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO10_AS_ACMP1_OUT, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO10_AS_ACMP1_OUT, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO11_AS_ACMP1_ANA_N3);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO11_AS_ACMP1_ANA_N3, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO11_AS_ACMP1_ANA_N3, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO11_AS_ACMP1_ANA_N3, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO11_AS_ACMP1_ANA_N3, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO12_AS_ACMP1_ANA_P3);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO12_AS_ACMP1_ANA_P3, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO12_AS_ACMP1_ANA_P3, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO12_AS_ACMP1_ANA_P3, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO12_AS_ACMP1_ANA_P3, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO21_AS_PGA0_ANA_P0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO21_AS_PGA0_ANA_P0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO21_AS_PGA0_ANA_P0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO21_AS_PGA0_ANA_P0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO21_AS_PGA0_ANA_P0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO22_AS_PGA0_ANA_N0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO22_AS_PGA0_ANA_N0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO22_AS_PGA0_ANA_N0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO22_AS_PGA0_ANA_N0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO22_AS_PGA0_ANA_N0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO23_AS_PGA0_ANA_EXT0);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO23_AS_PGA0_ANA_EXT0, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO23_AS_PGA0_ANA_EXT0, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO23_AS_PGA0_ANA_EXT0, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO23_AS_PGA0_ANA_EXT0, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO31_AS_ADC2_ANA_B1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO31_AS_ADC2_ANA_B1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO31_AS_ADC2_ANA_B1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO31_AS_ADC2_ANA_B1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO31_AS_ADC2_ANA_B1, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO36_AS_ADC2_ANA_A7);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO36_AS_ADC2_ANA_A7, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO36_AS_ADC2_ANA_A7, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO36_AS_ADC2_ANA_A7, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO36_AS_ADC2_ANA_A7, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO40_AS_ADC2_ANA_B2);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO40_AS_ADC2_ANA_B2, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO40_AS_ADC2_ANA_B2, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO40_AS_ADC2_ANA_B2, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO40_AS_ADC2_ANA_B2, DRIVER_RATE_2);  /* Output signal edge fast/slow */

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

    HAL_IOCMG_SetPinAltFuncMode(IO5_AS_GPIO2_2);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO5_AS_GPIO2_2, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO5_AS_GPIO2_2, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO5_AS_GPIO2_2, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO5_AS_GPIO2_2, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO56_AS_GPIO0_7);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO56_AS_GPIO0_7, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO56_AS_GPIO0_7, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO56_AS_GPIO0_7, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO56_AS_GPIO0_7, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO55_AS_GPIO0_6);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO55_AS_GPIO0_6, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO55_AS_GPIO0_6, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO55_AS_GPIO0_6, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO55_AS_GPIO0_6, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(IO9_AS_GPIO2_6);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(IO9_AS_GPIO2_6, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(IO9_AS_GPIO2_6, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(IO9_AS_GPIO2_6, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(IO9_AS_GPIO2_6, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

static void APT_SyncMasterInit(void)
{
    HAL_APT_MasterSyncInit(&g_apt0, APT_SYNC_OUT_ON_CNTR_ZERO); /* APT master synchronize */
}

static void APT_SyncSlaveInit(void)
{
    APT_SlaveSyncIn aptSlave;

    aptSlave.divPhase = 1 - 1; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value 0 */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_DOWN;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT0_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt1, &aptSlave);

    aptSlave.divPhase = 1 - 1; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value 0 */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_DOWN;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT0_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt2, &aptSlave);
}

void SystemInit(void)
{
    IOConfig();
    DMA_Init();
    UART0_Init();
    ACMP1_Init(); /* ACMP init */
    APT0_Init(); /* APT init */
    APT1_Init();
    APT2_Init();
    ADC0_Init(); /* ADC init */
    ADC1_Init();
    ADC2_Init();
    PGA0_Init(); /* PGA init */
    TIMER0_Init(); /* TIMER init */
    TIMER1_Init();
    GPIO_Init();

    APT_SyncMasterInit();
    APT_SyncSlaveInit();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}