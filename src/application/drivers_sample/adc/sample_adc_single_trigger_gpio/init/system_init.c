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
    crg.baseAddress     = CRG;
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
    /* enable adc clk */
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_ASYN_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_1);

    g_adc0.baseAddress = ADC0;
    g_adc0.socPriority = ADC_PRIMODE_ALL_ROUND;
    /* initialize adc config */
    HAL_ADC_Init(&g_adc0);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA15; /* PIN14(ADC AIN15) */
    socParam.sampleTotalTime = ADC_SOCSAMPLE_5CLK; /* adc sample total time 5 adc_clk */
    socParam.trigSource = ADC_TRIGSOC_GPIOPF1;
    socParam.continueMode = BASE_CFG_DISABLE;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM0, &socParam);
}

static void GPIO_Init(void)
{
    /* enable gpio clk */
    HAL_CRG_IpEnableSet(GPIO2_BASE, IP_CLK_ENABLE);
    g_gpio2.baseAddress = GPIO2;

    g_gpio2.pins = GPIO_PIN_1;
    HAL_GPIO_Init(&g_gpio2);
    /* set gpio output mode */
    HAL_GPIO_SetDirection(&g_gpio2, g_gpio2.pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(&g_gpio2, g_gpio2.pins, GPIO_LOW_LEVEL);
    HAL_GPIO_SetIrqType(&g_gpio2, g_gpio2.pins, GPIO_INT_TYPE_NONE);
    return;
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);  /* UART0 clock enable. */
    g_uart0.baseAddress = UART0;
    /* set UART0 bandrate. */
    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    /* set UART0 one stop bit. */
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    /* enable fifo */
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.fifoRxThr = UART_FIFODEPTH_SIZE8;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    /* set UART0 overSample. */
    g_uart0.handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    g_uart0.handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void IOConfig(void)
{
    HAL_IOCMG_SetPinAltFuncMode(GPIO2_1_AS_GPIO2_1);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO2_1_AS_GPIO2_1, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO2_1_AS_GPIO2_1, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO2_1_AS_GPIO2_1, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO2_1_AS_GPIO2_1, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO4_7_AS_ADC_AIN15);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO4_7_AS_ADC_AIN15, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO4_7_AS_ADC_AIN15, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO4_7_AS_ADC_AIN15, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO4_7_AS_ADC_AIN15, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO5_1_AS_ADC_EXT_TRIG3);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO5_1_AS_ADC_EXT_TRIG3, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO5_1_AS_ADC_EXT_TRIG3, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO5_1_AS_ADC_EXT_TRIG3, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO5_1_AS_ADC_EXT_TRIG3, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_3_AS_UART0_TXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_3_AS_UART0_TXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_3_AS_UART0_TXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_3_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_3_AS_UART0_TXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */

    HAL_IOCMG_SetPinAltFuncMode(GPIO0_4_AS_UART0_RXD);  /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_4_AS_UART0_RXD, PULL_NONE);  /* Pull-up and pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_4_AS_UART0_RXD, SCHMIDT_DISABLE);  /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_4_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW);  /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_4_AS_UART0_RXD, DRIVER_RATE_2);  /* Output signal edge fast/slow */
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    ADC0_Init();
    GPIO_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}