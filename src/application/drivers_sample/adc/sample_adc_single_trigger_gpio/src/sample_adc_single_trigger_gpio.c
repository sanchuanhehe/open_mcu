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
  * @file    sample_adc_single_trigger_it.c
  * @author  MCU Driver Team
  * @brief   adc sample module.
  * @details In single sampling mode, the ADC sampling is triggered by GPIO. After the sampling is complete,
  *          the ADC interrupt is triggered and the ADC conversion result is read in the interrupt callback function.
  *          (1) ADC strigger source is gpio. Use HAL_ADC_GetConvResult() to get adc data.
  *          (2) ADC sample source is ADCHANDLE. Select sample source in "g_adc.baseAddress" of SystemInit(),
  *           "ADC_SOC_NUM0" can be Modified. External input source: GPIO2_1
  */
#include "sample_adc_single_trigger_gpio.h"

/**
  * @brief ADC single channel sample with gpio.
  * @param None.
  * @retval None.
  */
float g_udc = 0.0f;
void ADC_SingleTrigger(void)
{
    SystemInit();
    while (1) {
    HAL_GPIO_TogglePin(&g_gpio2, GPIO_PIN_1);
    /* Delay 500ms */
    BASE_FUNC_DELAY_MS(500);
    /* udc is input voltage, 0.01289f is voltage sample coffecient */
    g_udc = ((float)HAL_ADC_GetConvResult(&ADCHANDLE, ADC_SOC_NUM0)) * 0.01289f;
    DBG_PRINTF("g_udc: %f\r\n", g_udc);
    }
}