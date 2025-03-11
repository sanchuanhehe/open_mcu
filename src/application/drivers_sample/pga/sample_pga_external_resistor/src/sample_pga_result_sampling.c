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
  * @file    sample_pga_result_sampling.c
  * @author  MCU Driver Team
  * @brief   pga sample module.
  * @details (1) In this example, the external resistor mode of the PGA is used,the PGA is amplified by 3.03 times,
  *              and the last sampled result is output through the ADC.
  *          (2) the external resistor mode is used, and the gain magnification value is 3.03.
  */

#include "sample_pga_result_sampling.h"

/**
  * @brief The PGA amplifies the voltage and uses the ADC to sample the output of the PGA.
  * @param None.
  * @retval None.
  */
void PGA_ReultSampling(void)
{
    SystemInit();
    DBG_PRINTF("The PGA amplifies the output and uses the ADC to sample the result.\r\n");

    /*  Configure ADC software triggering. */
    HAL_ADC_SoftTrigSample(&g_adc0, ADC_SOC_NUM1);
    BASE_FUNC_DELAY_MS(15);  /* delay 15 ms */
    if (HAL_ADC_CheckSocFinish(&g_adc0, ADC_SOC_NUM1) == BASE_STATUS_ERROR) {
        DBG_PRINTF("ADC sampling error output.\r\n");
        return;
    }
    /* Software trigger ADC sampling */
    unsigned int ret = HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM1);
    DBG_PRINTF("Sampling completed, result: %x\r\n", ret);
    float voltage = (float)ret / (float)4096 * 3.3f;      /* 4096 and 3.3 are for Sample Value Conversion */
    DBG_PRINTF("Real Output voltage of the PGA: %fV\r\n", voltage);
    DBG_PRINTF("Theoretical Output voltage of the PGA: 1.65V\r\n");
    return;
}