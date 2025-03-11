/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2024. All rights reserved.
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
 * @file    sample_apt_pwm_output.h
 * @author  MCU Driver Team
 * @brief   APT module sample of HAL API.
 *          This file provides some configuration example of APT module HAL API.
 *          + PWM waveform configuration and ADC trigger time configuration sample.
 *          + Output control protection configuration sample.
 *          + Interrupt callback function and user registration function sample.
 */

#ifndef McuMagicTag_SAMPLE_APT_PWM_OUTPUT_H
#define McuMagicTag_SAMPLE_APT_PWM_OUTPUT_H

#include "apt_ip.h"
#include "interrupt.h"
#include "apt.h"

/**
  * @brief ADC current sample mode.
  */
typedef enum {
    ADC_SINGLE_RESISTOR = 0x00000000U,
    ADC_THREE_RESISTORS = 0x00000001U,
} ADC_SampleMode;

/* Action of apt channel a and b */
typedef enum {
    APT_CHA_HIGH_CHB_LOW,
    APT_CHA_LOW_CHB_HIGH,
    APT_CHA_LOW_CHB_LOW,
    APT_CHA_HIGH_CHB_HIGH,
} APT_Act;

void APT_SampleMain(void);
void APT_SetADCTrgTime(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB, ADC_SampleMode mode);
void APT_SetPwmDuty(APT_Handle *aptHandle, unsigned int duty);
void APT_OutputEnable(APT_Handle *aptHandle);
void APT_OutputDisable(APT_Handle *aptHandle);
void APT_ForceOutput(APT_Handle *aptHandle, APT_Act aptAct);
#endif /* McuMagicTag_APT_HAL_SAMPLE_H */