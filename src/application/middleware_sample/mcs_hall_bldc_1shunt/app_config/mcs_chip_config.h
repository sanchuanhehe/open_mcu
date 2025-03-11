/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      mcs_chip_config.h
  * @author    MCU Algorithm Team
  * @brief     This file provides config macros for ECBMCU201MPC app.
  */

#ifndef McuMagicTag_MCS_CHIP_CONFIG_H
#define McuMagicTag_MCS_CHIP_CONFIG_H

#include "feature.h"

#if defined (CHIP_3061MNPICA) || defined (CHIP_3061MNPIKA) || defined (CHIP_3061MNNICA) || \
    defined (CHIP_3061MNNIKA) || defined (CHIP_3061MNPIC8) || defined(CHIP_3061MNNIC8) || \
    defined (CHIP_3061MNPIK8) || defined (CHIP_3061MNNIK8)
    /* 3061m */
    #define ADC_U_HANDLE         g_adc0   // ibus
    #define ADC_U_SOC_NUM        ADC_SOC_NUM0

    #define ADC_UDC_HANDLE       g_adc0   // vbus
    #define ADC_UDC_SOC_NUM      ADC_SOC_NUM1

    #define ADC_SPEED_HANDLE     g_adc0   // spd
    #define ADC_SPEED_SOC_NUM    ADC_SOC_NUM2

    #define ADC_TEMP_HANDLE      g_adc0   // temp
    #define ADC_TEMP_SOC_NUM     ADC_SOC_NUM3

    #define LED1_HANDLE          g_gpio1
    #define LED1_PIN             GPIO_PIN_0

    #define LED2_HANDLE          g_gpio2
    #define LED2_PIN             GPIO_PIN_3
#endif

#if defined CHIP_3065HRPIRZ || defined CHIP_3065ARPIRZ || defined CHIP_3066MNPIRH || \
    defined CHIP_3065PNPIRH || defined CHIP_3065PNPIRE || defined CHIP_3065PNPIRA
    /* 3065h */
    #define ADC_U_HANDLE         g_adc0   // ibus
    #define ADC_U_SOC_NUM        ADC_SOC_NUM0

    #define ADC_UDC_HANDLE       g_adc2   // vbus
    #define ADC_UDC_SOC_NUM      ADC_SOC_NUM1

    #define ADC_SPEED_HANDLE     g_adc2   // spd
    #define ADC_SPEED_SOC_NUM    ADC_SOC_NUM2

    #define ADC_TEMP_HANDLE      g_adc2   // temp
    #define ADC_TEMP_SOC_NUM     ADC_SOC_NUM3

    #define LED1_HANDLE          g_gpio0
    #define LED1_PIN             GPIO_PIN_7

    #define LED2_HANDLE          g_gpio0
    #define LED2_PIN             GPIO_PIN_6
#endif

#endif