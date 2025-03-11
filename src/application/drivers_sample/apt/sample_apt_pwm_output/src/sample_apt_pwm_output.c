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
 * @file    sample_apt_pwm_output.c
 * @author  MCU Driver Team
 * @brief   APT module sample of HAL API.
 *          This file provides some configuration example of APT module HAL API.
 *          + PWM waveform configuration and ADC trigger time configuration sample.
 *          + Output control protection configuration sample.
 *          + Interrupt callback function and user registration function sample.
 */

#include "sample_apt_pwm_output.h"
#include "apt.h"
#include "debug.h"
#include "crg.h"
#include "gpio.h"
#include "main.h"
#include "debug.h"
#include "iocmg_ip.h"

/* Some configuration values of APT modules. */
/* You can also use HAL_CRG_GetIpFreq(APT0) to get the CPU clock frequency (In units of Hz). */
#define APT_CLK_FREQ        HAL_CRG_GetIpFreq((void *)APT0)
#define APT_PWM_FREQ        5000U       /* Set PWM frequency to 5000Hz. */
#define APT_TIMER_PERIOD    (APT_CLK_FREQ / (APT_PWM_FREQ * 2)) /* Period value when using APT_COUNT_MODE_UP_DOWN. */
#define APT_DIVIDER_FACTOR  1U          /* The APT clock is not divided. */


/**
  * @brief Pwm force output of channel A and B in A_H_B_L mode.
  * @param aptHandle APT Handle.
  * @param aptAct Action of channel A and B.
  * @retval None.
  */
static void APT_AHBLModeForceOutput(APT_Handle *aptHandle, APT_Act aptAct)
{
    /* Enable force output of channel a and b */
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A);
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B);
    switch (aptAct) {
        case APT_CHA_HIGH_CHB_LOW:
            /* Channel A output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            /* Channel B output is Low because of inversion in A_H_B_L mode. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        case APT_CHA_LOW_CHB_HIGH:
            /* Channel A output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            /* Channel B output is High because of inversion in A_H_B_L mode. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        case APT_CHA_LOW_CHB_LOW:
            /* Channel A output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            /* Channel B output is Low because of inversion in A_H_B_L mode. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        case APT_CHA_HIGH_CHB_HIGH:
            /* Channel A output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            /* Channel B output is High because of inversion in A_H_B_L mode. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        default:
            break;
    }
}


/**
  * @brief Pwm force output of channel A and B in A_H_B_H mode.
  * @param aptHandle APT Handle.
  * @param aptAct Action of channel A and B.
  * @retval None.
  */
static void APT_AHBHModeForceOutput(APT_Handle *aptHandle, APT_Act aptAct)
{
    /* Enable force output of channel a and b */
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A);
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B);
    switch (aptAct) {
        case APT_CHA_HIGH_CHB_LOW:
            /* Channel A output is High level. */
            /* Channel B output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        case APT_CHA_LOW_CHB_HIGH:
            /* Channel A output is Low level. */
            /* Channel B output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        case APT_CHA_HIGH_CHB_HIGH:
            /* Channel A output is High level. */
            /* Channel B output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        case APT_CHA_LOW_CHB_LOW:
            /* Channel A output is Low level. */
            /* Channel B output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        default:
            break;
    }
}

/**
  * @brief Pwm force output of channel A and B in A_L_B_H mode.
  * @param aptHandle APT Handle.
  * @param aptAct Action of channel A and B.
  * @retval None.
  */
static void APT_ALBHModeForceOutput(APT_Handle *aptHandle, APT_Act aptAct)
{
    /* Enable force output of channel a and b */
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A);
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B);
    switch (aptAct) {
        case APT_CHA_HIGH_CHB_LOW:
            /* Channel A output is High level. */
            /* Channel B output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        case APT_CHA_LOW_CHB_LOW:
            /* Channel A output is Low level. */
            /* Channel B output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;
        
        case APT_CHA_LOW_CHB_HIGH:
            /* Channel A output is Low level. */
            /* Channel B output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        case APT_CHA_HIGH_CHB_HIGH:
            /* Channel A output is High level. */
            /* Channel B output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        default:
            break;
    }
}

/**
  * @brief Pwm force output of channel A and B in A_L_B_L mode.
  * @param aptHandle APT Handle.
  * @param aptAct Action of channel A and B.
  * @retval None.
  */
static void APT_ALBLModeForceOutput(APT_Handle *aptHandle, APT_Act aptAct)
{
    /* Enable force output of channel a and b */
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A);
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B);
    switch (aptAct) {
        case APT_CHA_HIGH_CHB_LOW:
            /* Channel A output is High level. */
            /* Channel B output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        case APT_CHA_LOW_CHB_LOW:
            /* Channel A output is Low level. */
            /* Channel B output is Low level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        case APT_CHA_LOW_CHB_HIGH:
            /* Channel A output is Low level. */
            /* Channel B output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        case APT_CHA_HIGH_CHB_HIGH:
            /* Channel A output is High level. */
            /* Channel B output is High level. */
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_HIGH);
            DCL_APT_SetSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        default:
            break;
    }
}

/**
  * @brief Pwm force output of channel A and B.
  * @param aptHandle APT Handle.
  * @param aptAct Action of channel A and B.
  * @retval None.
  */
void APT_ForceOutput(APT_Handle *aptHandle, APT_Act aptAct)
{
    switch (aptHandle->waveform.basicType) {
        case APT_PWM_BASIC_A_HIGH_B_LOW:
            /* Output type of channel a and b */
            APT_AHBLModeForceOutput(aptHandle, aptAct);
            break;

        case APT_PWM_BASIC_A_LOW_B_HIGH:
            /* Output type of channel a and b */
            APT_ALBHModeForceOutput(aptHandle, aptAct);
            break;

        case APT_PWM_BASIC_A_HIGH_B_HIGH:
            /* Output type of channel a and b */
            APT_AHBHModeForceOutput(aptHandle, aptAct);
            break;

        case APT_PWM_BASIC_A_LOW_B_LOW:
            /* Output type of channel a and b */
            APT_ALBLModeForceOutput(aptHandle, aptAct);
            break;

        default:
            break;
    }
}

/**
  * @brief Modify the ADC trigger time of master APT module (U phase).
  * @param cntCmpSOCA Count compare value of SOCA.
  * @param cntCmpSOCB Counnt compare value of SOCB.
  * @param mode Sample mode, include 1shunt/2shunt/3shunt sample.
  * @retval None.
  */
void APT_SetADCTrgTime(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB, ADC_SampleMode mode)
{
    /* AptU use CMPA and CMPB as the trigger source of SOCA and SOCB. */
    /* SOCA is used to trigger 1st ADC sampling when using single resistor sampling. */
    /* SOCB is used to trigger 2nd ADC sampling when using single resistor sampling. */
    if (mode == ADC_SINGLE_RESISTOR) {
        HAL_APT_SetADCTriggerTime(&g_apt0, cntCmpSOCA, cntCmpSOCB);
    } else {
        HAL_APT_SetADCTriggerTime(&g_apt0, cntCmpSOCA, cntCmpSOCB);
        HAL_APT_SetADCTriggerTime(&g_apt1, cntCmpSOCA, cntCmpSOCB);
        HAL_APT_SetADCTriggerTime(&g_apt2, cntCmpSOCA, cntCmpSOCB);
    }
}

/**
  * @brief Pwm output disable.
  * @param aptHandle APT Handle.
  * @retval None.
  */
void APT_OutputDisable(APT_Handle *aptHandle)
{
    /* Disable PWM waveform output. */
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A);
    DCL_APT_EnableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B);
    DCL_APT_ForcePWMOutputLow(aptHandle->baseAddress);
}

/**
  * @brief Pwm output enable.
  * @param aptHandle APT Handle.
  * @retval None.
  */
void APT_OutputEnable(APT_Handle *aptHandle)
{
    /* Disable PWM waveform output. */
    DCL_APT_DisableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_A);
    DCL_APT_DisableSwContPWMAction(aptHandle->baseAddress, APT_PWM_CHANNEL_B);
}

/**
  * @brief Set pwm duty.
  * @param aptHandle APT Handle.
  * @param duty Duty cycle.
  * @retval None.
  */
void APT_SetPwmDuty(APT_Handle *aptHandle, unsigned int duty)
{
    HAL_APT_SetPWMDutyByNumber(aptHandle, duty);
}

unsigned int g_aptDuty = 0;
/**
  * @brief Timer interrupt callback function of APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void APT0TimerCallback(void *aptHandle)
{
    g_aptDuty++;
    /* 100 means 100% duty */
    if (g_aptDuty > 100) {
        g_aptDuty = 0;
    }
    /* Five output mode */
    if (g_aptDuty == 0) {
        /** Four kinds of pwm out put action, please select an action for verification:
         *  APT_ForceOutput(&g_apt0, APT_CHA_LOW_CHB_LOW);
         *  APT_ForceOutput(&g_apt0, APT_CHA_LOW_CHB_HIGH);
         *  APT_ForceOutput(&g_apt0, APT_CHA_HIGH_CHB_LOW);
         *  APT_ForceOutput(&g_apt0, APT_CHA_HIGH_CHB_HIGH);
         */
        APT_ForceOutput(&g_apt0, APT_CHA_HIGH_CHB_HIGH);
    } else if (g_aptDuty >= 50) { /* 50: Pwm duty output from 50% to 100% */
        APT_OutputEnable(&g_apt0);
        APT_SetPwmDuty(&g_apt0, g_aptDuty);
    }
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief Event interrupt callback function of U phase APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void APT0EventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief Initialize and start the APT modules of U, V, W phases.
  * @param mode: ADC sampling mode.
  * @retval None.
  */
void APT_SampleMain(void)
{
    /* Initialize GPIO pin for timer interrupt test. */
    SystemInit();
    /* Set the buff load mode of forcible output to independent load and the update time to zero. */
    APT0->PG_BUF_EN.BIT.rg_frc_buf_en = 1;
    APT0->PG_ACT_LD.BIT.rg_pg_frcld_zroen = 1;
    APT1->PG_BUF_EN.BIT.rg_frc_buf_en = 1;
    APT1->PG_ACT_LD.BIT.rg_pg_frcld_zroen = 1;
    APT2->PG_BUF_EN.BIT.rg_frc_buf_en = 1;
    APT2->PG_ACT_LD.BIT.rg_pg_frcld_zroen = 1;

    /* Start APT module of U, V, W phases. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);
}