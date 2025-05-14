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
  * @file      mcs_config.h
  * @author    MCU Algorithm Team
  * @brief     This file provides config macros for ECBMCU201MPC app.
  */
#ifndef McuMagicTag_MCS_CONFIG_H
#define McuMagicTag_MCS_CONFIG_H

#if defined(CHIP_3061MNPICA) || defined(CHIP_3061MNNICA) || defined(CHIP_3061MNPIC8) || defined(CHIP_3061MNNIC8)
#define ADC_U_SOC_NUM ADC_SOC_NUM2
#define ADC_V_SOC_NUM ADC_SOC_NUM5
#define ADC_W_SOC_NUM ADC_SOC_NUM6
#define ADC_SPD_ADJ_SOC_NUM ADC_SOC_NUM9
#define ADC_HANDLE g_adc0
#endif

#if defined (CHIP_3066MNPIRH) || defined (CHIP_3065PNPIRH) || defined (CHIP_3065PNPIRE) || defined (CHIP_3065PNPIRA)
#define ADC_U_SOC_NUM ADC_SOC_NUM3
#define ADC_V_SOC_NUM ADC_SOC_NUM5
#define ADC_W_SOC_NUM ADC_SOC_NUM7
#define ADC_SPD_ADJ_SOC_NUM ADC_SOC_NUM9

#define ADC_HANDLE g_adc2
#endif

#define APT_PWM_FREQ        12000      /* PWM frequency 12K Hz. */
#define CTRL_CURR_PERIOD    (1.0f / APT_PWM_FREQ) /* carrier ISR period (S) */
#define CTRL_CURE_PERIOD_US  83

#define SYSTICK_PERIOD_US   500u        /* systick period. unit : us */

#define INV_CAP_CHARGE_MS   3u          /* Bootstrap capacitor charge time. unit : ms */

#define CTRL_SYSTICK_PERIOD 0.0005f     /* systick control period, 500us */

#define USER_SPD_SLOPE      60.0f       /* slope of velocity change */

#define IN_VOLTAGE_BUS      24          /* Motor input voltage */
#define POLES               14           /* 电机磁极对数 */
#define MATH_PI             3.14        /* Pi */
#define MOTOR_K             1.62       /* 转矩系数 */

#define RAMP_STP            60          /* Slope of forced draging acceleration */
#define RAMP_DUTY_PWM       0.5         /* 1.0% */

#define FORCE_DRAG_MINDUTY  15          /* APT minimum duty cycle during forced drag */
#define FORCE_DRAG_MAXDUTY  20          /* APT maximum duty cycle during forced drag */

#define DRAG_START_INTERVAL 1000        /* Force drag change phase time */
#define DRAG_STOP_INTERVAL  400         /* Force drag change phase time */

/* Parameters of the motor in the RUN */
#define FILTER_COUNT        3           /* Filter Times */
#define PHASE_OFFSET        9000        /* Phase delay due to filtering */

#define APT_DUTYLIMIT_MAX   99.9        /* 输出APT的最大占空比 */
#define APT_DUTYLIMIT_MIN   8.0        /* Minimum duty cycle of the output APT */

#define BRIDGE_CTR_APT_U    APT0        /* APT address that controls the U phase */
#define BRIDGE_CTR_APT_V    APT1        /* APT address that controls the V phase */
#define BRIDGE_CTR_APT_W    APT2        /* APT address that controls the W phase */

#define SDP_MAX_VALUE       58.8      /* 最大相位变化频率 */
#define SDP_MIN_VALUE       15.0        /* Minimum change phase frequency */
#define SDP_TARGET_VALUE    58.8      /* 目标相位变化频率 */

/* SPD PID Param */
#define SPD_PID_KP          0.9      /* P parameter of PID control */
#define SPD_PID_KI          40        /* I parameter of PID control */
#define SPD_PID_TS          0.0005   /* TS parameter of PID control cycle */

#define OP_TO_CL_INTERGRAL  8.0  /* Open-loop switching closed-loop integral term to prevent sudden current change. */

#define VOL_DIVIDER_COEFFICIENT  0.09091f  /* Division coefficient of the zero-crossing detection sampling circuit */

#define SPD_FILTER_FC       30

#endif