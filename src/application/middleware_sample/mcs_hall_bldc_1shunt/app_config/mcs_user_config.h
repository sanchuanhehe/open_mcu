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
  * @file      mcs_user_config.h
  * @author    MCU Algorithm Team
  * @brief     This file provides config macros for ECMCU105H app.
  */
#ifndef McuMagicTag_MCS_CONFIG_H
#define McuMagicTag_MCS_CONFIG_H


#include "typedefs.h"

#define MOTORPARAM_DEFAULTS  { \
    .mtrNp = 4, \
    .mtrRs = 0.5f, \
    .mtrLd = 0.000295f, \
    .mtrLq = 0.000295f, \
    .mtrPsif = 0.0f, \
    .mtrJ = 0.0f, \
    .maxElecSpd = 200.0f, \
    .maxCurr = 4.0f, \
}

#define SPD_KP                         0.005f
#define SPD_KI                         0.02f
#define SPD_UPPERLIM                   1.0f
#define SPD_LOWERLIM                   0.0f

#define SYSTICK_PERIOD_US              500u    /* systick period */

#define INV_CAP_CHARGE_MS              3u

#define INV_VOLTAGE_BUS                24.0f   /* Bus voltage, V */

#define CTRL_CURR_PERIOD               0.0001f /* carrier ISR period, 100us */
#define CTRL_SYSTICK_PERIOD            0.0005f /* systick control period, 500us */

/* Sampling resistance 200mOhm 0.0013295 */
#define ADC_CURR_COFFI              0.0013295f   /* 3.3/4096/3.03/0.2     pga: 3.03, 200mohm */
#define ADC_VOLT_COFFI              0.01289f     /* 3.3*192/12/4096 */

/* APT */
#define APT_SYNC_IN_SRC                APT_SYNCIN_SRC_APT0_SYNCOUT

#define APT_U_CP                       APT0_BASE    /* Base address of U phase APT module */
#define APT_V_CP                       APT1_BASE    /* Base address of V phase APT module */
#define APT_W_CP                       APT2_BASE    /* Base address of W phase APT module */

/* User_Commond */

#define USER_MIN_SPD_HZ                35.0         /* Minimum speed that can be set by the user. */
#define USER_MAX_SPD_HZ                200.0        /* Maximum speed that can be set by the user. */
#define USER_SPD_SLOPE                 28.0f        /* slope of velocity change */


/* Service Definition */
#define MOTOR_START_DELAY              100
#define ADC_READINIT_DELAY             200
#define ADC_READINIT_TIMES             20
#define ADC_TRIMVALUE_MIN              1820.0
#define ADC_TRIMVALUE_MAX              2280.0

#define HALL_PHASESHIFT                3.1415926f
#define HALL_ANGLE_PLL_BDW             50.0f
#define HALL_SPD_FILTER_FC             5.0f

/* Motor protection */
#define OVER_CURR_AMP_THRESHOLD        4.0f
#define OVER_CURR_TIME_THRESHOLD       0.5f

#define OVER_TEMP_AMP_THRESHOLD        40.0f
#define OVER_TEMP_TIME_THRESHOLD       2.0f

#define OVER_VOLT_AMP_THRESHOLD        26.0f
#define OVER_VOLT_TIME_THRESHOLD       2.0f

#define LOWER_VOLT_AMP_THRESHOLD       22.0f
#define LOWER_VOLT_TIME_THRESHOLD      2.0f

#define STALL_CURR_THRESHOLD           2.0f
#define STALL_SPEED_THRESHOLD          10.0f
#define STALL_TIME_THRESHOLD           1.0f

#endif