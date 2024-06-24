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
  * @file      mcs_user_config.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of user config parameters.
  */
#ifndef McuMagicTag_MCS_CONFIG_H
#define McuMagicTag_MCS_CONFIG_H

#include "debug.h"
#include "typedefs.h"

#define SYSTICK_PERIOD_US                 500u /* systick period */

#define INV_CAP_CHARGE_MS                 3u

#define INV_VOLTAGE_BUS                   24.0f   /* Bus voltage, V */

#define CTRL_CURR_PERIOD                  0.0001f /* carrier ISR period, 100us */
#define CTRL_SYSTICK_PERIOD               0.0005f /* systick control period, 500us */

/* Duty of sample window, the real time is 0.06*50us = 3us. */
#define SAMPLE_WINDOW_DUTY                0.06f

/* Duty of sample point shift as flip point, the real time is 0.008*50us = 0.4us. */
#define SAMPLE_POINT_SHIFT                0.008f

/* Sampling resistance 200mOhm 0.0013295 */
#define ADC_CURR_COFFI                    0.0013295f   /* 3.3/4096/3.03/0.2     pga: 3.03, 200mohm */
/* APT */
#define APT_SYNC_IN_SRC                   APT_SYNCIN_SRC_APT0_SYNCOUT

#define APT_U                             APT0_BASE /* Base address of U phase APT module */
#define APT_V                             APT1_BASE /* Base address of V phase APT module */
#define APT_W                             APT2_BASE /* Base address of W phase APT module */

/* User_Commond */
#define CTRL_IF_CURR_AMP_A                0.55f      /* IF control current amplitude */
#define USER_TARGET_SPD_HZ                100.0f  /* Parentheses are used to enter negative instructions */

#define USER_MIN_SPD_HZ                   10.0f    /* Motor minimum speed limit. */
#define USER_MAX_SPD_HZ                   200.0f
#define USER_SPD_SLOPE                    30.0f                       /* slope of velocity change */
#define USER_CURR_SLOPE                   (CTRL_IF_CURR_AMP_A * 5.0f) /* Current change slope  */

/* PID PARAMS */
#define CURRQAXIS_KP                      0.7414f
#define CURRQAXIS_KI                      1256.0f
#define CURRDAXIS_KP                      0.7414f
#define CURRDAXIS_KI                      1256.0f
#define CURR_LOWERLIM                     (-INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.95f)
#define CURR_UPPERLIM                     (INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.95f)

#define SPD_KP                            0.01f
#define SPD_KI                            0.1f
#define SPD_LOWERLIM                      -1.0f
#define SPD_UPPERLIM                      1.0f

#define POS_KP                            5.0f
#define POS_KI                            0.3f
#define POS_KD                            0.01f
#define POS_NS                            10.0f    /* Position loop Ns parameter. */
#define POS_LOWERLIM                      -200.0f
#define POS_UPPERLIM                      200.0f

/* MOTOR PARAMS */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
/* mtrPsif & mtrJ parameter is not used in this project, temporarily set to 0 */
#define MOTORPARAM_DEFAULTS  { \
    .mtrNp = 4, \
    .mtrRs = 0.5f, \
    .mtrLd = 0.000295f, \
    .mtrLq = 0.000295f, \
    .mtrPsif = 0.0f, \
    .mtrJ = 0.0f, \
    .maxElecSpd = 200.0f, \
    .maxCurr = 1.0f, \
    .mtrPPMR = 4000, \
    .zShift = 410, \
}

#define ADC_UDC_COFFI                     0.01289f      /* 0.01289 = 3.3/4096*192/12 */

#endif