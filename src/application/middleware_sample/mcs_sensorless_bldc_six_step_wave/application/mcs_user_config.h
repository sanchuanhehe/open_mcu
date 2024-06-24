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

/* APT synchronization */
#define APT_SYNC_IN_SRC     APT_SYNCIN_SRC_APT0_SYNCOUT
/* Overcurrent protection flag */
#define APT_EVT_IRQ_CP      IRQ_APT0_EVT

#define SYSTICK_PERIOD_US   500u        /* systick period. unit : us */

#define INV_CAP_CHARGE_MS   3u          /* Bootstrap capacitor charge time. unit : ms */

#define CTRL_SYSTICK_PERIOD 0.0005f     /* systick control period, 500us */

#define USER_SPD_SLOPE      60.0f       /* slope of velocity change */

#define IN_VOLTAGE_BUS      12          /* Motor input voltage */
#define POLES               7           /* logarithm of motor pole */
#define MATH_PI             3.14        /* Pi */
#define MOTOR_K             0.013       /* Torque Coefficient */

#define BOOST_FACTOR        0.1         /* Boosting factor */

#define RAMP_STP            60          /* Slope of forced draging acceleration */
#define RAMP_DUTY_PWM       0.5         /* 1.0% */

#define FORCE_DRAG_MINDUTY  10          /* APT minimum duty cycle during forced drag */
#define FORCE_DRAG_MAXDUTY  10          /* APT maximum duty cycle during forced drag */

#define DRAG_START_INTERVAL 1000        /* Force drag change phase every 48 ms.(1000 * 60us = 60ms) */
#define DRAG_STOP_INTERVAL  400         /* Force drag change phase every 24 ms.(200 * 60us = 12ms) */

/* Parameters of the motor in the RUN */
#define FILTER_COUNT        3           /* Filter Times */
#define PHASE_OFFSET        9000        /* Phase delay due to filtering */

#define APT_DUTYLIMIT_MAX   99.9        /* Maximum duty cycle of the output APT */
#define APT_DUTYLIMIT_MIN   8.0        /* Minimum duty cycle of the output APT */

#define BRIDGE_CTR_APT_U    APT0        /* APT address that controls the U phase */
#define BRIDGE_CTR_APT_V    APT1        /* APT address that controls the V phase */
#define BRIDGE_CTR_APT_W    APT2        /* APT address that controls the W phase */

#define SDP_MAX_VALUE       180.25      /* Maximum change phase frequency */
#define SDP_MIN_VALUE       30.0        /* Minimum change phase frequency */
#define SDP_TARGET_VALUE    180.25      /* Target change phase frequency */

/* SPD PID Param */
#define SPD_PID_KP          0.9      /* P parameter of PID control */
#define SPD_PID_KI          6        /* I parameter of PID control */
#define SPD_PID_TS          0.0005   /* TS parameter of PID control cycle */

#define OP_TO_CL_INTERGRAL  8.0  /* Open-loop switching closed-loop integral term to prevent sudden current change. */

#define VOL_DIVIDER_COEFFICIENT  0.09091f  /* Division coefficient of the zero-crossing detection sampling circuit */

#define SPD_FILTER_FC       30

#endif