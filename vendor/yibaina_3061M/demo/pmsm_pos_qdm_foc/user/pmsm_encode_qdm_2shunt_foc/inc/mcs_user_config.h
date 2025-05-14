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

#define SYSTICK_PERIOD_US                 500u /* Systick定时器的周期，单位为微秒 */

#define INV_CAP_CHARGE_MS                 3u   /* 反向电容充电时间，单位为毫秒 */

#define INV_VOLTAGE_BUS                   24.0f   // 总线电压，单位为伏特 

#define CTRL_CURR_PERIOD                  0.0001f /* 载波中断服务例程的周期，100微秒 */
#define CTRL_SYSTICK_PERIOD               0.0005f /* Systick控制周期，500微秒 */

/* 采样窗口的占空比，实际时间为0.06*50us = 3us。 */
#define SAMPLE_WINDOW_DUTY                0.06f

/* 采样点移位作为翻转点的占空比，实际时间为0.008*50us = 0.4us。 */
#define SAMPLE_POINT_SHIFT                0.008f

/* 采样电阻200毫欧姆，0.0013295 */
#define ADC_CURR_COFFI                    0.0013295f   /* 3.3/4096/3.03/0.2     pga: 3.03, 200mohm */
/* APT */
#define APT_SYNC_IN_SRC                   APT_SYNCIN_SRC_APT0_SYNCOUT /* APT同步输入源 */

#define APT_U                             APT0_BASE /* U相APT模块的基地址 */
#define APT_V                             APT1_BASE /* V相APT模块的基地址 */
#define APT_W                             APT2_BASE /* W相APT模块的基地址 */

/* User_Commond */
#define CTRL_IF_CURR_AMP_A                1.0f      /* IF控制电流幅值 */
#define USER_TARGET_SPD_HZ                40.0f  /* 用户目标速度，单位为赫兹，括号用于输入负指令 */

#define USER_MIN_SPD_HZ                   5.0f    /* 电机最小速度限制，单位为赫兹 */
#define USER_MAX_SPD_HZ                   200.0f   /* 电机最大速度限制，单位为赫兹 */
#define USER_SPD_SLOPE                    30.0f                       /* 速度变化斜率 */
#define USER_CURR_SLOPE                   (CTRL_IF_CURR_AMP_A * 5.0f) /* 电流变化斜率 */

/* PID PARAMS */
#define CURRQAXIS_KP                      0.7414f  /* Q轴电流PID控制器的比例系数 */
#define CURRQAXIS_KI                      1256.0f  /* Q轴电流PID控制器的积分系数 */
#define CURRDAXIS_KP                      0.7414f  /* D轴电流PID控制器的比例系数 */
#define CURRDAXIS_KI                      1256.0f  /* D轴电流PID控制器的积分系数 */
#define CURR_LOWERLIM                     (-INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.95f)  /* 电流下限 */
#define CURR_UPPERLIM                     (INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * 0.95f)   /* 电流上限 */

#define SPD_KP                            0.01f  /* 速度PID控制器的比例系数 */
#define SPD_KI                            0.1f    /* 速度PID控制器的积分系数 */
#define SPD_LOWERLIM                      -1.0f  /* 速度下限 */
#define SPD_UPPERLIM                      1.0f   /* 速度上限 */

#define POS_KP                            5.0f   /* 位置PID控制器的比例系数 */
#define POS_KI                            0.3f   /* 位置PID控制器的积分系数 */
#define POS_KD                            0.01f  /* 位置PID控制器的微分系数 */
#define POS_NS                            10.0f    /* 位置环Ns参数 */
#define POS_LOWERLIM                      -200.0f  /* 位置下限 */
#define POS_UPPERLIM                      200.0f   /* 位置上限 */

/* MOTOR PARAMS */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
/* mtrPsif & mtrJ parameter is not used in this project, temporarily set to 0 */

/* 电机参数 */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
/* mtrPsif & mtrJ 参数在这个项目中没有使用，暂时设置为0 */
// 电机的极对数 
/* 电机的电阻，单位为欧姆 */
/* 电机的直轴电感，单位为亨利 */
/* 电机的交轴电感，单位为亨利 */
/* 电机的磁链，本项目未使用 */
/* 电机的转动惯量，本项目未使用 */
/* 电机的最大电气转速，单位为赫兹 */
/* 电机的最大电流，单位为安培 */
/* 电机的每转脉冲数 */
/* 电机的零位偏移 */

#define MOTORPARAM_DEFAULTS  { \
    .mtrNp = 4, \
    .mtrRs = 0.5f, \
    .mtrLd = 0.000295f, \
    .mtrLq = 0.000295f, \
    .mtrPsif = 0.0f, \
    .mtrJ = 0.0f, \
    .maxElecSpd = 200.0f, \
    .maxCurr = 1.5f, \
    .mtrPPMR = 4000, \
    .zShift = 410, \
}

// // //1025电机
// #define MOTORPARAM_DEFAULTS  { \
//     .mtrNp = 21, \
//     .mtrRs = 2.5f, \
//     .mtrLd = 0.066f, \
//     .mtrLq = 0.066f, \
//     .mtrPsif = 0.0f, \
//     .mtrJ = 0.0f, \
//     .maxElecSpd = 200.0f, \
//     .maxCurr = 1.5f, \
//     .mtrPPMR = 4000, \
//     .zShift = 410, \
// }


// // GIM4310-10
// #define MOTORPARAM_DEFAULTS  { \
//     .mtrNp = 14, \
//     .mtrRs = 1.046f, \
//     .mtrLd = 0.000344f, \
//     .mtrLq = 0.000344f, \
//     .mtrPsif = 0.0f, \
//     .mtrJ = 0.0f, \
//     .maxElecSpd = 200.0f, \
//     .maxCurr = 1.5f, \
//     .mtrPPMR = 4000, \
//     .zShift = 410, \
// }

#define ADC_UDC_COFFI                     0.01289f      /* 0.01289 = 3.3/4096*192/12 */
// ADC和UDC的系数，用于将ADC值转换为实际电压值
#endif