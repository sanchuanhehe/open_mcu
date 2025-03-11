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
  * @file      mcs_brake.h
  * @author    MCU Algorithm Team
  * @brief     Math library.
  *            This file provides functions declaration of brake module.
  */

#ifndef McuMagicTag_MCS_BRAKE_H
#define McuMagicTag_MCS_BRAKE_H

#include "mcs_typedef.h"
#include "apt.h"
/* Typedef definitions ------------------------------------------------------------------------- */


/**
  * @brief Brake Param.
  */
typedef struct {
    float brkTime;              /**< brake time (s). */
    float maxBrkCurr;           /**< maximum brake current (A). */

    float blindDutyThr;         /**< brake duty cycle when entering open-loop braking state */
    float blindBrkDutyStep;     /**< large brake duty step per period. */

    float fastBrkCurrCoeff;     /**< current threshold coefficient for fast braking, recommend value: 0.5f. */
    float fastBrkDutyStep;      /**< large brake duty step per period. */

    float slowBrkDutyStep;      /**< small brake duty step per period. */

    unsigned short aptMaxCmp;   /**< apt maximum count. */
} BRAKE_Param;

/**
  * @brief Brake status enum.
  */
typedef enum {
    BRAKE_CLOSED_LOOP = 0,   /**< Closed loop. */
    BRAKE_OPEN_LOOP          /**< Open loop. */
} BRAKE_Status;


/**
  * @brief Brake Stage.
  */
typedef enum {
    BRAKE_INIT = 0,          /**< Brake init, include APT configuration. */
    BRAKE_EXEC,              /**< Brake exec, include brake current sample and brake duty adjust. */
    BRAKE_FINISHED           /**< Brake finished. */
} BRAKE_Stage;


/**
  * @brief Brake Struct.
  */
typedef struct {
    float ts;                   /**< control period (s). */
    float brkDuty;              /**< pwm duty ratio of lower switch during brake condition (0~1). */
    float brkCurr;              /**< brake current (A). */
    unsigned int tickCnt;       /**< counter for calculating brake time. */
    unsigned int tickNum;       /**< count number corresponding to brake time. */
    unsigned int brkFinished;   /**< brake finish flag. */
    BRAKE_Stage stage;          /**< brake stage. */
    BRAKE_Status status;        /**< brake status. */
    BRAKE_Param brkParam;       /**< brake parameter. */
} BRAKE_Handle;


void BRAKE_Init(BRAKE_Handle *brake, BRAKE_Param brkParam, float ts);
void BRAKE_Clear(BRAKE_Handle *brake);
void BRAKE_Exec(BRAKE_Handle *brake, APT_RegStruct **aptAddr, float brkCurr);

#endif