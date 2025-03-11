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
  * @file      mcs_brake.c
  * @author    MCU Algorithm Team
  * @brief     Math library.
  *            This file provides functions declaration of brake module.
  */

#include "mcs_brake.h"
#include "mcs_math.h"
#include "mcs_assert.h"

#define PHASE_MAX_NUM  3

/**
  * @brief Initialize brake handle.
  * @param brake Pointer of brake handle.
  * @param brkParam Brake parameters.
  * @param ts Brake cycle.
  * @retval None.
  */
void BRAKE_Init(BRAKE_Handle *brake, BRAKE_Param brkParam, float ts)
{
    MCS_ASSERT_PARAM(brake != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Set brake parameter. */
    brake->ts = ts;
    brake->brkFinished = 0;

    brake->brkParam.brkTime = brkParam.brkTime;
    brake->brkParam.maxBrkCurr = brkParam.maxBrkCurr;

    brake->brkParam.blindDutyThr = brkParam.blindDutyThr;
    brake->brkParam.blindBrkDutyStep = brkParam.blindBrkDutyStep;

    brake->brkParam.fastBrkCurrCoeff = brkParam.fastBrkCurrCoeff;
    brake->brkParam.fastBrkDutyStep = brkParam.fastBrkDutyStep;

    brake->brkParam.slowBrkDutyStep = brkParam.slowBrkDutyStep;

    brake->brkParam.aptMaxCmp = brkParam.aptMaxCmp;

    /* Brake count. */
    brake->tickNum = (unsigned int)(brkParam.brkTime / ts);
}

/**
  * @brief Clear historical values of brake handle.
  * @param brake Pointer of brake handle.
  * @retval None.
  */
void BRAKE_Clear(BRAKE_Handle *brake)
{
    MCS_ASSERT_PARAM(brake != NULL);
    brake->brkDuty = 0.0f; /* brake duty */
    /* counter for calculating brake time */
    brake->tickCnt = 0;
    brake->brkCurr = 0.0f;
    brake->brkFinished = 0;
    /* brake status */
    brake->status = BRAKE_CLOSED_LOOP;
    brake->stage = BRAKE_INIT;
}

/**
  * @brief Set brake duty.
  * @param aptAddr APT address pointer.
  * @param maxCmp  Maximum count of APT.
  * @param brkDuty Braje duty.
  * @retval None.
  */
static void BRAKE_SetPwmDuty(APT_RegStruct **aptAddr, unsigned short maxCmp, float brkDuty)
{
    /* Calc brake compare point according to the duty. */
    unsigned short comparePoint = (unsigned short)Clamp((brkDuty * maxCmp), (float)(maxCmp - 1), 1.0f);
    /* Set pwm waveform according C D point. */
    for (int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptAddrx = aptAddr[i];
        DCL_APT_SetCounterCompare(aptAddrx, APT_COMPARE_REFERENCE_C, comparePoint);
        DCL_APT_SetCounterCompare(aptAddrx, APT_COMPARE_REFERENCE_D, comparePoint);
    }
}

/**
  * @brief Brake execution.
  * @param BRAKE_Handle Pointer of brake handle.
  * @param aptAddr APT address pointer.
  * @param brkCurr Sampling result of current during brake condition (A).
  * @retval None.
  */
void BRAKE_Exec(BRAKE_Handle *brake, APT_RegStruct **aptAddr, float brkCurr)
{
    MCS_ASSERT_PARAM(brake != NULL);
    float curr = Abs(brkCurr);
    float maxCurr = brake->brkParam.maxBrkCurr;
    float dutyMax = 1.0f;
    unsigned int tickNum = brake->tickNum;

    if (brake->stage != BRAKE_EXEC) {
        return;
    }
    /* Collect statistics on the total braking duration */
    brake->tickCnt++;
    if (brake->tickCnt >= tickNum) {
        /* Time to push out the brakes.  */
        brake->brkFinished = 1;
        return;
    }

    /**
     * Brake process:
     * (1) when brake current is more than maxCurr, brake duty decrease BRAKE_DUTY_FAST every period.
     * (2) when brake current is less than maxCurr, brake duty increase BRAKE_DUTY_SLOW every period.
     * (3) when brake current is less than (coeff * maxCurr), brake duty increase BRAKE_DUTY_FAST every period.
     * (4) when brake current is less than (maxCurr - 0.1) and brake duty is more than dutyThr,
     *     brake duty increase BRAKE_DUTY_BLIND every period.
     * Attention: The max brake duty is 1.0.
    */
    switch (brake->status) {
        case BRAKE_CLOSED_LOOP:
            if (curr < brake->brkParam.fastBrkCurrCoeff * maxCurr) {
                brake->brkDuty += brake->brkParam.fastBrkDutyStep; /* (3) */
            } else if (curr < maxCurr) {
                brake->brkDuty += brake->brkParam.slowBrkDutyStep; /* (2) */
                brake->brkDuty = Clamp(brake->brkDuty, brake->brkParam.blindDutyThr, 0.0f);
            } else {
                brake->brkDuty -= brake->brkParam.fastBrkDutyStep; /* (1) */
            }

            brake->status = (brake->brkDuty >= brake->brkParam.blindDutyThr && curr <= (maxCurr - 0.1f))?
                            BRAKE_OPEN_LOOP : BRAKE_CLOSED_LOOP; /* (4) */
            break;

        case BRAKE_OPEN_LOOP:
            brake->brkDuty += brake->brkParam.blindBrkDutyStep;  /* (4) */
            break;
 
        default:
              brake->brkDuty = dutyMax;
            break;
    }

    brake->brkDuty = Clamp(brake->brkDuty, dutyMax, 0.0f);
    /* Set brake duty. */
    BRAKE_SetPwmDuty(aptAddr, brake->brkParam.aptMaxCmp, brake->brkDuty);
}