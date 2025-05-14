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
  * @file      mcs_carrier.c
  * @author    MCU Algorithm Team
  * @brief     This file provides carrier interrupt application for motor control.
  */

#include "mcs_carrier.h"
#include "mcs_user_config.h"
#include "mcs_assert.h"
#include "debug.h"

#define S_TO_US         1000000
#define S_TO_SYSTICK    100000000

/**
  * @brief Sets the duty cycle of the H-bridge APT.
  * @param aptHandle APT module handle.
  * @param duty PWM duty. Range: 0.1 ~ 99.9.
  * @retval None.
  */
void MCS_SetCtrAptDuty(MtrCtrlHandle *mtrCtrl, unsigned int duty)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(duty > 0);
    /* Set pwm duty of uvw pahse */
    HAL_APT_SetPWMDutyByNumber(mtrCtrl->stepCtrl.controlApt.u, duty);
    HAL_APT_SetPWMDutyByNumber(mtrCtrl->stepCtrl.controlApt.v, duty);
    HAL_APT_SetPWMDutyByNumber(mtrCtrl->stepCtrl.controlApt.w, duty);
}

/**
  * @brief Strong drag start.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ForceDragAcc(MtrCtrlHandle *mtrCtrl)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    unsigned int voltageDValue;
    unsigned int dragChangeFreq;
    unsigned int timerperiod;
    unsigned int freqLast;
    unsigned int aptCountsOneUs;

    mtrCtrl->sysVar.accTimeCnt++;
    if (mtrCtrl->sysVar.accTimeCnt < mtrCtrl->sysVar.dragChangePhaseTime) {
        return;
    }

    mtrCtrl->sysVar.accTimeCnt = 0;
    /* Step 1: Calculate the voltage difference. */
    voltageDValue = IN_VOLTAGE_BUS * RAMP_DUTY_PWM;
    /* Step 2: Calculate the commutation frequency. */
    freqLast = S_TO_US / (mtrCtrl->sysVar.dragChangePhaseTime * CTRL_CURE_PERIOD_US);
    dragChangeFreq = voltageDValue * STEP_MAX_NUM * POLES / (MOTOR_K * 2 * MATH_PI) + freqLast; /* 2*PI = 360° */
    /* Step 3: Convert the commutation frequency to the commutation count value. */
    timerperiod = mtrCtrl->stepCtrl.controlApt.u->waveform.timerPeriod * CTRL_CURE_PERIOD_US;
    mtrCtrl->sysVar.dragChangePhaseTime = S_TO_US / (timerperiod * dragChangeFreq);

    /* Determine whether the change phase speed has reached the speed of stopping forced drag. */
    if (mtrCtrl->sysVar.dragChangePhaseTime < DRAG_STOP_INTERVAL) {
        mtrCtrl->sysVar.dragChangePhaseTime = DRAG_STOP_INTERVAL;
        mtrCtrl->sysVar.accTimeCnt = 0;
        mtrCtrl->sysVar.bemfFilterCnt = 0;
        mtrCtrl->stateMachine = FSM_RUN;
        mtrCtrl->sysVar.lastZeroPoint = DCL_SYSTICK_GetTick();
        mtrCtrl->spdPi.integral = OP_TO_CL_INTERGRAL;
    }

    /* Shorten the time interval of forced drag change phase. */
    mtrCtrl->pwmDuty += RAMP_DUTY_PWM;
    aptCountsOneUs = HAL_CRG_GetIpFreq(APT0_BASE) / S_TO_US;
    mtrCtrl->sysVar.waitTime = (mtrCtrl->sysVar.dragChangePhaseTime * CTRL_CURE_PERIOD_US * aptCountsOneUs) >> 1;
    /* Change phase time is 2 the waiting time. */
    mtrCtrl->spdRefHz = (float)(HAL_CRG_GetIpFreq(SYSTICK_BASE) / (mtrCtrl->sysVar.waitTime * 2)) / STEP_MAX_NUM;
    mtrCtrl->spdEstHz = mtrCtrl->spdRefHz;
    mtrCtrl->spdRmg.yLast = mtrCtrl->spdRefHz;
    mtrCtrl->stepCtrl.phaseStep = (mtrCtrl->stepCtrl.phaseStep + 1) % STEP_MAX_NUM;
    SixStepPwm(&mtrCtrl->stepCtrl);
    /* Limiting the PWM Duty Cycle Range. */
    if (mtrCtrl->pwmDuty > FORCE_DRAG_MAXDUTY) {
        mtrCtrl->pwmDuty = FORCE_DRAG_MAXDUTY;
    }
    if (mtrCtrl->pwmDuty < FORCE_DRAG_MINDUTY) {
        mtrCtrl->pwmDuty = FORCE_DRAG_MINDUTY;
    }
    MCS_SetCtrAptDuty(mtrCtrl, mtrCtrl->pwmDuty);
}

/**
  * @brief Zero-crossing filter.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void BemfZeroSampleFilter(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->sysVar.bemfFilterCnt++;
    /* Filter the number of samples. */
    if (mtrCtrl->sysVar.bemfFilterCnt >= FILTER_COUNT) {
        mtrCtrl->sysVar.bemfFilterCnt = 0;
        unsigned int currentTick = DCL_SYSTICK_GetTick();
        if (mtrCtrl->sysVar.firstEventFilterFlag == 0) {
            mtrCtrl->sysVar.lastZeroPoint = currentTick;
            mtrCtrl->sysVar.changePhaseFlag = 1;
            mtrCtrl->sysVar.firstEventFilterFlag = 1;
            return;
        }
        /* Calculate the time interval between the last zero crossing. */
        mtrCtrl->sysVar.stepTime[mtrCtrl->stepCtrl.phaseStep] = currentTick >= mtrCtrl->sysVar.lastZeroPoint
            ? currentTick - mtrCtrl->sysVar.lastZeroPoint
            : SYSTICK_MAX_VALUE - mtrCtrl->sysVar.lastZeroPoint + currentTick + 1;
        mtrCtrl->sysVar.stepTimeNum++;
        /* Wait time is half the interval. */
        mtrCtrl->sysVar.waitTime = mtrCtrl->sysVar.stepTime[mtrCtrl->stepCtrl.phaseStep] >> 1;
        mtrCtrl->sysVar.waitTime -= PHASE_OFFSET;
        mtrCtrl->sysVar.lastZeroPoint = currentTick;
        /* Update flag. */
        mtrCtrl->sysVar.changePhaseFlag = 1;
    }
}

/**
  * @brief Back EMF zero crossing check.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void BemfZeroCheck(MtrCtrlHandle *mtrCtrl)
{
    /* The step corresponds to different conditional judgments. */
    unsigned int bemfCheckTable[STEP_MAX_NUM] = {mtrCtrl->bemf.w < mtrCtrl->zeroPoint ? 1 : 0,
                                                 mtrCtrl->bemf.v > mtrCtrl->zeroPoint ? 1 : 0,
                                                 mtrCtrl->bemf.u < mtrCtrl->zeroPoint ? 1 : 0,
                                                 mtrCtrl->bemf.w > mtrCtrl->zeroPoint ? 1 : 0,
                                                 mtrCtrl->bemf.v < mtrCtrl->zeroPoint ? 1 : 0,
                                                 mtrCtrl->bemf.u > mtrCtrl->zeroPoint ? 1 : 0};

    if (bemfCheckTable[mtrCtrl->stepCtrl.phaseStep] == 1) {
        BemfZeroSampleFilter(mtrCtrl);
    }
}

/**
  * @brief Change phase and speed estimation.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ChangePhase(MtrCtrlHandle *mtrCtrl)
{
    unsigned int currentTick = DCL_SYSTICK_GetTick(); /* Get the current tick value. */
    unsigned int intervalTick = (currentTick >= mtrCtrl->sysVar.lastZeroPoint)
                              ? currentTick - mtrCtrl->sysVar.lastZeroPoint
                              : SYSTICK_MAX_VALUE - mtrCtrl->sysVar.lastZeroPoint + currentTick + 1;
    /* Start average filtering when the number of recorded data reaches six. */
    if (mtrCtrl->sysVar.stepTimeNum >= STEP_MAX_NUM) {
        mtrCtrl->sysVar.stepTimeNum = 0;
        mtrCtrl->sysVar.stepTimeFilterEnable = 1;
    }
    /* If the accumulated time is greater than the waiting commutation time, commutation is performed. */
    if (intervalTick > mtrCtrl->sysVar.waitTime) {
        if (mtrCtrl->sysVar.stepTimeFilterEnable) { /* 6-step commutation time averaging if filtering is enabled. */
            unsigned int totalTime = mtrCtrl->sysVar.stepTime[STEP1] + mtrCtrl->sysVar.stepTime[STEP2]
                                   + mtrCtrl->sysVar.stepTime[STEP3] + mtrCtrl->sysVar.stepTime[STEP4]
                                   + mtrCtrl->sysVar.stepTime[STEP5] + mtrCtrl->sysVar.stepTime[STEP6];
            mtrCtrl->spdEstHz = (float)HAL_CRG_GetIpFreq(SYSTICK_BASE) / totalTime;
        } else {
            mtrCtrl->spdEstHz = (float)HAL_CRG_GetIpFreq(SYSTICK_BASE) / (mtrCtrl->sysVar.waitTime << 1) / STEP_MAX_NUM;
        }
        /* Change phase. */
        mtrCtrl->stepCtrl.phaseStep = (mtrCtrl->stepCtrl.phaseStep + 1) % STEP_MAX_NUM;
        SixStepPwm(&mtrCtrl->stepCtrl);
        mtrCtrl->sysVar.changePhaseFlag = 0;
    }
}

/**
 * @brief Zero-crossing detection and change phase
 * @param mtrCtrl The motor control handle.
 * @retval None.
 */
void MCS_CarrierProcess(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    /* Offset value of the calibration value of the three-phase current */

    switch (mtrCtrl->stateMachine) {
        case FSM_STARTUP:
            ForceDragAcc(mtrCtrl); /* Forced drag. */
            break;

        case FSM_RUN:
        case FSM_WAIT_STOP:
            mtrCtrl->readBemfUVW(&mtrCtrl->bemf);
            MCS_SetCtrAptDuty(mtrCtrl, mtrCtrl->pwmDuty);
            if (mtrCtrl->sysVar.changePhaseFlag) {
                /* Change phase process. */
                ChangePhase(mtrCtrl);
            } else {
                /* Zero-crossing detection procedure. */
                BemfZeroCheck(mtrCtrl);
            }
            break;

        default:
            break;
    }
}