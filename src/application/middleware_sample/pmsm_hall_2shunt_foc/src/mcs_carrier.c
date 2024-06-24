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
#include "mcs_math.h"
#include "typedefs.h"
#include "mcs_assert.h"
#include "mcs_user_config.h"
#include "mcs_ctlmode_config.h"
#include "mcs_math_const.h"

/**
  * @brief Synchronous rotation coordinate system angle.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_SyncCoorAngle(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    /* Synchronous rotation coordinate system angle. */
    switch (mtrCtrl->stateMachine) {
        case FSM_STARTUP:
        case FSM_RUN:
            if (mtrCtrl->controlMode == SIXSTEPWAVE_CONTROLMODE) {
                mtrCtrl->axisAngle = mtrCtrl->hallSixStepAngle;  /* Sixstep angle drive at start-up stage. */
            } else if (mtrCtrl->controlMode == FOC_CONTROLMODE_SPEED) {
                mtrCtrl->axisAngle = mtrCtrl->hallAxisAngle;     /* Foc angle drive at mid-high speed stage. */
            }
            break;
        default:
            mtrCtrl->axisAngle = 0;
            break;
    }
}

/**
  * @brief PWM waveform setting and sampling point setting for a single resistors and dual resistors.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_PwmAdcSet(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    SampleMode sampleMode = mtrCtrl->sampleMode;
    /* Set the duty cycle according to the sampling mode. */
    if (sampleMode == DUAL_RESISTORS) {
        SVPWM_Exec(&mtrCtrl->sv, &mtrCtrl->vabRef, &mtrCtrl->dutyUvw);
        mtrCtrl->setPwmDutyCb(&mtrCtrl->dutyUvw, &mtrCtrl->dutyUvw);
    } else if (sampleMode == SINGLE_RESISTOR) {
        R1SVPWM_Exec(&mtrCtrl->r1Sv, &mtrCtrl->vabRef, &mtrCtrl->dutyUvwLeft, &mtrCtrl->dutyUvwRight);
        mtrCtrl->setPwmDutyCb(&mtrCtrl->dutyUvwLeft, &mtrCtrl->dutyUvwRight);
        /* The ADC sampling point position needs to be set based on the phase shift of a single resistors. */
        mtrCtrl->setADCTriggerTimeCb(mtrCtrl->r1Sv.samplePoint[0] * mtrCtrl->aptMaxcntCmp, \
            mtrCtrl->r1Sv.samplePoint[1] * mtrCtrl->aptMaxcntCmp);
    }
}

/**
  * @brief Carrier interrupt function.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
void MCS_CarrierProcess(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    static unsigned short sixStepToFocAngleCnt = 0;
    UvwAxis *currUvw = &mtrCtrl->currUvw;
    AlbeAxis *currAlbe = &mtrCtrl->iabFbk;
    AlbeAxis *vab = &mtrCtrl->vabRef;
    /* Read the three-phase current value. */
    mtrCtrl->readCurrUvwCb(currUvw);
    /* Get hall speed & angle. */
    mtrCtrl->getHallAngSpd(&mtrCtrl->hallSpeed, &mtrCtrl->hallAxisAngle);
    mtrCtrl->hallAxisAngle = Mod(mtrCtrl->hallAxisAngle, DOUBLE_PI);
    /* Limit angle range to -pi ~ pi. */
    if (mtrCtrl->hallAxisAngle > ONE_PI) {
        mtrCtrl->hallAxisAngle -= DOUBLE_PI;
    }
    if (mtrCtrl->hallAxisAngle < -ONE_PI) {
        mtrCtrl->hallAxisAngle += DOUBLE_PI;
    }
    /* Synchronization angle */
    MCS_SyncCoorAngle(mtrCtrl);
    /* Clark Calc */
    ClarkeCalc(currUvw, currAlbe);
    /* Park transformation */
    ParkCalc(currAlbe, mtrCtrl->axisAngle, &mtrCtrl->idqFbk);
    /* statemachine */
    switch (mtrCtrl->stateMachine) {
        case FSM_STARTUP:
        case FSM_RUN:{
            /* If speed is over set Speed threshold, switch control angle */
            if (Abs(mtrCtrl->hallSpeed) >= SIXSTEPTOFOC) {
                sixStepToFocAngleCnt++;
                /* 3000 is delay tick,delay 3000 tick cut from sixwave angle to foc control,
                    make sure cut speed point is steady */
                if (sixStepToFocAngleCnt >= 3000) {
                    mtrCtrl->controlMode = FOC_CONTROLMODE_SPEED;
                }
            } else {
                sixStepToFocAngleCnt = 0;
            }
            /* Current loop control */
            CURRCTRL_Exec(&mtrCtrl->currCtrl, &mtrCtrl->vdqRef, mtrCtrl->hallSpeed, 0);
            InvParkCalc(&mtrCtrl->vdqRef, mtrCtrl->axisAngle, vab);
            MCS_PwmAdcSet(mtrCtrl);
            break;
        }
        case FSM_CAP_CHARGE:
        case FSM_CLEAR:
        case FSM_IDLE:
            mtrCtrl->smo4th.spdEst = 0.0f;
            break;

        default:
            vab->alpha = 0.0f;
            vab->beta = 0.0f;
            MCS_PwmAdcSet(mtrCtrl);
            break;
    }
}