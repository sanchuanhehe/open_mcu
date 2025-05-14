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

/**
  * @brief 同步旋转坐标系角度。
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_SyncCoorAngle(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    /* Synchronous rotation coordinate system angle. */
    switch (mtrCtrl->stateMachine) {
        case FSM_STARTUP:
            /* 当前斜坡角度为0。 */
            if (mtrCtrl->startup.stage == STARTUP_STAGE_CURR) {
                mtrCtrl->axisAngle = 0;
            } else if (mtrCtrl->startup.stage == STARTUP_STAGE_SPD) { /* 中频控制相位角自加。 */
                mtrCtrl->axisAngle = IF_CurrAngleCalc(&mtrCtrl->ifCtrl, mtrCtrl->spdRefHz);
            }
            break;

        case FSM_RUN:
            mtrCtrl->axisAngle = mtrCtrl->encAxisAngle;
            break;

        default:
            mtrCtrl->axisAngle = 0;
            break;
    }
}

/**
  * @brief 单电阻器和双电阻器的PWM波形设置和采样点设置。
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_PwmAdcSet(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    SampleMode sampleMode = mtrCtrl->sampleMode;
    /* 根据采样模式设置占空比. */
    if (sampleMode == DUAL_RESISTORS) {
        //三相上开关PWM波的占空比为在两相静止坐标系（albe）中计算。
        SVPWM_Exec(&mtrCtrl->sv, &mtrCtrl->vabRef, &mtrCtrl->dutyUvw);
        mtrCtrl->setPwmDutyCb(&mtrCtrl->dutyUvw, &mtrCtrl->dutyUvw);
    } else if (sampleMode == SINGLE_RESISTOR) {
        R1SVPWM_Exec(&mtrCtrl->r1Sv, &mtrCtrl->vabRef, &mtrCtrl->dutyUvwLeft, &mtrCtrl->dutyUvwRight);
        mtrCtrl->setPwmDutyCb(&mtrCtrl->dutyUvwLeft, &mtrCtrl->dutyUvwRight);
        /* ADC采样点位置需要根据单个电阻器的相移进行设置。*/
        mtrCtrl->setADCTriggerTimeCb(mtrCtrl->r1Sv.samplePoint[0] * mtrCtrl->aptMaxcntCmp, \
            mtrCtrl->r1Sv.samplePoint[1] * mtrCtrl->aptMaxcntCmp);
    }
}

/**
  * @brief 载波中断功能。
  * @param mtrCtrl The motor control handle.
  * @retval None.
  * 电流环
  */
void MCS_CarrierProcess(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    UvwAxis *currUvw = &mtrCtrl->currUvw;
    AlbeAxis *currAlbe = &mtrCtrl->iabFbk;
    AlbeAxis *vab = &mtrCtrl->vabRef;
    SampleMode sampleMode = mtrCtrl->sampleMode;
    /* 采样模式验证 */
    if (sampleMode > SINGLE_RESISTOR || mtrCtrl->setADCTriggerTimeCb == NULL) {
        return;
    }
    /* 参数验证 */
    if (mtrCtrl->setPwmDutyCb == NULL || mtrCtrl->readCurrUvwCb == NULL) {
        return;
    }
    /* 读取三相电流值。 */
    mtrCtrl->readCurrUvwCb(currUvw);
    mtrCtrl->getEncAngSpd(&mtrCtrl->encSpeed, &mtrCtrl->encAxisAngle);//读取反馈速度与角度
    /* 同步角度 */
    MCS_SyncCoorAngle(mtrCtrl);

    /* 克拉克变化  UVW->A、B    */
    ClarkeCalc(currUvw, currAlbe);
    /* 帕克变化 A、B +角度->q、d */
    ParkCalc(currAlbe, mtrCtrl->axisAngle, &mtrCtrl->idqFbk);

    /* 状态机 */
    switch (mtrCtrl->stateMachine) {
        case FSM_STARTUP:
        case FSM_RUN:
            CURRCTRL_Exec(&mtrCtrl->currCtrl, &mtrCtrl->vdqRef, mtrCtrl->smo.spdEst, 0);
            //帕克逆变化 ，将d，q轴电流＋角度逆变换为α，β。
            InvParkCalc(&mtrCtrl->vdqRef, mtrCtrl->axisAngle, vab);
            //单电阻器和双电阻器的PWM波形设置和采样点设置。
            MCS_PwmAdcSet(mtrCtrl);
            break;

        case FSM_CAP_CHARGE:
        case FSM_CLEAR:
        case FSM_IDLE:
            break;

        default:
            vab->alpha = 0.0f;
            vab->beta = 0.0f;
            MCS_PwmAdcSet(mtrCtrl);
            break;
    }
}