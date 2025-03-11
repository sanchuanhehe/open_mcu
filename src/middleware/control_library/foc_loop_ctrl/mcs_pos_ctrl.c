/**
  * Copyright (c) 2022 - 2024, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mcs_pos_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of motor position control.
  */

#include "typedefs.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_pos_ctrl.h"

/**
  * @brief Reset the position controller, fill with zero, NULL.
  * @param posHandle position controller struct handle.
  * @retval None.
  */
void POSCTRL_Clear(POSCTRL_Handle *posHandle)
{
    MCS_ASSERT_PARAM(posHandle != NULL);
    /* PID controller history values clear */
    PID_Clear(&posHandle->posPid);

    /* trajectory mode: history values clear */
    posHandle->runTime = 0.0f;
    posHandle->timeTick = 0.0f;
    posHandle->deltaTime = 0.0f;
    posHandle->accMax = 0.0f; /* the maximum accelerator.  */
    posHandle->jerk = 0.0f;
    posHandle->posTarget = 0.0f; /* Target position is 0. */
    posHandle->posIncRefPrev = 0.0f;
    posHandle->posIncRef = 0.0f;
    posHandle->posErr = 0.0f;
    posHandle->posFbkPrev = 0.0f;
}

/**
 * @brief Position control initialization function.
 * @param posHandle position controller struct handle.
 * @param ts Control period.
 */
void POSCTRL_Init(POSCTRL_Handle *posHandle, const PID_Param *pidCtrlTable, float ts, float posSlope)
{
    MCS_ASSERT_PARAM(posHandle != NULL);
    MCS_ASSERT_PARAM(pidCtrlTable != NULL);
    POSCTRL_Clear(posHandle);
    posHandle->ts = ts;
    
    /* position PID controller initialization */
    posHandle->posPid.ts = posHandle->ts;
    posHandle->posPid.kp = pidCtrlTable->kp;
    posHandle->posPid.ki = pidCtrlTable->ki;
    posHandle->posPid.kd = pidCtrlTable->kd;
    posHandle->posPid.ns = pidCtrlTable->ns;
    posHandle->posPid.ka = 1.0f / posHandle->posPid.kp;
    posHandle->posPid.upperLimit = pidCtrlTable->upperLim;
    posHandle->posPid.lowerLimit = -pidCtrlTable->upperLim;

    /* continuous mode: ramp controller initialization */
    RMG_Init(&posHandle->posRmg, posHandle->ts, posSlope);
    posHandle->posRmg.ts = posHandle->ts;
    
    /* position feedback history values clear */
    posHandle->angFbkLoop = 0;
    posHandle->angFbkPrev = 0.0f;

    /* position control mode configuration */
    posHandle->mode = POSCTRL_MODE_CONTINUOUS;
}

/**
 * @brief Position control mode settings.
 * @param posHandle position controller struct handle.
 * @param mode control mode.
 */
void POSCTRL_ModeSelect(POSCTRL_Handle *posHandle, POSCTRL_Mode mode)
{
    posHandle->mode = mode;
}

/**
  * @brief Set position change rate.
  * @param posHandle Position controller struct handle.
  * @param slope position change rate (Hz).
  * @retval None.
  */
void POSCTRL_SetSlope(POSCTRL_Handle *posHandle, float slope)
{
    RMG_SetSlope(&posHandle->posRmg, slope * DOUBLE_PI);
}

/**
 * @brief Position ring target position setting.
 * @param posHandle Position controller struct handle.
 * @param posTarget Target location.
 */
void POSCTRL_SetTarget(POSCTRL_Handle *posHandle, float posTarget)
{
    posHandle->posTarget = posTarget;
    posHandle->posTargetShadow = posTarget;
}

/**
 * @brief Absolute position calculation.
 * @param posHandle Position controller struct handle.
 * @param angFbk Angle feedback.
 * @return float, Position feedback.
 */
float POSCTRL_AngleExpand(POSCTRL_Handle *posHandle, float angFbk)
{
    float angFbkPrevFloat = posHandle->angFbkPrev;
    int   loopPrev = posHandle->angFbkLoop;
    int   loop; /* Number of rotation */

    /* unify feedback angle to ±2*pi */
    angFbk = Mod(angFbk, DOUBLE_PI);
    /* unify feedback angle to 0 ~ 2*pi */
    if (angFbk < 0.0f) {
        angFbk += DOUBLE_PI;
    }

    /* check if angle rotates one cycle */
    if (angFbkPrevFloat > THREE_PI_DIV_TWO && angFbkPrevFloat <= DOUBLE_PI && angFbk < HALF_PI) {
        /* if theta_pre is at [270, 360), and theta is at [0, 90), that means one cycle increased. */
        loop = loopPrev + 1;
    } else if (angFbk > THREE_PI_DIV_TWO && angFbk <= DOUBLE_PI && angFbkPrevFloat < HALF_PI) {
        /* if theta is at [270, 360), and theta_pre is at [0, 90), that means one cycle decreased. */
        loop = loopPrev - 1;
    } else {
        loop = loopPrev;
    }

    /* update prev value */
    posHandle->angFbkLoop = loop;
    posHandle->angFbkPrev = angFbk;

    /* update output value, the absolute position is (360 * loop + theta). */
    posHandle->posFbk = angFbk + loop * DOUBLE_PI;

    return posHandle->posFbk;
}

/**
 * @brief Position ring PID execution function.
 * @param posHandle Position controller struct handle.
 * @param posErr position error.
 * @return Speed reference.
 */
float POSCTRL_PidExec(POSCTRL_Handle *posHandle, float posErr)
{
    float spdRef;
    posHandle->posPid.error = posErr;
    spdRef = PID_Exec(&posHandle->posPid);
    return spdRef;
}

/**
 * @brief Trajectory planning calculation.
 * @param posHandle Position controller struct handle.
 * @param posFbk Position feedback.
 * @param posIncRef.
 */
void POSCTRL_TrajCtrlPrepare(POSCTRL_Handle *posHandle, float posFbk, float *posIncRef)
{
    POSCTRL_Clear(posHandle); /* clear history values */
    posHandle->posTarget = posHandle->posTargetShadow; /* update position target */
    posHandle->posFbkPrev = posFbk;                    /* store position feedback before trajectory starts */
    posHandle->posErr = posHandle->posTarget - posHandle->posFbkPrev; /* position increment for this trajectory */

    /* limit the maximum position change rate according to the speed limitation */
    float posSlope = posHandle->posRmg.slope;

    if (posSlope > posHandle->posPid.upperLimit * TWO_DIV_THREE) {
        posSlope = posHandle->posPid.upperLimit * TWO_DIV_THREE;
    } else if (posSlope < 0.0f) {
        posSlope = 1.0f;
    }
    /* calulate whole trajectory lasts time (s) */
    posHandle->runTime = Abs(posHandle->posErr) / posSlope;
    /* calulate each stage lasts time (s) */
    posHandle->deltaTime = posHandle->runTime * ONE_DIV_NINE;
    /* square of deltaTime */
    posHandle->deltaTimeSq = posHandle->deltaTime * posHandle->deltaTime;
    /* cube of deltaTime */
    posHandle->deltaTimeCu = posHandle->deltaTimeSq * posHandle->deltaTime;
    /* jerk of the trajectory (m/s^3) */
    posHandle->jerk = posHandle->posErr * ONE_DIV_TWELVE / posHandle->deltaTimeCu;
    /* maximum accelerate of the trajectory (m/s^2) */
    posHandle->accMax = posHandle->jerk * posHandle->deltaTime;

    /* trajectory time stage (s) , of the index 0, 1, 2, 3, 4, 5, 6. */
    posHandle->timeStg[TRAJ_STAGE_INDEX_0] = posHandle->deltaTime;
    posHandle->timeStg[TRAJ_STAGE_INDEX_1] = 2.0f * posHandle->deltaTime;
    posHandle->timeStg[TRAJ_STAGE_INDEX_2] = 3.0f * posHandle->deltaTime;
    posHandle->timeStg[TRAJ_STAGE_INDEX_3] = 6.0f * posHandle->deltaTime;
    posHandle->timeStg[TRAJ_STAGE_INDEX_4] = 7.0f * posHandle->deltaTime;
    posHandle->timeStg[TRAJ_STAGE_INDEX_5] = 8.0f * posHandle->deltaTime;
    posHandle->timeStg[TRAJ_STAGE_INDEX_6] = 9.0f * posHandle->deltaTime;

    *posIncRef = 0.0f;
}

/**
 * @brief Calculate the output position state.
 * @param posHandle Position controller struct handle.
 * @return stage.
 */
static POSCTRL_TRAJ_STAGE TrajPosState(POSCTRL_Handle *posHandle)
{
    POSCTRL_TRAJ_STAGE stage;
    /**< trajectory control inner timer (s) */
    float timeTick = posHandle->timeTick;
    /* Seven-segment trajectory time */
    float *timeStg = posHandle->timeStg;
    /* Acceleration deacceleration state transition */
    if (timeTick < timeStg[TRAJ_STAGE_INDEX_0]) {
        stage = TRAJ_STAGE_INDEX_0;
    } else if (timeTick < timeStg[TRAJ_STAGE_INDEX_1]) {
        stage = TRAJ_STAGE_INDEX_1;
    } else if (timeTick < timeStg[TRAJ_STAGE_INDEX_2]) {
        stage = TRAJ_STAGE_INDEX_2;
    } else if (timeTick < timeStg[TRAJ_STAGE_INDEX_3]) {
        stage = TRAJ_STAGE_INDEX_3;
    } else if (timeTick < timeStg[TRAJ_STAGE_INDEX_4]) {
        stage = TRAJ_STAGE_INDEX_4;
    } else if (timeTick < timeStg[TRAJ_STAGE_INDEX_5]) {
        stage = TRAJ_STAGE_INDEX_5;
    } else if (timeTick < timeStg[TRAJ_STAGE_INDEX_6]) {
        stage = TRAJ_STAGE_INDEX_6;
    } else {
        stage = TRAJ_STAGE_INDEX_7;
    }
    /* return trajectory state */
    return stage;
}

/**
 * @brief Calculate the output position increment reference.
 * @param posHandle Position controller struct handle.
 * @return postion.
 */
static float TrajPosCalc(POSCTRL_Handle *posHandle)
{
    float posIncRef; /* output position increment reference (rad) */
    /* local variables */
    float timeTick = posHandle->timeTick;
    float *timeStg = posHandle->timeStg;
    float jerk = posHandle->jerk;
    float acc  = posHandle->accMax;
    float half_acc = 0.5f * acc; /* 0.5f : half */
        /* temporary variables */
    float deltaTimeSq = 0.0f;
    float deltaTimeCu = 0.0f;
    float deltaTime = 0.0f;
    float jerkDtcu = jerk * posHandle->deltaTimeCu;
    float jerkDtsq = jerk * posHandle->deltaTimeSq;
    POSCTRL_TRAJ_STAGE stage = TrajPosState(posHandle);
    if (TRAJ_STAGE_INDEX_1 <= stage && stage <= TRAJ_STAGE_INDEX_6) {
        deltaTime = timeTick - timeStg[stage - 1];
        deltaTimeSq = deltaTime * deltaTime;
        deltaTimeCu = deltaTimeSq * deltaTime;
    }
    switch (stage) {
        case TRAJ_STAGE_INDEX_0:
            /* jerk > 0, acc > 0 */
            deltaTimeCu = timeTick * timeTick * timeTick;
            posIncRef = ONE_DIV_SIX * jerk * deltaTimeCu;
            break;
        case TRAJ_STAGE_INDEX_1:
            /* jerk = 0, acc > 0 */
            posIncRef = half_acc * deltaTimeSq + 0.5f * jerkDtsq * deltaTime + ONE_DIV_SIX * jerkDtcu;
            break;
        case TRAJ_STAGE_INDEX_2:
            /* jerk < 0, acc > 0 */
            posIncRef = - ONE_DIV_SIX * jerk * deltaTimeCu +
                        half_acc * deltaTimeSq + 1.5f * jerkDtsq * deltaTime + SEVEN_DIV_SIX * jerkDtcu;
            break;
        case TRAJ_STAGE_INDEX_3:
            /* jerk = 0, acc = 0 */
            posIncRef = 3.0f * jerkDtcu + 2.0f * jerkDtsq * deltaTime;
            break;
        case TRAJ_STAGE_INDEX_4:
            /* jerk < 0, acc < 0 */
            posIncRef = - ONE_DIV_SIX * jerk * deltaTimeCu + 2.0f * jerkDtsq * deltaTime + 9.0f * jerkDtcu;
            break;
        case TRAJ_STAGE_INDEX_5:
            /* jerk = 0, acc < 0 */
            posIncRef = - half_acc * deltaTimeSq + 1.5f * jerkDtsq * deltaTime + SIXTY_FIVE_DIV_SIX * jerkDtcu;
            break;
        case TRAJ_STAGE_INDEX_6:
            /* jerk > 0, acc < 0 */
            posIncRef = ONE_DIV_SIX * jerk * deltaTimeCu - half_acc * deltaTimeSq +
                        0.5f * jerkDtsq * deltaTime + SEVENTY_ONE_DIV_SIX * jerkDtcu;
            break;
        default:
            posIncRef = posHandle->posIncRefPrev; /* maintain last position */
            break;
    }

    return posIncRef;
}

/**
 * @brief Executing location track planning.
 * @param posHandle Position controller struct handle.
 * @param posFbk Position feedback.
 * @return postion.
 */
float POSCTRL_TrajCtrlExec(POSCTRL_Handle *posHandle, float posFbk)
{
    float posIncRef; /* output position increment reference (rad) */
    /* every start of a new trajectory run this function */
    /* Expanded by 10000 times and converted to an integer to avoid floating-point comparison. */
    if ((int)(posHandle->posTargetBk * 10000.0f) != (int)(posHandle->posTargetShadow * 10000.0f)) {
        POSCTRL_TrajCtrlPrepare(posHandle, posFbk, &posIncRef);
        posHandle->posTargetBk = posHandle->posTargetShadow;
    }

    /* time accumulates */
    posHandle->timeTick += posHandle->ts; /* tick time updates */

    posIncRef = TrajPosCalc(posHandle);

    posHandle->posIncRefPrev = posIncRef;
    posHandle->posIncRef = posIncRef;

    return (posIncRef + posHandle->posFbkPrev);
}

/**
 * @brief position loop execution function.
 * @param posHandle Position controller struct handle.
 * @param posFbk Position feedback.
 * @return float, Speed reference value.
 */
float POSCTRL_Exec(POSCTRL_Handle *posHandle, float posFbk)
{
    float posRef, spdRef;

    if (posHandle->mode == POSCTRL_MODE_CONTINUOUS) {
        posRef = RMG_Exec(&posHandle->posRmg, posHandle->posTarget);
        posHandle->posTargetBk = 0.0f;
    } else {
        /* posHandle->mode == POSCTRL_MODE_TRAJ */
        posRef = POSCTRL_TrajCtrlExec(posHandle, posFbk);
        posHandle->posRmg.yLast = posRef;
    }
    posHandle->posRef = posRef;
    spdRef = POSCTRL_PidExec(posHandle, posRef - posFbk);
    spdRef *= ONE_DIV_DOUBLE_PI; /* transfer spdRef from rad/s to Hz */
    posHandle->spdRef = spdRef;
    return spdRef;
}

/**
 * @brief Set position loop kp parameter function.
 * @param posHandle Position controller struct handle.
 * @param kp PID-kp paramter.
 */
void POSCTRL_SetKp(POSCTRL_Handle *posHandle, float kp)
{
    posHandle->posPid.kp = kp;
}

/**
 * @brief Set position loop ki parameter function.
 * @param posHandle Position controller struct handle.
 * @param ki PID-ki paramter.
 */
void POSCTRL_SetKi(POSCTRL_Handle *posHandle, float ki)
{
    posHandle->posPid.ki = ki;
}

/**
 * @brief Set position loop kd parameter function.
 * @param posHandle Position controller struct handle.
 * @param kd PID-kd paramter.
 */
void POSCTRL_SetKd(POSCTRL_Handle *posHandle, float kd)
{
    posHandle->posPid.kd = kd;
}

/**
 * @brief Set position loop Ns parameter function.
 * @param posHandle Position controller struct handle.
 * @param ns ns paramter.
 */
void POSCTRL_SetNs(POSCTRL_Handle *posHandle, float ns)
{
    posHandle->posPid.ns = ns;
}