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
  * @file      mcs_sensor_hall.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of hall sensor signal process.
  */

#include "mcs_sensor_hall.h"
#include "mcs_assert.h"
#include "mcs_math_const.h"
#include "mcs_math.h"

#define HALL_INSTALL_CALIBRATION_ENABLE (1)

#define     EANGLE0     (0.0f)
#define     EANGLE30    (0.5235988f)
#define     EANGLE60    (1.0471976f)
#define     EANGLE90    (1.5707963f)
#define     EANGLE120   (2.0943951f)
#define     EANGLE150   (2.6179939f)
#define     EANGLE180   (3.1415927f)
#define     EANGLE210   (-2.6179939f)
#define     EANGLE240   (-2.0943951f)
#define     EANGLE270   (-1.5707963f)
#define     EANGLE300   (-1.0471976f)
#define     EANGLE330   (-0.5235988f)

/**
  * @brief
  */
typedef enum {
    APT_CHA_PWM_CHB_LOW,
    APT_CHA_LOW_CHB_HIGH,
    APT_CHA_LOW_CHB_LOW
} SIXSTEP_AptAct;

/**
  * @brief Configure three kinds of APT action, H_PWM_L_ON mode.
  * @param aptAddr APT base address.
  * @param aptAct The APT action.
  */
static void SIXSTEP_ForcePwmOut(APT_RegStruct* aptAddr, SIXSTEP_AptAct aptAct)
{
    switch (aptAct) {
        case APT_CHA_PWM_CHB_LOW:
            /* Channel A: 0 means not force output enable, channel A output PWM. */
            DCL_APT_DisableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            /* Channel B: 1 means force output enable. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            /* Channel B: 2 means channel B force output LOW due to the A_H_B_L invert. */
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;

        case APT_CHA_LOW_CHB_HIGH:
            /* Channel A: 1 means force output enable. */
            /* Channel A: 1 means channel A force output LOW. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            /* Channel B: 1 means force output enable. */
            /* Channel B: 1 means channel A force output HIGH due to the A_H_B_L invert. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_LOW);
            break;

        case APT_CHA_LOW_CHB_LOW:
            /* Channel A: 1 means force output enable. */
            /* Channel A: 1 means channel A force output LOW. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_A, APT_PWM_CONTINUOUS_ACTION_LOW);
            /* Channel B: 1 means force output enable. */
            /* Channel B: 2 means channel A force output LOW due to the A_H_B_L invert. */
            DCL_APT_EnableSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B);
            DCL_APT_SetSwContPWMAction(aptAddr, APT_PWM_CHANNEL_B, APT_PWM_CONTINUOUS_ACTION_HIGH);
            break;
        default:
            break;
    }
}

/**
  * @brief Configure the APT action mode for six sectors.
  * @param aptUvw APT base address of U V W.
  * @param sector Sector.
  */
void SIXSTEP_AptConfig(APT_RegStruct **aptUvw, HALL_SECTOR sector)
{
    APT_RegStruct *aptU = aptUvw[0];
    APT_RegStruct *aptV = aptUvw[1];
    APT_RegStruct *aptW = aptUvw[2]; /* index 2. */
    switch (sector) {
        case SECTOR1: /* A+, C- */
            SIXSTEP_ForcePwmOut(aptU, APT_CHA_PWM_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptV, APT_CHA_LOW_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptW, APT_CHA_LOW_CHB_HIGH);
            break;
        case SECTOR2: /* B+, C- */
            SIXSTEP_ForcePwmOut(aptU, APT_CHA_LOW_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptV, APT_CHA_PWM_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptW, APT_CHA_LOW_CHB_HIGH);
            break;
        case SECTOR3: /* B+, A- */
            SIXSTEP_ForcePwmOut(aptU, APT_CHA_LOW_CHB_HIGH);
            SIXSTEP_ForcePwmOut(aptV, APT_CHA_PWM_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptW, APT_CHA_LOW_CHB_LOW);
            break;
        case SECTOR4: /* C+, A- */
            SIXSTEP_ForcePwmOut(aptU, APT_CHA_LOW_CHB_HIGH);
            SIXSTEP_ForcePwmOut(aptV, APT_CHA_LOW_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptW, APT_CHA_PWM_CHB_LOW);
            break;
        case SECTOR5: /* C+, B- */
            SIXSTEP_ForcePwmOut(aptU, APT_CHA_LOW_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptV, APT_CHA_LOW_CHB_HIGH);
            SIXSTEP_ForcePwmOut(aptW, APT_CHA_PWM_CHB_LOW);
            break;
        case SECTOR6: /* A+, B- */
            SIXSTEP_ForcePwmOut(aptU, APT_CHA_PWM_CHB_LOW);
            SIXSTEP_ForcePwmOut(aptV, APT_CHA_LOW_CHB_HIGH);
            SIXSTEP_ForcePwmOut(aptW, APT_CHA_LOW_CHB_LOW);
            break;
        default:
            break;
    }
}


/**
  * @brief Set six step angel at start up stage.
  * @param sector 0-60: sec1, 60-120: sec2.
  * @retval None.
  */
static float CalcSixStepRadian(unsigned int sector)
{
    /* Angle assignment in different sector */
    float sixStepAngle = 0.0f;
    switch (sector) {
        /* Sector one the angle is 30 */
        case SECTOR1:
            sixStepAngle = EANGLE30;
            break;
        /* Sector two the angle is 90 degree */
        case SECTOR2:
            sixStepAngle = EANGLE90;
            break;
        /* Sector three the angle is 150 degree */
        case SECTOR3:
            sixStepAngle = EANGLE150;
            break;
        /* Sector four the angle is 210 degree */
        case SECTOR4:
            sixStepAngle = EANGLE210;
            break;
        /* Sector five the angle is 270 degree */
        case SECTOR5:
            sixStepAngle = EANGLE270;
            break;
        /* Sector six the angle is 330 degree */
        case SECTOR6:
            sixStepAngle = EANGLE330;
            break;

        default:
            break;
    }
    return sixStepAngle;
}


/**
 * @brief Hall sensor initialization interface.
 * @param handle Hall sensor handle.
 * @param  Hall sensor synchronous electrical angle.
 * @param ts Execution period.
 * @retval None.
 */
void HALL_Init(HALL_Handle *hall, int dir, float pllBdw, float spdCutOffFre, float ts)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(hall != NULL);
    hall->ts = ts;
    hall->timer = 0.0f;
    hall->t1 = 0.0f;
    hall->t2 = 0.0f;
    hall->t3 = 0.0f;
    hall->tAvg = 0.0f;
    /* Initialization direction. */
    hall->dir = dir;
    /* Initialize the basic angle based on the sector. */
    hall->angStart = 0.0f;

    hall->angleComp = 0.0f;
    hall->elecAngle = 0.0f;
    hall->spdEst = 0.0f;
    hall->firstEdgeFilterFlag = 0;

    PLL_Init(&hall->pll, ts, pllBdw);
    FOLPF_Init(&hall->spdLpf, ts, spdCutOffFre);
}

/**
  * @brief Calculate the start angle of FOC six-step control in clockwise direction.
  * @param sector Motor sector.
  * @retval None.
  */
static float HALL_StartAngleCw(HALL_SECTOR sector)
{
    float angStart = 0.0f;
        switch (sector) {
            case SECTOR1:
                angStart = EANGLE0; /* 0 */
                break;
            case SECTOR2:
                angStart = EANGLE60; /* 60 */
                break;
            case SECTOR3:
                angStart = EANGLE120; /* 120 */
                break;
            case SECTOR4:
                angStart = EANGLE180; /* 180 */
                break;
            case SECTOR5:
                angStart = EANGLE240; /* 240 */
                break;
            case SECTOR6:
                angStart = EANGLE300; /* 300 */
                break;
            default:
                break;
        }
    return angStart;
}

/**
  * @brief Calculate the start angle of FOC six-step control in counter clockwise direction.
  * @param sector Motor sector.
  * @retval None.
  */
static float HALL_StartAngleCcw(HALL_SECTOR sector)
{
    float angStart = 0.0f;
    switch (sector) {
        case SECTOR6:
            angStart = EANGLE0; /* 0 */
            break;
        case SECTOR5:
            angStart = EANGLE300; /* 300 */
            break;
        case SECTOR4:
            angStart = EANGLE240; /* 240 */
            break;
        case SECTOR3:
            angStart = EANGLE180; /* 180 */
            break;
        case SECTOR2:
            angStart = EANGLE120; /* 120 */
            break;
        case SECTOR1:
            angStart = EANGLE60; /* 60 */
            break;
        default:
            break;
    }
    return angStart;
}

/**
  * @brief Calculate the start angle of FOC six-step control
  * @param sector 0-60: sec1, 60-120: sec2.
  * @param dir Rotate direction.
  * @retval None.
  */
static float HALL_StartAngleUpdate(HALL_SECTOR sector, int dir)
{
    if (dir == HALL_DIR_CW) {
        /* Clockwise */
        return HALL_StartAngleCw(sector);
    } else if (dir == HALL_DIR_CCW) {
        /* Counter clockwise */
        return HALL_StartAngleCcw(sector);
    } else {
        return 0.0f;
    }
}


/**
 * @brief Updating Hall Sensor Information
 * @param hall Hall sensor handle.
 * @retval None.
 */
void HALL_CapmEvtCallBack(HALL_Handle *hall, HALL_SECTOR sector)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(hall != NULL);

    hall->sector = sector;
    hall->sixStepAngle = CalcSixStepRadian(sector);
    /* The first handle edge change is incorrect and needs to be filtered out. */
    if (hall->firstEdgeFilterFlag == 0) {
        hall->firstEdgeFilterFlag = 1;
        /* Sector start angle is (current_sector) * 60 degree. */
        hall->angStart = HALL_StartAngleUpdate(hall->sector, hall->dir);
        hall->sectorLast = hall->sector;
        return;
    }
    hall->angStart = HALL_StartAngleUpdate(hall->sector, hall->dir);
    /* Updated the Hall sensor jump duration. */
    if (hall->sector != hall->sectorLast) {
        hall->t3 = hall->t2;
        hall->t2 = hall->t1;
        hall->t1 = hall->timer;
        hall->timer = 0.0f;
        hall->sectorLast = hall->sector;
    }
}

/**
 * @brief Calculate the electrical angle and electrical speed.
 * @param hall Hall sensor handle.
 * @retval None.
 */
void HALL_Exec(HALL_Handle *hall, HALL_SECTOR sector)
{
    float spdEst;
    float runAngle;
    float elecAngle;
    hall->sector = sector;
    hall->sixStepAngle = CalcSixStepRadian(sector);
    /* Average time of three sectors. */
    hall->tAvg = ONE_DIV_THREE * (hall->t1 + hall->t2 + hall->t3);
    /* Handle motor stationary state. */
    if (hall->tAvg <= 0.00001f) {
        spdEst = 0.0f;
        elecAngle = 0.0f;
        hall->angleComp = 0.0f;
    } else {
        if (hall->spdCalcCnt < SPEED_CALC_SECTOR_NUMS) {
            runAngle = hall->timer / hall->tAvg * ONE_PI_DIV_THREE;
            elecAngle = hall->angStart + hall->dir * runAngle;
            spdEst = 1.0f / hall->t1 / SECTOR_MAX_NUM;
            hall->spdCalcCnt++;
        } else {
            /* Hall installing calibration. */
            hall->angleComp = (ONE_PI_DIV_THREE * ONE_DIV_THREE * (hall->t1 - hall->t3) / hall->tAvg);
            /* Angle estimation. */
            runAngle = hall->timer / hall->tAvg * ONE_PI_DIV_THREE;
            elecAngle = hall->angStart + hall->dir * runAngle + hall->dir * hall->angleComp;
            /* Speed estimation (Hz). */
            spdEst = 1.0f / hall->tAvg / SECTOR_MAX_NUM;
        }
    }
    PLL_Exec(&hall->pll, GetSin(elecAngle), GetCos(elecAngle));
    hall->elecAngle = hall->pll.angle;
    hall->spdEst = FOLPF_Exec(&hall->spdLpf, spdEst);
    
    /* Update history values. */
    hall->timer += hall->ts;

    HALL_CapmEvtCallBack(hall, sector);
}

/**
 * @brief Clearing historical data of hall sensors.
 * @param hall Hall sensor handle.
 * @retval None.
 */
void HALL_Clear(HALL_Handle *hall)
{
    MCS_ASSERT_PARAM(hall != NULL);
    /* Clear hall parameters. */
    hall->timer = 0.0f;
    /* Get motor current value. */
    hall->t1 = 0.0f;
    hall->t2 = 0.0f;
    hall->t3 = 0.0f;
    hall->tAvg = 0.0f;
    
    /* Init current angle. */
    hall->angStart = 0.0f;
    hall->elecAngle = 0.0f;
    hall->sixStepAngle = 0.0f;
    hall->spdEst = 0.0f;
    hall->firstEdgeFilterFlag = 0;
    hall->spdCalcCnt = 0;

    PLL_Clear(&hall->pll);
    FOLPF_Clear(&hall->spdLpf);
}