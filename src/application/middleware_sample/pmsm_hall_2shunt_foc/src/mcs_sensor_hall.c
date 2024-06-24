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
/**
 * @brief Hall sensor initialization interface.
 * @param handle Hall sensor handle.
 * @param phaseShift Hall sensor synchronous electrical angle.
 * @param ts Execution period.
 * @retval None.
 */
void HALL_Init(HALL_Handle *handle, float phaseShift, float ts)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(handle != NULL);
    handle->ts = ts;
    handle->timer = 0.0f;
    /* Initializing hall sector. */
    handle->currentHallValue = HALL_SectorCalc(handle->getHallValue());
    handle->lastHallValue = HALL_SectorCalc(handle->getHallValue());
    handle->durationLast1 = 0.0f;
    handle->durationLast2 = 0.0f;
    handle->durationLast3 = 0.0f;
    handle->durationAvg = 0.0f;
    /* Initialization direction. */
    handle->dir = HALL_DIR_CW;
    /* Initialize the basic angle based on the sector. */
    if (handle->currentHallValue) {
        handle->angleStartPoint = (handle->currentHallValue - 1) * DOUBLE_PI;
    } else {
        handle->angleStartPoint = 0.0f;
    }
    handle->angleComp = 0.0f;
    handle->phaseShift = phaseShift;
    handle->angle = handle->angleStartPoint;
    handle->spd = 0.0f;
    handle->firstEdgeFilterFlag = 0;
}

/**
 * @brief Obtain the corresponding sector based on the value of the hall sensor.
 * @param hallValue Hall sensor value.
 * @retval None.
 */
unsigned int HALL_SectorCalc(unsigned int hallValue)
{
    /* Hall install mode is HALL_120_DEGREE */
    switch (hallValue) {
        case 5: /* 5 is current hall summation value */
            return SECTOR_ONE;
            break;
        case 4: /* 4 is current hall summation value */
            return SECTOR_TWO;
            break;
        case 6: /* 6 is current hall summation value */
            return SECTOR_THREE;
            break;
        case 2: /* 2 is current hall summation value */
            return SECTOR_FOUR;
            break;
        case 3: /* 3 is current hall summation value */
            return SECTOR_FIVE;
            break;
        case 1: /* 1 is current hall summation value */
            return SECTOR_SIX;
            break;

        default:
            return SECTOR_ONE;
            break;
    }
}

/**
 * @brief Updated the Hall sensor jump duration.
 * @param handle Hall sensor handle.
 * @retval None.
 */
static void HALL_DurationUpdate(HALL_Handle *handle)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(handle != NULL);
    handle->durationLast3 = handle->durationLast2;
    handle->durationLast2 = handle->durationLast1;
    handle->durationLast1 = handle->timer;
    handle->timer = 0.0f;
}

/**
 * @brief Judge motor rotation CW direction.
 * @param handle Hall sensor handle.
 * @retval None.
 */
static void HALL_DirCw(HALL_Handle *handle)
{
    /* Hall rotation: CW. */
    if (handle->currentHallValue == SECTOR_ONE && handle->lastHallValue == SECTOR_SIX) {
        /* Init angele shift is pi. */
        handle->angleStartPoint = handle->phaseShift;
    } else if (handle->currentHallValue == SECTOR_TWO && handle->lastHallValue == SECTOR_ONE) {
        /* Sector 2 && 1, the start angle shift is shift+60 degre. */
        handle->angleStartPoint = handle->phaseShift + S32_60_PHASE_SHIFT;
    } else if (handle->currentHallValue == SECTOR_THREE && handle->lastHallValue == SECTOR_TWO) {
        /* Sector 3 && 2, the start angle shift is shift+120 degre. */
        handle->angleStartPoint = handle->phaseShift + S32_120_PHASE_SHIFT;
    } else if (handle->currentHallValue == SECTOR_FOUR && handle->lastHallValue == SECTOR_THREE) {
        /* Sector 4 && 3, the start angle shift is shift+180 degre. */
        handle->angleStartPoint = handle->phaseShift + S32_60_PHASE_SHIFT + S32_120_PHASE_SHIFT;
    } else if (handle->currentHallValue == SECTOR_FIVE && handle->lastHallValue == SECTOR_FOUR) {
        /* Sector 5 && 4, the start angle shift is shift-120 degre. */
        handle->angleStartPoint = handle->phaseShift - S32_120_PHASE_SHIFT;
    } else if (handle->currentHallValue == SECTOR_SIX && handle->lastHallValue == SECTOR_FIVE) {
        /* Sector 6 && 5, the start angle shift is shift-60 degre. */
        handle->angleStartPoint = handle->phaseShift - S32_60_PHASE_SHIFT;
    }
}

/**
 * @brief Updating Hall Sensor Information
 * @param handle Hall sensor handle.
 * @retval None.
 */
void HALL_InformationUpdate(HALL_Handle *handle)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(handle != NULL);

    handle->currentHallValue = HALL_SectorCalc(handle->getHallValue());

    /* The first handle edge change is incorrect and needs to be filtered out. */
    if (handle->firstEdgeFilterFlag == 0) {
        handle->firstEdgeFilterFlag = 1;
        if (handle->currentHallValue) {
            /* Sector start angle is (current_sector - 1) * 60 degree. */
            handle->angleStartPoint = (handle->currentHallValue - 1) * S32_60_PHASE_SHIFT;
        } else {
            handle->angleStartPoint = 0.0f;
        }
        handle->angle = handle->angleStartPoint;
        handle->lastHallValue = handle->currentHallValue;
        return;
    }
    /* Judgement motor rotation direction */
    HALL_DirCw(handle);

    /* Updated the Hall sensor jump duration. */
    if (handle->currentHallValue != handle->lastHallValue) {
        HALL_DurationUpdate(handle);
        handle->lastHallValue = handle->currentHallValue;
    }
}

/**
 * @brief Calculate the electrical angle and electrical speed.
 * @param handle Hall sensor handle.
 * @retval None.
 */
void HALL_AngSpdCalcExec(HALL_Handle *handle)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(handle != NULL);
    handle->durationAvg = ONE_DIV_THREE * (handle->durationLast1 + handle->durationLast2 + handle->durationLast3);

    if (handle->durationAvg <= 0.00001f) {
        handle->spd = 0.0f;
        handle->angleComp = 0.0f;
    } else {
        /* Hall installing calibration. */
        #if (HALL_INSTALL_CALIBRATION_ENABLE == 0)
            handle->angleComp = 0.0f;
        #elif (HALL_INSTALL_CALIBRATION_ENABLE == 1)
            handle->angleComp = (S32_60_PHASE_SHIFT * ONE_DIV_THREE *
                (handle->durationLast1 - handle->durationLast3) / handle->durationAvg);
        #endif
        /* Speed estimation (Hz). */
        handle->spd = (float)handle->dir / handle->durationAvg / 6.0f;
        /* Angle estimation. */
        handle->angle = handle->angleStartPoint + handle->dir *
            (handle->timer / handle->durationAvg * ONE_PI_DIV_THREE + handle->angleComp);
    }
    /* Update history values. */
    handle->timer += handle->ts;
}

/**
 * @brief Clearing historical data of hall sensors.
 * @param handle Hall sensor handle.
 * @retval None.
 */
void HALL_ParamClear(HALL_Handle *handle)
{
    MCS_ASSERT_PARAM(handle != NULL);
    /* Clear hall parameters. */
    handle->timer = 0.0f;
    /* Get motor current value. */
    handle->currentHallValue = HALL_SectorCalc(handle->getHallValue());
    /* Get motor current position sector. */
    handle->lastHallValue = HALL_SectorCalc(handle->getHallValue());
    handle->durationLast1 = 0.0f;
    handle->durationLast2 = 0.0f;
    handle->durationLast3 = 0.0f;
    handle->durationAvg = 0.0f;
    /* Set motor init direction. */
    handle->dir = HALL_DIR_CW;
    if (handle->currentHallValue) {
        handle->angleStartPoint = (handle->currentHallValue - 1) * ONE_PI_DIV_THREE;
    } else {
        handle->angleStartPoint = 0;
    }
    /* Init current angle. */
    handle->angle = handle->angleStartPoint;
    handle->spd = 0.0f;
    handle->firstEdgeFilterFlag = 0;
}