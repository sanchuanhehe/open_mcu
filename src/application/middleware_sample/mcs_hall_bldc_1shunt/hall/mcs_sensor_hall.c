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
  * @file      mcs_sensor_hall.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of hall sensor signal process.
  */

#include "mcs_sensor_hall.h"
#include "mcs_assert.h"
#include "mcs_math.h"
#include "mcs_math_const.h"

/* Hall sector. */
#define SECTOR_ONE      1U
#define SECTOR_TWO      2U
#define SECTOR_THREE    3U
#define SECTOR_FOUR     4U
#define SECTOR_FIVE     5U
#define SECTOR_SIX      6U

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
    /* Init timer. */
    handle->timer = 0.0f;
    /* Initializing hall sector. */
    handle->sec = HALL_SectorCalc(handle->getHallValue());
    handle->secLast = HALL_SectorCalc(handle->getHallValue());
    handle->durationLast1 = 0.0f;
    handle->durationLast2 = 0.0f;
    handle->durationLast3 = 0.0f;
    handle->durationAvg = 0.0f;
    /* Initialization direction. */
    handle->dir = HALL_DIR_CW;
    /* Initialize the basic angle based on the sector. */
    if (handle->sec) {
        handle->angleStartPoint = (float)(handle->sec - 1) * ONE_PI_DIV_THREE; // S16_60_PHASE_SHIFT
    } else {
        handle->angleStartPoint = 0;
    }
    handle->angleComp = 0.0f;
    handle->phaseShift = phaseShift;
    handle->angle = handle->angleStartPoint;
    handle->spd = 0.0f;
    handle->firstEdgeFilterFlag = 0;
}

/**
 * @brief Obtain sector based on the 120 degree Installation.
 * @param hallValue Hall sensor value.
 * @retval None.
 */
static unsigned int HALL_InstallDegree120(unsigned int hallValue)
{
    if (hallValue == 5) {           /* Hall value 5 : HALL_A = 1, HALL_B = 0, HALL_C = 1 */
        return SECTOR_ONE;
    } else if (hallValue == 4) {    /* Hall value 4 : HALL_A = 1, HALL_B = 0, HALL_C = 0 */
        return SECTOR_TWO;
    } else if (hallValue == 6) {    /* Hall value 6 : HALL_A = 1, HALL_B = 1, HALL_C = 0 */
        return SECTOR_THREE;
    } else if (hallValue == 2) {    /* Hall value 2 : HALL_A = 0, HALL_B = 1, HALL_C = 0 */
        return SECTOR_FOUR;
    } else if (hallValue == 3) {    /* Hall value 3 : HALL_A = 0, HALL_B = 1, HALL_C = 1 */
        return SECTOR_FIVE;
    } else if (hallValue == 1) {    /* Hall value 1 : HALL_A = 0, HALL_B = 0, HALL_C = 1 */
        return SECTOR_SIX;
    } else {
        return 0;
    }
}


/**
 * @brief Obtain the corresponding sector based on the value of the hall sensor.
 * @param hallValue Hall sensor value.
 * @retval None.
 */
unsigned int HALL_SectorCalc(unsigned int hallValue)
{
    /* Different installation, choose different function. */
    return HALL_InstallDegree120(hallValue);
}

/**
 * @brief Updated the Hall sensor jump duration.
 * @param hall Hall sensor handle.
 * @retval None.
 */
static void HALL_DurationUpdate(HALL_Handle *hall)
{
    /* Verifying Parameters. */
    MCS_ASSERT_PARAM(hall != NULL);
    hall->durationLast3 = hall->durationLast2;
    hall->durationLast2 = hall->durationLast1;
    hall->durationLast1 = hall->timer;
    hall->timer = 0.0f;
}

/**
 * @brief Hall rotation direction.
 * @param handle Hall sensor handle.
 * @retval None.
 */
static void HALL_RotationDir(HALL_Handle *handle)
{
    /* Specail handle. */
    if (handle->sec == SECTOR_ONE && handle->secLast == SECTOR_SIX) {
        handle->dir = HALL_DIR_CW;
        handle->angleStartPoint = handle->phaseShift;
    }
    /* CW direction handle. */
    if (handle->sec - handle->secLast == 1) {
        handle->dir = HALL_DIR_CW;
        handle->angleStartPoint = handle->phaseShift + (float)handle->secLast * ONE_PI_DIV_THREE;
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

    handle->sec = HALL_SectorCalc(handle->getHallValue());

    /* The first handle edge change is incorrect and needs to be filtered out. */
    if (handle->firstEdgeFilterFlag == 0) {
        handle->firstEdgeFilterFlag = 1;
        if (handle->sec) {
            handle->angleStartPoint = (float)(handle->sec - 1) * ONE_PI_DIV_THREE; // S16_60_PHASE_SHIFT;
        } else {
            handle->angleStartPoint = 0;
        }
        handle->angle = handle->angleStartPoint;
        handle->secLast = handle->sec;
        return;
    }

    /* Hall rotation: CW. */
    HALL_RotationDir(handle);

    /* Updated the Hall sensor jump duration. */
    if (handle->sec != handle->secLast) {
        HALL_DurationUpdate(handle);
        handle->secLast = handle->sec;
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

    if (Abs(handle->durationAvg) <= 1e-5f) {
        handle->spd = 0.0f;
        handle->angleComp = 0.0f;
    } else {
        /* Hall installing calibration. */
        #if (HALL_INSTALL_CALIBRATION_ENABLE == 0)
            handle->angleComp = 0.0f;
        #elif (HALL_INSTALL_CALIBRATION_ENABLE == 1)
            handle->angleComp = (ONE_PI_DIV_THREE * ONE_DIV_THREE * \
                                (handle->durationLast1 - handle->durationLast3) / handle->durationAvg);
        #endif
        /* Speed estimation (Hz). */
        handle->spd = (float)handle->dir / handle->durationAvg / 6.0f;
        /* Angle estimation (float). */
        handle->angle = handle->angleStartPoint + (float)handle->dir * \
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
void HALL_Clear(HALL_Handle *handle)
{
    /* Clear duration. */
    handle->durationLast1 = 0.0f;
    handle->durationLast2 = 0.0f;
    handle->durationLast3 = 0.0f;
    handle->durationAvg = 0.0f;
    /* Clear hall sector and last sector. */
    handle->sec = HALL_SectorCalc(handle->getHallValue());
    handle->secLast = HALL_SectorCalc(handle->getHallValue());
    /* Default direction is CW. */
    handle->dir = HALL_DIR_CW;
    /* Handle angle start point. */
    if (handle->sec) {
        handle->angleStartPoint = (float)(handle->sec - 1) * ONE_PI_DIV_THREE;
    } else {
        handle->angleStartPoint = 0;
    }
    handle->angle = handle->angleStartPoint;
    handle->spd = 0.0f;
    handle->firstEdgeFilterFlag = 0;
    handle->timer = 0.0f; /* Hall timer. */
}