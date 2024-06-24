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
  * @file      mcs_sensor_hall.h
  * @author
  * @brief
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_SENSOR_HALL_H
#define McuMagicTag_MCS_SENSOR_HALL_H
/* Includes ------------------------------------------------------------------------------------ */
#include "capm.h"

#define SPEED_BUFFER_SIZE 24

#define S32_120_PHASE_SHIFT DOUBLE_PI_DIV_THREE
#define S32_60_PHASE_SHIFT  ONE_PI_DIV_THREE

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief
  */

 typedef unsigned int (*MCS_GetHallValue)(void);

typedef enum {
    SECTOR_ONE = 1,
    SECTOR_TWO,
    SECTOR_THREE,
    SECTOR_FOUR,
    SECTOR_FIVE,
    SECTOR_SIX
} HALL_SECTOR;

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
  * @brief Rotation direction state define.
  * @details Rotation direction state type:
  *          + HALL_DIR_CW  -- Forward rotation.
  *          + HALL_DIR_CCW -- Reverse rotation.
  */
typedef enum {
    HALL_DIR_CW = 1,
    HALL_DIR_CCW = -1
} HALL_DIR_STATE;

/**
  * @brief Hall sensor control data structure
  */
typedef struct {
    float ts;                       /**< Control period (s). */
    float timer;                    /**< Timer count value. */
    int currentHallValue;           /**< Current sector. */
    int lastHallValue;              /**< Last sector. */
    float durationLast1;            /**< Last sector jump interval. */
    float durationLast2;            /**< The sector jump interval before durationLast1. */
    float durationLast3;            /**< The sector jump interval before durationLast2. */
    float durationAvg;              /**< Average transition interval. */
    int dir;                        /**< Rotation direction. */

    float angleComp;                /**< Angle correction parameters. */
    float angleStartPoint;          /**< The start electrical angle of the current sector.. */
    float phaseShift;               /**< Synchronous electrical angle. */
    float spd;                      /**< Electrical angle. */
    float angle;                    /**< Electrical speed. */

    short firstEdgeFilterFlag;      /**< First jump filter flag. */

    MCS_GetHallValue getHallValue;  /**< Pointer to the function for obtaining the value of the hall sensor. */
} HALL_Handle;

void HALL_Init(HALL_Handle *hall, float phaseShift, float ts);
unsigned int HALL_SectorCalc(unsigned int hallValue);
void HALL_InformationUpdate(HALL_Handle *hall);
void HALL_AngSpdCalcExec(HALL_Handle *hall);
void HALL_ParamClear(HALL_Handle *hall);

#endif
