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
  * @file      mcs_sensor_hall.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of hall sensor data process related structures and functions.
  */

 /* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_HALL_Q15_H
#define McuMagicTag_MCS_HALL_Q15_H

#define HALL_INSTALL_60_DEGREE          (1)
#define HALL_INSTALL_120_DEGREE         (2)
#define HALL_INSTALL_MODE               (HALL_INSTALL_120_DEGREE)

#define HALL_INSTALL_CALIBRATION_ENABLE (1)

#define S16_120_PHASE_SHIFT (short)(65536.0f/3.0f)
#define S16_60_PHASE_SHIFT  (short)(65536.0f/6.0f)

typedef unsigned int (*MCS_GetHallValue)(void);

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
    unsigned int sec;                        /**< Current sector. */
    unsigned int secLast;                    /**< Last sector. */
    float durationLast1;            /**< Last sector jump interval. */
    float durationLast2;            /**< The sector jump interval before durationLast1. */
    float durationLast3;            /**< The sector jump interval before durationLast2. */
    float durationAvg;              /**< Average transition interval. */
    HALL_DIR_STATE dir;             /**< Rotation direction. */

    float angleComp;
    float angleStartPoint;
    float phaseShift;
    float angle;
    float spd;                      /**< Electrical speed. */
    short firstEdgeFilterFlag;      /**< First jump filter flag. */

    MCS_GetHallValue getHallValue;  /**< Pointer to the function for obtaining the value of the hall sensor. */
} HALL_Handle;

void HALL_Init(HALL_Handle *hall, float phaseShift, float ts);
unsigned int HALL_SectorCalc(unsigned int hallValue);
void HALL_InformationUpdate(HALL_Handle *hall);
void HALL_AngSpdCalcExec(HALL_Handle *hall);
void HALL_Clear(HALL_Handle *hall);

#endif