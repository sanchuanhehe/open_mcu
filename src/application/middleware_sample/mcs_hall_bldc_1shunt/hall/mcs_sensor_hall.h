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
#include "apt.h"
#include "mcs_pll.h"
#include "mcs_filter.h"

/* Typedef definitions ------------------------------------------------------------------------- */

#define SPEED_CALC_SECTOR_NUMS   3u

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

typedef enum {
    SECTOR1 = 0,
    SECTOR2,
    SECTOR3,
    SECTOR4,
    SECTOR5,
    SECTOR6,
    SECTOR_MAX_NUM
} HALL_SECTOR;

/**
  * @brief Hall sensor control data structure
  */
typedef struct {
    float ts;                      /**< Control period (s). */
    float timer;                   /**< Timer count value. */
    unsigned int sector;           /**< Current sector. */
    unsigned int sectorLast;       /**< Last sector. */
    unsigned int spdCalcCnt;
    float t1;                      /**< Last sector jump interval. */
    float t2;                      /**< The sector jump interval before t1. */
    float t3;                      /**< The sector jump interval before t2. */
    float tAvg;                    /**< Average transition interval. */
    int dir;                       /**< Rotation direction. */

    float angleComp;               /**< Angle correction parameters. */
    float angStart;                /**< The start electrical angle of the current sector.. */

    float spdEst;                  /**< Electrical angle. */
    float elecAngle;               /**< Electrical speed. */
    float sixStepAngle;

    short firstEdgeFilterFlag;     /**< First jump filter flag. */
    
    PLL_Handle pll;
    FOFLT_Handle spdLpf;
} HALL_Handle;

/*** Hall six step control ***/
void SIXSTEP_AptConfig(APT_RegStruct **aptUvw, HALL_SECTOR sector);

/*** Hall foc control ***/
void HALL_Init(HALL_Handle *hall, int dir, float pllBdw, float spdCutOffFre, float ts);

void HALL_Exec(HALL_Handle *hall, HALL_SECTOR sector);

void HALL_CapmEvtCallBack(HALL_Handle *hall, HALL_SECTOR sector);

void HALL_Clear(HALL_Handle *hall);

#endif
