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
  * @file     mcs_carrier.h
  * @author   MCU Algorithm Team
  * @brief    This file provides functions declaration for carrier interrupt processing function.
  */
#ifndef McuMagicTag_MCS_CARRIER_H
#define McuMagicTag_MCS_CARRIER_H

#include "mcs_status.h"
#include "mcs_six_step.h"
#include "mcs_ramp_mgmt.h"
#include "mcs_spd_ctrl.h"
#include "mcs_fsm.h"

/**
  * @brief Three-phase static coordinate frame variable.
  */
typedef struct {
    unsigned int u; /**< Component u of the three-phase static coordinate frame variable. */
    unsigned int v; /**< Component v of the three-phase static coordinate frame variable. */
    unsigned int w; /**< Component w of the three-phase static coordinate frame variable. */
} UVWBemf;

typedef void (*ReadBemf)(UVWBemf *bemfUVW);

/**
  * @brief The definition of the systematic global variables
  */
typedef struct {
    unsigned char changePhaseFlag;          /**< Flag indicating that delay change phase is required */
    unsigned char firstEventFilterFlag;     /**< First zero-crossing event filtering flag. */
    unsigned int  dragChangePhaseTime;      /**< Interval for forced drag acceleration */
    unsigned int  accTimeCnt;               /**< Acceleration time count */

    unsigned int  bemfFilterCnt;            /**< Number of sample filtering times for zero-crossing detection. */

    unsigned int  lastZeroPoint;            /**< Time of the last zero point */
    unsigned int  waitTime;                 /**< Time to wait for change phase */
    unsigned int  stepTime[STEP_MAX_NUM];   /**< Record the time spent on each step */
    unsigned int  stepTimeNum;              /**< Number of data records in the steptime array. */
    unsigned int  stepTimeFilterEnable;
} SysVariable;

typedef struct {
    volatile float spdCmdHz;                         /**< Set target change phase frequency */
    float spdRefHz;                         /**< Command values after speed ramp management */
    float spdCurrHz;                        /**< Actual change phase frequency of feedback */
    float spdEstHz;                         /**< Actual change phase frequency of feedback */
    float pwmDuty;                          /**< APT duty cycle  */
    unsigned int zeroPoint;                 /**< Adc value of zero point */
    
    unsigned short aptMaxcntCmp;            /**< Apt Maximum Comparison Count */

    unsigned short sysTickCnt;              /**< System Timer Tick Count */
    unsigned short capChargeTickNum;        /**< Bootstrap Capacitor Charge Tick Count */
    unsigned short msTickNum;               /**< Number of ticks corresponding to 1 ms */
    volatile unsigned int msTickCnt;        /**< Millisecond-level counter, which can be used in 1-ms and 5-ms tasks. */

    RMG_Handle spdRmg;                       /**< Ramp management struct for the speed controller input reference */
    PID_Handle spdPi;                        /**< PI controller struct in the speed controller. */

    UVWBemf bemf;                           /**< Three-phase back electromotive force */
    SixStepHandle stepCtrl;                 /**< Control structure of six-step square wave */

    SysVariable sysVar;                     /**< System Variables */
    SysStatusReg statusReg;                 /**< System Status */
    FsmState stateMachine;                  /**< BLDC Motor Control State Machine */

    ReadBemf readBemfUVW;                   /**< Function interface for obtaining the three-phase electromotive force */
} MtrCtrlHandle;

void MCS_CarrierProcess(MtrCtrlHandle *mtrCtrl);

void MCS_SetCtrAptDuty(MtrCtrlHandle *mtrCtrl, unsigned int duty);

#endif