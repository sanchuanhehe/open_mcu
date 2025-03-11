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
  * @file     mcs_carrier.h
  * @author   MCU Algorithm Team
  * @brief    This file provides functions declaration for carrier interrupt processing function.
  */
#ifndef McuMagicTag_MCS_CARRIER_H
#define McuMagicTag_MCS_CARRIER_H

#include "mcs_sys_status.h"
#include "mcs_mtr_param.h"
#include "mcs_ramp_mgmt.h"
#include "mcs_spd_ctrl.h"
#include "mcs_pll.h"
#include "mcs_startup.h"
#include "mcs_filter.h"
#include "apt_ip.h"
#include "apt.h"
#include "mcs_adcCalibr.h"
#include "mcs_fsm.h"
#include "mcs_sensor_hall.h"
#include "fault_det.h"

typedef struct {
    unsigned int iuAdcBias; /* iu adc temperature calibration */
    unsigned int ivAdcBias; /* iv adc temperature calibration */
    unsigned int iwAdcBias; /* iw adc temperature calibration */
    unsigned int iBusAdcBias; /* ibus adc temperature calibration */
} IBIAS_Handle; /* Current Bias */

/**
  * @brief Sampling mode.
  */
typedef enum {
    DUAL_RESISTORS = 0,
    SINGLE_RESISTOR = 1
} SampleMode;

/**
  * @brief Motor control data structure
  */
typedef struct {
    float spdCmd;              /**< External input speed command value */
    float spdRef;              /**< Command value after speed ramp management */
    float spdFbk;              /**< Feedback speed */
    float currCtrlPeriod;        /**< current loop control period */
    unsigned short aptMaxCntCmp; /**< Apt Maximum Comparison Count */
    
    UvwAxis iuvw;
    float idc;              /**< Bus current. */
    float udc;              /**< Bus voltage. */
    float powerBoardTemp;   /**< Power boart surface temperature */

    float pwmDuty;          /**< pwm duty. */

    unsigned short sysTickCnt;       /**< System Timer Tick Count */
    unsigned short capChargeTickNum; /**< Bootstrap Capacitor Charge Tick Count */
    volatile unsigned int msTickCnt; /**< Millisecond-level counter, which can be used in 1-ms and 5-ms tasks. */
    unsigned short msTickNum;        /**< Number of ticks corresponding to 1 ms */
    
    SysStatusReg statusReg;         /**< System status */
    volatile FSM_State stateMachine; /**< Motor Control State Machine */
    
    MOTOR_Param mtrParam;        /**< Motor parameters */
    RMG_Handle spdRmg;           /**< Ramp management struct for the speed controller input reference */
    PID_Handle spdPi;            /**< Speed PI controller. */
    
    HALL_Handle *hall;
    ADC_CALIBR_Handle adcCalibr;                 /**< adc calibration */
    IBIAS_Handle iuvwAdcBias;                    /**< Phase current ADC calibration handle. */

    FAULT_Status faultStatus;
    FP_OCD_Handle ocd;
    FP_UOVD_Handle uovd;
    FP_OTD_Handle otd;
    FP_STD_Handle std;
    FP_OPD_Handle opd;
} MTRCTRL_Handle;

void MCS_CarrierProcess(MTRCTRL_Handle *mtrCtrl);


#endif