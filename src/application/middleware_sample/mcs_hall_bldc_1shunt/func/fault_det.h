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
  * @file      fault_det.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of fault detection.
  *            Include open phase, statll, over current, under voltage, interturn.
  */

#ifndef McuMagicTag_FAULT_DET_H
#define McuMagicTag_FAULT_DET_H

#include "mcs_typedef.h"
#include "mcs_assert.h"

/**
  * @brief System fault status definations.硬件电流/软件电流/欠压/过压/过温/缺相/堵转
  */
typedef union {
    unsigned short all;
    struct {
        unsigned short hardOverCurFault     : 1;    // indicates over-current fault
        unsigned short softOverCurFault     : 1;    // indicates over-current fault
        unsigned short stallFault           : 1;    // indicates rotor stall fault
        unsigned short overVoltFault        : 1;    // indicates over-voltage fault
        unsigned short underVoltFault       : 1;    // indicates under-voltage fault
        unsigned short mtrOverTempFault     : 1;    // indicates motor over-temperature fault
        unsigned short brdOverTempFault     : 1;    // indicates board over-temperature fault
        unsigned short openPhaseFault       : 1;    // indicates open-phase fault in two or three phases
        unsigned short openPhaseFault_U     : 1;    // indicates phase U open-phase fault
        unsigned short openPhaseFault_V     : 1;    // indicates phase V open-phase fault
        unsigned short openPhaseFault_W     : 1;    // indicates phase W open-phase fault
    } Bit;
} FAULT_Status;

void Fault_Clear(FAULT_Status *faultStatus);

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief OCD_Handle Struct.
  */
typedef struct {
    float ts;
    float overCurrThr;            // Software overcurrent value (A)

    float detWindow;
    float timer;
} FP_OCD_Handle;

void FP_OCD_Init(FP_OCD_Handle *ocd, float overCurrThr, float detWindow, float ts);
void FP_OCD_Exec(FP_OCD_Handle *ocd, FAULT_Status *faultStatus, UvwAxis iuvw);
void FP_OCD_Clear(FP_OCD_Handle *ocd);

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief UOVD_Handle Struct.
  */
typedef struct {
    float ts;
    float overProThr;             // over-voltage protection value
    float overRecThr;             // over-voltage recovery value
    float underProThr;            // under-voltage protection value
    float underRecThr;            // under-voltage recovery value

    unsigned int proTicks;        // ticks of under/over-voltage protection
    unsigned int recTicks;        // ticks of under/over-voltage recovery

    unsigned int overVolCnt;      // over voltage protection/recovery counter
    unsigned int underVolCnt;     // under voltage protection/recovery counter
    unsigned char recoveryFlag;   //
} FP_UOVD_Handle;

typedef struct {
    float overProThr;     // over-voltage protection value
    float overRecThr;     // over-voltage recovery value
    float underProThr;    // under-voltage protection value
    float underRecThr;    // under-voltage recovery value
    float detWindow;        // response time of under/over-voltage protection (s)
    float recWindow;        // response time of under/over-voltage recovery (s)
} FP_UOVD_Param;

void FP_UOVD_Init(FP_UOVD_Handle *uovdHandle, FP_UOVD_Param *uovdInit, float ts);
void FP_UOVD_Exec(FP_UOVD_Handle *uovdHandle, FAULT_Status *faultStatus, float busVolt);
void FP_UOVR_Exec(FP_UOVD_Handle *uovdHandle, FAULT_Status *faultStatus, float busVolt);
void FP_UOVD_Clear(FP_UOVD_Handle *uovdHandle);

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief UOVD_Handle Struct.
  */
typedef struct {
    float ts;

    float mtrTempThr;
    float brdTempThr;

    float mtrDetWindow;
    float brdDetWindow;

    float mtrTimer;
    float brdTimer;
} FP_OTD_Handle;

void FP_OTD_Init(FP_OTD_Handle *otd, float mtrTempThr, float brdTempThr,
                 float mtrDetWindow, float brdDetWindow, float ts);
void FP_OTD_Exec(FP_OTD_Handle *otd, FAULT_Status *faultStatus, float mtrTemp, float brdTemp);
void FP_OTD_Clear(FP_OTD_Handle *otd);

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief UOVD_Handle Struct.
  * 两种情景
  * （1）原地不动，timer累加，可用timer判据;
  * （2）原地震荡，timer变化，可用失速判据：速度在时速区间内，超过xx时间。
  */
typedef struct {
    float ts;
    float detWindow;
    float spdLowerLim;
    float spdUpperLim;

    float timer;
    char loseSpdFlag;
    char timeOutFlag;
} FP_STD_Handle;

void FP_STD_Init(FP_STD_Handle *std, float spdLowerLim, float spdUpperLim, float detWindow, float ts);
void FP_STD_Exec(FP_STD_Handle *std, FAULT_Status *faultStatus, float timer, float spdFbk);
void FP_STD_Clear(FP_STD_Handle *std);

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief UOVD_Handle Struct.
  */
typedef struct {
    float ts;
    float opdCurrThr;      // 缺相电流，理论为0
    float detWindow;

    float iuAbsIntegral;
    float ivAbsIntegral;
    float iwAbsIntegral;
    float timer;
} FP_OPD_Handle;

void FP_OPD_Init(FP_OPD_Handle *opd, float opdCurrThr, float detWindow, float ts);
void FP_OPD_Exec(FP_OPD_Handle *opd, FAULT_Status *faultStatus, UvwAxis iuvw);
void FP_OPD_Clear(FP_OPD_Handle *opd);

#endif