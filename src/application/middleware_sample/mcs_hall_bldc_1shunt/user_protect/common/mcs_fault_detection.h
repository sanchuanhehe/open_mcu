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
  * @file      mcs_fault_detection.h
  * @author    MCU Algorithm Team
  * @brief     This file contains fault dection struct and api declaration.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_FAULT_DETECTION_H
#define McuMagicTag_MCS_FAULT_DETECTION_H

#include "mcs_openphs_det.h"
#include "mcs_stall_det.h"
#include "mcs_unbalance_det.h"
#include "mcs_curr_det.h"
#include "mcs_temp_det.h"
#include "mcs_volt_det.h"
#include "mcs_stall_det.h"
#include "mcs_user_config.h"


typedef union {
    unsigned short all;
    struct {
        unsigned short overCurrErr      : 1; /**< Indicates that phase current(s) is over protected value. */
        unsigned short overVoltErr      : 1; /**< Indicates that dc-link voltage is over protected value. */
        unsigned short lowerVoltErr     : 1; /**< Indicates that dc-link voltage is lower than protected value */
        unsigned short overTempErr      : 1; /**< Indicates that temperature is over protected value. */
        unsigned short motorStallErr    : 1; /**< Indicates that rotor is stalling. */
        unsigned short unbalanceErr     : 1; /**< Indicates that three phase currents is out-of-balance. */
        unsigned short phsOpenErr       : 1; /**< Indicates that phase winding(s) is open. */
        unsigned short phsOpenU         : 1; /**< Indicates that u phase fails when phsOpenErr occurs. */
        unsigned short phsOpenV         : 1; /**< Indicates that v phase fails when phsOpenErr occurs. */
        unsigned short phsOpenW         : 1; /**< Indicates that w phase fails when phsOpenErr occurs. */
        unsigned short multiPhs         : 1; /**< Indicates that multi-phases fail when phsOpenErr occurs.*/
    } Bit;
} FAULT_Status;


typedef struct {
    FAULT_Status faultStatus;
    OCD_Handle ocd;
    OVD_Handle ovd;
    LVD_Handle lvd;
    OTD_Handle otd;
    OPD_Handle opd;
    STD_Handle std;
    UNBAL_Handle unbal;
} FAULT_DET_Handle;

void FaultDetect_Init(FAULT_DET_Handle *faultDet);

bool FaultDetect_Exec(FAULT_DET_Handle *faultDet, float currAmp, float tempAmp, float voltAmp, float spdFbk);

void FaultDetect_Clear(FAULT_DET_Handle *faultDet);


#endif