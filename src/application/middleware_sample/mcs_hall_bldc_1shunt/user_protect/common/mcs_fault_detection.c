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

#include "mcs_fault_detection.h"
#include "mcs_assert.h"

/**
  * @brief Motor fault detection initialization.
  * @param faultDet Fault detection handle.
  * @retval None.
  */
void FaultDetect_Init(FAULT_DET_Handle *faultDet)
{
    MCS_ASSERT_PARAM(faultDet != NULL);
    /* Clear the fault status. */
    faultDet->faultStatus.all = 0;
    /* Initialize the fault detection structure. */
    OCD_Init(&faultDet->ocd, OVER_CURR_AMP_THRESHOLD, OVER_CURR_TIME_THRESHOLD, CTRL_CURR_PERIOD);
    OTD_Init(&faultDet->otd, OVER_TEMP_AMP_THRESHOLD, OVER_TEMP_TIME_THRESHOLD, CTRL_CURR_PERIOD);
    OVD_Init(&faultDet->ovd, OVER_VOLT_AMP_THRESHOLD, OVER_VOLT_TIME_THRESHOLD, CTRL_CURR_PERIOD);
    LVD_Init(&faultDet->lvd, LOWER_VOLT_AMP_THRESHOLD, LOWER_VOLT_TIME_THRESHOLD, CTRL_CURR_PERIOD);
    STD_Init(&faultDet->std, STALL_CURR_THRESHOLD, STALL_SPEED_THRESHOLD, STALL_TIME_THRESHOLD, CTRL_CURR_PERIOD);
}

/**
  * @brief Motor fault detection.
  * @param faultDet Fault detection handle.
  * @param currAmp Motor current amplitude.
  * @param tempAmp Motor temperature amplitude.
  * @param voltAmp Motor voltage amplitude.
  * @retval Whether the motor happen faults.
  */
bool FaultDetect_Exec(FAULT_DET_Handle *faultDet, float currAmp, float tempAmp, float voltAmp, float spdFbk)
{
    MCS_ASSERT_PARAM(faultDet != NULL);
    /* Executing Fault Detection */
    faultDet->faultStatus.Bit.overCurrErr = OCD_Exec(&faultDet->ocd, currAmp);
    faultDet->faultStatus.Bit.overTempErr = OTD_Exec(&faultDet->otd, tempAmp);
    faultDet->faultStatus.Bit.overVoltErr = OVD_Exec(&faultDet->ovd, voltAmp);
    faultDet->faultStatus.Bit.lowerVoltErr = LVD_Exec(&faultDet->lvd, voltAmp);
    faultDet->faultStatus.Bit.motorStallErr = STD_Exec_ByCurrSpd(&faultDet->std, spdFbk, currAmp);

    /* Return to Fault Status */
    return faultDet->faultStatus.all;
}

/**
  * @brief Clear Motor fault detection history value.
  * @param faultDet Fault detection handle.
  * @retval None.
  */
void FaultDetect_Clear(FAULT_DET_Handle *faultDet)
{
    MCS_ASSERT_PARAM(faultDet != NULL);
    /* Clear the fault status. */
    faultDet->faultStatus.all = 0;
}