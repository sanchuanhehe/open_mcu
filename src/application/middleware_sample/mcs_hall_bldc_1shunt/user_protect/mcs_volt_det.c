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
  * @file      mcs_volt_det.c
  * @author    MCU Algorithm Team
  * @brief     This file contains dc-link voltage protection api declaration.
  */


#include "mcs_volt_det.h"
#include "mcs_assert.h"


/**
  * @brief Initilization over voltage detection function.
  * @param ovd Over temperature dectection handle.
  * @param overVoltThr Over voltage value threshold.
  * @param timeThr Over current time threshold (s).
  * @param ts Ctrl period (s).
  * @retval None.
  */
void OVD_Init(OVD_Handle *ovd, float overVoltThr, float timeThr, float ts)
{
    MCS_ASSERT_PARAM(ovd != NULL);
    MCS_ASSERT_PARAM(overVoltThr > 0.0f);
    MCS_ASSERT_PARAM(timeThr > 0.0f);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Time threshold and current threshold. */
    ovd->overVoltThr = overVoltThr;
    ovd->timeThr = timeThr;
    
    ovd->ts = ts;
    /* Detection time limit calculation. */
    ovd->detCntLimit = (unsigned int)(ovd->timeThr / ovd->ts);
}


/**
  * @brief Over voltage detection.
  * @param ovd Over voltage dectection handle.
  * @param voltAmp Voltage vlaue.
  * @retval Whether the motor is over current.
  */
bool OVD_Exec(OVD_Handle *ovd, float voltAmp)
{
    MCS_ASSERT_PARAM(ovd != NULL);
    MCS_ASSERT_PARAM(voltAmp > 0.0f);
    /* Voltage threshold judgment. */
    if (voltAmp < ovd->overVoltThr) {
        ovd->detCnt = 0;
        return false;
    }
    /* Over voltage detection time judgment. */
    if (ovd->detCnt < ovd->detCntLimit) {
        ovd->detCnt++;
        return false;
    }

    return true;
}


/**
  * @brief Clear history value.
  * @param ovd Over temperature dectection handle.
  * @retval None.
  */
void OVD_Clear(OVD_Handle *ovd)
{
    MCS_ASSERT_PARAM(ovd != NULL);
    ovd->detCnt = 0;
}


/**
  * @brief Initilization lower voltage detection function.
  * @param lvd Over temperature dectection handle.
  * @param lowerVoltThr Over voltage value threshold.
  * @param timeThr Over current time threshold (s).
  * @param ts Ctrl period (s).
  * @retval None.
  */
void LVD_Init(LVD_Handle *lvd, float lowerVoltThr, float timeThr, float ts)
{
    MCS_ASSERT_PARAM(lvd != NULL);
    MCS_ASSERT_PARAM(lowerVoltThr > 0.0f);
    MCS_ASSERT_PARAM(timeThr > 0.0f);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Time threshold and current threshold. */
    lvd->lowerVoltThr = lowerVoltThr;
    lvd->timeThr = timeThr;
    
    lvd->ts = ts;
    /* Detection time limit calculation. */
    lvd->detCntLimit = (unsigned int)(lvd->timeThr / lvd->ts);
}

/**
  * @brief Over temperature detection.
  * @param lvd Over temperature dectection handle.
  * @param voltAmp Voltage vlaue.
  * @retval Whether the motor is over current.
  */
bool LVD_Exec(LVD_Handle *lvd, float voltAmp)
{
    MCS_ASSERT_PARAM(lvd != NULL);
    MCS_ASSERT_PARAM(voltAmp > 0.0f);
    /* Voltage threshold judgment. */
    if (voltAmp > lvd->lowerVoltThr) {
        lvd->detCnt = 0;
        return false;
    }
    /* Lower voltage detection time judgment. */
    if (lvd->detCnt < lvd->detCntLimit) {
        lvd->detCnt++;
        return false;
    }
    return true;
}

/**
  * @brief Clear history value.
  * @param lvd Over temperature dectection handle.
  * @retval None.
  */
void LVD_Clear(LVD_Handle *lvd)
{
    MCS_ASSERT_PARAM(lvd != NULL);
    lvd->detCnt = 0;
}