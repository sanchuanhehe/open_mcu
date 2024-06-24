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
  * @file      mcs_curr_det.c
  * @author    MCU Algorithm Team
  * @brief     This file contains current protection data struct and api declaration.
  */


#include "mcs_curr_det.h"
#include "mcs_assert.h"
#include "mcs_math.h"


/**
  * @brief Initilization over current detection function.
  * @param ocd Over current detection handle.
  * @param currThr Over current value threshold.
  * @param timeThr Over current time threshold (s).
  * @param ts Ctrl period (s).
  * @retval None.
  */
void OCD_Init(OCD_Handle *ocd, float currThr, float timeThr, float ts)
{
    MCS_ASSERT_PARAM(ocd != NULL);
    MCS_ASSERT_PARAM(currThr > 0.0f);
    MCS_ASSERT_PARAM(timeThr > 0.0f);
    MCS_ASSERT_PARAM(ts > 0.0f);
    /* Time threshold and current threshold. */
    ocd->currThr = currThr;
    ocd->timeThr = timeThr;

    ocd->ts = ts;
    /* Detection time limit calculation. */
    ocd->detCntLimit = (unsigned int)(ocd->timeThr / ocd->ts);
}

/**
  * @brief Over current detection.
  * @param ocd Over current detection handle.
  * @param currAmp Current vlaue.
  * @retval Whether the motor is over current.
  */
bool OCD_Exec(OCD_Handle *ocd, float currAmp)
{
    MCS_ASSERT_PARAM(ocd != NULL);
    float currAbs = Abs(currAmp);
    /* Current threshold judgment. */
    if (currAbs < ocd->currThr) {
        ocd->detCnt = 0;
        return false;
    }
    /* Over current detection time judgment. */
    if (ocd->detCnt < ocd->detCntLimit) {
        ocd->detCnt++;
        return false;
    }
    return true;
}

/**
  * @brief Clear history value.
  * @param ocd Over current detection handle.
  * @retval None.
  */
void OCD_Clear(OCD_Handle *ocd)
{
    MCS_ASSERT_PARAM(ocd != NULL);
    ocd->detCnt = 0;
}