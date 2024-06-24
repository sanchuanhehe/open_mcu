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
  * @file      mcs_volt_det.h
  * @author    MCU Algorithm Team
  * @brief     This file contains dc-link voltage protection api declaration.
  */

 /* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS__VOLT_DET_H
#define McuMagicTag_MCS__VOLT_DET_H

#include "typedefs.h"

typedef struct {
    float overVoltThr;         /**< Over voltage threshold. */
    float timeThr;             /**< Detection time threshold. */

    unsigned int detCnt;       /**< Detection count. */
    unsigned int detCntLimit;  /**< Detection count threshold. */
    float ts;                  /**< Control period (s). */
} OVD_Handle;

typedef struct {
    float lowerVoltThr;        /**< Lower voltage threshold. */
    float timeThr;             /**< Detection time threshold. */

    unsigned int detCnt;       /**< Detection count. */
    unsigned int detCntLimit;  /**< Detection count threshold. */
    float ts;                  /**< Control period (s). */
} LVD_Handle;

void OVD_Init(OVD_Handle *ovd, float overVoltThr, float timeThr, float ts);
bool OVD_Exec(OVD_Handle *ovd, float voltAmp);
void OVD_Clear(OVD_Handle *ovd);

void LVD_Init(LVD_Handle *lvd, float lowerVoltThr, float timeThr, float ts);
bool LVD_Exec(LVD_Handle *lvd, float voltAmp);
void LVD_Clear(LVD_Handle *lvd);
#endif