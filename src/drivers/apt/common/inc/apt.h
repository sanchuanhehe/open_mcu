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
  * @file    apt.h
  * @author  MCU Driver Team
  * @brief   APT module driver.
  * @details This file provides functions declaration of the APT module.
  *           + APT handle structure definition.
  *           + Initialization and de-initialization functions.
  *           + APT Service Functions.
  */

#ifndef McuMagicTag_APT_H
#define McuMagicTag_APT_H

#include "apt_ip.h"

/**
  * @brief The definition of the APT handle structure.
  */
typedef struct _APT_Handle {
    APT_RegStruct          *baseAddress;      /**< Register base address. */
    APT_PWMWaveForm         waveform;         /**< PWM waveform configuration handle. */
    APT_ADCTrigger          adcTrg;           /**< ADC trigger configuration handle. */
    APT_TimerInterrupt      tmrInterrupt;     /**< Timer interrupt configuration handle. */
    APT_UserCallBack        userCallBack;     /**< Interrupt callback function when APT event happens. */
    APT_ExtendHandle            handleEx;         /**< extra handle */
} APT_Handle;

/**
  * @defgroup APT_API_Declaration APT HAL API
  * @{
  */


/**
  * @defgroup APT_API_Declaration
  * @brief APT HAL API.
  * @{
  */

BASE_StatusType HAL_APT_PWMInit(APT_Handle *aptHandle);
BASE_StatusType HAL_APT_PWMDeInit(APT_Handle *aptHandle);
BASE_StatusType HAL_APT_ProtectInit(APT_Handle *aptHandle, APT_OutCtrlProtect *protect);
BASE_StatusType HAL_APT_ProtectDeInit(APT_Handle *aptHandle, APT_OutCtrlProtect *protect);
BASE_StatusType HAL_APT_ProtectInitEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect);
BASE_StatusType HAL_APT_ProtectDeInitEx(APT_Handle *aptHandle, APT_OutCtrlProtectEx *protect);
void HAL_APT_ForcePWMOutputLow(APT_Handle *aptHandle);
BASE_StatusType HAL_APT_MasterSyncInit(APT_Handle *aptHandle, unsigned short syncOutSrc);
BASE_StatusType HAL_APT_SlaveSyncInit(APT_Handle *aptHandle, APT_SlaveSyncIn *slaveSyncIn);
void HAL_APT_StartModule(unsigned int aptRunMask);
void HAL_APT_StopModule(unsigned int aptRunMask);
BASE_StatusType HAL_APT_SetPWMDuty(APT_Handle *aptHandle, unsigned short cntCmpLeftEdge, \
                                   unsigned short cntCmpRightEdge);
BASE_StatusType HAL_APT_SetPWMDutyByNumber(APT_Handle *aptHandle, unsigned int duty);
BASE_StatusType HAL_APT_SetADCTriggerTime(APT_Handle *aptHandle, unsigned short cntCmpSOCA, unsigned short cntCmpSOCB);
void HAL_APT_EventIrqHandler(void *handle);
void HAL_APT_TimerIrqHandler(void *handle);
void HAL_APT_RegisterCallBack(APT_Handle *aptHandle, APT_InterruputType typeID, APT_CallbackType pCallback);
BASE_StatusType HAL_APT_EMInit(APT_Handle *aptHandle, APT_EventManage *eventManage);
unsigned short HAL_APT_EMGetCapValue(APT_Handle *aptHandle);
void HAL_APT_EMSetWdOffsetAndWidth(APT_Handle *aptHandle, unsigned short offset, unsigned short width);
void HAL_APT_EMSetValleySwitchSoftDelay(APT_Handle *aptHandle, unsigned short calibrate);
BASE_StatusType HAL_APT_ChangeOutputType(APT_Handle *aptHandle,
                                         APT_PWMChannel channel,
                                         APT_PWMChannelOutType aptAction);
/**
  * @defgroup APT_API_Declaration
  * @brief Attribute configuration of each reference point.
  * @{
  */

BASE_StatusType APT_ConfigRefA(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
BASE_StatusType APT_ConfigRefB(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
BASE_StatusType APT_ConfigRefC(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
BASE_StatusType APT_ConfigRefD(APT_Handle *aptHandle, APT_RefDotParameters *refDotParameters);
/**
  * @}
  */

/**
  * @defgroup APT_API_Declaration
  * @brief Combination configuration of reference point attributes.
  * @{
  */

BASE_StatusType HAL_APT_ConfigRefDot(APT_Handle *aptHandle, APT_RefDotSelect refDotSelect,
                                     APT_RefDotParameters *refDotParameters);

BASE_StatusType HAL_APT_SetTimerPeriod(APT_Handle *aptHandle, unsigned short newPeriod, \
                                       APT_BufferLoadMode prdLoadMode, unsigned int prdLoadEvt);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_APT_H */