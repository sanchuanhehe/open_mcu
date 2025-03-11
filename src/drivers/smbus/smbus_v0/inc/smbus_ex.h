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
  * @file      smbus_ex.h
  * @author    MCU Driver Team
  * @brief     SMBUS module driver
  * @details   The header file contains the following declaration:
  *             + Setting the Special Function Configuration.
  */

#ifndef McuMagicTag_SMBUS_EX_H
#define McuMagicTag_SMBUS_EX_H

/* Includes ------------------------------------------------------------------*/
#include "smbus.h"
/* Macro definitions ---------------------------------------------------------*/
/**
  * @addtogroup SMBUS_IP
  * @{
  */

/**
  * @defgroup SMBUS_EX_API_Declaration SMBUS HAL API EX
  * @{
  */
BASE_StatusType HAL_SMBUS_SetDataTransferSequenceEx(SMBUS_Handle *handle, SMBUS_DataTransferSequenceType sequence);
BASE_StatusType HAL_SMBUS_SetSclStretchModeEx(SMBUS_Handle *handle, SMBUS_ClockStretchType clkStretch);
BASE_StatusType HAL_SMBUS_SetSclLowTimeoutEx(SMBUS_Handle *handle, unsigned int sclLowTimeout);
BASE_StatusType HAL_SMBUS_SetBusFreeTimeEx(SMBUS_Handle *handle, unsigned int busFreeTime);
BASE_StatusType HAL_SMBUS_Set10BitSlaveEnableEx(SMBUS_Handle *handle);
BASE_StatusType HAL_SMBUS_SetDeviceIdAddressEnableEx(SMBUS_Handle *handle);
BASE_StatusType HAL_SMBUS_SetStartByteEnableEx(SMBUS_Handle *handle);
BASE_StatusType HAL_SMBUS_SetOwnAddressMaskEx(SMBUS_Handle *handle, unsigned int addrMask);
BASE_StatusType HAL_SMBUS_SetOwnXmbAddressMaskEx(SMBUS_Handle *handle, unsigned int addrMask);
/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_SMBUS_H */