/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
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
  * @file    DAP_vendor_ex.h
  * @author  MCU Driver Team
  * @brief   Process DAP Vendor Command from the Extended Command range.
  */
#ifndef DAP_VENDOR_EX_H
#define DAP_VENDOR_EX_H
#include "DAP.h"

#define ID_DAP_VENDOR_OFFLINE_HEAD      0xB0U
#define ID_DAP_WENDOR_OFFLINE_DATA      0xB1U
#define ID_DAP_WENDOR_OFFLINE_STOP      0xB2U

typedef enum {
    ID_DAP_VENDOR_EX32_I2C_READ = ID_DAP_VendorExFirst,
    ID_DAP_VENDOR_EX33_I2C_WRITE,
    ID_DAP_VENDOR_EX34_GPIO,
    ID_DAP_VENDOR_EX35_DUT_PWR_CTRL,
    ID_DAP_VENDOR_EX36_VERSION_DETAILS,
    ID_DAP_VENDOR_EX37_HOLD_IN_BL,
    ID_DAP_VENDOR_EX38_RESET_DAPLINK,
    ID_DAP_VENDOR_EX39_READ_UDC_ADAPTER_TYPE_ADC,
    ID_DAP_VENDOR_EX40_VAR_MONITOR = ID_DAP_VENDOR_VAR_MONITOR,
    ID_DAP_VENDOR_EX41_READ_VAR = ID_DAP_VENDOR_READ_VAR,
    ID_DAP_VENDOR_EX42_WRITE_VAR = ID_DAP_VENDOR_WRITE_VAR,
    ID_DAP_VENDOR_EX43_STOP_VARMONITOR = ID_DAP_VENDOR_STOP_VARMONITOR,
    ID_DAP_VENDOR_EX44_PAUSE_VARMONITOR = ID_DAP_VENDOR_PAUSE_VARMONITOR,

    ID_DAP_VENDOR_EX50_OFFLINE_HEAD = ID_DAP_VENDOR_OFFLINE_HEAD,
    ID_DAP_WENDOR_EX51_OFFLINE_DATA = ID_DAP_WENDOR_OFFLINE_DATA,
    ID_DAP_WENDOR_EX52_OFFLINE_STOP = ID_DAP_WENDOR_OFFLINE_STOP,
    // Add new commands before the last command
    ID_DAP_VENDOR_EX126_LAST = ID_DAP_VendorExLast,
} DapVendorExCmdE;

uint32_t DAP_ProcessVendorCommandEx(const uint8_t *request, uint8_t *response);
#endif /* #ifndef DAP_VENDOR_EX_H */
