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
  * @file    DAP_vendor_ex.c
  * @author  MCU Driver Team
  * @brief   Process DAP Vendor Command from the Extended Command range.
  */
#include <string.h>
#include "DAP.h"
#include "DAP_config.h"
#include "settings.h"
#include "cortex_m.h"
#include "var_monitor_process.h"
#include "dap_offline_load_process.h"
#include "DAP_vendor_ex.h"


/**
  * @brief Process DAP Vendor Command from the Extended Command range and prepare Response Data.
  * @param request pointer to request data.
  * @param response pointer to response data.
  * @retval number of bytes in response (lower 16 bits),number of bytes in request (upper 16 bits).
  */
uint32_t DAP_ProcessVendorCommandEx(const uint8_t *request, uint8_t *response)
{
    uint32_t num = (1U << 16) | 1U;     // count the Command ID byte

    *response++ = *request;             // copy Command ID

    switch (*request++) {               // first byte in request is Command ID
        case ID_DAP_VENDOR_EX40_VAR_MONITOR: {
            num += DAP_VarMonitor(request, response);
            break;
        }
        case ID_DAP_VENDOR_EX41_READ_VAR: {
            num += DAP_ReadVar(request, response);
            break;
        }
        case ID_DAP_VENDOR_EX42_WRITE_VAR: {
            num += DAP_WriteVar(request, response);
            break;
        }
        case ID_DAP_VENDOR_EX43_STOP_VARMONITOR: {
            num += DAP_StopVarMonitor(response);
            break;
        }
        case ID_DAP_VENDOR_EX44_PAUSE_VARMONITOR: {
            num += DAP_PauseVarMonitor(request, response);
            break;
        }
        case ID_DAP_VENDOR_EX50_OFFLINE_HEAD: {
            num += DAP_OfflineHeadProc(request, response);
            break;
        }
        case ID_DAP_WENDOR_EX51_OFFLINE_DATA: {
            num += DAP_OfflineDataProc(request, response);
            break;
        }
        case ID_DAP_WENDOR_EX52_OFFLINE_STOP: {
            num += DAP_OfflineStopProc(request, response);
            break;
        }
        default: break;
    }

    return (num);
}

