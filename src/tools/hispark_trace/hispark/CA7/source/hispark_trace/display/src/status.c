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
  * @file    status.c
  * @author  MCU Driver Team
  * @brief   status process
  */

#include <stdbool.h>
#include "display_common.h"
#include "swd_jtag_config.h"
#include "display.h"
#include "menu_tbl.h"
#include "status.h"

DapLinkStatus g_dapLinkStatus = {
    .connectInfo = {.status = TARGET_DISCONNECT, .type = TARGET_CONNECT_SWD, .rate = 0},
    .imageStoreStatus = IMAGE_STORE_IDLE,
    .imageUpgradeStatus = IMAGE_UPGRADE_IDLE,
    .algoStoreStatus = ALGORITHM_STORE_IDLE,
    .fwUpgradeStatus = FIMRWARE_UPGRADE_SUCCESS,
    .imageDeleteStatus = IMAGE_DELETE_IDLE,
    .algoDeleteStatus = ALGORITHM_DELETE_IDLE,
    .statusChangeMask = STATUS_CHANGE_MASK_CONNECT_STATUS,
    .dapDbgInfo = {.status = DAP_STATUS_NOTWORKING, .totalSampleCount = 0,
                   .totalOverflowCount = 0, .totalErrorCount = 0, .errorRate = 0},
};

/**
  * @brief Check Whether the Connect Information changes
  * @retval None
  */
static bool IsConnectInfoChanged(TargetConnectInfo *info)
{
    return (info->status != g_dapLinkStatus.connectInfo.status) ||
           (info->type != g_dapLinkStatus.connectInfo.type) ||
           (info->rate != g_dapLinkStatus.connectInfo.rate);
}

/**
  * @brief Check Whether the Debug Information changes
  * @retval None
  */
static bool IsDapDbgInfoChanged(DapDebugInfo *info)
{
    /* status or statistics changed */
    return (info->status != g_dapLinkStatus.dapDbgInfo.status) ||
           (info->totalSampleCount != g_dapLinkStatus.dapDbgInfo.totalSampleCount) ||
           (info->totalOverflowCount != g_dapLinkStatus.dapDbgInfo.totalOverflowCount) ||
           (info->totalErrorCount != g_dapLinkStatus.dapDbgInfo.totalErrorCount) ||
           (info->errorRate != g_dapLinkStatus.dapDbgInfo.errorRate);
}

/**
  * @brief Get Daplink status
  * @retval None
  */
void DapLinkStatusGet(DapLinkStatus *status)
{
    *status = g_dapLinkStatus;
}

/**
  * @brief Get Daplink status changed mask
  * @retval changed mask
  */
static unsigned int DaplinkGetImageAlgoStatusMask(DapLinkStatus *status)
{
    unsigned int mask;
    uint32_t rate = SwdJtagClockGet(); /* Get Connect rate */
    if (status->imageStoreStatus != g_dapLinkStatus.imageStoreStatus) {
        mask = STATUS_CHANGE_MASK_IMAGE_STORE_STATUS;
    } else if ((status->imageUpgradeStatus != g_dapLinkStatus.imageUpgradeStatus) ||
               (status->upgradeRate != 0 && status->upgradeRate != rate)) {
        mask = STATUS_CHANGE_MASK_IMAGE_UPGRADE_STATUS; /* image upgrade status changed */
        status->upgradeRate = rate;
    } else if (status->algoStoreStatus != g_dapLinkStatus.algoStoreStatus) {
        mask = STATUS_CHANGE_MASK_ALGO_STORE_STATUS;    /* algo upgrade status changed */
    } else if (status->fwUpgradeStatus != g_dapLinkStatus.fwUpgradeStatus) {
        mask = STATUS_CHANGE_MASK_FW_UPGRADE_STATUS;    /* fireware upgrade status changed */
    } else if (status->imageDeleteStatus != g_dapLinkStatus.imageDeleteStatus) {
        mask = STATUS_CHANGE_MASK_IMAGE_DELETE_STATUS;  /* delete image status changed */
    } else if (status->algoDeleteStatus != g_dapLinkStatus.algoDeleteStatus) {
        mask = STATUS_CHANGE_MASK_ALGO_DELETE_STATUS;   /* delete algo status changed */
    } else {
        mask = 0;
    }
    return mask;
}

/**
  * @brief Get Daplink status changed mask
  * @retval changed mask
  */
static unsigned int DaplinkGetStatusMask(DapLinkStatus *status)
{
    unsigned int mask = 0;

    if (IsConnectInfoChanged(&status->connectInfo)) {
        mask = STATUS_CHANGE_MASK_CONNECT_STATUS;  /* connect info changed */
    } else if (IsDapDbgInfoChanged(&status->dapDbgInfo)) {
        mask = STATUS_CHANGE_MASK_DAP_SAMPLING_STATUS;
    } else {
        mask = DaplinkGetImageAlgoStatusMask(status);
    }
    return mask;
}

/**
  * @brief Set Daplink status
  * @retval None
  */
void DapLinkStatusSet(DapLinkStatus *status)
{
    unsigned int mask;
    if (status == NULL) {
        return;
    }
    mask = DaplinkGetStatusMask(status); /* Get Status changed mask */
    if (mask != 0) {
        g_dapLinkStatus = *status; /* update status and changed mask */
        g_dapLinkStatus.statusChangeMask = mask;
        if ((mask == STATUS_CHANGE_MASK_IMAGE_STORE_STATUS) ||
            (mask == STATUS_CHANGE_MASK_ALGO_STORE_STATUS)) {
            if (IsDbgStatusFrame() == false) {
                return;
            }
        }
        /* call daplink hook process */
        if (g_dapLinkStatus.cbFunc != NULL) {
            g_dapLinkStatus.cbFunc();
        }
    }
}

/**
  * @brief Register Daplink hook process
  * @retval None
  */
void RegisterDaplinkStatusCallBackFunc(StatusChangeCallback func)
{
    g_dapLinkStatus.cbFunc = func;
}
