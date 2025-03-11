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
  * @file    status.h
  * @author  MCU Driver Team
  * @brief   status header
  */

#ifndef STATUS_H
#define STATUS_H

typedef enum {
    TARGET_DISCONNECT,
    TARGET_CONNECTED,
} TARGET_CONNECT_STATUS;

typedef enum {
    TARGET_CONNECT_SWD,
    TARGET_CONNECT_JTAG,
} TARGET_CONNECT_TYPE;

typedef struct {
    TARGET_CONNECT_STATUS status;
    TARGET_CONNECT_TYPE   type;
    unsigned int          rate;
} TargetConnectInfo;

typedef enum {
    IMAGE_STORE_SUCCESS,
    IMAGE_STORE_FAIL,
    IMAGE_STORE_NOT_SUPPORT_MULTI_COPIES,
    IMAGE_STORE_CRC_ERROR,
    IMAGE_STORE_FILE_ERROR,
    IMAGE_STORE_ALGO_ERROR,
    IMAGE_STORE_IDLE,
    IMAGE_STORE_STATUS_NUM,
} IMAGE_STORE_STATUS;

typedef enum {
    IMAGE_UPGRADE_START,
    IMAGE_UPGRADE_SUCCESS,
    IMAGE_UPGRADE_TARGET_CONNECT_FAIL,
    IMAGE_UPGRADE_GET_ALGO_FAIL,
    IMAGE_UPGRADE_WRITE_RUNNING,
    IMAGE_UPGRADE_VERIFY_RUNNING,
    IMAGE_UPGRADE_WRITE_FAIL,
    IMAGE_UPGRADE_GET_IMAGE_FAIL,
    IMAGE_UPGRADE_VERIFY_FAIL,
    IMAGE_UPGRADE_IMAGE_ERROR,
    IMAGE_UPGRADE_ALGO_ERROR,
    IMAGE_UPGRADE_ALGO_RUN_ERROR,
    IMAGE_UPGRADE_IDLE,
    IMAGE_UPGREADE_MAX_LOAD_ERROR,
    IMAGE_UPGRADE_STATUS_NUM,
} IMAGE_UPGRADE_STATUS;

typedef enum {
    ALGORITHM_STORE_SUCCESS,
    ALGORITHM_STORE_FAIL,
    ALGORITHM_STORE_NOT_SUPPORT_MULTI_COPIES,
    ALGORITHM_STORE_CRC_ERROR,
    ALGORITHM_STORE_IDLE,
    ALGORITHM_STORE_STATUS_NUM,
} ALGORITHM_STORE_STATUS;

typedef enum {
    FIRMWARE_UPGRADE_RUNNING,
    FIMRWARE_UPGRADE_SUCCESS,
    FIMRWARE_UPGRADE_CRC_ERROR,
    FIRMWARE_UPGRADE_IDLE,
    FIRMWARE_UPGRADE_STATUS_NUM,
} FIRMWARE_UPGRADE_STATUS;

typedef enum {
    IMAGE_DELETE_RUNNING,
    IMAGE_DELETE_SUCCESS,
    IMAGE_DELETE_FAIL,
    IMAGE_DELETE_IDLE,
    IMAGE_DELETE_STATUS_NUM,
} IMAGE_DELETE_STATUS;

typedef enum {
    ALGORITHM_DELETE_RUNNING,
    ALGORITHM_DELETE_SUCCESS,
    ALGORITHM_DELETE_FAIL,
    ALGORITHM_DELETE_IDLE,
    ALGORITHM_DELETE_STATUS_NUM,
} ALGORITHM_DELETE_STATUS;

typedef enum {
    DAP_STATUS_NOTWORKING,
    DAP_STATUS_DEBUGING,
    DAP_STATUS_SAMPLING,
    DAP_STATUS_SAMPLEOFFEXCEPTION,
    DAP_STATUS_GDBEXCEPTIONEXIT,
    DAP_STATUS_IDLE,
    DAP_STATUS_NUM,
} DAP_STATUS;

typedef struct {
    DAP_STATUS  status;
    unsigned long long totalSampleCount;
    unsigned long long totalOverflowCount;
    unsigned long long totalErrorCount;
    unsigned int       errorRate;
} DapDebugInfo;

typedef enum {
    STATUS_STATUS_NO_CHANGE                 = 0x00000000,
    STATUS_CHANGE_MASK_CONNECT_STATUS       = 0x00000001,
    STATUS_CHANGE_MASK_IMAGE_STORE_STATUS   = 0x00000002,
    STATUS_CHANGE_MASK_IMAGE_UPGRADE_STATUS = 0x00000004,
    STATUS_CHANGE_MASK_ALGO_STORE_STATUS    = 0x00000008,
    STATUS_CHANGE_MASK_FW_UPGRADE_STATUS    = 0x00000010,
    STATUS_CHANGE_MASK_IMAGE_DELETE_STATUS  = 0x00000020,
    STATUS_CHANGE_MASK_ALGO_DELETE_STATUS   = 0x00000040,
    STATUS_CHANGE_MASK_DAP_SAMPLING_STATUS  = 0x00000080,
} STATUS_CHANGE_MASK;

typedef void (*StatusChangeCallback)(void);
typedef struct {
    TargetConnectInfo       connectInfo;
    IMAGE_STORE_STATUS      imageStoreStatus;
    IMAGE_UPGRADE_STATUS    imageUpgradeStatus;
    ALGORITHM_STORE_STATUS  algoStoreStatus;
    FIRMWARE_UPGRADE_STATUS fwUpgradeStatus;
    IMAGE_DELETE_STATUS     imageDeleteStatus;
    ALGORITHM_DELETE_STATUS algoDeleteStatus;
    DapDebugInfo            dapDbgInfo;
    unsigned int            statusChangeMask;
    unsigned int            upgradeRate;
    StatusChangeCallback    cbFunc;
} DapLinkStatus;

void DapLinkStatusGet(DapLinkStatus *status);
void DapLinkStatusSet(DapLinkStatus *status);
void RegisterDaplinkStatusCallBackFunc(StatusChangeCallback func);
void UnRegisterDaplinkStatusCallBackFunc(void);

#endif