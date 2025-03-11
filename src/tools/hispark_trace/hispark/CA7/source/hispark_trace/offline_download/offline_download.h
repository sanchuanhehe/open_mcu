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
  * @file    offline_download.h
  * @author  MCU Driver Team
  * @brief   offline download driver.
  */
#ifndef OFFLINE_DOWNLOAD_H
#define OFFLINE_DOWNLOAD_H
#include <stdint.h>

typedef enum {
    OFFLINE_DOWNLOAD_DISPLAY_LOADLIG,
    OFFLINE_DOWNLOAD_DISPLAY_SELECT_ALGO_FAIL,
    OFFLINE_DOWNLOAD_DISPLAY_VERIFI_FAIL,
    OFFLINE_DOWNLOAD_DISPLAY_VERIFI_OK,
    OFFLINE_DOWNLOAD_DISPLAY_LOAD_FAIL,
    OFFLINE_DOWNLOAD_DISPLAY_LOAD_OK,
} OfflineDownLoadDisplayState;

/* error state */
#define OFFLINE_DOWNLOAD_OK                  0
#define OFFLINE_DOWNLOAD_ERROR               1
#define OFFLINE_DOWNLOAD_ERROR_OPEN          2
#define OFFLINE_DOWNLOAD_ERROR_SELECT_ALGO   3
#define OFFLINE_DOWNLOAD_VERIFI_ERROR        4
#define OFFLINE_DOWNLOAD_ERROR_LOADING       5
#define OFFLINE_DOWNLOAD_ERROR_CLOSE         6
#define OFFLINE_DOWNLOAD_ERROR_NOT_CONNECT   7
#define OFFLINE_DOWNLOAD_ERROR_ERASE_FAIL    8


int32_t OfflineDownLoadInit(void);
int32_t OfflineDownLoadDeinit(void);
int32_t OfflineDownLoadStart(void);
int32_t OfflineDownLoadStop(void);
int32_t OfflineEraseChip(int32_t index);
void OfflineDownLoadHandler(void);
OfflineDownLoadDisplayState OfflineDownLoadDisplayStateGet(void);
int32_t OfflineDownLoadDisplayStateSet(OfflineDownLoadDisplayState state);
bool IsOfflineDownLoad(void);

#endif /* #ifndef OFFLINE_DOWNLOAD_H */