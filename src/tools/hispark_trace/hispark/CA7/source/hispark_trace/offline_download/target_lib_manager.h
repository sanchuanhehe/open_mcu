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
  * @file    target_lib_manager.h
  * @author  MCU Driver Team
  * @brief   target flash library mamager driver.
  */
#ifndef TARGET_LIB_MANAGER_H
#define TARGET_LIB_MANAGER_H
#include <stdint.h>
#include <stdbool.h>
#include "daplink_addr.h"

#define TARGET_LIB_OK                        0
#define TARGET_LIB_ERROR                     1

#define TARGET_LIB_FILE_STATUS_BAD           0
#define TARGET_LIB_FILE_STATUS_NORMAL        1
#define TARGET_LIB_FILE_STATUS_SECURITY      2

#define TARGET_LIB_SEC_REGION                0x000A0000
#define TARGET_LIB_APP_REGION                0x00010000
#define TARGET_LIB_DATA_REGION               0x00020000

#define TARGET_LIB_FLASH_BASE_ADDR           0x10048000
#define TARGET_LIB_FLASH_BLOCK_SIZE          4096
#define TARGET_LIB_ALGO_OFFSET               0x300000
#define TARGET_LIB_IMAGE_OFFSET              0x400000
#define TARGET_LIB_FACTORY_IMAGE_OFFSET      0x900000

#define TARGET_LIB_IMAGE_IN_NAME_MAX_LEN     32
#define TARGET_LIB_IMAGE_NAME_MAX_LEN        64
#define TARGET_LIB_IMAGE_MAX_NUM             10
#define TARGET_LIB_ALGO_MAX_NUM              10
#define TARGET_LIB_FACTORY_ALGO_MAX_NUM      TARGET_LIB_ALGO_MAX_NUM
#define TARGET_LIB_FACTORY_IMAGE_MAX_NUM     10
#define TARGET_LIB_LOAD_CNT_INVALIA          0xFAFAFAFA

#define TARGET_LIB_IMAGE_START              (TARGET_LIB_FLASH_BASE_ADDR + TARGET_LIB_IMAGE_OFFSET)
#define TARGET_LIB_ONE_IMAGE_SIZE           (125 * TARGET_LIB_FLASH_BLOCK_SIZE) /* 125*4096 = 500k */

#define TARGET_LIB_ALGO_START               (TARGET_LIB_FLASH_BASE_ADDR + TARGET_LIB_ALGO_OFFSET)
#define TARGET_LIB_ONE_ALGO_SIZE            (2 * TARGET_LIB_FLASH_BLOCK_SIZE) /* 2*4096 = 8K */

#define TARGET_LIB_FACTORY_IMAGE_START      (TARGET_LIB_FLASH_BASE_ADDR + TARGET_LIB_FACTORY_IMAGE_OFFSET)
#define TARGET_LIB_ONE_FACTORY_IMAGE_SIZE   (125 * TARGET_LIB_FLASH_BLOCK_SIZE) /* 125*4096 = 500k */

typedef enum {
    TARGET_LIB_TYPE_ALGO,
    TARGET_LIB_TYPE_IMAGE
} TargetLibType;

typedef enum {
    TARGET_LIB_DISPLAY_LOADING,
    TARGET_LIB_DISPLAY_VERIFI_FAIL,
    TARGET_LIB_DISPLAY_LOAD_FAIL,
    TARGET_LIB_DISPLAY_LOAD_OK,
    TARGET_LIB_DISPLAY_DELETE_OK,
    TARGET_LIB_DISPLAY_DELETE_FAIL,
}TargetLibDisplayState;

typedef struct {
    uint32_t regioninfo;
    uint32_t version;
    uint32_t vendorID;
    uint32_t modelID;
    uint32_t flashID;
    uint32_t fileSize;
    uint32_t fileCrc32;
    uint32_t oldNamelen;
    uint32_t targetAddr;
    uint32_t maxLoads;
} TargetFlashInfo;

typedef struct {
    uint32_t             nameLen;
    char                 name[TARGET_LIB_IMAGE_NAME_MAX_LEN];
    TargetFlashInfo      info;
    uint32_t             fileStatus;
    uint32_t             startAddr;
} TargetAlgoLibInfo;

typedef struct {
    uint32_t             nameLen;
    char                 name[TARGET_LIB_IMAGE_NAME_MAX_LEN];
    TargetFlashInfo      info;
    uint32_t             fileStatus;
    uint32_t             startAddr;
} TargetImageLibInfo;

typedef struct {
    int32_t           count;
    TargetAlgoLibInfo  libInfo[TARGET_LIB_ALGO_MAX_NUM];
} TargetAlgoHandle;

typedef struct {
    int32_t            count;
    TargetImageLibInfo  libInfo[TARGET_LIB_IMAGE_MAX_NUM];
} TargetImageHandle;

typedef struct {
    uint32_t index;
    uint32_t nameLen;
    char name[TARGET_LIB_IMAGE_NAME_MAX_LEN];
} TargetEraseInfo;

typedef struct {
    int32_t count;
    TargetEraseInfo eraseList[TARGET_LIB_ALGO_MAX_NUM];
}TargetEraseList;

typedef struct {
    uint32_t list[TARGET_LIB_IMAGE_MAX_NUM];
} TargetImageLoadCntList;

int32_t TargetLibInit(void);
int32_t TargetLibUninit(void);

int32_t TargetLibAlgoListGet(TargetAlgoHandle *handle);
int32_t TargetLibAlgoAdd(TargetFlashInfo info);
int32_t TargetLibAlgoSelect(int32_t index);
int32_t TargetLibAlgoDelete(int32_t index);
int32_t TargetLibAlgoDeleteAll(void);
int32_t TargetLibCurrentAlgoGet(TargetAlgoLibInfo *info);
int32_t TargetLibAlgoIndexGetFormStartAddr(uint32_t startAddr);
int32_t TargetLibAlgoIndexGetFormFlashInfo(TargetFlashInfo info);

int32_t TargetLibImageListGet(TargetImageHandle *handle);
int32_t TargetLibImageAdd(TargetFlashInfo info, uint8_t *imageName, uint32_t nameLen);
int32_t TargetLibImageSelect(int32_t index);
int32_t TargetLibImageDelete(uint32_t index);
int32_t TargetLibImageDeleteAll(void);
int32_t TargetLibCurrentImageGet(TargetImageLibInfo *info, int32_t *index);
int32_t TargetLibImageIndexGetFormStartAddr(uint32_t startAddr);

int32_t TargetLibStartAddrSet(TargetLibType type);
uint32_t TargetLibStartAddrGet(TargetLibType type);

int32_t TargetLibFileStatusChange(TargetLibType type, int32_t index, uint32_t status);

TargetLibDisplayState TargetLibDisplayStateGet(void);
int32_t TargetLibDisplayStateSet(TargetLibDisplayState state);

int32_t TargetLibEraseListGet(TargetEraseList *list, uint32_t listLen);

uint32_t TargetImageLoadCntGet(int32_t index);
int32_t TargetImageLoadCntAdd(int32_t index);

#endif