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
  * @file    factory_manager.h
  * @author  MCU Driver Team
  * @brief   factory manager driver.
  */
#ifndef FACTORY_MANAGER_H
#define FACTORY_MANAGER_H
#include "target_lib_manager.h"

#define FACTORY_CRC_CHECK_SPE_VALUE  0x2144DF1C

#define FACTORY_FLAG_IS_SET    0xFAFBF0AF
#define FACTORY_DEFAULT_COUNT  1

#define FACTORY_ALGO_NAME_LEN    48
#define FACTORY_IMAGE_NAME_LEN   64

#define FACTORY_STATUS_OK      0
#define FACTORY_STATUS_ERROR   (-1)
#define FACTORY_STATUS_CRC_ERROR   (-2)
#define FACTORY_STATUS_ALGO_NO_MATCH (-3)

#define FACTORY_CRC_CHECK_RES_SUCCESS       USER_CRC_CHECK_SUCCESS
#define FACTORY_CRC_CHECK_RES_FAIL          USER_CRC_CHECK_FAIL
#define FACTORY_CRC_CHECK_RES_NOT_CONNECT   USER_CRC_CHECK_NOT_CONNECT
#define FACTORY_CRC_CHECK_RES_ERROR         USER_CRC_CHECK_ERROR
#define FACTORY_CRC_CHECK_RES_READ_FAIL     USER_CRC_CHECK_READ_FAIL
#define FACTORY_CRC_CHECK_RES_BAD_FILE      USER_CRC_CHECK_BAD_FILE
#define FACTORY_CRC_CHECK_RES_ALGO_ERROR    USER_CRC_CHECK_ALGO_ERROR
#define FACTORY_CRC_CHECK_RES_NO_ALGO       USER_CRC_CHECK_NO_ALGO

typedef struct {
    uint32_t preCrc;
    uint32_t currentCrc;
    int32_t results;
    uint32_t checkLen;
    uint32_t checkStartAddr;
} FactoryCrcStatus;

typedef  TargetAlgoHandle FactoryAlgoHandle;

typedef struct {
    int32_t            count;
    TargetImageLibInfo  libInfo[TARGET_LIB_FACTORY_IMAGE_MAX_NUM];
} FactoryImageHandle;

typedef struct {
    uint32_t nameLen;
    char name[FACTORY_ALGO_NAME_LEN];
} FactoryAlgoInfo;

typedef struct {
    uint32_t count;
    FactoryAlgoInfo algoInfo[TARGET_LIB_FACTORY_ALGO_MAX_NUM];
} FactoryAlgoNoMatchList;

typedef struct {
    uint32_t nameLen;
    char name[FACTORY_IMAGE_NAME_LEN];
} FactoryImageInfo;

typedef struct {
    uint32_t count;
    FactoryImageInfo imageInfo[TARGET_LIB_FACTORY_IMAGE_MAX_NUM];
} FactoryRestrictedImageList;

typedef struct {
    uint32_t list[TARGET_LIB_FACTORY_IMAGE_MAX_NUM];
} FactoryImageLoadCntList;

int32_t FactoryInit(void);
int32_t FactoryModeEnter(void);
int32_t FactoryModeExit(void);
uint32_t FactoryStatusGet(void);
int32_t FactoryCrcget(FactoryCrcStatus *status, int32_t index);

int32_t FactoryAlgoListGet(FactoryAlgoHandle *handle);
int32_t FactoryAlgoListGetWithCrcCheck(FactoryAlgoHandle *handle);
int32_t FactoryAlgoChange(TargetFlashInfo info);
int32_t FactoryAlgoFree(int32_t index);
int32_t FactoryAlgoFreeAll(void);

int32_t FactoryImageListGet(FactoryImageHandle *handle);
int32_t FactoryImageListGetWithCrcCheck(FactoryImageHandle *handle);
int32_t FactoryImageChange(TargetFlashInfo info, uint8_t *imageName, uint32_t nameLen);
int32_t FactoryImageFree(int32_t index);
int32_t FactoryImageFreeAll(void);

int32_t FactoryLibFileStatusChange(TargetLibType type, int32_t index, uint32_t status);
uint32_t FactoryLibStartAddrGet(TargetLibType type);
int32_t FactoryLibCurrentImageGet(TargetImageLibInfo *info);
int32_t FactoryLibImageIndexGetFormStartAddr(uint32_t startAddr);

bool FactoryAlgoMatchCheck(FactoryAlgoNoMatchList *list);

bool FactoryImageLoadCntCheck(FactoryRestrictedImageList *list);
int32_t FactoryImageLoadCntAdd(uint32_t index);
uint32_t FactoryImageLoadCntGet(uint32_t index);
#endif