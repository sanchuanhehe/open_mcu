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
  * @file    target_lib_manager.c
  * @author  MCU Driver Team
  * @brief   target flash library mamager driver.
  */
#include <stdbool.h>
#include <string.h>
#include "securec.h"
#include "FlashPrg.h"
#include "target_vendor_config.h"
#include "config_storage_update.h"
#include "status.h"
#include "debug.h"
#include "target_lib_manager.h"

static TargetImageHandle g_targetImageInfo;
static TargetAlgoHandle g_targetAlgoInfo;

static TargetImageLoadCntList g_targetImageLoadCntList;

static int32_t g_currentAlgoIndex = 0;
static int32_t g_currentImageIndex = 0;
static bool g_freeAlgoFlag[TARGET_LIB_ALGO_MAX_NUM];
static bool g_freeImageFlag[TARGET_LIB_IMAGE_MAX_NUM];

static TargetLibDisplayState g_targetLibDisplayState;

/**
  * @brief Update the image list of the target chip.
  * @retval None
  */
static void TargetLibImageUpdateList(void)
{
    TargetImageLibInfo tempInfo;
    uint32_t i;
    int32_t tempCnt;
    uint32_t tempLoadCnt;
    if (g_currentImageIndex == (g_targetImageInfo.count - 1) ||
        g_targetImageInfo.count <= 0) {
        return;
    }
    tempInfo = g_targetImageInfo.libInfo[g_targetImageInfo.count - 1];
    tempLoadCnt = g_targetImageLoadCntList.list[g_targetImageInfo.count - 1];
    g_targetImageInfo.libInfo[g_targetImageInfo.count - 1] = g_targetImageInfo.libInfo[g_currentImageIndex];
    g_targetImageLoadCntList.list[g_targetImageInfo.count - 1] = g_targetImageLoadCntList.list[g_currentImageIndex];
    /* To obtain the number of moves, subtract 2 from the number of moves. */
    tempCnt = g_targetImageInfo.count - g_currentImageIndex - 2;
    for (i = 0; i < tempCnt; i++) {
        g_targetImageInfo.libInfo[g_currentImageIndex + i] = g_targetImageInfo.libInfo[g_currentImageIndex + 1 + i];
        g_targetImageLoadCntList.list[g_currentImageIndex + i] = \
            g_targetImageLoadCntList.list[g_currentImageIndex + 1 + i];
    }
    g_targetImageInfo.libInfo[g_targetImageInfo.count - 2] = tempInfo; /* Last Move Forward 2 */
    g_targetImageLoadCntList.list[g_targetImageInfo.count - 2] = tempLoadCnt; /* Last Move Forward 2 */
    g_currentImageIndex = g_targetImageInfo.count - 1;
}

/**
  * @brief Update the algorithm list of the target chip.
  * @retval None
  */
static void TargetLibAlgoUpdateList(void)
{
    TargetAlgoLibInfo tempInfo;
    uint32_t i;
    int32_t tempCnt;
    if (g_currentAlgoIndex == (g_targetAlgoInfo.count - 1) ||
        g_targetAlgoInfo.count <= 0) {
        return;
    }
    tempInfo = g_targetAlgoInfo.libInfo[g_targetAlgoInfo.count - 1];
    g_targetAlgoInfo.libInfo[g_targetAlgoInfo.count - 1] = g_targetAlgoInfo.libInfo[g_currentAlgoIndex];
    /* To obtain the number of moves, subtract 2 from the number of moves. */
    tempCnt = g_targetAlgoInfo.count - g_currentAlgoIndex - 2;
    for (i = 0; i < tempCnt; i++) {
        g_targetAlgoInfo.libInfo[g_currentAlgoIndex + i] = g_targetAlgoInfo.libInfo[g_currentAlgoIndex + 1 + i];
    }
    g_targetAlgoInfo.libInfo[g_targetAlgoInfo.count - 2] = tempInfo; /* Last Move Forward 2 */
    g_currentAlgoIndex = g_targetAlgoInfo.count - 1;
}

/**
  * @brief Obtaining the address of the idle algorithm space
  * @retval Idle algorithm space address
  */
static uint32_t FreeAlgoAddrGet(void)
{
    int i;
    for (i = 0; i < TARGET_LIB_ALGO_MAX_NUM; i++) {
        if (g_freeAlgoFlag[i]) {
            g_freeAlgoFlag[i] = false;
            ConfigFreeAlgoFlagSave(g_freeAlgoFlag, sizeof(g_freeAlgoFlag));
            return TARGET_LIB_ALGO_START + i * TARGET_LIB_ONE_ALGO_SIZE;
        }
    }
    return g_targetAlgoInfo.libInfo[0].startAddr;
}

/**
  * @brief Address status configuration of the idle algorithm space.
  * @param addr Algorithm space address
  * @param state Algorithm space address status
  * @retval TARGET_LIB_OK
  */
static int32_t FreeAlgoAddrSet(uint32_t addr, bool state)
{
    uint32_t index;
    index = (addr - TARGET_LIB_ALGO_START) / TARGET_LIB_ONE_ALGO_SIZE;
    g_freeAlgoFlag[index] = state;
    ConfigFreeAlgoFlagSave(g_freeAlgoFlag, sizeof(g_freeAlgoFlag));
    return TARGET_LIB_OK;
}

/**
  * @brief Obtaining the free space address of the image.
  * @retval Specifies the free space address of the image
  */
static uint32_t FreeImageAddrGet(void)
{
    int i;
    for (i = 0; i < TARGET_LIB_IMAGE_MAX_NUM; i++) {
        if (g_freeImageFlag[i]) {
            g_freeImageFlag[i] = false;
            ConfigFreeImageFlagSave(g_freeImageFlag, sizeof(g_freeImageFlag));
            return TARGET_LIB_IMAGE_START + i * TARGET_LIB_ONE_IMAGE_SIZE;
        }
    }
    return g_targetImageInfo.libInfo[0].startAddr;
}

/**
  * @brief Address status configuration of the idle image space.
  * @param addr image space address
  * @param state image space address status
  * @retval TARGET_LIB_OK
  */
static int32_t FreeImageAddrSet(uint32_t addr, bool state)
{
    uint32_t index;
    index = (addr - TARGET_LIB_IMAGE_START) / TARGET_LIB_ONE_IMAGE_SIZE;
    g_freeImageFlag[index] = state;
    ConfigFreeImageFlagSave(g_freeImageFlag, sizeof(g_freeImageFlag));
    return TARGET_LIB_OK;
}

/**
  * @brief Initialize the target file library.
  * @retval Success or Failure Result
  */
int32_t TargetLibInit(void)
{
    ConfigAlgoListRead(&g_targetAlgoInfo, sizeof(g_targetAlgoInfo));
    if (g_targetAlgoInfo.count > TARGET_LIB_ALGO_MAX_NUM) {
        g_targetAlgoInfo.count = 0;
    }
    ConfigImageListRead(&g_targetImageInfo, sizeof(g_targetImageInfo));
    if (g_targetImageInfo.count > TARGET_LIB_IMAGE_MAX_NUM) {
        g_targetImageInfo.count = 0;
    }

    ConfigImageLoadCntRead(&g_targetImageLoadCntList, sizeof(g_targetImageLoadCntList));
    ConfigFreeAlgoFlagRead(g_freeAlgoFlag, sizeof(g_freeAlgoFlag));
    ConfigFreeImageFlagRead(g_freeImageFlag, sizeof(g_freeImageFlag));
    g_currentAlgoIndex = ConfigCurrentAlgoIndexRead();
    return 0;
}

/**
  * @brief Deinitialize the target file library.
  * @retval Success or Failure Result
  */
int32_t TargetLibUninit(void)
{
    return TARGET_LIB_OK;
}

/**
  * @brief Obtaining the Algorithm List of Target Chips.
  * @param handle Algorithm list handle
  * @retval Success or Failure Result
  */
int32_t TargetLibAlgoListGet(TargetAlgoHandle *handle)
{
    if (handle == NULL) {
        return TARGET_LIB_ERROR;
    }

    *handle = g_targetAlgoInfo;
    return TARGET_LIB_OK;
}

/**
  * @brief Concatenate the algorithm name based on the algorithm list index.
  * @param info Target chip information
  * @param index Index of the target algorithm list
  * @retval Success or Failure Result
  */
static int32_t TargetAlgoNameSplicing(TargetFlashInfo info, int32_t index)
{
    errno_t rc = EOK;
    switch (info.vendorID) {
        case TARGET_VENDOR_HISILICON:
            rc = strcat_s(g_targetAlgoInfo.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                          g_hiModelName[info.modelID]);
            break;
        case TARGET_VENDOR_HM:
            rc = strcat_s(g_targetAlgoInfo.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                          g_vendorHMModelName[info.modelID]);
            break;
        case TARGET_VENDOR_AUCU:
            rc = strcat_s(g_targetAlgoInfo.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                          g_aucuModelName[info.modelID]);
            break;
        default:
            break;
    }
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    rc = strcat_s(g_targetAlgoInfo.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                  g_regionInfo[info.regioninfo & 0xFF]);
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    return TARGET_LIB_OK;
}

/**
  * @brief Check whether indexes exist.
  * @param info Target flash information
  * @retval index
  */
static int32_t CheckAddIndex(TargetFlashInfo info)
{
    int32_t index = -1;

    for (int i = 0; i < g_targetAlgoInfo.count; i++) {
        if (info.regioninfo != g_targetAlgoInfo.libInfo[i].info.regioninfo) {
            continue;
        }

        if (info.vendorID != g_targetAlgoInfo.libInfo[i].info.vendorID) {
            continue;
        }

        if (info.modelID != g_targetAlgoInfo.libInfo[i].info.modelID) {
            continue;
        }

        if (info.flashID != g_targetAlgoInfo.libInfo[i].info.flashID) {
            continue;
        }
        index = i;
        break;
    }
    return index;
}

/**
  * @brief Adding an algorithm file to the target algorithm list
  * @param info Target flash information
  * @retval Success or Failure Result
  */
int32_t TargetLibAlgoAdd(TargetFlashInfo info)
{
    int32_t index = -1;
    errno_t rc = EOK;

    if (g_targetAlgoInfo.count > TARGET_LIB_ALGO_MAX_NUM) {
        return TARGET_LIB_ERROR;
    }

    index = CheckAddIndex(info);
    if (index == -1) {
        if (g_targetAlgoInfo.count == TARGET_LIB_ALGO_MAX_NUM || g_targetAlgoInfo.count < 0) {
            index = 0;
        } else {
            index = g_targetAlgoInfo.count;
            g_targetAlgoInfo.libInfo[index].startAddr = FreeAlgoAddrGet();
            g_targetAlgoInfo.count++;
        }
        rc = memset_s(g_targetAlgoInfo.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                      0, sizeof(g_targetAlgoInfo.libInfo[index].name));
        if (rc != EOK) {
            return TARGET_LIB_ERROR;
        }
        TargetAlgoNameSplicing(info, index);
        g_targetAlgoInfo.libInfo[index].nameLen = strlen(g_targetAlgoInfo.libInfo[index].name);
    }
    g_targetAlgoInfo.libInfo[index].info = info;
    g_targetAlgoInfo.libInfo[index].fileStatus = TARGET_LIB_FILE_STATUS_BAD;
    g_currentAlgoIndex = index;
    TargetLibAlgoUpdateList();
    ConfigAlgoListSave(&g_targetAlgoInfo, sizeof(g_targetAlgoInfo));
    ConfigCurrentAlgoIndexSave(g_currentAlgoIndex);
    return TARGET_LIB_OK;
}

/**
  * @brief Select the target algorithm based on the list index.
  * @param index Target algorithm index
  * @retval Success or Failure Result
  */
int32_t TargetLibAlgoSelect(int32_t index)
{
    if (index >= g_targetAlgoInfo.count ||
        index < 0 ||
        g_targetAlgoInfo.count > TARGET_LIB_ALGO_MAX_NUM) {
        return TARGET_LIB_ERROR;
    }
    if (index != g_currentAlgoIndex) {
        g_currentAlgoIndex = index;
        ConfigCurrentAlgoIndexSave(g_currentAlgoIndex);
    }
    return TARGET_LIB_OK;
}

/**
  * @brief Obtains the currently selected target chip algorithm.
  * @param info Flash information of the target chip
  * @retval Success or Failure Result
  */
int32_t TargetLibCurrentAlgoGet(TargetAlgoLibInfo *info)
{
    if (info == NULL) {
        return TARGET_LIB_ERROR;
    }
    *info = g_targetAlgoInfo.libInfo[g_currentAlgoIndex];
    return TARGET_LIB_OK;
}

/**
  * @brief Obtains the algorithm file index based on the start address.
  * @param startAddr Start address of the algorithm file
  * @retval Algorithm file index
  */
int32_t TargetLibAlgoIndexGetFormStartAddr(uint32_t startAddr)
{
    int32_t i;
    int32_t index = -1;

    for (i = 0; i < g_targetAlgoInfo.count; i++) {
        if (startAddr == g_targetAlgoInfo.libInfo[i].startAddr) {
            index = i;
            break;
        }
    }
    return index;
}

/**
  * @brief Obtain the algorithm file index based on the flash information of the target chip.
  * @param info Flash information of the target chip
  * @retval Algorithm file index
  */
int32_t TargetLibAlgoIndexGetFormFlashInfo(TargetFlashInfo info)
{
    int32_t i;
    int32_t index = -1;
    for (i = 0; i < g_targetAlgoInfo.count; i++) {
        if (g_targetAlgoInfo.libInfo[i].info.regioninfo != info.regioninfo) {
            continue;
        }

        if (g_targetAlgoInfo.libInfo[i].info.vendorID != info.vendorID) {
            continue;
        }

        if (g_targetAlgoInfo.libInfo[i].info.modelID != info.modelID) {
            continue;
        }

        if (g_targetAlgoInfo.libInfo[i].info.flashID != info.flashID) {
            continue;
        }
        index = i;
    }
    return index;
}

/**
  * @brief Obtaining the image list of the target chip
  * @param handle Image handle of the target chip
  * @retval Success or Failure Result
  */
int32_t TargetLibImageListGet(TargetImageHandle *handle)
{
    if (handle == NULL) {
        return TARGET_LIB_ERROR;
    }
    *handle = g_targetImageInfo;
    return TARGET_LIB_OK;
}

/**
  * @brief Assemble the image file name based on the target chip information.
  * @param info Flash information of the target chip
  * @param tempName Specifies the image file name
  * @param nameLen Name len
  * @retval Success or Failure Result
  */
static int32_t TargetImageNameSplicing(TargetFlashInfo info, char *tempName, uint32_t nameLen)
{
    errno_t rc = EOK;
    rc = strcat_s(tempName, nameLen, "(");    /* Splice Brackets */
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    switch (info.vendorID) {                 /* Splicing Manufacturer */
        case TARGET_VENDOR_HISILICON:
            rc = strcat_s(tempName, nameLen, g_hiModelName[info.modelID]);
            break;
        case TARGET_VENDOR_HM:               /* vendor open harmony */
            rc = strcat_s(tempName, nameLen, g_vendorHMModelName[info.modelID]);
            break;
        case TARGET_VENDOR_AUCU:
            rc = strcat_s(tempName, nameLen, g_aucuModelName[info.modelID]);
            break;
        default:
            break;
    }
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    rc = strcat_s(tempName, nameLen, g_regionInfo[info.regioninfo & 0xFF]);    /* Stitching Partitions */
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    rc = strcat_s(tempName, nameLen, ")");  /* Splice Brackets */
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
}

/**
  * @brief Updating the Image Index
  * @param info Flash information of the target chip
  * @retval Image file index
  */
static int32_t UpdatingImageIndex(TargetFlashInfo info, int32_t index)
{
    int32_t tmpIndex = index;
    if (tmpIndex == -1) {
        if (g_targetImageInfo.count == TARGET_LIB_IMAGE_MAX_NUM) {
            tmpIndex = 0;
        } else {
            tmpIndex = g_targetImageInfo.count;
            g_targetImageInfo.libInfo[tmpIndex].startAddr = FreeImageAddrGet();
            g_targetImageInfo.count++;
        }
    }
    g_targetImageInfo.libInfo[tmpIndex].info = info;
    g_targetImageInfo.libInfo[tmpIndex].fileStatus = TARGET_LIB_FILE_STATUS_BAD;
    return tmpIndex;
}

/**
  * @brief Adding a File to the Image File List.
  * @param info Flash information of the target chip
  * @param imageName Image File Name
  * @param nameLen Length of the image file name
  * @retval Success or Failure Result
  */
int32_t TargetLibImageAdd(TargetFlashInfo info, uint8_t *imageName, uint32_t nameLen)
{
    int32_t i;
    int32_t index = -1;
    uint32_t templen = 0;
    errno_t rc = EOK;
    char tempName[TARGET_LIB_IMAGE_NAME_MAX_LEN];
    if (g_targetImageInfo.count > TARGET_LIB_IMAGE_MAX_NUM) {
        return TARGET_LIB_ERROR;
    }

    if (imageName == NULL || nameLen > TARGET_LIB_IMAGE_IN_NAME_MAX_LEN) {
        return TARGET_LIB_ERROR;
    }
    memset_s(tempName, TARGET_LIB_IMAGE_NAME_MAX_LEN, 0, sizeof(tempName));
    rc = memcpy_s(tempName, TARGET_LIB_IMAGE_NAME_MAX_LEN, imageName, nameLen);
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    TargetImageNameSplicing(info, tempName, TARGET_LIB_IMAGE_NAME_MAX_LEN);
    templen = strlen(tempName);

    for (i = 0; i < g_targetImageInfo.count; i++) {
        if (g_targetImageInfo.libInfo[i].nameLen != templen) {
            continue;
        }

        if (strncmp(tempName, g_targetImageInfo.libInfo[i].name, templen) == 0) {
            index = i;
            break;
        }
    }
    index = UpdatingImageIndex(info, index);
    memset_s(g_targetImageInfo.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
             0, sizeof(g_targetImageInfo.libInfo[index].name));
    rc = memcpy_s(g_targetImageInfo.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                  tempName, templen);
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    g_targetImageInfo.libInfo[index].nameLen = templen;

    g_currentImageIndex = index;
    g_targetImageLoadCntList.list[index] = 0;
    TargetLibImageUpdateList();
    ConfigImageListSave(&g_targetImageInfo, sizeof(g_targetImageInfo));
    ConfigImageLoadCntSave(&g_targetImageLoadCntList, sizeof(g_targetImageLoadCntList));
    return TARGET_LIB_OK;
}

/**
  * @brief Selecting an image file based on the index.
  * @param index Image file index
  * @retval Success or Failure Result
  */
int32_t TargetLibImageSelect(int32_t index)
{
    if (index >= g_targetImageInfo.count || index < 0) {
        return TARGET_LIB_ERROR;
    }
    g_currentImageIndex = index;
    return TARGET_LIB_OK;
}

/**
  * @brief Obtains the information and index of the currently selected file.
  * @param info Image file information
  * @param index Index of the currently selected file
  * @retval Success or Failure Result
  */
int32_t TargetLibCurrentImageGet(TargetImageLibInfo *info, int32_t *index)
{
    if (info == NULL || index == NULL) {
        return TARGET_LIB_ERROR;
    }
    *info = g_targetImageInfo.libInfo[g_currentImageIndex];
    *index = g_currentImageIndex;
    return TARGET_LIB_OK;
}

/**
  * @brief Obtains the image index based on the start address.
  * @param startAddr Image start address
  * @retval Indicates the image index
  */
int32_t TargetLibImageIndexGetFormStartAddr(uint32_t startAddr)
{
    int32_t i;
    int32_t index = -1;

    for (i = 0; i < g_targetImageInfo.count; i++) {
        if (startAddr == g_targetImageInfo.libInfo[i].startAddr) {
            index = i;
            break;
        }
    }
    return index;
}

/**
  * @brief Start address configuration of the target chip
  * @param type TargetLibType
  * @retval TARGET_LIB_OK
  */
int32_t TargetLibStartAddrSet(TargetLibType type)
{
    return TARGET_LIB_OK;
}

/**
  * @brief Obtain the start address of the current image file or algorithm file.
  * @param type TargetLibType
  * @retval Start address of the current image file or algorithm file
  */
uint32_t TargetLibStartAddrGet(TargetLibType type)
{
    if (type == TARGET_LIB_TYPE_IMAGE) {
        return g_targetImageInfo.libInfo[g_currentImageIndex].startAddr;
    } else {
        return g_targetAlgoInfo.libInfo[g_currentAlgoIndex].startAddr;
    }
}

/**
  * @brief Display Status Obtaining
  * @retval Display Status
  */
TargetLibDisplayState TargetLibDisplayStateGet(void)
{
    return g_targetLibDisplayState;
}

/**
  * @brief Display Status Obtaining
  * @param state Display Status
  * @retval TARGET_LIB_OK
  */
int32_t TargetLibDisplayStateSet(TargetLibDisplayState state)
{
    g_targetLibDisplayState = state;
    return TARGET_LIB_OK;
}

/**
  * @brief Delete algorithm files based on indexes.
  * @param index Algorithm file index
  * @retval Success or Failure Result
  */
int32_t TargetLibAlgoDelete(int32_t index)
{
    uint32_t i;
    int32_t updataCnt;
    DapLinkStatus status;
    errno_t rc = EOK;

    DapLinkStatusGet(&status);
    status.algoDeleteStatus = ALGORITHM_DELETE_RUNNING;
    DapLinkStatusSet(&status);

    if (index >= g_targetAlgoInfo.count || g_targetAlgoInfo.count <= 0 || index < 0) {
        DapLinkStatusGet(&status);
        status.algoDeleteStatus = ALGORITHM_DELETE_FAIL;
        DapLinkStatusSet(&status);
        return TARGET_LIB_ERROR;
    }
    FreeAlgoAddrSet(g_targetAlgoInfo.libInfo[index].startAddr, true);

    if (index < g_targetAlgoInfo.count - 1) {
        updataCnt = g_targetAlgoInfo.count - index - 1;
        for (i = 0; i < updataCnt; i++) {
            g_targetAlgoInfo.libInfo[index + i] = g_targetAlgoInfo.libInfo[index + i + 1];
        }
    }
    rc = memset_s(&g_targetAlgoInfo.libInfo[g_targetAlgoInfo.count - 1], sizeof(TargetAlgoLibInfo),
                  0, sizeof(TargetAlgoLibInfo));
    if (rc != EOK) {
        DapLinkStatusGet(&status);
        status.algoDeleteStatus = ALGORITHM_DELETE_FAIL;
        DapLinkStatusSet(&status);
        return TARGET_LIB_ERROR;
    }
    g_targetAlgoInfo.count--;
    ConfigAlgoListSave(&g_targetAlgoInfo, sizeof(g_targetAlgoInfo));
    if (index < g_currentAlgoIndex) {
        g_currentAlgoIndex--;
    } else if (index == g_currentAlgoIndex) {
        g_currentAlgoIndex = g_targetAlgoInfo.count - 1;
    }
    ConfigCurrentAlgoIndexSave(g_currentAlgoIndex);

    DapLinkStatusGet(&status);
    status.algoDeleteStatus = ALGORITHM_DELETE_SUCCESS;
    DapLinkStatusSet(&status);

    return TARGET_LIB_OK;
}

/**
  * @brief Delete all algorithm files.
  * @retval Success or Failure Result
  */
int32_t TargetLibAlgoDeleteAll(void)
{
    uint32_t i;
    DapLinkStatus status;
    errno_t rc = EOK;

    DapLinkStatusGet(&status);
    status.algoDeleteStatus = ALGORITHM_DELETE_RUNNING;
    DapLinkStatusSet(&status);

    for (i = 0; i < TARGET_LIB_ALGO_MAX_NUM; i++) {
        g_freeAlgoFlag[i] = true;
        rc = memset_s(&g_targetAlgoInfo.libInfo[i], sizeof(g_targetAlgoInfo.libInfo[i]),
                      0, sizeof(g_targetAlgoInfo.libInfo[i]));
        if (rc != EOK) {
            DapLinkStatusGet(&status);
            status.algoDeleteStatus = ALGORITHM_DELETE_FAIL;
            DapLinkStatusSet(&status);
            return TARGET_LIB_ERROR;
        }
    }
    ConfigFreeAlgoFlagSave(g_freeAlgoFlag, sizeof(g_freeAlgoFlag));
    g_targetAlgoInfo.count = 0;
    ConfigAlgoListSave(&g_targetAlgoInfo, sizeof(g_targetAlgoInfo));
    g_currentAlgoIndex = 0;
    ConfigCurrentAlgoIndexSave(g_currentAlgoIndex);

    DapLinkStatusGet(&status);
    status.algoDeleteStatus = ALGORITHM_DELETE_SUCCESS;
    DapLinkStatusSet(&status);
    return TARGET_LIB_OK;
}

/**
  * @brief Delete image files based on indexes.
  * @param index image file index
  * @retval Success or Failure Result
  */
int32_t TargetLibImageDelete(uint32_t index)
{
    uint32_t i;
    uint32_t updataCnt;
    errno_t rc = EOK;

    DapLinkStatus status;

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_RUNNING;
    DapLinkStatusSet(&status);
    if (index >= g_targetImageInfo.count) {
        return TARGET_LIB_ERROR;
    }
    FreeImageAddrSet(g_targetImageInfo.libInfo[index].startAddr, true);

    if (index < g_targetImageInfo.count - 1) {
        updataCnt = g_targetImageInfo.count - index - 1;
        for (i = 0; i < updataCnt; i++) {
            g_targetImageInfo.libInfo[index + i] = g_targetImageInfo.libInfo[index + i + 1];
            g_targetImageLoadCntList.list[index + i] = g_targetImageLoadCntList.list[index + i + 1];
        }
    }
    rc = memset_s(&g_targetImageInfo.libInfo[g_targetImageInfo.count - 1], sizeof(TargetImageLibInfo),
                  0, sizeof(TargetImageLibInfo));
    if (rc != EOK) {
        DapLinkStatusGet(&status);
        status.imageDeleteStatus = IMAGE_DELETE_FAIL;
        DapLinkStatusSet(&status);
        return TARGET_LIB_ERROR;
    }
    g_targetImageLoadCntList.list[g_targetImageInfo.count - 1] = TARGET_LIB_LOAD_CNT_INVALIA;
    g_targetImageInfo.count--;
    ConfigImageListSave(&g_targetImageInfo, sizeof(g_targetImageInfo));
    ConfigImageLoadCntSave(&g_targetImageLoadCntList, sizeof(g_targetImageLoadCntList));

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_SUCCESS;
    DapLinkStatusSet(&status);
    return TARGET_LIB_OK;
}

/**
  * @brief Delete all image files.
  * @retval Success or Failure Result
  */
int32_t TargetLibImageDeleteAll(void)
{
    uint32_t i;
    DapLinkStatus status;
    errno_t rc = EOK;

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_RUNNING;
    DapLinkStatusSet(&status);

    for (i = 0; i < TARGET_LIB_IMAGE_MAX_NUM; i++) {
        g_freeImageFlag[i] = true;
        rc = memset_s(&g_targetImageInfo.libInfo[i], sizeof(g_targetImageInfo.libInfo[i]),
                      0, sizeof(g_targetImageInfo.libInfo[i]));
        if (rc != EOK) {
            DapLinkStatusGet(&status);
            status.imageDeleteStatus = IMAGE_DELETE_FAIL;
            DapLinkStatusSet(&status);
            return TARGET_LIB_ERROR;
        }
        g_targetImageLoadCntList.list[i] = TARGET_LIB_LOAD_CNT_INVALIA;
    }
    ConfigFreeImageFlagSave(g_freeImageFlag, sizeof(g_freeImageFlag));
    g_targetImageInfo.count = 0;
    ConfigImageListSave(&g_targetImageInfo, sizeof(g_targetImageInfo));
    ConfigImageLoadCntSave(&g_targetImageLoadCntList, sizeof(g_targetImageLoadCntList));

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_SUCCESS;
    DapLinkStatusSet(&status);
    return TARGET_LIB_OK;
}

/**
  * @brief Modifying the File Status
  * @param type Image file or algorithm file
  * @param index file index
  * @param status File Status
  * @retval Success or Failure Result
  */
int32_t TargetLibFileStatusChange(TargetLibType type, int32_t index, uint32_t status)
{
    if (index >= TARGET_LIB_IMAGE_MAX_NUM || index < 0) {
        return TARGET_LIB_ERROR;
    }

    if (type == TARGET_LIB_TYPE_IMAGE) {
        g_targetImageInfo.libInfo[index].fileStatus = status;
        ConfigImageListSave(&g_targetImageInfo, sizeof(g_targetImageInfo));
    } else if (type == TARGET_LIB_TYPE_ALGO) {
        g_targetAlgoInfo.libInfo[index].fileStatus = status;
        ConfigAlgoListSave(&g_targetAlgoInfo, sizeof(g_targetAlgoInfo));
        if (status == TARGET_LIB_FILE_STATUS_BAD) {
            g_currentAlgoIndex = 0;
            ConfigCurrentAlgoIndexSave(g_currentAlgoIndex);
        }
    } else {
        return TARGET_LIB_ERROR;
    }
    return TARGET_LIB_OK;
}

/**
  * @brief Obtaining the Chip Erasure List
  * @param list Chip Erasure List
  * @param listLen List len
  * @retval Success or Failure Result
  */
int32_t TargetLibEraseListGet(TargetEraseList *list, uint32_t listLen)
{
    char *tempStr = NULL;
    int32_t tempCnt = 0;
    uint32_t templen = 0;
    errno_t rc = EOK;
    if (list == NULL) {
        return TARGET_LIB_ERROR;
    }

    rc = memset_s(list, listLen, 0, listLen);
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    if (g_targetAlgoInfo.count == 0 ||
        g_targetAlgoInfo.count > TARGET_LIB_ALGO_MAX_NUM) {
        list->count = 0;
        return TARGET_LIB_OK;
    }

    for (uint32_t i = 0; i < g_targetAlgoInfo.count; i++) {
        if ((g_targetAlgoInfo.libInfo[i].info.regioninfo & 0xFFFF0000) !=
            TARGET_LIB_APP_REGION) {
            continue;
        }

        if (g_targetAlgoInfo.libInfo[i].fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
            continue;
        }
        tempStr = strchr(g_targetAlgoInfo.libInfo[i].name, '_');
        templen = tempStr - g_targetAlgoInfo.libInfo[i].name;
        rc = memcpy_s(list->eraseList[tempCnt].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                      g_targetAlgoInfo.libInfo[i].name, templen);
        if (rc != EOK) {
            return TARGET_LIB_ERROR;
        }
        list->eraseList[tempCnt].nameLen = templen;
        list->eraseList[tempCnt].index = i;
        tempCnt++;
    }
    list->count = tempCnt;
    return TARGET_LIB_OK;
}

/**
  * @brief Obtains the number of times that an image has been downloaded based on the index.
  * @param index image index
  * @retval Number of times the image has been downloaded
  */
uint32_t TargetImageLoadCntGet(int32_t index)
{
    if (index >= TARGET_LIB_IMAGE_MAX_NUM || index < 0) {
        return TARGET_LIB_LOAD_CNT_INVALIA;
    }
    return g_targetImageLoadCntList.list[index];
}

/**
  * @brief Accumulate the number of image download times based on the index.
  * @param index image index
  * @retval Success or Failure Result
  */
int32_t TargetImageLoadCntAdd(int32_t index)
{
    if (index >= TARGET_LIB_IMAGE_MAX_NUM || index < 0) {
        return TARGET_LIB_ERROR;
    }
    g_targetImageLoadCntList.list[index]++;
    ConfigImageLoadCntSave(&g_targetImageLoadCntList, sizeof(g_targetImageLoadCntList));
    return TARGET_LIB_OK;
}