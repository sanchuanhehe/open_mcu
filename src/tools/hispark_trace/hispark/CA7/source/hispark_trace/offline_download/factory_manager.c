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
  * @file    factory_manager.c
  * @author  MCU Driver Team
  * @brief   factory manager driver.
  */
#include "securec.h"
#include "FlashPrg.h"
#include "target_board.h"
#include "flash_manager.h"
#include "target_vendor_config.h"
#include "config_storage_update.h"
#include "target_algo_parse.h"
#include "swd_jtag_config.h"
#include "user_crc.h"
#include "status.h"
#include "factory_manager.h"

static uint32_t g_factoryFlag = 0;
static FactoryImageHandle g_factoryImageList;
static int32_t g_currentFactoryImageIndex = 0;
static bool g_freeFactoryImageFlag[TARGET_LIB_FACTORY_IMAGE_MAX_NUM];
static FactoryImageLoadCntList g_factoryImageLoadCntList;

/**
  * @brief Updates the factory image list.
  * @retval None.
  */
static void TargetFactoryImageUpdateList(void)
{
    TargetImageLibInfo tempInfo;
    uint32_t i;
    int32_t tempCnt;
    uint32_t tempLoadCnt;
    if (g_currentFactoryImageIndex == (g_factoryImageList.count - 1) ||
        g_factoryImageList.count <= 0) {
        return;
    }
    /* Temporary Storage List End File */
    tempInfo = g_factoryImageList.libInfo[g_factoryImageList.count - 1];
    tempLoadCnt = g_factoryImageLoadCntList.list[g_factoryImageList.count - 1];
    g_factoryImageList.libInfo[g_factoryImageList.count - 1] = g_factoryImageList.libInfo[g_currentFactoryImageIndex];
    g_factoryImageLoadCntList.list[g_factoryImageList.count - 1] = \
        g_factoryImageLoadCntList.list[g_currentFactoryImageIndex];
    /* To obtain the number of moves, subtract 2 from the number of moves. */
    tempCnt = g_factoryImageList.count - g_currentFactoryImageIndex - 2;
    /* Move the file position to the left. */
    for (i = 0; i < tempCnt; i++) {
        g_factoryImageList.libInfo[g_currentFactoryImageIndex + i] = \
            g_factoryImageList.libInfo[g_currentFactoryImageIndex + 1 + i];
        g_factoryImageLoadCntList.list[g_currentFactoryImageIndex + i] = \
            g_factoryImageLoadCntList.list[g_currentFactoryImageIndex + 1 + i];
    }
    g_factoryImageList.libInfo[g_factoryImageList.count - 2] = tempInfo; /* Last Move Forward 2 */
    g_factoryImageLoadCntList.list[g_factoryImageList.count - 2] = tempLoadCnt; /* Last Move Forward 2 */
    g_currentFactoryImageIndex = g_factoryImageList.count - 1;
}

/**
  * @brief Get the factory mode idle address and save it.
  * @retval Free Address
  */
static uint32_t FreeFactoryImageAddrGet(void)
{
    int i;
    /* Query the first idle area address in the list. */
    for (i = 0; i < TARGET_LIB_FACTORY_IMAGE_MAX_NUM; i++) {
        if (g_freeFactoryImageFlag[i]) {
            g_freeFactoryImageFlag[i] = false;
            ConfigFactoryFreeImageFlagSave(g_freeFactoryImageFlag, sizeof(g_freeFactoryImageFlag));
            return TARGET_LIB_FACTORY_IMAGE_START + i * TARGET_LIB_ONE_FACTORY_IMAGE_SIZE;
        }
    }

    return g_factoryImageList.libInfo[0].startAddr;
}

/**
  * @brief Set the idle address state in factory mode without saving it.
  * @param addr Image addr
  * @param state Free state
  * @retval TARGET_LIB_OK
  */
static int32_t FreeFactoryImageAddrSetNoSave(uint32_t addr, bool state)
{
    uint32_t index;
    index = (addr - TARGET_LIB_FACTORY_IMAGE_START) / TARGET_LIB_ONE_FACTORY_IMAGE_SIZE;
    g_freeFactoryImageFlag[index] = state;
    return TARGET_LIB_OK;
}

/**
  * @brief Set the idle address state in factory mode and saving it.
  * @param addr Image addr
  * @param state Free state
  * @retval TARGET_LIB_OK
  */
static int32_t FreeFactoryImageAddrSet(uint32_t addr, bool state)
{
    FreeFactoryImageAddrSetNoSave(addr, state);
    ConfigFactoryFreeImageFlagSave(g_freeFactoryImageFlag, sizeof(g_freeFactoryImageFlag));
    return TARGET_LIB_OK;
}

/**
  * @brief The factory mode image is released based on the index and is not saved.
  * @param index Image index
  * @retval FACTORY_STATUS_OK or FACTORY_STATUS_ERROR
  */
static int32_t FactoryImageFreeNosave(uint32_t index)
{
    uint32_t updataCnt;
    uint32_t i;
    errno_t rc = EOK;

    if (g_factoryImageList.count >= TARGET_LIB_FACTORY_IMAGE_MAX_NUM) {
        return FACTORY_STATUS_ERROR;
    }
    /* Idle address flag configuration */
    FreeFactoryImageAddrSetNoSave(g_factoryImageList.libInfo[index].startAddr, true);
    if (index < g_factoryImageList.count - 1) {
        updataCnt = g_factoryImageList.count - index - 1;
        for (i = 0; i < updataCnt; i++) {
            g_factoryImageList.libInfo[index + i] = g_factoryImageList.libInfo[index + i + 1];
            g_factoryImageLoadCntList.list[index + i] = g_factoryImageLoadCntList.list[index + i + 1];
        }
    }
    rc = memset_s(&g_factoryImageList.libInfo[g_factoryImageList.count - 1], sizeof(TargetImageLibInfo),
                  0, sizeof(TargetImageLibInfo));
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;
    }
    g_factoryImageLoadCntList.list[g_factoryImageList.count - 1] = TARGET_LIB_LOAD_CNT_INVALIA;
    g_factoryImageList.count--;
    return FACTORY_STATUS_OK;
}

/**
  * @brief Factory Mode Module Initialization.
  * @retval FACTORY_STATUS_OK
  */
int32_t FactoryInit(void)
{
    g_factoryFlag = ConfigFactoryFlagRead();
    ConfigFactoryImageListRead(&g_factoryImageList, sizeof(g_factoryImageList));
    if (g_factoryImageList.count > TARGET_LIB_FACTORY_IMAGE_MAX_NUM) {
        g_factoryImageList.count = 0;
    }
    ConfigFactoryFreeImageFlagRead(g_freeFactoryImageFlag, sizeof(g_freeFactoryImageFlag));
    ConfigFactoryImageLoadCntRead(&g_factoryImageLoadCntList,
                                  sizeof(g_factoryImageLoadCntList));
    return FACTORY_STATUS_OK;
}

/**
  * @brief Enter factory mode.
  * @retval FACTORY_STATUS_OK
  */
int32_t FactoryModeEnter(void)
{
    g_factoryFlag = FACTORY_FLAG_IS_SET;
    ConfigFactoryFlagSave(g_factoryFlag);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Exit factory mode.
  * @retval FACTORY_STATUS_OK
  */
int32_t FactoryModeExit(void)
{
    g_factoryFlag = 0;
    ConfigFactoryFlagSave(g_factoryFlag);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Obtains the factory mode status.
  * @retval factory mode status
  */
uint32_t FactoryStatusGet(void)
{
    return g_factoryFlag;
}

/**
  * @brief Configuration for obtaining the CRC value in factory mode
  * @param verifyStartAddr CRC check start address
  * @param index Indicates the image index for CRC check
  * @retval Node
  */
static void FactoryCrcGetConfig(uint32_t *verifyStartAddr, int32_t index)
{
    if (g_board_info.target_cfg) {
        region_info_t *region = g_board_info.target_cfg->flash_regions;
        for (; region->start != 0 || region->end != 0; ++region) {
            if (g_factoryImageList.libInfo[index].info.version > 0) {
                *verifyStartAddr = g_factoryImageList.libInfo[index].info.targetAddr;
            } else {
                *verifyStartAddr = region->start;
            }
            if (kRegionIsDefault == region->flags) {
                break;
            }
        }
    }
}

/**
  * @brief Obtains the CRC value and length based on the index.
  * @param checkValue Crc check value
  * @param checkLen Crc check len
  * @param index Indicates the image index for CRC check
  * @retval Node
  */
static void FactoryCrcCheckValueAndLenGet(uint32_t *checkValue, uint32_t *checkLen, int32_t index)
{
    uint32_t tempAddr;

    *checkValue = g_factoryImageList.libInfo[index].info.fileCrc32;
    if (*checkValue == FACTORY_CRC_CHECK_SPE_VALUE) {
        *checkLen = g_factoryImageList.libInfo[index].info.fileSize - 4; /* Subtract the last 4 bytes */
        Init(0, 0, 0);
        tempAddr = g_factoryImageList.libInfo[index].startAddr + *checkLen;
        if (FlashRead(tempAddr, 4, (uint8_t*)checkValue) != 0) { /* Read the last 4 bytes */
            UnInit(0);
            return;
        }
        UnInit(0);
    } else {
        *checkLen = g_factoryImageList.libInfo[index].info.fileSize;
    }
}

/**
  * @brief Check input var.
  * @param status CRC check result
  * @param index Indicates the image index for CRC check
  * @retval CRC var check result
  */
static int32_t FactoryCrcVarCheck(FactoryCrcStatus *status, int32_t index)
{
    if (status == NULL || index >= TARGET_LIB_FACTORY_IMAGE_MAX_NUM || index < 0) {
        return FACTORY_STATUS_ERROR;
    }
    return FACTORY_STATUS_OK;
}

/**
  * @brief CRC check target chip
  * @param flash_intf Flash drive interface
  * @param startAddr Check Start Address
  * @param len Check length
  * @retval CRC var check result
  */
static int32_t FactoryTargetCrcCheck(uint32_t startAddr, uint32_t len)
{
    int32_t clockLevel;
    int32_t ret = USER_CRC_CHECK_NOT_CONNECT;
    const flash_intf_t *currentIntf = NULL;

    currentIntf = flash_intf_target;
    clockLevel = SwdJtagClockLevelGet();
    while (1) {
        if (clockLevel <= SWD_JTAG_CLOCK_LEVEL_9) {
            SwJtagClockLevelLevelSet(clockLevel);
            clockLevel++;
        } else {
            break;
        }
        ret = CheckCrcFromFlash(currentIntf, startAddr, len);
        if (ret == USER_CRC_CHECK_SUCCESS || ret == USER_CRC_CHECK_ERROR) {
            return ret;
        }
    }
    return ret;
}

/**
  * @brief Obtaining the Image CRC of the Target Board in Factory Mode
  * @param status CRC check result
  * @param index Indicates the image index for CRC check
  * @retval CRC check result
  */
int32_t FactoryCrcget(FactoryCrcStatus *status, int32_t index)
{
    uint32_t verifyStartAddr = 0;
    FactoryCrcStatus tempStatus;
    uint32_t checkValue;
    uint32_t checkLen;
    int32_t algoIndex;

    if (FactoryCrcVarCheck(status, index) != FACTORY_STATUS_OK) {
        return FACTORY_STATUS_ERROR;
    }
    tempStatus.results = USER_CRC_CHECK_FAIL;
    *status = tempStatus;

    algoIndex = TargetLibAlgoIndexGetFormFlashInfo(g_factoryImageList.libInfo[index].info);
    if (algoIndex < 0) {
        tempStatus.results = USER_CRC_CHECK_NO_ALGO;
        *status = tempStatus;
        return FACTORY_STATUS_OK;
    }

    if (TargetAlgoConfig(algoIndex) != TARGET_ALGO_PARSE_OK) {
        return FACTORY_STATUS_ERROR;
    }

    if (g_factoryImageList.libInfo[index].fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
        return FACTORY_STATUS_ERROR;
    }

    FactoryCrcGetConfig(&verifyStartAddr, index);
    FactoryCrcCheckValueAndLenGet(&checkValue, &checkLen, index);

    tempStatus.checkLen = checkLen;
    UserCrcInit(0, checkValue);
    tempStatus.preCrc = checkValue;
    tempStatus.checkStartAddr = verifyStartAddr;
    int32_t ret = FactoryTargetCrcCheck(verifyStartAddr, checkLen);
    if (ret != USER_CRC_CHECK_SUCCESS) {
        if (ret == USER_CRC_CHECK_ERROR) {
            tempStatus.currentCrc = UserCurrentCrc32Get();
            UserCrcInit(0, g_factoryImageList.libInfo[index].info.fileCrc32);
            if (CheckCrcFromFlash(flash_intf_iap_protected, g_factoryImageList.libInfo[index].startAddr,
                                  g_factoryImageList.libInfo[index].info.fileSize) != USER_CRC_CHECK_SUCCESS) {
                FactoryLibFileStatusChange(TARGET_LIB_TYPE_IMAGE, index, TARGET_LIB_FILE_STATUS_BAD);
                ret = FACTORY_CRC_CHECK_RES_BAD_FILE;
            }
        }
        tempStatus.results = ret;
    } else {
        tempStatus.results = USER_CRC_CHECK_SUCCESS;
        tempStatus.currentCrc = checkValue;
    }
    *status = tempStatus;
    return FACTORY_STATUS_OK;
}

/**
  * @brief Obtaining the Flash Algorithm List in Factory Mode
  * @param handle Flash Algorithm List handle
  * @retval FACTORY_STATUS_ERROR or FACTORY_STATUS_OK
  */
int32_t FactoryAlgoListGet(FactoryAlgoHandle *handle)
{
    if (handle == NULL) {
        return FACTORY_STATUS_ERROR;
    }
    TargetLibAlgoListGet(handle);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Obtaining the Flash Algorithm List in Factory Mode and Performing CRC
  * @param handle Flash Algorithm List handle
  * @retval FACTORY_STATUS_ERROR or FACTORY_STATUS_OK
  */
int32_t FactoryAlgoListGetWithCrcCheck(FactoryAlgoHandle *handle)
{
    int32_t ret;
    int32_t status = FactoryAlgoListGet(handle);
    if (status != FACTORY_STATUS_OK) {
        return status;
    }
    for (uint32_t i = 0; i < handle->count; i++) {
        TargetAlgoLibInfo *libInfo = &handle->libInfo[i];
        UserCrcInit(0, libInfo->info.fileCrc32);
        ret = CheckCrcFromFlash(flash_intf_iap_protected, libInfo->startAddr, libInfo->info.fileSize);
        if (ret != USER_CRC_CHECK_SUCCESS) {
            return FACTORY_STATUS_CRC_ERROR;
        }
    }
    return FACTORY_STATUS_OK;
}

/**
  * @brief List of algorithms to be changed in factory mode
  * @param info Flash information of the target chip
  * @retval FACTORY_STATUS_OK
  */
int32_t FactoryAlgoChange(TargetFlashInfo info)
{
    TargetLibAlgoAdd(info);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Delete the flash algorithm file based on the index
  * @param index Indicates the index of the flash algorithm file
  * @retval FACTORY_STATUS_OK
  */
int32_t FactoryAlgoFree(int32_t index)
{
    TargetLibAlgoDelete(index);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Delete all flash algorithm files in factory mode.
  * @retval FACTORY_STATUS_OK
  */
int32_t FactoryAlgoFreeAll(void)
{
    TargetLibAlgoDeleteAll();
    return FACTORY_STATUS_OK;
}

/**
  * @brief Obtaining the File List in Factory Mode
  * @param handle Factory image Information handle
  * @retval FACTORY_STATUS_OK or FACTORY_STATUS_ERROR
  */
int32_t FactoryImageListGet(FactoryImageHandle *handle)
{
    if (handle == NULL) {
        return FACTORY_STATUS_ERROR;
    }
    *handle = g_factoryImageList;
    return FACTORY_STATUS_OK;
}

/**
  * @brief Obtaining and Verifying the File List in Factory Mode
  * @param handle Factory image Information handle
  * @retval Obtain the result status
  */
int32_t FactoryImageListGetWithCrcCheck(FactoryImageHandle *handle)
{
    int32_t ret;
    if (FactoryImageListGet(handle) != FACTORY_STATUS_OK) {
        return FACTORY_STATUS_ERROR;
    }

    for (uint32_t i = 0; i < handle->count; i++) {
        TargetImageLibInfo *libInfo = &handle->libInfo[i];
        UserCrcInit(0, libInfo->info.fileCrc32);
        ret = CheckCrcFromFlash(flash_intf_iap_protected, libInfo->startAddr, libInfo->info.fileSize);
        if (ret != USER_CRC_CHECK_SUCCESS) {
            return FACTORY_STATUS_CRC_ERROR;
        }
    }
    return USER_CRC_CHECK_SUCCESS;
}

/**
  * @brief Concatenate the image name in factory mode.
  * @param info Flash chip information
  * @param tempName Name after splicing
  * @param nameLen Name len
  * @retval Refreshing succeeded or failed
  */
static int32_t FactorySplicingImageName(TargetFlashInfo info, char *tempName, uint32_t nameLen)
{
    errno_t rc = EOK;
    rc = strcat_s(tempName, nameLen, "(");  /* Splice Brackets */
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;    /* Error check */
    }
    switch (info.vendorID) {      /* Splicing Manufacturer */
        case TARGET_VENDOR_HM:    /* vendor open harmony */
            rc = strcat_s(tempName, nameLen, g_vendorHMModelName[info.modelID]);
            break;
        case TARGET_VENDOR_AUCU:
            rc = strcat_s(tempName, nameLen, g_aucuModelName[info.modelID]);
            break;
        case TARGET_VENDOR_HISILICON:
            rc = strcat_s(tempName, nameLen, g_hiModelName[info.modelID]);
            break;
        default:
            break;
    }
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;  /* Error check */
    }
    rc = strcat_s(tempName, nameLen, g_regionInfo[info.regioninfo & 0xFF]);
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;
    }
    rc = strcat_s(tempName, nameLen, ")");  /* Splice Brackets */
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;
    }
    return FACTORY_STATUS_OK;
}

/**
  * @brief Refreshing the image list in factory mode.
  * @param info Flash chip information
  * @param imageName target image name
  * @param nameLen target image name len
  * @param index target image index
  * @retval Refreshing succeeded or failed
  */
static int32_t FactoryImageRefresh(TargetFlashInfo info, const char *imageName, uint32_t nameLen, int32_t index)
{
    errno_t rc = EOK;
    g_factoryImageList.libInfo[index].fileStatus = TARGET_LIB_FILE_STATUS_BAD;
    g_factoryImageList.libInfo[index].info = info;
    rc = memset_s(g_factoryImageList.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN,
                  0, sizeof(g_factoryImageList.libInfo[index].name));
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;
    }
    rc = memcpy_s(g_factoryImageList.libInfo[index].name, TARGET_LIB_IMAGE_NAME_MAX_LEN, imageName, nameLen);
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;
    }
    g_factoryImageList.libInfo[index].nameLen = nameLen;
    g_currentFactoryImageIndex = index;
    g_factoryImageLoadCntList.list[index] = 0;
    TargetFactoryImageUpdateList();
    ConfigFactoryImageListSave(&g_factoryImageList, sizeof(g_factoryImageList));
    ConfigFactoryImageLoadCntSave(&g_factoryImageLoadCntList, sizeof(g_factoryImageLoadCntList));
    return FACTORY_STATUS_OK;
}

/**
  * @brief Factory Mode Duplicate Address Handling.
  * @param info Flash chip information
  * @retval None
  */
static void FactoryImageDuplicateAddressHandle(TargetFlashInfo info)
{
    uint32_t freeIndex = 0;

    while (freeIndex < g_factoryImageList.count) {
        if (((info.targetAddr <= g_factoryImageList.libInfo[freeIndex].info.targetAddr) &&
             ((info.targetAddr + info.fileSize) > g_factoryImageList.libInfo[freeIndex].info.targetAddr)) ||
            ((g_factoryImageList.libInfo[freeIndex].info.targetAddr < info.targetAddr) &&
             ((g_factoryImageList.libInfo[freeIndex].info.targetAddr +
               g_factoryImageList.libInfo[freeIndex].info.fileSize) > info.targetAddr))) {
            FactoryImageFreeNosave(freeIndex);
        } else {
            freeIndex++;
        }
    }
}

/**
  * @brief Refresh the image list in factory mode and save it.
  * @param info Flash chip information
  * @param imageName target image name
  * @param nameLen target image name len
  * @retval Refreshing succeeded or failed
  */
int32_t FactoryImageChange(TargetFlashInfo info, uint8_t *imageName, uint32_t nameLen)
{
    int32_t index = -1;
    uint32_t templen = 0;
    errno_t rc = EOK;

    char tempName[TARGET_LIB_IMAGE_NAME_MAX_LEN];

    if (imageName == NULL || nameLen > TARGET_LIB_IMAGE_IN_NAME_MAX_LEN) {
        return TARGET_LIB_ERROR;
    }

    FactoryImageDuplicateAddressHandle(info);
    ConfigFactoryFreeImageFlagSave(g_freeFactoryImageFlag, sizeof(g_freeFactoryImageFlag));
    ConfigFactoryImageListSave(&g_factoryImageList, sizeof(g_factoryImageList));
    memset_s(tempName, TARGET_LIB_IMAGE_NAME_MAX_LEN, 0, sizeof(tempName));
    rc = memcpy_s(tempName, TARGET_LIB_IMAGE_NAME_MAX_LEN, imageName, nameLen);
    if (rc != EOK) {
        return TARGET_LIB_ERROR;
    }
    FactorySplicingImageName(info, tempName, TARGET_LIB_IMAGE_NAME_MAX_LEN);
    templen = strlen(tempName);

    for (int i = 0; i < g_factoryImageList.count; i++) {
        if (g_factoryImageList.libInfo[i].nameLen != templen) {
            continue;
        }

        if (strncmp(tempName, g_factoryImageList.libInfo[i].name, templen) == 0) {
            index = i;
            break;
        }
    }

    if (index == -1) {
        if (g_factoryImageList.count == TARGET_LIB_IMAGE_MAX_NUM) {
            index = 0;
        } else {
            index = g_factoryImageList.count;
            g_factoryImageList.libInfo[index].startAddr = FreeFactoryImageAddrGet();
            g_factoryImageList.count++;
        }
    }
    FactoryImageRefresh(info, tempName, templen, index);
    return FACTORY_STATUS_OK;
}

/**
  * @brief In factory mode, the image file is deleted based on the index.
  * @param index Indicates the image index.
  * @retval Refreshing succeeded or failed
  */
int32_t FactoryImageFree(int32_t index)
{
    DapLinkStatus status;
    int32_t updataCnt;
    uint32_t i;
    errno_t rc = EOK;

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_RUNNING;
    DapLinkStatusSet(&status);
    if (g_factoryImageList.count >= TARGET_LIB_FACTORY_IMAGE_MAX_NUM) {
        return FACTORY_STATUS_ERROR;
    }
    FreeFactoryImageAddrSet(g_factoryImageList.libInfo[index].startAddr, true);
    if (index < g_factoryImageList.count - 1) {
        updataCnt = g_factoryImageList.count - index - 1;
        for (i = 0; i < updataCnt; i++) {
            g_factoryImageLoadCntList.list[index + i] = g_factoryImageLoadCntList.list[index + i + 1];
            g_factoryImageList.libInfo[index + i] = g_factoryImageList.libInfo[index + i + 1];
        }
    }
    rc = memset_s(&g_factoryImageList.libInfo[g_factoryImageList.count - 1], sizeof(TargetImageLibInfo),
                  0, sizeof(TargetImageLibInfo));
    if (rc != EOK) {
        DapLinkStatusGet(&status);
        status.imageDeleteStatus = IMAGE_DELETE_FAIL;
        DapLinkStatusSet(&status);
        return FACTORY_STATUS_ERROR;
    }
    g_factoryImageLoadCntList.list[g_factoryImageList.count - 1] = TARGET_LIB_LOAD_CNT_INVALIA;
    g_factoryImageList.count--;
    ConfigFactoryImageListSave(&g_factoryImageList, sizeof(g_factoryImageList));
    ConfigFactoryImageLoadCntSave(&g_factoryImageLoadCntList, sizeof(g_factoryImageLoadCntList));

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_SUCCESS;
    DapLinkStatusSet(&status);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Delete all image files in factory mode.
  * @retval Refreshing succeeded or failed
  */
int32_t FactoryImageFreeAll(void)
{
    uint32_t i;
    errno_t rc = EOK;

    DapLinkStatus status;

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_RUNNING;
    DapLinkStatusSet(&status);

    for (i = 0; i < TARGET_LIB_FACTORY_IMAGE_MAX_NUM; i++) {
        g_freeFactoryImageFlag[i] = true;
        rc = memset_s(&g_factoryImageList.libInfo[i], sizeof(g_factoryImageList.libInfo[i]),
                      0, sizeof(g_factoryImageList.libInfo[i]));
        if (rc != EOK) {
            DapLinkStatusGet(&status);
            status.imageDeleteStatus = IMAGE_DELETE_FAIL;
            DapLinkStatusSet(&status);
            return FACTORY_STATUS_ERROR;
        }
        g_factoryImageLoadCntList.list[i] = TARGET_LIB_LOAD_CNT_INVALIA;
    }
    ConfigFactoryFreeImageFlagSave(g_freeFactoryImageFlag, sizeof(g_freeFactoryImageFlag));
    g_factoryImageList.count = 0;
    ConfigFactoryImageListSave(&g_factoryImageList, sizeof(g_factoryImageList));
    ConfigFactoryImageLoadCntSave(&g_factoryImageLoadCntList, sizeof(g_factoryImageLoadCntList));

    DapLinkStatusGet(&status);
    status.imageDeleteStatus = IMAGE_DELETE_SUCCESS;
    DapLinkStatusSet(&status);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Modifying the File Status Based on the Index in Factory Mode.
  * @param type File Type
  * @param index File index
  * @param status Status of the file to be modified
  * @retval Refreshing succeeded or failed
  */
int32_t FactoryLibFileStatusChange(TargetLibType type, int32_t index, uint32_t status)
{
    if (index >= TARGET_LIB_FACTORY_IMAGE_MAX_NUM || index < 0) {
        return FACTORY_STATUS_ERROR;
    }

    if (type == TARGET_LIB_TYPE_IMAGE) {
        g_factoryImageList.libInfo[index].fileStatus = status;
        ConfigFactoryImageListSave(&g_factoryImageList, sizeof(g_factoryImageList));
    } else {
        return FACTORY_STATUS_ERROR;
    }
    return FACTORY_STATUS_OK;
}

/**
  * @brief Obtaining the Start Address of the Image File in Factory Mode.
  * @param type File Type
  * @retval image file start address
  */
uint32_t FactoryLibStartAddrGet(TargetLibType type)
{
    if (type == TARGET_LIB_TYPE_IMAGE) {
        return g_factoryImageList.libInfo[g_currentFactoryImageIndex].startAddr;
    } else {
        return 0;
    }
}

/**
  * @brief Obtaining the File Index Based on the Start Address in Factory Mode.
  * @param startAddr image start address
  * @retval image file index
  */
int32_t FactoryLibImageIndexGetFormStartAddr(uint32_t startAddr)
{
    int32_t i;
    int32_t index = -1;

    for (i = 0; i < g_factoryImageList.count; i++) {
        if (startAddr == g_factoryImageList.libInfo[i].startAddr) {
            index = i;
            break;
        }
    }
    return index;
}

/**
  * @brief Obtain the image file that is being processed in factory mode.
  * @param info Image file information
  * @retval Failure or Success Result
  */
int32_t FactoryLibCurrentImageGet(TargetImageLibInfo *info)
{
    if (info == NULL) {
        return TARGET_LIB_ERROR;
    }
    *info = g_factoryImageList.libInfo[g_currentFactoryImageIndex];
    return TARGET_LIB_OK;
}

/**
  * @brief Check whether there is a matching algorithm based on the image file information.
  * @param info Image file information
  * @retval true or false
  */
static bool HasMatchAlgo(TargetFlashInfo info)
{
    TargetAlgoHandle tmpAlgoList;
    TargetLibAlgoListGet(&tmpAlgoList);
    for (int i = 0; i < tmpAlgoList.count; i++) {
        if (info.regioninfo != tmpAlgoList.libInfo[i].info.regioninfo) {
            continue;
        }

        if (info.vendorID != tmpAlgoList.libInfo[i].info.vendorID) {
            continue;
        }

        if (info.modelID != tmpAlgoList.libInfo[i].info.modelID) {
            continue;
        }

        if (info.flashID != tmpAlgoList.libInfo[i].info.flashID) {
            continue;
        }
        return true;
    }
    return false;
}

/**
  * @brief The stitching algorithm does not match the file name in factory mode.
  * @param list List of files whose algorithms do not match
  * @param cnt Indicates the number of files whose algorithms do not match.
  * @param index File index with algorithm mismatch
  * @retval Failure or Success Result
  */
static int32_t FactorySplicingAlgoName(FactoryAlgoNoMatchList *list, uint32_t cnt, uint32_t index)
{
    errno_t rc = EOK;
    switch (g_factoryImageList.libInfo[index].info.vendorID) {
        case TARGET_VENDOR_HISILICON:
            rc = strcat_s(list->algoInfo[cnt].name, FACTORY_ALGO_NAME_LEN,
                          g_hiModelName[g_factoryImageList.libInfo[index].info.modelID]);
            break;
        case TARGET_VENDOR_ST:
            rc = strcat_s(list->algoInfo[cnt].name, FACTORY_ALGO_NAME_LEN,
                          g_stModelName[g_factoryImageList.libInfo[index].info.modelID]);
            break;
        case TARGET_VENDOR_TI:
            rc = strcat_s(list->algoInfo[cnt].name, FACTORY_ALGO_NAME_LEN,
                          g_tiModelName[g_factoryImageList.libInfo[index].info.modelID]);
            break;
        case TARGET_VENDOR_RENESAS:
            rc = strcat_s(list->algoInfo[cnt].name, FACTORY_ALGO_NAME_LEN,
                          g_renesasModelName[g_factoryImageList.libInfo[index].info.modelID]);
            break;
        case TARGET_VENDOR_HM:
            rc = strcat_s(list->algoInfo[cnt].name, FACTORY_ALGO_NAME_LEN,
                          g_vendorHMModelName[g_factoryImageList.libInfo[index].info.modelID]);
            break;
        case TARGET_VENDOR_AUCU:
            rc = strcat_s(list->algoInfo[cnt].name, FACTORY_ALGO_NAME_LEN,
                          g_aucuModelName[g_factoryImageList.libInfo[index].info.modelID]);
            break;
        default:
            break;
    }
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;
    }
    rc = strcat_s(list->algoInfo[cnt].name, FACTORY_ALGO_NAME_LEN,
                  g_regionInfo[g_factoryImageList.libInfo[index].info.regioninfo & 0xFF]);
    if (rc != EOK) {
        return FACTORY_STATUS_ERROR;
    }
    list->algoInfo[cnt].nameLen = strlen(list->algoInfo[cnt].name);
    return FACTORY_STATUS_OK;
}

/**
  * @brief Check the algorithm matching in factory mode.
  * @param list List of files whose algorithms do not match
  * @retval true or false
  */
bool FactoryAlgoMatchCheck(FactoryAlgoNoMatchList *list)
{
    FactoryAlgoNoMatchList tempList;
    uint32_t tempCnt = 0;
    uint32_t j;
    errno_t rc = EOK;

    if (list == NULL) {
        return false;
    }
    rc = memset_s(&tempList, sizeof(tempList), 0, sizeof(tempList));
    if (rc != EOK) {
        return false;
    }
    for (uint32_t i = 0; i < g_factoryImageList.count; i++) {
        if (HasMatchAlgo(g_factoryImageList.libInfo[i].info)) {
            continue;
        }
        FactorySplicingAlgoName(&tempList, tempCnt, i);
        for (j = 0; j < tempCnt;) {
            if (strcmp(tempList.algoInfo[j].name, tempList.algoInfo[tempCnt].name) == 0) {
                rc = memset_s(&tempList.algoInfo[tempCnt].name, FACTORY_ALGO_NAME_LEN,
                              0, tempList.algoInfo[tempCnt].nameLen);
                break;
            }
            j++;
        }
        if (rc != EOK) {
            return false;
        }
        if (j >= tempCnt) {
            tempCnt++;
        }
    }

    if (tempCnt > 0) {
        tempList.count = tempCnt;
        *list = tempList;
        return true;
    }
    return false;
}

/**
  * @brief Check the number of image burning times in factory mode.
  * @param list Factory restricted image list
  * @retval true or false
  */
bool FactoryImageLoadCntCheck(FactoryRestrictedImageList *list)
{
    FactoryRestrictedImageList tempList;
    uint32_t tempCnt = 0;
    errno_t rc = EOK;

    if (list == NULL) {
        return false;
    }

    rc = memset_s(&tempList, sizeof(tempList), 0, sizeof(tempList));
    if (rc != EOK) {
        return false;
    }
    for (int i = 0; i < g_factoryImageList.count; i++) {
        if (g_factoryImageLoadCntList.list[i] >= g_factoryImageList.libInfo[i].info.maxLoads &&
            g_factoryImageList.libInfo[i].info.maxLoads != 0) {
            rc = memcpy_s(tempList.imageInfo[tempCnt].name, FACTORY_IMAGE_NAME_LEN,
                          g_factoryImageList.libInfo[i].name,
                          g_factoryImageList.libInfo[i].nameLen);
            if (rc != EOK) {
                return false;
            }
            tempList.imageInfo[tempCnt].nameLen = g_factoryImageList.libInfo[i].nameLen;
            tempCnt++;
        }
    }

    if (tempCnt > 0) {
        tempList.count = tempCnt;
        *list = tempList;
        return true;
    } else {
        return false;
    }
}

/**
  * @brief Accumulated number of image downloads in factory mode.
  * @param index image index
  * @retval Success or Failure Result
  */
int32_t FactoryImageLoadCntAdd(uint32_t index)
{
    if (index >= TARGET_LIB_FACTORY_IMAGE_MAX_NUM) {
        return TARGET_LIB_ERROR;
    }

    g_factoryImageLoadCntList.list[index]++;
    ConfigFactoryImageLoadCntSave(&g_factoryImageLoadCntList, sizeof(g_factoryImageLoadCntList));
    return FACTORY_STATUS_OK;
}

/**
  * @brief Obtaining the Number of Burning Times of an Image File in Factory Mode.
  * @param index image index
  * @retval Number of burnt images
  */
uint32_t FactoryImageLoadCntGet(uint32_t index)
{
    return g_factoryImageLoadCntList.list[index];
}