/**
  * @copyright Copyright (c) 2024, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file    dap_offline_load_process.c
  * @author  MCU Driver Team
  * @details This file is used to describe the implementation of the interface for storing offline burning files
  *          through the DAP.
  */
#include "securec.h"
#include "flash_decoder.h"
#include "flash_intf.h"
#include "config_storage_update.h"
#include "target_lib_manager.h"
#include "offline_sys_config.h"
#include "user_crc.h"
#include "status.h"
#include "dap_offline_load_process.h"

#define SHIFTS_8_BIT                    8
#define SHIFTS_16_BIT                   16
#define SHIFTS_24_BIT                   24
#define REQUEST_DATA_LENGTH             4

#define MASK_POS_0                      0x000000ff
#define MASK_POS_8                      0x0000ff00
#define MASK_POS_16                     0x00ff0000
#define MASK_POS_24                     0xff000000

#define OFFLINE_STOP_STATUS_DONE          1
#define OFFLINE_STOP_STATUS_FAIL          2

#define OFFLINE_LOAD_HEAD_LEN             FLASH_DECODER_MIN_SIZE

typedef struct {
    uint8_t headBuf[OFFLINE_LOAD_HEAD_LEN];
    uint32_t flashAddr;
    uint8_t nameBuf[TARGET_LIB_IMAGE_NAME_MAX_LEN];
    uint8_t nameBufPos;
    uint8_t imageFlag;
    const flash_intf_t *flash_intf;
} OfflineLoadState;

static OfflineLoadState g_offlineLoadState = {0};

/**
  * @brief Processing file headers.
  * @param info File header information.
  * @param targetFlashInfo Target flash information.
  * @retval errno_t.
  */
static int ParsingHeader(flash_decoder_head_t *info, TargetFlashInfo *targetFlashInfo)
{
    errno_t rc = EOK;

    /* Copy the file header data and structured data from the global buffer. */
    rc = memcpy_s(info, OFFLINE_LOAD_HEAD_LEN, g_offlineLoadState.headBuf, OFFLINE_LOAD_HEAD_LEN);
    if (rc != EOK) {
        return rc;
    }
    /* Parsing header data */
    targetFlashInfo->regioninfo = info->region_info;
    targetFlashInfo->flashID = info->vendor_flash_algo;
    targetFlashInfo->fileSize = info->size;
    targetFlashInfo->modelID = info->vendor_model;
    targetFlashInfo->vendorID = info->vendor;
    targetFlashInfo->version = info->version;
    targetFlashInfo->fileCrc32 = info->crc_value;
    targetFlashInfo->oldNamelen = info->nameLen;
    /* Burned target address. */
    targetFlashInfo->targetAddr = info->targetAddr;
    /* Maximum number of times that the program can be programmed. */
    targetFlashInfo->maxLoads = info->maxLoads;
    return rc;
}

/**
  * @brief Processing the image header.
  * @param request request buffer pointer.
  * @param dataLen request data len.
  * @retval errno_t.
  */
static int ProcessImageHead(const uint8_t *request, uint32_t dataLen)
{
    errno_t rc = EOK;
    flash_decoder_head_t info;
    TargetFlashInfo targetFlashInfo;
    flash_decoder_type_t flashType;

    g_offlineLoadState.nameBufPos = 0;
    /* Parse the file data header. */
    rc = ParsingHeader(&info, &targetFlashInfo);
    if (rc != EOK) {
        return rc;
    }

    if (dataLen < info.nameLen) {
        return rc;
    }
    /* Parse and store file names */
    rc = memcpy_s(g_offlineLoadState.nameBuf, TARGET_LIB_IMAGE_NAME_MAX_LEN, request, info.nameLen);
    if (rc != EOK) {
        return rc;
    }

    /* Processes the offline file list based on whether it is factory mode or normal mode. */
    if (FactoryStatusGet() == FACTORY_FLAG_IS_SET) {
        FactoryImageChange(targetFlashInfo, g_offlineLoadState.nameBuf, info.nameLen);
    } else {
        TargetLibImageAdd(targetFlashInfo, g_offlineLoadState.nameBuf, info.nameLen);
    }
    g_offlineLoadState.nameBufPos = info.nameLen;
    /* Parse the file header and obtain the type and storage address. */
    flashType = flash_decoder_detect_type(g_offlineLoadState.headBuf, OFFLINE_LOAD_HEAD_LEN, 0, false);
    rc = flash_decoder_get_flash(flashType, 0, false, &g_offlineLoadState.flashAddr,
                                 &g_offlineLoadState.flash_intf);
    if (rc != EOK) {
        return rc;
    }
    /* Storage header data. */
    rc = flash_decoder_write(g_offlineLoadState.flashAddr, g_offlineLoadState.headBuf, OFFLINE_LOAD_HEAD_LEN);
    return rc;
}

/**
  * @brief Interface file transfer completion processing function.
  * @param None.
  * @retval None.
  */
static void InterfaceDoneHandle(void)
{
    DapLinkStatus daplinkStatus;

    /* Obtain the partition where the current app is running and
        configure the partition flag for the app to be upgraded. */
    if (SysStartFlagGet() == CONFIG_STRAT_FLAG_A) {
        ConfigStartFlagSave(CONFIG_STRAT_FLAG_B);
    } else {
        ConfigStartFlagSave(CONFIG_STRAT_FLAG_A);
    }
    /* The debugger shows that the upgrade is successful. */
    DapLinkStatusGet(&daplinkStatus);
    daplinkStatus.fwUpgradeStatus = FIMRWARE_UPGRADE_SUCCESS;
    DapLinkStatusSet(&daplinkStatus);
}

/**
  * @brief Image file transfer completion processing function.
  * @param None.
  * @retval None.
  */
static void ImageDoneHandle(void)
{
    DapLinkStatus daplinkStatus;
    TargetImageLibInfo currentLibImage;
    int currentIndex;
    const flash_intf_t *currentIntf;

    /* Obtains the current file information. */
    if (FactoryStatusGet() == FACTORY_FLAG_IS_SET) {
        FactoryLibCurrentImageGet(&currentLibImage);
    } else {
        TargetLibCurrentImageGet(&currentLibImage, (int32_t *)&currentIndex);
    }
    /* Perform CRC verification on the file. */
    UserCrcInit(0, currentLibImage.info.fileCrc32);
    currentIntf = flash_intf_iap_protected;
    if (CheckCrcFromFlash(currentIntf, currentLibImage.startAddr, currentLibImage.info.fileSize)
        != USER_CRC_CHECK_SUCCESS) {
        /* The debugger displays the crc error status. */
        DapLinkStatusGet(&daplinkStatus);
        daplinkStatus.imageStoreStatus = IMAGE_STORE_CRC_ERROR;
        DapLinkStatusSet(&daplinkStatus);
    } else {
        /* Set the stored file to the available state. */
        if (FactoryStatusGet() == FACTORY_FLAG_IS_SET) {
            currentIndex = FactoryLibImageIndexGetFormStartAddr(currentLibImage.startAddr);
            FactoryLibFileStatusChange(TARGET_LIB_TYPE_IMAGE, currentIndex, TARGET_LIB_FILE_STATUS_NORMAL);
        } else {
            currentIndex = TargetLibImageIndexGetFormStartAddr(currentLibImage.startAddr);
            TargetLibFileStatusChange(TARGET_LIB_TYPE_IMAGE, currentIndex, TARGET_LIB_FILE_STATUS_NORMAL);
        }
        /* The debugger displays the success status. */
        DapLinkStatusGet(&daplinkStatus);
        daplinkStatus.imageStoreStatus = IMAGE_STORE_SUCCESS;
        DapLinkStatusSet(&daplinkStatus);
    }
}

/**
  * @brief Algo file transfer completion processing function.
  * @param None.
  * @retval None.
  */
static void AlgoDoneHandle(void)
{
    DapLinkStatus daplinkStatus;
    int currentIndex;
    uint32_t fileStatus;
    TargetAlgoLibInfo currentLibAlgo;
    const flash_intf_t *currentIntf;

    /* Obtains the current file information. */
    TargetLibCurrentAlgoGet(&currentLibAlgo);
    UserCrcInit(0, currentLibAlgo.info.fileCrc32);
    currentIntf = flash_intf_iap_protected;
    if (CheckCrcFromFlash(currentIntf, currentLibAlgo.startAddr, currentLibAlgo.info.fileSize)
        != USER_CRC_CHECK_SUCCESS) {
        /* The debugger displays the crc error status. */
        DapLinkStatusGet(&daplinkStatus);
        daplinkStatus.algoStoreStatus = ALGORITHM_STORE_CRC_ERROR;
        DapLinkStatusSet(&daplinkStatus);
    } else {
        /* Set the stored file to the available state. */
        if ((currentLibAlgo.info.regioninfo & 0xFFFF0000) == TARGET_LIB_SEC_REGION) {
            fileStatus = TARGET_LIB_FILE_STATUS_SECURITY;
        } else {
            fileStatus = TARGET_LIB_FILE_STATUS_NORMAL;
        }

        currentIndex = TargetLibAlgoIndexGetFormStartAddr(currentLibAlgo.startAddr);
        TargetLibFileStatusChange(TARGET_LIB_TYPE_ALGO, currentIndex, fileStatus);
        /* The debugger displays the success status. */
        DapLinkStatusGet(&daplinkStatus);
        daplinkStatus.algoStoreStatus = ALGORITHM_STORE_SUCCESS;
        DapLinkStatusSet(&daplinkStatus);
    }
}


/**
  * @brief Processing function for data transmission completion.
  * @param None.
  * @retval None.
  */
static void TransferUpdateDoneHandle(void)
{
    int loadType;

    /* Obtains the file type that is being processed. */
    loadType = ConfigLoadTypeGet();
    switch (loadType) {
        case CONFIG_LOAD_TYPE_INTERFACE: /* Debugger APP Firmware. */
            InterfaceDoneHandle();
            break;
        case CONFIG_LOAD_TYPE_IMAGE: /* Target image file */
            ImageDoneHandle();
            break;
        case CONFIG_LOAD_TYPE_ALGO: /* Target algo file */
            AlgoDoneHandle();
            break;
        default:
            break;
    }
}

/**
  * @brief Parses the flash drive and storage address based on the file type.
  * @param flashType file type.
  * @retval errno_t.
  */
static int ParseAddrAndFlashIntf(flash_decoder_type_t flashType)
{
    errno_t rc = EOK;
    flash_decoder_head_t info;
    TargetFlashInfo targetFlashInfo;

    /* Processing algorithm file headers */
    if (flashType == FLASH_DECODER_TYPE_TARGET_ALGO) {
        rc = ParsingHeader(&info, &targetFlashInfo);
        if (rc != EOK) {
            return rc;
        }

        if (FactoryStatusGet() == FACTORY_FLAG_IS_SET) {
            FactoryAlgoChange(targetFlashInfo);
        } else {
            TargetLibAlgoAdd(targetFlashInfo);
        }
    }
    /* Obtains the flash drive and file storage start address based on the file type. */
    rc = flash_decoder_get_flash(flashType, 0, false, &g_offlineLoadState.flashAddr,
                                 &g_offlineLoadState.flash_intf);
    if (rc != EOK) {
        return rc;
    }
    flash_decoder_open();
    /* Delayed processing of image files */
    if (flashType == FLASH_DECODER_TYPE_TARGET_IMAGE) {
        g_offlineLoadState.imageFlag = 1; /* image flag set 1 */
        return EOK;
    }
    /* Storage header data. */
    rc = flash_decoder_write(g_offlineLoadState.flashAddr, g_offlineLoadState.headBuf, OFFLINE_LOAD_HEAD_LEN);
    if (rc != EOK) {
        return rc;
    }
    return EOK;
}

/**
  * @brief Processing the Header of an Offline Burning File.
  * @param request request buffer pointer.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_OfflineHeadProc(const uint8_t *request, uint8_t *response)
{
    errno_t rc = EOK;
    flash_decoder_type_t flashType;

    unsigned int maskPos0;
    unsigned int maskPos8;
    unsigned int maskPos16;
    unsigned int maskPos24;
    uint32_t dataLen = 0;

    if (request == NULL || response == NULL) {
        /* high 16-bits is request length, low 16-bits is responce length. */
        return (0U << 16U) | 1U;
    }

    maskPos0 = (*(request++) & MASK_POS_0);
    maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
    maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
    maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
    dataLen = maskPos0 | maskPos8 | maskPos16 | maskPos24;
    
    if (dataLen < FLASH_DECODER_MIN_SIZE) {
        *response = DAP_ERROR;
        /* high 16-bits is request length, low 16-bits is responce length is 1. */
        return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
    }
    g_offlineLoadState.imageFlag = 0; /* image type flag set 0 */
    rc = memset_s(g_offlineLoadState.headBuf, OFFLINE_LOAD_HEAD_LEN,
                  0, OFFLINE_LOAD_HEAD_LEN);
    if (rc != EOK) {
        *response = DAP_ERROR;
        /* high 16-bits is request length, low 16-bits is responce length is 1. */
        return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
    }

    rc = memcpy_s(g_offlineLoadState.headBuf, OFFLINE_LOAD_HEAD_LEN,
                  request, OFFLINE_LOAD_HEAD_LEN);
    if (rc != EOK) {
        *response = DAP_ERROR;
        /* high 16-bits is request length, low 16-bits is responce length is 1. */
        return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
    }

    flashType = flash_decoder_detect_type(g_offlineLoadState.headBuf, OFFLINE_LOAD_HEAD_LEN, 0, false);
    if (flashType == FLASH_DECODER_TYPE_UNKNOWN || flashType == FLASH_DECODER_TYPE_TARGET) {
            *response = DAP_ERROR;
        /* high 16-bits is request length, low 16-bits is responce length is 1. */
        return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
    }

    rc = ParseAddrAndFlashIntf(flashType);
    if (rc != EOK) {
        *response = DAP_ERROR;
        /* high 16-bits is request length, low 16-bits is responce length is 1. */
        return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
    }

    *response = DAP_OK;
    /* high 16-bits is request length, low 16-bits is responce length is 1. */
    return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
}

/**
  * @brief Processing Offline Burning File Data.
  * @param request request buffer pointer.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_OfflineDataProc(const uint8_t *request, uint8_t *response)
{
    errno_t rc = EOK;
    flash_decoder_type_t flashType;
    uint32_t startAddr;

    unsigned int maskPos0;
    unsigned int maskPos8;
    unsigned int maskPos16;
    unsigned int maskPos24;
    uint32_t dataLen = 0;
    uint32_t writeLen = 0;

    if (request == NULL || response == NULL) {
        flash_decoder_close();
        /* high 16-bits is request length, low 16-bits is responce length is 1. */
        return (0U << 16U) | 1U;
    }
    /* Length of the parsed data */
    maskPos0 = (*(request++) & MASK_POS_0);
    maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
    maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
    maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
    dataLen = maskPos0 | maskPos8 | maskPos16 | maskPos24;
    writeLen = dataLen;
    /* process image head */
    if (g_offlineLoadState.imageFlag == 1) { /* image flag set 1 */
        g_offlineLoadState.imageFlag = 0;
        rc = ProcessImageHead(request, dataLen);
        if ((rc != EOK) && (rc != ERROR_SUCCESS_DONE)) {
            flash_decoder_close();
            *response = DAP_ERROR;
            /* high 16-bits is request length, low 16-bits is responce length is 1. */
            return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
        }
        request += g_offlineLoadState.nameBufPos;
        writeLen -= g_offlineLoadState.nameBufPos;
    }

    rc = flash_decoder_write(g_offlineLoadState.flashAddr, request, writeLen);
    if ((rc != EOK) && (rc != ERROR_SUCCESS_DONE)) {
        flash_decoder_close();
        *response = DAP_ERROR;
        /* high 16-bits is request length, low 16-bits is responce length is 1. */
        return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
    }
    g_offlineLoadState.flashAddr += writeLen;
    *response = DAP_OK;
    /* high 16-bits is request length, low 16-bits is responce length is 1. */
    return ((dataLen + REQUEST_DATA_LENGTH) << 16U) | 1U;
}

/**
  * @brief Stop processing offline burning.
  * @param request request buffer pointer.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_OfflineStopProc(const uint8_t *request, uint8_t *response)
{
    uint8_t status = 0;
    int loadType;
    DapLinkStatus daplinkStatus;

    flash_decoder_close();
    status = *(request++);
    if (status == OFFLINE_STOP_STATUS_DONE) {
        TransferUpdateDoneHandle();
    } else {
        /* Debugger displays failure information */
        loadType = ConfigLoadTypeGet();
        switch (loadType) {
            case CONFIG_LOAD_TYPE_INTERFACE:  /* Debugger APP Firmware. */
                DapLinkStatusGet(&daplinkStatus);
                daplinkStatus.fwUpgradeStatus = FIMRWARE_UPGRADE_CRC_ERROR;
                DapLinkStatusSet(&daplinkStatus);
                break;
            case CONFIG_LOAD_TYPE_IMAGE: /* Target image file */
                DapLinkStatusGet(&daplinkStatus);
                daplinkStatus.imageStoreStatus = IMAGE_STORE_FAIL;
                DapLinkStatusSet(&daplinkStatus);
                break;
            case CONFIG_LOAD_TYPE_ALGO: /* Target algo file */
                DapLinkStatusGet(&daplinkStatus);
                daplinkStatus.algoStoreStatus = ALGORITHM_STORE_FAIL;
                DapLinkStatusSet(&daplinkStatus);
                break;
            default :
                break;
        }
    }
    *response = DAP_OK;
    /* high 16-bits is request length, low 16-bits is responce length is 1. */
    return (sizeof(uint8_t) << 16U) | (1U);
}