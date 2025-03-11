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
  * @file    offline_download.c
  * @author  MCU Driver Team
  * @brief   offline download driver.
  */
#include <stdbool.h>
#include "securec.h"
#include "target_config.h"
#include "flash_manager.h"
#include "FlashPrg.h"
#include "target_board.h"
#include "target_lib_manager.h"
#include "target_algo_parse.h"
#include "status.h"
#include "swd_jtag_config.h"
#include "factory_manager.h"
#include "user_crc.h"
#include "config_storage_update.h"
#include "offline_download.h"

#define DOWNLOAD_BUFF_SIZE        1024
#define OFFLINE_ERASE_TYR_TIME    3

/* offline state */
#define OFFLINE_DOWNLOAD_STATE_IDLE          0
#define OFFLINE_DOWNLOAD_STATE_OPEN          1
#define OFFLINE_DOWNLOAD_STATE_SELECT_ALGO   2
#define OFFLINE_DOWNLOAD_STATE_LOADING       3
#define OFFLINE_DOWNLOAD_VERIFI              4
#define OFFLINE_DOWNLOAD_STATE_DONE          5
#define OFFLINE_DOWNLOAD_STATE_ERROR         6

typedef struct {
    TargetAlgoLibInfo currentLibAlgo;
    TargetImageLibInfo currentLibImage;
    int32_t currentImageIndex;
    uint32_t currentDownloadAdrr;
    uint32_t currentDataAdrr;
    uint32_t currentDownloadDataEndAddr;
    uint32_t currentDownloadSize;
    uint32_t currentFactoryFileCnt;
} OfflineDownloadHandle;

static OfflineDownloadHandle g_offlineDownloadHandle;
volatile static int32_t g_offlineDownloadState;
volatile static bool g_downloadStartFlag;
volatile static bool g_loadingInitialized;
volatile static OfflineDownLoadDisplayState g_offlineDownloadDisplayState;
static uint8_t g_downloadBuf[DOWNLOAD_BUFF_SIZE];

/**
  * @brief Gets the smaller of two values.
  * @param a Input value a
  * @param b Input value b
  * @retval smaller of two values
  */
static uint32_t Min(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

/**
  * @brief Idle processing function.
  * @retval None
  */
static void IdleHandler(void)
{
    if (g_downloadStartFlag) {
        g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_OPEN;
        g_offlineDownloadHandle.currentFactoryFileCnt = 0;
    }
}

/**
  * @brief Initialization parameter configuration.
  * @retval None
  */
static void OpenParameterInit(void)
{
    g_offlineDownloadHandle.currentDataAdrr = 0;
    g_offlineDownloadHandle.currentDownloadAdrr = 0;
    g_offlineDownloadHandle.currentDownloadSize = 0;
    g_loadingInitialized = false;
}

/**
  * @brief Open processing function.
  * @retval None
  */
static void OpenHandler(void)
{
    errno_t rc = EOK;
    DapLinkStatus status;
    uint32_t tempCnt;
    FactoryImageHandle tempFactoryImageList;
    /* Initialize global parameters. */
    memset_s(g_downloadBuf, sizeof(g_downloadBuf), 0xFF, sizeof(g_downloadBuf));

    OpenParameterInit();
    /* Check whether the factory mode is used. Perform different processing in factory mode and normal mode. */
    if (FactoryStatusGet() == FACTORY_FLAG_IS_SET) {
        FactoryImageListGet(&tempFactoryImageList);
        /* Check whether the number of download times exceeds the limit. */
        if (tempFactoryImageList.count == 0 ||
            tempFactoryImageList.count > TARGET_LIB_FACTORY_IMAGE_MAX_NUM) {
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
            return;
        }
        /* In factory mode, firmware is downloaded in the image list sequence. All images stored in factory
           mode can be downloaded in one-click mode. */
        if (g_offlineDownloadHandle.currentFactoryFileCnt < tempFactoryImageList.count) {
            g_offlineDownloadHandle.currentLibImage =
                tempFactoryImageList.libInfo[g_offlineDownloadHandle.currentFactoryFileCnt];
            g_offlineDownloadHandle.currentFactoryFileCnt++;
        }
    } else {
        /* Obtains the information about the image to be downloaded. */
        int ret = TargetLibCurrentImageGet(&g_offlineDownloadHandle.currentLibImage,
                                           &g_offlineDownloadHandle.currentImageIndex);
        if (ret != TARGET_LIB_OK) {
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
            return;
        }
        tempCnt = TargetImageLoadCntGet(g_offlineDownloadHandle.currentImageIndex);
        /* Check whether the number of download times exceeds the limit. */
        if (tempCnt >= g_offlineDownloadHandle.currentLibImage.info.maxLoads &&
            g_offlineDownloadHandle.currentLibImage.info.maxLoads != 0) {
            DapLinkStatusGet(&status);
            status.imageUpgradeStatus = IMAGE_UPGREADE_MAX_LOAD_ERROR;
            DapLinkStatusSet(&status);
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
            return;
        }
    }
    /* Check whether the file is damaged. */
    if (g_offlineDownloadHandle.currentLibImage.fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
            DapLinkStatusGet(&status);
            status.imageUpgradeStatus = IMAGE_UPGRADE_IMAGE_ERROR;
            DapLinkStatusSet(&status);
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
            return;
    }
    g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_SELECT_ALGO;
    /* Turn on On-Demand Erase Configuration */
    flash_manager_set_page_erase(true);
}

/**
  * @brief Select algo processing function.
  * @retval None
  */
static void SelectAlgoHandler(void)
{
    int32_t index;
    DapLinkStatus status;
    /* Obtains the index of the currently selected algorithm. */
    index = TargetLibAlgoIndexGetFormFlashInfo(g_offlineDownloadHandle.currentLibImage.info);
    if (index < 0) {
        g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
        g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_SELECT_ALGO_FAIL;
        /* Display status */
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_GET_ALGO_FAIL;
        DapLinkStatusSet(&status);
        return;
    }
    /* Flash burning algorithm configuration */
    if (TargetAlgoConfig(index) != TARGET_ALGO_PARSE_OK) {
        g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
        g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_SELECT_ALGO_FAIL;

        /* Display status */
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_GET_ALGO_FAIL;
        DapLinkStatusSet(&status);
        return;
    }

    g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_LOADING;
}

/**
  * @brief Download Processing Initialization.
  * @retval true or false
  */
static bool LoadingHandlerInit(void)
{
    DapLinkStatus status;
    int32_t clockLevel;
    const flash_intf_t *currentIntf = NULL;

    g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_LOADLIG;
    if (g_board_info.target_cfg) {
        region_info_t *region = g_board_info.target_cfg->flash_regions;
        for (; region->start != 0 || region->end != 0; ++region) {
            /* Version 1 and later versions add the burning start address
               of the target board to the file header. */
            if (g_offlineDownloadHandle.currentLibImage.info.version > 0) {
                g_offlineDownloadHandle.currentDownloadAdrr = \
                    g_offlineDownloadHandle.currentLibImage.info.targetAddr;
            } else {
                g_offlineDownloadHandle.currentDownloadAdrr = region->start;
            }

            if (kRegionIsDefault == region->flags) {
                break;
            }
        }
        /* Configure the current flash operation algorithm. */
        currentIntf = flash_intf_target;
    }
    /* Connect and initialize the target chip. */
    if (flash_manager_init(currentIntf) != ERROR_SUCCESS) {
        clockLevel = SwdJtagClockLevelGet();
        /* Speed Down Retry */
        if (clockLevel < SWD_JTAG_CLOCK_LEVEL_9) {
            clockLevel++;
            SwJtagClockLevelLevelSet(clockLevel);
            flash_manager_uninit();
        } else {
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
            g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_LOAD_FAIL;

            /* Display status */
            DapLinkStatusGet(&status);
            status.imageUpgradeStatus = IMAGE_UPGRADE_TARGET_CONNECT_FAIL;
            DapLinkStatusSet(&status);
        }
        return false;
    }
    /* Initialize the flash memory for storing firmware on the target chip and configure the
       burning start address and end address of the target chip. */
    Init(0, 0, 0);
    g_offlineDownloadHandle.currentDataAdrr = g_offlineDownloadHandle.currentLibImage.startAddr;
    g_offlineDownloadHandle.currentDownloadDataEndAddr = g_offlineDownloadHandle.currentLibImage.startAddr +
                                                            g_offlineDownloadHandle.currentLibImage.info.fileSize;
    g_loadingInitialized = true;
    return true;
}

/**
  * @brief Speed Down Retry.
  * @retval None
  */
static void LoadingSpeedDownRerty(int32_t clockLevel)
{
    DapLinkStatus status;
    int32_t level = clockLevel + 1; /* Reduce the rate by 1 level. */
    if (clockLevel < SWD_JTAG_CLOCK_LEVEL_9) {
        SwJtagClockLevelLevelSet(level);
        flash_manager_uninit();
        UnInit(0);
        g_loadingInitialized = false;
    } else {
        g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
        g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_LOAD_FAIL;

        /* Display status */
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_WRITE_FAIL;
        DapLinkStatusSet(&status);
    }
}

/**
  * @brief Download Processing Function.
  * @retval None
  */
static void LoadingHandler(void)
{
    DapLinkStatus status;
    int32_t clockLevel;
    errno_t rc = EOK;
    if ((g_offlineDownloadHandle.currentLibImage.info.regioninfo & 0xFFFF0000) == TARGET_LIB_SEC_REGION) {
        SysSoftResetFlagSet(false);
    }
    if (!g_loadingInitialized) {
        /* Download Initial Configuration */
        if (!LoadingHandlerInit()) {
            return;
        }
    } else {
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_WRITE_RUNNING;
        DapLinkStatusSet(&status);
        rc = memset_s(g_downloadBuf, sizeof(g_downloadBuf), 0xFF, sizeof(g_downloadBuf));
        if (rc != EOK) {
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
             /* Display status */
            DapLinkStatusGet(&status);
            status.imageUpgradeStatus = IMAGE_UPGRADE_GET_IMAGE_FAIL;
            DapLinkStatusSet(&status);
            return;
        }
        /* Configure the current download size. */
        g_offlineDownloadHandle.currentDownloadSize = Min(sizeof(g_downloadBuf),
                                                          (g_offlineDownloadHandle.currentDownloadDataEndAddr -
                                                           g_offlineDownloadHandle.currentDataAdrr));
        /* Read the firmware of the target chip from the flash memory. */
        if (FlashRead(g_offlineDownloadHandle.currentDataAdrr,
                      g_offlineDownloadHandle.currentDownloadSize, g_downloadBuf) != 0) {
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
            g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_LOAD_FAIL;

            /* Display status */
            DapLinkStatusGet(&status);
            status.imageUpgradeStatus = IMAGE_UPGRADE_GET_IMAGE_FAIL;
            DapLinkStatusSet(&status);
            return;
        }
        /* Burn the firmware of the target chip. */
        if (flash_manager_data(g_offlineDownloadHandle.currentDownloadAdrr, g_downloadBuf,
                               g_offlineDownloadHandle.currentDownloadSize) != ERROR_SUCCESS) {
            clockLevel = SwdJtagClockLevelGet();
            /* Burning failed. Reduce the speed and try again. */
            LoadingSpeedDownRerty(clockLevel);
            return;
        }
        /* Segmentation processing */
        g_offlineDownloadHandle.currentDataAdrr += g_offlineDownloadHandle.currentDownloadSize;
        g_offlineDownloadHandle.currentDownloadAdrr += g_offlineDownloadHandle.currentDownloadSize;
        if (g_offlineDownloadHandle.currentDataAdrr >= g_offlineDownloadHandle.currentDownloadDataEndAddr) {
            /* After the download is complete, enter the verification phase. */
            flash_manager_uninit();
            g_offlineDownloadState = OFFLINE_DOWNLOAD_VERIFI;
        }
    }
}

/**
  * @brief Check the source image file.
  * @retval true or false
  */
static bool CheckSoureImage(void)
{
    int32_t currentIndex;
    DapLinkStatus status;
    /* Initial configuration of CRC check processing */
    UserCrcInit(0, g_offlineDownloadHandle.currentLibImage.info.fileCrc32);
    /* Check the CRC value of the source file to determine whether the source file is damaged. */
    if (CheckCrcFromFlash(flash_intf_iap_protected,\
        g_offlineDownloadHandle.currentLibImage.startAddr,\
        g_offlineDownloadHandle.currentLibImage.info.fileSize) !=USER_CRC_CHECK_SUCCESS) {
        if (FactoryStatusGet() == FACTORY_FLAG_IS_SET) {
            currentIndex = \
                FactoryLibImageIndexGetFormStartAddr(g_offlineDownloadHandle.currentLibImage.startAddr);
            if (currentIndex < 0) {
                return false;
            }
            /* Flag corrupted file */
            FactoryLibFileStatusChange(TARGET_LIB_TYPE_IMAGE, currentIndex, TARGET_LIB_FILE_STATUS_BAD);
        } else {
            currentIndex = \
                TargetLibImageIndexGetFormStartAddr(g_offlineDownloadHandle.currentLibImage.startAddr);
            if (currentIndex < 0) {
                return false;
            }
            /* Flag corrupted file */
            TargetLibFileStatusChange(TARGET_LIB_TYPE_IMAGE, currentIndex, TARGET_LIB_FILE_STATUS_BAD);
        }
        /* Display status */
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_IMAGE_ERROR;
        DapLinkStatusSet(&status);
        return false;
    }
    return true;
}

/**
  * @brief Check the source algo file.
  * @retval true or false
  */
static bool CheckSoureAlgo(void)
{
    DapLinkStatus status;
    int32_t currentIndex;

    if (TargetLibCurrentAlgoGet(&g_offlineDownloadHandle.currentLibAlgo) != TARGET_LIB_OK) {
        /* Display status */
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_ALGO_ERROR;
        DapLinkStatusSet(&status);
        return false;
    }
    /* Initial configuration of CRC check processing */
    UserCrcInit(0, g_offlineDownloadHandle.currentLibAlgo.info.fileCrc32);
    /* Check the CRC value of the source file to determine whether the source file is damaged. */
    if (CheckCrcFromFlash(flash_intf_iap_protected,\
        g_offlineDownloadHandle.currentLibAlgo.startAddr,\
        g_offlineDownloadHandle.currentLibAlgo.info.fileSize) !=USER_CRC_CHECK_SUCCESS) {
        currentIndex = TargetLibAlgoIndexGetFormStartAddr(g_offlineDownloadHandle.currentLibAlgo.startAddr);
        /* Flag corrupted file */
        TargetLibFileStatusChange(TARGET_LIB_TYPE_ALGO, currentIndex, TARGET_LIB_FILE_STATUS_BAD);
        /* Display status */
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_ALGO_ERROR;
        DapLinkStatusSet(&status);
        return false;
    }
    return true;
}

/**
  * @brief Verification Processing Configuration
  * @param verifyStartAddr Check Start Address
  * @retval None
  */
static void VerifyHandleConfig(uint32_t *verifyStartAddr)
{
    if (g_board_info.target_cfg) {
        region_info_t *region = g_board_info.target_cfg->flash_regions;
        for (; region->start != 0 || region->end != 0; ++region) {
            /* Version 1 and later versions add the burning start address
               of the target board to the file header. */
            if (g_offlineDownloadHandle.currentLibImage.info.version > 0) {
                *verifyStartAddr = g_offlineDownloadHandle.currentLibImage.info.targetAddr;
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
  * @brief check processing function.
  * @retval None
  */
static void VerifyHandler(void)
{
    DapLinkStatus status;
    const flash_intf_t *currentIntf = NULL;
    uint32_t verifyStartAddr = 0;
    uint32_t checkValue;
    uint32_t checkLen;
    int32_t clockLevel;

    /* Display status */
    DapLinkStatusGet(&status);
    status.imageUpgradeStatus = IMAGE_UPGRADE_VERIFY_RUNNING;
    DapLinkStatusSet(&status);
    /* Verification Parameter Configuration */
    VerifyHandleConfig(&verifyStartAddr);
    /* Configure the current flash operation algorithm. */
    currentIntf = flash_intf_target;
    checkLen = g_offlineDownloadHandle.currentLibImage.info.fileSize;
    checkValue = g_offlineDownloadHandle.currentLibImage.info.fileCrc32;

    UserCrcInit(0, checkValue);
    /* Verify the target file. */
    if (CheckCrcFromFlash(currentIntf, verifyStartAddr, checkLen) != USER_CRC_CHECK_SUCCESS) {
        clockLevel = SwdJtagClockLevelGet();
        /* Speed Down Retry */
        if (clockLevel < SWD_JTAG_CLOCK_LEVEL_9) {
            clockLevel++;
            SwJtagClockLevelLevelSet(clockLevel);
        } else {
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_ERROR;
            g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_VERIFI_FAIL;

            /* Check soruce image */
            if (!CheckSoureImage()) {
                return;
            }
            /* Check soruce algo */
            if (!CheckSoureAlgo()) {
                return;
            }
            /* Display status */
            DapLinkStatusGet(&status);
            status.imageUpgradeStatus = IMAGE_UPGRADE_VERIFY_FAIL;
            DapLinkStatusSet(&status);
        }
        return;
    }

    g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_DONE;
    g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_VERIFI_OK;
}

/**
  * @brief Done processing function.
  * @retval None
  */
static void DoneHandler(void)
{
    DapLinkStatus status;
    FactoryImageHandle tempFactoryImageList;

    UnInit(0);
    /* Factory mode and normal mode are handled differently. */
    if (FactoryStatusGet() == FACTORY_FLAG_IS_SET) {
        if (g_offlineDownloadHandle.currentLibImage.info.maxLoads != 0) {
            FactoryImageLoadCntAdd(g_offlineDownloadHandle.currentFactoryFileCnt - 1);
        }
        FactoryImageListGet(&tempFactoryImageList);
        /* Check whether all files are downloaded. */
        if (tempFactoryImageList.count > g_offlineDownloadHandle.currentFactoryFileCnt) {
            g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_OPEN;
            return;
        }
    } else {
        if (g_offlineDownloadHandle.currentLibImage.info.maxLoads != 0) {
            TargetImageLoadCntAdd(g_offlineDownloadHandle.currentImageIndex);
        }
    }
    /* Restoring the Default Download Configuration */
    g_downloadStartFlag = false;
    g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_IDLE;
    g_offlineDownloadDisplayState = OFFLINE_DOWNLOAD_DISPLAY_LOAD_OK;

    /* Display status */
    DapLinkStatusGet(&status);
    status.imageUpgradeStatus = IMAGE_UPGRADE_SUCCESS;
    DapLinkStatusSet(&status);
}

/**
  * @brief Error processing function.
  * @retval None
  */
static void ErrorHandler(void)
{
    flash_manager_uninit();
    UnInit(0);
    g_downloadStartFlag = false;
    g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_IDLE;
}

/**
  * @brief Offline burning initialization.
  * @retval Success or Failure Result
  */
int32_t OfflineDownLoadInit(void)
{
    int32_t index;
    TargetAlgoHandle temptargetAlgoInfo;

    g_offlineDownloadState = OFFLINE_DOWNLOAD_STATE_IDLE;
    g_downloadStartFlag = false;
    /* Read the algorithm list. */
    ConfigAlgoListRead(&temptargetAlgoInfo, sizeof(temptargetAlgoInfo));
    if (temptargetAlgoInfo.count == 0 || temptargetAlgoInfo.count > TARGET_LIB_ALGO_MAX_NUM) {
        return OFFLINE_DOWNLOAD_ERROR;
    }
    /* Obtains the index of the currently configured algorithm. */
    index = ConfigCurrentAlgoIndexRead();
    TargetAlgoConfig(index);
    return OFFLINE_DOWNLOAD_OK;
}

/**
  * @brief Offline burning deinitialization.
  * @retval OFFLINE_DOWNLOAD_OK
  */
int32_t OfflineDownLoadDeinit(void)
{
    return OFFLINE_DOWNLOAD_OK;
}

/**
  * @brief Processing function of the offline burning task.
  * @retval None
  */
void OfflineDownLoadHandler(void)
{
    switch (g_offlineDownloadState) {
        case OFFLINE_DOWNLOAD_STATE_IDLE:
            /* The program is idle most of the time. As long as
               there is no download task, the state machine is set back to idle. */
            IdleHandler();
            break;
        case OFFLINE_DOWNLOAD_STATE_OPEN:
            /* Download Preparation Phase */
            SysSoftResetFlagSet(true);
            OpenHandler();
            break;
        case OFFLINE_DOWNLOAD_STATE_SELECT_ALGO:
            /* Configuration algorithm phase */
            SelectAlgoHandler();
            break;
        case OFFLINE_DOWNLOAD_STATE_LOADING:
            /* Multipart download processing phase */
            LoadingHandler();
            break;
        case OFFLINE_DOWNLOAD_VERIFI:
            /* Verification phase */
            VerifyHandler();
            break;
        case OFFLINE_DOWNLOAD_STATE_DONE:
            DoneHandler();
            SysSoftResetFlagSet(true);
            break;
        case OFFLINE_DOWNLOAD_STATE_ERROR:
            /* Error Handling */
            ErrorHandler();
            SysSoftResetFlagSet(true);
            break;
        default:
            break;
    }
}

/**
  * @brief Start Offline Burning.
  * @retval Success or Failure Result
  */
int32_t OfflineDownLoadStart(void)
{
    DapLinkStatus status;
    if (g_downloadStartFlag) {
        return OFFLINE_DOWNLOAD_ERROR;
    }
    /* Set the start flag. */
    g_downloadStartFlag = true;
    DapLinkStatusGet(&status);
    status.imageUpgradeStatus = IMAGE_UPGRADE_START;
    DapLinkStatusSet(&status);
    /* By default, the download starts at the maximum rate. */
    SwJtagClockLevelLevelSet(0);
    return OFFLINE_DOWNLOAD_OK;
}

/**
  * @brief Stop Offline Burning.
  * @retval OFFLINE_DOWNLOAD_OK
  */
int32_t OfflineDownLoadStop(void)
{
    return OFFLINE_DOWNLOAD_OK;
}

/**
  * @brief Entire erase configuration
  * @param startAddr Start address of the erase partition
  * @retval None
  */
static void EraseChipConfig(uint32_t *startAddr)
{
    region_info_t *region = g_board_info.target_cfg->flash_regions;
    for (; region->start != 0 || region->end != 0; ++region) {
        if (kRegionIsDefault == region->flags) {
            *startAddr = region->start;
            break;
        }
    }
}

/**
  * @brief Erase chip handle.
  * @param startAddr Erase Start Address
  * @retval Success or Failure Result
  */
static int32_t OfflineEraseChipHandle(uint32_t startAddr)
{
    int ret;
    int32_t clockLevel;
    const flash_intf_t *currentIntf  = flash_intf_target;

    clockLevel = SwdJtagClockLevelGet();
    while (1) {
        if (clockLevel <= SWD_JTAG_CLOCK_LEVEL_9) {
            SwJtagClockLevelLevelSet(clockLevel);
            clockLevel++;
        } else {
            break;
        }
        /* Initialize the target chip. */
        if (ERROR_SUCCESS != currentIntf->init()) {
            currentIntf->uninit();
            continue;
        }

        /* Configure and load the operation algorithm of the target chip. */
        if (currentIntf->flash_algo_set) {
            if (ERROR_SUCCESS != currentIntf->flash_algo_set(startAddr)) {
                currentIntf->uninit();
                continue;
            }
        }

        /* Erases the target chip. If the chip fails to be erased, the system retries
        the chip based on the configured retry times. */
        for (uint32_t i = 0; i < OFFLINE_ERASE_TYR_TIME; i++) {
            ret = currentIntf->erase_chip();
            if (ret == ERROR_SUCCESS) {
                return OFFLINE_DOWNLOAD_OK;
            }
        }
    }
    return OFFLINE_DOWNLOAD_ERROR_ERASE_FAIL;
}

/**
  * @brief Erases the entire partition of the target chip based on the algorithm index.
  * @param index Algorithm file index
  * @retval Success or Failure Result
  */
int32_t OfflineEraseChip(int32_t index)
{
    DapLinkStatus status;
    int ret;
    uint32_t startAddr = 0;
    const flash_intf_t *currentIntf = NULL;

    if (index < 0 || index >= TARGET_LIB_ALGO_MAX_NUM) {
        return OFFLINE_DOWNLOAD_ERROR;
    }

    if (TargetAlgoConfig(index) != TARGET_ALGO_PARSE_OK) {
        /* Display status */
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_GET_ALGO_FAIL;
        DapLinkStatusSet(&status);
        return OFFLINE_DOWNLOAD_ERROR_SELECT_ALGO;
    }
    /* Entire erase configuration */
    if (g_board_info.target_cfg) {
        EraseChipConfig(&startAddr);
        /* Configure the current flash operation algorithm. */
        currentIntf = flash_intf_target;
    } else {
        DapLinkStatusGet(&status);
        status.imageUpgradeStatus = IMAGE_UPGRADE_GET_ALGO_FAIL;
        DapLinkStatusSet(&status);
        return OFFLINE_DOWNLOAD_ERROR_SELECT_ALGO;
    }
    ret = OfflineEraseChipHandle(startAddr);
    /* Deinitialize the target chip. */
    currentIntf->uninit();
    return ret;
}

/**
  * @brief Obtaining the Display Status of Offline Download.
  * @retval OfflineDownLoadDisplayState
  */
OfflineDownLoadDisplayState OfflineDownLoadDisplayStateGet(void)
{
    return g_offlineDownloadDisplayState;
}

/**
  * @brief Offline Download Display Status Configuration.
  * @param state Offline burning status
  * @retval OFFLINE_DOWNLOAD_OK
  */
int32_t OfflineDownLoadDisplayStateSet(OfflineDownLoadDisplayState state)
{
    g_offlineDownloadDisplayState = state;
    return OFFLINE_DOWNLOAD_OK;
}

/**
  * @brief In OfflineDownLoad.
  * @param none
  * @retval download flag
  */
bool IsOfflineDownLoad(void)
{
    return g_downloadStartFlag;
}