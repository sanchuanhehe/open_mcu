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
  * @file    proc.c
  * @author  MCU Driver Team
  * @brief   display process
  */
#include <string.h>
#include "securec.h"
#include "res_en.h"
#include "res_zh.h"
#include "display.h"
#include "util.h"
#include "oled.h"
#include "target_lib_manager.h"
#include "offline_download.h"
#include "status.h"
#include "DAP_vendor_ex.h"
#include "factory_manager.h"
#include "offline_sys_config.h"
#include "user_crc.h"
#include "menu_tbl.h"
#include "target_algo_parse.h"
#include "config_storage_update.h"
#include "var_monitor_process.h"
#include "swd_jtag_config.h"
#include "proc.h"

char *g_hwVersion[] = {
    "NULL",
    "NULL",
    "NULL",
    "VER.B 16M",
    "NULL",
    "NULL",
    "VER.B 10M",
    "VER.A 10M"
};

DeviceConfig devConfig = {
    .enableMultiCopies = 1,
    .swVersion = {0},
    .hwVersion = HARDWARE_VERSION,
    .enableHsSampling = 0,
    .enableFactory = 0,
    .sn = "HISI001ABCDE",
};

#define MSG_LINE_SIZE  LED_LINE_LEN
#define IMAGE_ALGO_NAME_EXT_SIZE  35
#define DIGITAL_HEX_MSG_LEN  11
#define DIGITAL_RATE_MSG_LEN 12
#define DAP_MSG_LINES   4
#define CRC_MSG_LINES   4
#define DAP_DATA_RATE_CHARS 20
#define PROC_NAME_MAX_LEN   (TARGET_LIB_IMAGE_NAME_MAX_LEN + IMAGE_ALGO_NAME_EXT_SIZE)

#define IMAGE_UPGRADE_FAIL_MSG_MAX_LEN 24

typedef enum {
    DYN_ITEM_SELECT,
    DYN_ITEM_DELETE,
    DYN_ITEM_SHOW,
    FACTORY_ITEM_SELECT,
    DYN_ITEM_NUM,
} DynItemType;

static Item g_imageDynItem[DYN_ITEM_NUM][TARGET_LIB_IMAGE_MAX_NUM + 1];
static Item g_algoDynItem[DYN_ITEM_NUM][TARGET_LIB_ALGO_MAX_NUM + 1];
static Item g_imageAlgoDynItem[DYN_ITEM_NUM][TARGET_LIB_IMAGE_MAX_NUM + TARGET_LIB_ALGO_MAX_NUM + 3];
static char g_imageName[TARGET_LIB_IMAGE_MAX_NUM][TARGET_LIB_IMAGE_NAME_MAX_LEN + IMAGE_ALGO_NAME_EXT_SIZE];
static char g_algoName[TARGET_LIB_IMAGE_MAX_NUM][TARGET_LIB_IMAGE_NAME_MAX_LEN + IMAGE_ALGO_NAME_EXT_SIZE];
static Item g_imageUnMatchDynItem[TARGET_LIB_IMAGE_MAX_NUM + 1];
static Item g_imageRestrictedDynItem[TARGET_LIB_IMAGE_MAX_NUM + 1];
static Item g_eraseListItem[TARGET_LIB_ALGO_MAX_NUM + 1];
static bool g_loadChkSucc = false;
char g_imageUpgradeFailMsg[IMAGE_UPGRADE_FAIL_MSG_MAX_LEN + 1];

static bool FormatConnectRate(unsigned int rate, char *const rateStr, unsigned int length);
static bool ConnectStatusShow(Frame *frame, TargetConnectInfo *connectInfo);
static void ImageStoreStatusShow(Frame *frame, IMAGE_STORE_STATUS status);
static void ImageUpgradeStatusShow(Frame *frame, IMAGE_UPGRADE_STATUS status);
static void AlgoStoreStatusShow(Frame *frame, ALGORITHM_STORE_STATUS status);
static void FirmwareUpgradeStatusShow(Frame *frame, FIRMWARE_UPGRADE_STATUS status);
static void ImageSelectedProc(Frame *frame);
static void ImageDeleteProc(Frame *frame);
static void AlgoSelectedProc(Frame *frame);
static void AlgoDeleteProc(Frame *frame);
static void ImageDeleteStatusShow(const Frame *frame, IMAGE_DELETE_STATUS status);
static bool DapSamplingStatusShow(const Frame *frame, const DapDebugInfo *dbgInfo);
static void AlgoDeleteStatusShow(const Frame *frame, ALGORITHM_DELETE_STATUS status);
static bool CreateDynAlgoListFrame(Frame *frame, Item *dynItems, SelectedProc selectProc);
static bool CreateDynImageListFrame(Frame *frame, Item *dynItems, SelectedProc selectProc);
static void WelcomeMsg(const Frame *frame);
static bool HasValidImage(int imageCnt);
static bool HasValidAlgo(int algoCnt);
static void CreateFactoryImageAndAlgoFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc,
                                           SelectedProc algoSelectProc);
static void CreateDynEraseListFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc);
static void FactoryImageDeleteProc(Frame *frame);
static bool GreateImageDownloadDetailInfo(char *const name, uint32_t nameLen, uint32_t maxLoads, uint32_t loadCnt);
static void FactoryImageListDummyProc(Frame *frame);
static void SetImageUpgradeFailMsg(const char *msg);
static char *GetImageUpgradeFailMsg(void);
static void ImageUpgradeFailPeriodShow(const Frame *frame);

#define KHZ (1000)
#define MHZ (KHZ * KHZ)

/**
  * @brief Debug Frame Process, base on the Daplink Status
  * @retval None
  */
void DbgFrameProc(Frame *frame)
{
    DapLinkStatus status;
    DapLinkStatusGet(&status); /* Get daplink status */

    /* What type of status is changed? Invoke the corresponding function. */
    switch (status.statusChangeMask) {
        case STATUS_CHANGE_MASK_CONNECT_STATUS:
            ConnectStatusShow(frame, &status.connectInfo);
            break;
        case STATUS_CHANGE_MASK_IMAGE_STORE_STATUS:
            ImageStoreStatusShow(frame, status.imageStoreStatus);
            status.statusChangeMask &= ~STATUS_CHANGE_MASK_IMAGE_STORE_STATUS; /* Clear image store status */
            break;
        case STATUS_CHANGE_MASK_IMAGE_UPGRADE_STATUS:
            ImageUpgradeStatusShow(frame, status.imageUpgradeStatus);
            status.statusChangeMask &= ~STATUS_CHANGE_MASK_IMAGE_UPGRADE_STATUS; /* Clear image upgrade status */
            break;
        case STATUS_CHANGE_MASK_ALGO_STORE_STATUS:
            AlgoStoreStatusShow(frame, status.algoStoreStatus);
            status.statusChangeMask &= ~STATUS_CHANGE_MASK_ALGO_STORE_STATUS; /* Clear algo store status */
            break;
        case STATUS_CHANGE_MASK_FW_UPGRADE_STATUS:
            FirmwareUpgradeStatusShow(frame, status.fwUpgradeStatus);
            status.statusChangeMask &= ~STATUS_CHANGE_MASK_FW_UPGRADE_STATUS;
            break;
        case STATUS_CHANGE_MASK_DAP_SAMPLING_STATUS:
            DapSamplingStatusShow(frame, &status.dapDbgInfo);
            status.statusChangeMask &= ~STATUS_CHANGE_MASK_DAP_SAMPLING_STATUS;
            break;
        default:
            WelcomeMsg(frame); /* Default： welcome */
            break;
    }
    DapLinkStatusSet(&status); /* Set daplink status */
    return;
}

/**
  * @brief Show Welcome message
  * @retval None
  */
static void WelcomeMsg(const Frame *frame)
{
    char *msg[] = MULTI_LANGUAGE(WELCOME);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief Formate Data and Time
  * @retval None
  */
static void SetDateAndTimeStr(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    char *p = SOFTWARE_VERSION;
    rc = strncpy_s(buf, len, p, len - 1);
    if (rc != EOK) {
        return;
    }
    buf[len - 1] = 0;
}

/**
  * @brief Proc Init
  * @retval None
  */
void ProcInit(void)
{
    errno_t rc = EOK;
    /* Get Factory, hs-sampling and powerSupply from flash */
    devConfig.enableFactory = (FactoryStatusGet() == FACTORY_FLAG_IS_SET) ? true : false;
    devConfig.enableHsSampling = GetPerformanceTest();
    devConfig.enablePowerSupply = (SysExternalPowerStatusGet() == SYS_EXTERNAL_POWER_ON) ? true : false;

    /* Get board id and software version */
    uint32_t boardId = SysBoardIdGet();
    rc = strcpy_s(devConfig.hwVersion, VER_MAX_LEN, g_hwVersion[boardId & 0x7]);
    if (rc != EOK) {
        return;
    }
    SetDateAndTimeStr(devConfig.swVersion, sizeof(devConfig.swVersion));
}

/**
  * @brief firmware upgrade popup
  * @retval None
  */
void FirmwareUpgradeMsgPopup(Frame *frame)
{
    DapLinkStatus status;
    DapLinkStatusGet(&status);
    if (status.statusChangeMask == STATUS_CHANGE_MASK_FW_UPGRADE_STATUS) {
        /* Refresh upgrade status */
        FirmwareUpgradeStatusShow(frame, status.fwUpgradeStatus);
        return;
    }
    /* show upgrading */
    char *msg[] = MULTI_LANGUAGE(FW_UPGRADE_MSG);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief get more image popup with title
  * @retval None
  */
static void GetMoreImageMsgPopupWithTitle(const Frame *frame, const char *title)
{
    Language lang;

    lang = GetWinLanguage();
    const char *msg[1];
    msg[0] = GET_MULTILANGE_STR(GET_MORE_IMAGE, lang);
    FramePopMsgWithTitle(frame, title, msg, sizeof(msg) / sizeof(msg[0]), ALIGN_LEFT);
}

/**
  * @brief get more image popup
  * @retval None
  */
void GetMoreImageMsgPopup(Frame *frame)
{
    Language lang = GetWinLanguage();
    char *title = GET_MULTILANGE_STR(END_OF_LIST, lang);
    GetMoreImageMsgPopupWithTitle(frame, title);
}

/**
  * @brief no image popup
  * @retval None
  */
void NoImageMsgPopup(Frame *frame)
{
    if (frame->lineNumber > frame->staticItemNum) {
        /* At least one image is available, return */
        return;
    }
    /* Show Message with title style, title=message */
    Language lang = GetWinLanguage();
    char *title = GET_MULTILANGE_STR(NO_IMAGE, lang);
    GetMoreImageMsgPopupWithTitle(frame, title);
}

/**
  * @brief get more algo popup with title
  * @retval None
  */
static void GetMoreAlgoMsgPopupWithTitle(const Frame *frame, const char *title)
{
    Language lang;
    lang = GetWinLanguage();
    const char *msg[1];
    msg[0] = GET_MULTILANGE_STR(GET_MORE_ALGO, lang);
    FramePopMsgWithTitle(frame, title, msg, sizeof(msg) / sizeof(msg[0]), ALIGN_LEFT);
}

/**
  * @brief get flash algo popup with title
  * @retval None
  */
static void GetFlashAlgoMsgPopupWithTitle(const Frame *frame, const char *title)
{
    Language lang;
    lang = GetWinLanguage();
    const char *msg[1];
    msg[0] = GET_MULTILANGE_STR(GET_FLASH_ALGO, lang);
    FramePopMsgWithTitle(frame, title, msg, sizeof(msg) / sizeof(msg[0]), ALIGN_LEFT);
}

/**
  * @brief get more algo popup
  * @retval None
  */
void GetMoreAlgoMsgPopup(Frame *frame)
{
    Language lang = GetWinLanguage();
    char *title = GET_MULTILANGE_STR(END_OF_LIST, lang);
    GetMoreAlgoMsgPopupWithTitle(frame, title);
}

/**
  * @brief no algo popup
  * @retval None
  */
void NoAlgoMsgPopup(Frame *frame)
{
    if (frame->lineNumber > frame->staticItemNum) {
        /* At least one algo is available, return */
        return;
    }
    Language lang = GetWinLanguage();
    char *title = GET_MULTILANGE_STR(NO_ALGO, lang);
    GetMoreAlgoMsgPopupWithTitle(frame, title);
}

/**
  * @brief no flash algo popup
  * @retval None
  */
void NoFlashAlgoMsgPopup(Frame *frame)
{
    if (frame->lineNumber > frame->staticItemNum) {
        /* At least one flash algo is available, return */
        return;
    }
    Language lang = GetWinLanguage();
    char *title = GET_MULTILANGE_STR(NO_FLASH_ALGO, lang);
    GetFlashAlgoMsgPopupWithTitle(frame, title);
}

/**
  * @brief recovery factory mode
  * @retval None
  */
void RecoveryFactoryMsgPopup(Frame *frame)
{
    DapLinkStatus status;
    DapLinkStatusGet(&status);
    /* Check algo delete status, if not success, show reseting */
    if (status.statusChangeMask == STATUS_CHANGE_MASK_ALGO_DELETE_STATUS) {
        if (status.algoDeleteStatus != ALGORITHM_DELETE_SUCCESS) {
            char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(DEBUGGER_RST);
            FramePopMsg(frame, msg[GetWinLanguage()]);
        }
    }
    /* Algo delete success, show reset success */
    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(DEBUGGER_RST_SUCC);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief image upgrade popup
  * @retval None
  */
void ImageUpgradeMsgPopup(Frame *frame)
{
    DapLinkStatus status;
    DapLinkStatusGet(&status);
    if (status.statusChangeMask == STATUS_CHANGE_MASK_IMAGE_UPGRADE_STATUS) {
        ImageUpgradeStatusShow(frame, status.imageUpgradeStatus);
        status.statusChangeMask &= ~STATUS_CHANGE_MASK_IMAGE_UPGRADE_STATUS;
        DapLinkStatusSet(&status);
        return;
    }
    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(IMAGE_UPGRADEING);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief error image upgrade popup
  * @retval None
  */
void ErrorImageUpgradeMsgPopup(Frame *frame)
{
    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(TARGET_IMAGE_ERROR);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief image delete popup
  * @retval None
  */
void ImageDeleteMsgPopup(Frame *frame)
{
    DapLinkStatus status;

    DapLinkStatusGet(&status);
    if (status.statusChangeMask == STATUS_CHANGE_MASK_IMAGE_DELETE_STATUS) {
        ImageDeleteStatusShow(frame, status.imageDeleteStatus);
        return;
    }
    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(DELETEING);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief algo delete popup
  * @retval None
  */
void AlgoDeleteMsgPopup(Frame *frame)
{
    DapLinkStatus status;

    DapLinkStatusGet(&status);
    if (status.statusChangeMask == STATUS_CHANGE_MASK_ALGO_DELETE_STATUS) {
        AlgoDeleteStatusShow(frame, status.algoDeleteStatus);
        return;
    }
    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(DELETEING);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief error algo select popup
  * @retval None
  */
void ErrorAlgoSelectedMsgPopup(Frame *frame)
{
    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(TARGET_ALGO_ERROR);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief factory mode image or algo delete Popup
  * @retval None
  */
void FactoryDeleteMsgPopup(Frame *frame)
{
    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(DELETE_SUCCESS);
    FramePopMsg(frame, msg[GetWinLanguage()]);
}

/**
  * @brief Erase Chip Popup
  * @retval None
  */
void EraseChipMsgPopup(Frame *frame)
{
    int algoIdx = frame->userData;
    Language lang = GetWinLanguage();

    char *msg[LANGUAGE_NUM] = MULTI_LANGUAGE(ERASEING);
    FramePopMsg(frame, msg[lang]);

    TargetEraseList algoHandle;
    int ret = TargetLibEraseListGet(&algoHandle, sizeof(algoHandle));
    if ((ret != TARGET_LIB_OK) || (algoIdx >= algoHandle.count)) {
        return;
    }

    const char *eraseMsg[2]; // 2 msg, 1 for name, 2 for result
    eraseMsg[0] = algoHandle.eraseList[algoIdx].name;
    if (OfflineEraseChip(algoHandle.eraseList[algoIdx].index) == OFFLINE_DOWNLOAD_OK) {
        eraseMsg[1] = GET_MULTILANGE_STR(ERASE_SUCCESS, lang);
    } else {
        eraseMsg[1] = GET_MULTILANGE_STR(ERASE_FAIL, lang);
    }
    FramePopMsgWithTitle(frame, 0, eraseMsg, sizeof(eraseMsg) / sizeof(eraseMsg[0]), ALIGN_MID);
}

static void FactoryNoImageAlgoMsgPopup(Frame *frame, int imageCnt, int algoCnt)
{
    Language lang = GetWinLanguage();
    const char *title;
    const char *msg[1];
    if (!HasValidImage(imageCnt) && !HasValidAlgo(algoCnt)) {
        title = GET_MULTILANGE_STR(MISS_IMAGE_ALGO, lang);
        msg[0] = GET_MULTILANGE_STR(GET_MORE_IMAGE_ALGO, lang);
    } else if (!HasValidImage(imageCnt)) {
        title = GET_MULTILANGE_STR(MISS_IMAGE, lang);
        msg[0] = GET_MULTILANGE_STR(GET_MORE_IMAGE, lang);
    } else {
        title = GET_MULTILANGE_STR(MISS_ALGO, lang);
        msg[0] = GET_MULTILANGE_STR(GET_MORE_ALGO, lang);
    }
    FramePopMsgWithTitle(frame, title, msg, sizeof(msg)/sizeof(msg[0]), ALIGN_LEFT);
    return;
}

static void FactoryImageAlgoErrorMsgPopup(Frame *frame, int imageStatus, int algoStatus)
{
    Language lang = GetWinLanguage();
    char *title;
    const char *msg[1];
    if ((imageStatus == TARGET_LIB_FILE_STATUS_BAD) && (algoStatus == TARGET_LIB_FILE_STATUS_BAD)) {
        title = GET_MULTILANGE_STR(IMAGE_ALGO_ERROR, lang);
        msg[0] = GET_MULTILANGE_STR(GET_MORE_IMAGE_ALGO, lang);
    } else if (imageStatus == TARGET_LIB_FILE_STATUS_BAD) {
        title = GET_MULTILANGE_STR(IMAGE_ERROR, lang);
        msg[0] = GET_MULTILANGE_STR(GET_MORE_IMAGE, lang);
    } else {
        title = GET_MULTILANGE_STR(ALGO_ERROR, lang);
        msg[0] = GET_MULTILANGE_STR(GET_MORE_ALGO, lang);
    }
    FramePopMsgWithTitle(frame, title, msg, sizeof(msg)/sizeof(msg[0]), ALIGN_LEFT);
    return;
}

static void FactoryImageAlgoNoMatchMsgPopup(Frame *frame)
{
    frame = frame;
    Frame *popupFrame = GetFactoryImageAlgoNoMatchFrame();
    UpdateActiveFrame(popupFrame);
}

static void FactoryImageRestrictedMsgPopup(Frame *frame)
{
    frame = frame;
    Frame *popupFrame = GetFactoryImageRestrictedFrame();
    UpdateActiveFrame(popupFrame);
}

/**
  * @brief Image upgrade message popup
  * @retval None
  */
void FactoryImageUpgradeMsgPopup(Frame *frame)
{
    FactoryImageHandle imageHandle;

    FactoryImageListGet(&imageHandle);
    if (!HasValidImage(imageHandle.count)) {
        FactoryNoImageAlgoMsgPopup(frame, imageHandle.count, 1);
        return;
    }

    if (g_loadChkSucc == false) {
        FactoryImageRestrictedMsgPopup(frame);
        return;
    }

    FactoryAlgoNoMatchList noMatchList;
    if (FactoryAlgoMatchCheck(&noMatchList)) {
        FactoryImageAlgoNoMatchMsgPopup(frame);
        return;
    }
    ImageUpgradeMsgPopup(frame);
}

/**
  * @brief Flash protect off
  * @retval None
  */
void FlashProtectOffMsgPopup(Frame *frame)
{
    Language lang = GetWinLanguage();
    char *msg = GET_MULTILANGE_STR(FLASH_PROTECT_OFF_RUNING, lang);
    FramePopMsg(frame, msg);

    if (SysUnlockSecurity() == true) {
        msg = GET_MULTILANGE_STR(FLASH_PROTECT_OFF_SUCC, lang);
    } else {
        msg = GET_MULTILANGE_STR(FLASH_PROTECT_OFF_FAIL, lang);
    }
    FramePopMsg(frame, msg);
}

/**
  * @brief Show CRC value
  * @retval true or false
  */
static bool CrcCheckShowCrc(Frame *frame, FactoryCrcStatus *status)
{
    char *title = NULL;
    char msgBody[CRC_MSG_LINES][MSG_LINE_SIZE] = {0};
    char hex[DIGITAL_HEX_MSG_LEN] = {'0', 'x'};
    char len[DIGITAL_RATE_MSG_LEN] = {0};
    char *p = len;
    const char *msg[CRC_MSG_LINES];
    Language lang = GetWinLanguage();
    /* Get title */
    if ((status->results == USER_CRC_CHECK_SUCCESS) && (status->preCrc == status->currentCrc)) {
        title = GET_MULTILANGE_STR(CRC_CHK_SUCC, lang);
    } else {
        title = GET_MULTILANGE_STR(CRC_CHK_FAIL, lang);
    }
    int i = 0;  /* Avoid using magic words directly, code check */
    char *pHex = &hex[strlen("0x")];  // skip 2 bytes of "0x"
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, "    ") != EOK) {
        return false;
    }
    /* Expect CRC */
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, GET_MULTILANGE_STR(CRC_EXPECT, lang)) != EOK) {
        return false;
    }
    util_write_hex32(pHex, status->preCrc);
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, hex) != EOK) {
        return false;
    }
    /* Actual CRC */
    if (strcat_s(msgBody[++i], MSG_LINE_SIZE, "    ") != EOK) {
        return false;
    }
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, GET_MULTILANGE_STR(CRC_ACTUAL, lang)) != EOK) {
        return false;
    }
    util_write_hex32(pHex, status->currentCrc);
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, hex) != EOK) {
        return false;
    }
    /* CRC length */
    if (strcat_s(msgBody[++i], MSG_LINE_SIZE, "    ") != EOK) {
        return false;
    }
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, GET_MULTILANGE_STR(CRC_CHK_LENGTH, lang)) != EOK) {
        return false;
    }
    util_write_uint32(p, status->checkLen);
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, p) != EOK) {
        return false;
    }
    /* CRC offset in image */
    if (strcat_s(msgBody[++i], MSG_LINE_SIZE, "    ") != EOK) {
        return false;
    }
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, GET_MULTILANGE_STR(CRC_CHK_ADDR, lang)) != EOK) {
        return false;
    }
    util_write_hex32(pHex, status->checkStartAddr);
    if (strcat_s(msgBody[i], MSG_LINE_SIZE, hex) != EOK) {
        return false;
    }

    for (i = 0; i < sizeof(msg) / sizeof(msg[0]); ++i) {
        msg[i] = msgBody[i];
    }
    /* show title and crc items */
    FramePopMsgWithTitle(frame, title, msg, sizeof(msg)/sizeof(msg[0]), ALIGN_LEFT);
    return true;
}

/**
  * @brief CRC Error Popup
  * @retval None
  */
static void CrcCheckErrorStatusPopup(Frame *frame, FactoryCrcStatus *status)
{
    Language lang = GetWinLanguage();
    char *msg = 0;
    /* init message */
    switch (status->results) {
        case FACTORY_CRC_CHECK_RES_NOT_CONNECT:
            msg = GET_MULTILANGE_STR(TARGET_CONNECTED_FAIL, lang); /* Connection failed. */
            break;
        case FACTORY_CRC_CHECK_RES_FAIL:
            msg = GET_MULTILANGE_STR(TARGET_VERIFY_FAIL, lang);   /* Verification  failed. */
            break;
        case FACTORY_CRC_CHECK_RES_READ_FAIL:
            msg = GET_MULTILANGE_STR(GET_IMAGE_FAIL, lang);       /* Failed to obtain the image. */
            break;
        case FACTORY_CRC_CHECK_RES_BAD_FILE:
            msg = GET_MULTILANGE_STR(IMAGE_ERROR, lang);          /* The image is incorrect. */
            break;
        case FACTORY_CRC_CHECK_RES_ALGO_ERROR:
            msg = GET_MULTILANGE_STR(TARGET_RUN_ALGO_FAIL, lang); /* Failed to run the algorithm. */
            break;
        case FACTORY_CRC_CHECK_RES_NO_ALGO:
            msg = GET_MULTILANGE_STR(TARGET_GET_ALGO_FAIL, lang); /* Failed to obtain the algorithm. */
            break;
        default:
            break;
    }
    /* popup msg */
    if (msg) {
        FramePopMsg(frame, msg);
    }
}

/**
  * @brief CRC Check result popup
  * @retval None
  */
void CrcCheckMsgPopup(Frame *frame)
{
    FactoryCrcStatus status;
    FactoryImageHandle imageHandle;
    int ret;
    int imageIdx = frame->userData;
    Language lang = GetWinLanguage();

    FactoryImageListGet(&imageHandle);
    if (imageHandle.libInfo[imageIdx].fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
        /* if image is bad, don't check alog and force it's normal */
        FactoryImageAlgoErrorMsgPopup(frame, imageHandle.libInfo[imageIdx].fileStatus, TARGET_LIB_FILE_STATUS_NORMAL);
        return;
    }
    /* if image is ok, show verify, crc check maybe take some seconds or one minute */
    FramePopMsg(frame, GET_MULTILANGE_STR(TARGET_VERIFYING, lang));

    /* Do Crc check */
    ret = FactoryCrcget(&status, imageIdx);
    if (ret != FACTORY_STATUS_OK) { /* fail */
        FramePopMsg(frame, GET_MULTILANGE_STR(GET_IMAGE_FAIL, lang));
        return;
    }

    if ((status.results == FACTORY_CRC_CHECK_RES_SUCCESS) || (status.results == FACTORY_CRC_CHECK_RES_ERROR)) {
        /* crc check done, show result */
        CrcCheckShowCrc(frame, &status);
    } else {
        /* other crc check status, popup */
        CrcCheckErrorStatusPopup(frame, &status);
    }
}

/**
  * @brief Restory Factory Frame
  * @retval None
  */
void RestoreFactory(Frame *frame)
{
    ConfigRestoreFactory();
}

/**
  * @brief Generate dynmaic itmes
  * @retval None
  */
static void CreateDynItem(Item *item, char *filename, const SelectedProc proc, Frame *linkFrame, unsigned int userData)
{
    Item fileItem;
    for (int i = 0; i < sizeof(fileItem.text) / sizeof(fileItem.text[0]); ++i) {
        fileItem.text[i] = filename; /* Chinese and English use the same filename */
    }
    fileItem.itemSelectedProc = proc;
    fileItem.linkFrame = linkFrame;
    fileItem.getInfo = 0;
    fileItem.userData = userData;
    *item = fileItem;
}

/**
  * @brief copy static items to dynamic itmes
  * @retval None
  */
static unsigned int CopyStaticItem(Item *dstItem, const Frame *frame)
{
    unsigned int i = 0;
    for (i = 0; i < frame->staticItemNum; ++i) {
        dstItem[i] = frame->pItem[i];
    }
    return i;
}

/**
  * @brief combine file name and index
  * @retval true or false
  */
static bool Translate2ItemNameByFileName(char *itemName, const char *filename, int idx)
{
    errno_t rc = EOK;
    int2str(itemName, idx);
    rc = strcat_s(itemName, PROC_NAME_MAX_LEN, ".");
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(itemName, PROC_NAME_MAX_LEN, filename);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Create selected images Frame
  * @retval None
  */
void CreateSelectImageListFrame(Frame *frame)
{
    CreateDynImageListFrame(frame, g_imageDynItem[DYN_ITEM_SELECT], ImageSelectedProc);
}

/**
  * @brief Create delete images Frame
  * @retval None
  */
void CreateDeleteImageListFrame(Frame *frame)
{
    CreateDynImageListFrame(frame, g_imageDynItem[DYN_ITEM_DELETE], ImageDeleteProc);
}

/**
  * @brief Create show images Frame
  * @retval None
  */
void CreateShowImageListFrame(Frame *frame) /* for factory mode */
{
    CreateDynImageListFrame(frame, g_imageDynItem[DYN_ITEM_SHOW], NULL);
}

/**
  * @brief Create factory images Frame
  * @retval None
  */
void CreateFactoryImageListFrame(Frame *frame)
{
    CreateDynImageListFrame(frame, g_imageDynItem[FACTORY_ITEM_SELECT], FactoryImageListDummyProc);
}

/**
  * @brief Create Dynamic images Frame
  * @retval true or false
  */
static bool CreateDynImageListFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc)
{
    TargetImageHandle imageHandle;
    int idx = 0;
    errno_t rc = EOK;
    Language lang = GetWinLanguage();
    /* Obtains images in factory or non-factory mode. */
    if (devConfig.enableFactory) {
        FactoryImageListGet((FactoryImageHandle *)((void *)&imageHandle));
    } else {
        TargetLibImageListGet(&imageHandle);
    }
    idx = CopyStaticItem(&dynItems[idx], frame); /* Copy Static Items first */
    Frame *nextFrame;
    SelectedProc proc;
    for (int i = 0; i < imageHandle.count; ++i, ++idx) {
        memset_s(g_imageName[i], sizeof(g_imageName[i]), 0, sizeof(g_imageName[i]));
        char *p = g_imageName[i];
        /* Add an index before the file name. */
        Translate2ItemNameByFileName(p, imageHandle.libInfo[i].name, i + 1);
        proc = selectedProc;
        nextFrame = frame->dynItemSelectFrame; /* Jump frame after confirmation */
        if (imageHandle.libInfo[i].fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
            rc = strcat_s(p, sizeof(g_imageName[i]), GET_MULTILANGE_STR(FILE_ERROR, lang));
            if (rc != EOK) {
                return false;
            }
            if (selectedProc == ImageSelectedProc) {
                nextFrame = frame->dynFailItemSelectFrame; /* show fail frame */
                proc = 0;
            }
        } else {
            if (selectedProc != FactoryImageListDummyProc) {
                /* Need add load/max load information behind image name */
                uint32_t maxLoads = imageHandle.libInfo[i].info.maxLoads;
                uint32_t loadCnt = TargetImageLoadCntGet(i);
                GreateImageDownloadDetailInfo(p, sizeof(g_imageName[0]), maxLoads, loadCnt);
            }
        }
        CreateDynItem(&dynItems[idx], p, proc, nextFrame, idx - frame->staticItemNum);
    }
    /* Update frame */
    frame->lineNumber = idx;
    frame->pItem = dynItems;
    return true;
}

/**
  * @brief Show images and algo Frame
  * @retval None
  */
void CreateShowImageAndAlgoListFrame(Frame *frame) /* for factory mode */
{
    CreateFactoryImageAndAlgoFrame(frame, g_imageAlgoDynItem[DYN_ITEM_SHOW], NULL, NULL);
}

/**
  * @brief Delete images and algo Frame
  * @retval None
  */
void CreateDeleteImageAndAlgoListFrame(Frame *frame)
{
    CreateFactoryImageAndAlgoFrame(frame, g_imageAlgoDynItem[DYN_ITEM_DELETE], FactoryImageDeleteProc, AlgoDeleteProc);
}

/**
  * @brief Create images download detail information
  * @retval true or false
  */
static bool GreateImageDownloadDetailInfo(char *const name, uint32_t nameLen, uint32_t maxLoads, uint32_t loadCnt)
{
    errno_t rc = EOK;
    if (maxLoads == 0) {
        return false;
    }
    Language lang = GetWinLanguage();
    char number[11] = {0};  // 11 is the max digits number of max value of unsigned int
    char *p = number;
    rc = strcat_s(name, nameLen, "(");
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(name, nameLen,  GET_MULTILANGE_STR(DOWNLOAD, lang));
    if (rc != EOK) {
        return false;
    }
    util_write_uint32(p, loadCnt); /* download times */
    rc = strcat_s(name, nameLen, p);
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(name, nameLen, GET_MULTILANGE_STR(TIMES, lang));
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(name, nameLen, "/");
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(name, nameLen, GET_MULTILANGE_STR(PERIMIT, lang));
    if (rc != EOK) {
        return false;
    }
    memset_s(p, sizeof(number), 0, sizeof(number));
    util_write_uint32(p, maxLoads); /* max permit times */
    rc = strcat_s(name, nameLen, p);
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(name, nameLen, GET_MULTILANGE_STR(TIMES, lang));
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(name, nameLen, ")");
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Create Factory images items
  * @retval true or false
  */
static bool GreateFactoryImageFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc, int *index)
{
    FactoryImageHandle imageHandle;
    Language lang = GetWinLanguage();
    int idx = 0;
    errno_t rc = EOK;

    FactoryImageListGet(&imageHandle);
    idx = CopyStaticItem(&dynItems[idx], frame);
    Frame *nextFrame = frame->dynItemSelectFrame; /* Display frame if items is selected */
    if (HasValidImage(imageHandle.count)) { /* valid images */
        CreateDynItem(&dynItems[idx++], GET_MULTILANGE_STR(IMAGE, lang), 0, 0, 0);
        for (int i = 0; i < imageHandle.count; ++i, ++idx) {
            memset_s(g_imageName[i], sizeof(g_imageName[i]), 0, sizeof(g_imageName[i]));
            Translate2ItemNameByFileName(g_imageName[i], imageHandle.libInfo[i].name, i + 1);
            if (imageHandle.libInfo[i].fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
                rc = strcat_s(g_imageName[i], sizeof(g_imageName[i]), GET_MULTILANGE_STR(FILE_ERROR, lang));
            } else {
                /* Add extern info of images behind the file name */
                uint32_t maxLoads = imageHandle.libInfo[i].info.maxLoads;
                uint32_t loadCnt = FactoryImageLoadCntGet(i);
                GreateImageDownloadDetailInfo(g_imageName[i], sizeof(g_imageName[i]), maxLoads, loadCnt);
            }
            if (rc != EOK) {
                return false;
            }
            CreateDynItem(&dynItems[idx], g_imageName[i], selectedProc, nextFrame, i);
        }
    } else { /* No valid image */
        CreateDynItem(&dynItems[idx++], GET_MULTILANGE_STR(NO_IMAGE, lang), 0, 0, 0);
    }
    *index = idx;
    return true;
}

/**
  * @brief Create Factory Algo items
  * @retval true or false
  */
static bool GreateFactoryAlgoFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc, int *index)
{
    FactoryAlgoHandle algoHandle;
    Language lang = GetWinLanguage();
    int idx = *index;
    errno_t rc = EOK;
    Frame *nextFrame = frame->dynItemSelectFrame; /* Display frame if items is selected */

    FactoryAlgoListGet(&algoHandle);
    if (HasValidAlgo(algoHandle.count)) { /* valid algo */
        CreateDynItem(&dynItems[idx++], GET_MULTILANGE_STR(ALGO, lang), 0, 0, 0);
        for (int i = 0; i < algoHandle.count; ++i, ++idx) {
            memset_s(g_algoName[i], sizeof(g_algoName[i]), 0, sizeof(g_algoName[i]));
            Translate2ItemNameByFileName(g_algoName[i], algoHandle.libInfo[i].name, i + 1);
            if (algoHandle.libInfo[i].fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
                /* add error */
                rc = strcat_s(g_algoName[i], sizeof(g_algoName[i]), GET_MULTILANGE_STR(FILE_ERROR, lang));
            }
            if (rc != EOK) {
                return false;
            }
            CreateDynItem(&dynItems[idx], g_algoName[i], selectedProc, nextFrame, i);
        }
    } else { /* No valid algo */
        CreateDynItem(&dynItems[idx++], GET_MULTILANGE_STR(NO_ALGO, lang), 0, 0, 0);
    }
    *index = idx;
    return true;
}

/**
  * @brief Create Factory Image and Algo Frame
  * @retval None
  */
static void CreateFactoryImageAndAlgoFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc,
    SelectedProc algoSelectProc)
{
    int idx = 0;
    GreateFactoryImageFrame(frame, dynItems, selectedProc, &idx); /* out: idx */
    GreateFactoryAlgoFrame(frame, dynItems, algoSelectProc, &idx); /* inout：idx */
    frame->lineNumber = idx;
    frame->pItem = dynItems;
}

/**
  * @brief Show unmatch algo List frame
  * @retval None
  */
void CreateUnMatchAlgoListFrame(Frame *frame)
{
    FactoryAlgoNoMatchList list;
    FactoryAlgoMatchCheck(&list);
    Item *dynItems = &g_imageUnMatchDynItem[0];
    int idx = CopyStaticItem(dynItems, frame);

    /* Generate dynamic items of un match algo names */
    for (int i = 0; i < list.count; ++i, ++idx) {
        memset_s(g_algoName[i], sizeof(g_algoName[i]), 0, sizeof(g_algoName[i]));
        char *p = g_algoName[i];
        char *name = list.algoInfo[i].name;
        Translate2ItemNameByFileName(p, name, i + 1);
        CreateDynItem(&dynItems[idx], p, 0, 0, 0);
    }
    /* Updata frame */
    frame->lineNumber = idx;
    frame->pItem = dynItems;
}

/**
  * @brief Show Restricted Images List
  * @retval None
  */
void CreateImageRestrictedListFrame(Frame *frame)
{
    FactoryRestrictedImageList list;
    FactoryImageLoadCntCheck(&list);  /* Get Image with load times restrict */
    Item *dynItems = &g_imageRestrictedDynItem[0];
    int idx = CopyStaticItem(dynItems, frame);

    /* Generate dynamic items of image names */
    for (int i = 0; i < list.count; ++i, ++idx) {
        memset_s(g_algoName[i], sizeof(g_algoName[i]), 0, sizeof(g_algoName[i]));
        char *p = g_algoName[i];
        char *name = list.imageInfo[i].name;
        Translate2ItemNameByFileName(p, name, i + 1);
        CreateDynItem(&dynItems[idx], p, 0, 0, 0);
    }
    /* Updata frame */
    frame->lineNumber = idx;
    frame->pItem = dynItems;
}

/**
  * @brief Show Algo Select List
  * @retval None
  */
void AlgoSelectFrameProc(Frame *frame)
{
    CreateDynAlgoListFrame(frame, g_algoDynItem[DYN_ITEM_SELECT], AlgoSelectedProc);
}

/**
  * @brief Show Algo Delete List
  * @retval None
  */
void CreateDeleteAlgoListFrame(Frame *frame)
{
    CreateDynAlgoListFrame(frame, g_algoDynItem[DYN_ITEM_DELETE], AlgoDeleteProc);
}

/**
  * @brief Show Algo List
  * @retval None
  */
void CreateShowAlgoListFrame(Frame *frame)
{
    CreateDynAlgoListFrame(frame, g_algoDynItem[DYN_ITEM_SHOW], NULL);
}

/**
  * @brief Genearte Algo List
  * @retval true or false
  */
static bool CreateDynAlgoListFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc)
{
    TargetAlgoHandle algoHandle;
    unsigned int idx = 0;
    Frame *nextFrame;
    SelectedProc proc;
    Language lang = GetWinLanguage();
    errno_t rc = EOK;

    TargetLibAlgoListGet(&algoHandle);
    idx = CopyStaticItem(&dynItems[idx], frame);
    for (int i = 0, algoIdx = 0; i < algoHandle.count; ++i) {
        memset_s(g_algoName[i], sizeof(g_algoName[i]), 0, sizeof(g_algoName[i]));
        proc = selectedProc;
        nextFrame = frame->dynItemSelectFrame;
        /* Add Item Index of file */
        Translate2ItemNameByFileName(g_algoName[algoIdx], algoHandle.libInfo[i].name, idx);
        if (algoHandle.libInfo[i].fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
            rc = strcat_s(g_algoName[algoIdx], sizeof(g_algoName[algoIdx]), GET_MULTILANGE_STR(FILE_ERROR, lang));
            if (rc != EOK) {
                return false;
            }
            if (selectedProc == AlgoSelectedProc) {
                /* If select algo file is bad, show special frame */
                nextFrame = frame->dynFailItemSelectFrame;
                proc = 0;
            }
        }

        if (selectedProc == AlgoSelectedProc) {
            if (algoHandle.libInfo[i].fileStatus == TARGET_LIB_FILE_STATUS_SECURITY) {
                /* Don't show if file status is security */
                continue;
            }
            /* Show (Selected) behind file name */
            if (ConfigCurrentAlgoIndexRead() == i) {
                rc = strcat_s(g_algoName[algoIdx], sizeof(g_algoName[algoIdx]), GET_MULTILANGE_STR(SELECTED, lang));
            }
            if (rc != EOK) {
                return false;
            }
        }
        CreateDynItem(&dynItems[idx], g_algoName[algoIdx], proc, nextFrame, i);
        /* Update item index and algo index */
        ++idx;
        ++algoIdx;
    }
    /* Updata frame items number and dynamic items */
    frame->lineNumber = idx;
    frame->pItem = dynItems;
    return true;
}

/**
  * @brief Create erase file list frame
  * @retval None
  */
void CreateEraseListFrame(Frame *frame)
{
    CreateDynEraseListFrame(frame, g_eraseListItem, NULL);
}

/**
  * @brief Create Dynamic Erase file list
  * @retval None
  */
static void CreateDynEraseListFrame(Frame *frame, Item *dynItems, SelectedProc selectedProc)
{
    TargetEraseList algoHandle;
    int ret = TargetLibEraseListGet(&algoHandle, sizeof(algoHandle));
    if (ret != TARGET_LIB_OK) { /* Get erase list fail and ret */
        return;
    }
    unsigned int idx = 0;
    Frame *nextFrame;
    SelectedProc proc;

    idx = CopyStaticItem(&dynItems[idx], frame);
    /* Copy the file name to items */
    for (int i = 0, algoIdx = 0; i < algoHandle.count; ++i) {
        memset_s(g_algoName[i], sizeof(g_algoName[i]), 0, sizeof(g_algoName[i]));
        proc = selectedProc;
        nextFrame = frame->dynItemSelectFrame;
        Translate2ItemNameByFileName(g_algoName[algoIdx], algoHandle.eraseList[i].name, idx);
        CreateDynItem(&dynItems[idx], g_algoName[algoIdx], proc, nextFrame, i);
        ++idx;
        ++algoIdx;
    }
    /* update the frame line number and dynmaic items pointer */
    frame->lineNumber = idx;
    frame->pItem = dynItems;
}

/**
  * @brief Displayed in Chinese
  * @retval None
  */
void SetLanguageToCN(Frame *frame)
{
    if (GetWinLanguage() != LANGUAGE_CN) {
        SetWinLanguage(LANGUAGE_CN);
        FrameShow(frame);
    }
}

/**
  * @brief Displayed in English
  * @retval None
  */
void SetLanguageToEN(Frame *frame)
{
    if (GetWinLanguage() != LANGUAGE_EN) {
        SetWinLanguage(LANGUAGE_EN);
        FrameShow(frame);
    }
}

/**
  * @brief Set debug interface to SWD
  * @retval None
  */
void SetInterfaceToSwd(Frame *frame)
{
    if (SwdJtagDebugPortGet() != DAP_PORT_SWD) {
        SwdJtagDebugPortSet(DAP_PORT_SWD);
        FrameShow(frame);
    }
}

/**
  * @brief Set debug interface to Jtag
  * @retval None
  */
void SetInterfaceToJtag(Frame *frame)
{
    if (SwdJtagDebugPortGet() != DAP_PORT_JTAG) {
        SwdJtagDebugPortSet(DAP_PORT_JTAG);
        FrameShow(frame);
    }
}

/**
  * @brief Enable Multi Copies and Show
  * @retval None
  */
void EnableMultiCopies(Frame *frame)
{
    if (devConfig.enableMultiCopies != true) {
        devConfig.enableMultiCopies = true;
        FrameShow(frame);
    }
}

/**
  * @brief Disable Multi Copies and Show
  * @retval None
  */
void DisableMultiCopies(Frame *frame)
{
    if (devConfig.enableMultiCopies != false) {
        devConfig.enableMultiCopies = false;
        FrameShow(frame);
    }
}

/**
  * @brief Enable High speed sampling and Show
  * @retval None
  */
void EnableHsSampling(Frame *frame)
{
    if (devConfig.enableHsSampling != true) {
        devConfig.enableHsSampling = true;
        SetPerformanceTest(true);
        FrameShow(frame);
    }
}

/**
  * @brief Disable High speed sampling and Show
  * @retval None
  */
void DisableHsSampling(Frame *frame)
{
    if (devConfig.enableHsSampling != false) {
        devConfig.enableHsSampling = false;
        SetPerformanceTest(false);
        FrameShow(frame);
    }
}

/**
  * @brief Factory Image upgrade
  * @retval None
  */
void EnablePowerSupply(Frame *frame)
{
    if (devConfig.enablePowerSupply != true) {
        devConfig.enablePowerSupply = true;
        SysExternalPowerSet(SYS_EXTERNAL_POWER_ON);
        FrameShow(frame);
    }
}

/**
  * @brief Factory Image upgrade
  * @retval None
  */
void DisablePowerSupply(Frame *frame)
{
    if (devConfig.enablePowerSupply != false) {
        devConfig.enablePowerSupply = false;
        SysExternalPowerSet(SYS_EXTERNAL_POWER_OFF);
        FrameShow(frame);
    }
}

/**
  * @brief Exit Factory Mode
  * @retval None
  */
void ExitFactoryMode(Frame *frame)
{
    frame->pointer = 1; /* 指向工厂模式 */
    devConfig.enableFactory = false;
    FactoryModeExit();
}

/**
  * @brief Entry Factory Mode
  * @retval None
  */
void EntryFactoryMode(Frame *frame)
{
    frame->pointer = 1; /* 始终指向烧录 */
    devConfig.enableFactory = true;
    FactoryModeEnter();
}

/**
  * @brief Deleting All Algorithms and Images in Factory Mode
  * @retval None
  */
void FactoryImageAlgoDelete(Frame *frame)
{
    frame = frame;
    FactoryImageFreeAll();
    FactoryAlgoFreeAll();
}

/**
  * @brief Factory Image upgrade
  * @retval None
  */
void FactoryImageUpgrade(Frame *frame)
{
    FactoryImageHandle imageHandle;

    FactoryImageListGet(&imageHandle);
    if (!HasValidImage(imageHandle.count)) { /* valid check */
        return;
    }
    FactoryRestrictedImageList restrictedList;
    g_loadChkSucc = true;
    /* check image upgrade times */
    if (FactoryImageLoadCntCheck(&restrictedList)) {
        g_loadChkSucc = false;
        return;
    }
    /* Check image and algo match or not */
    FactoryAlgoNoMatchList noMatchList;
    if (FactoryAlgoMatchCheck(&noMatchList)) {
        return;
    }
    OfflineDownLoadStart();
}

/**
  * @brief Deleting All Image in No-Factory
  * @retval None
  */
void DeleteAllImage(Frame *frame)
{
    TargetLibImageDeleteAll();
    FrameShow(frame);
}

/**
  * @brief Deleting All Algorithms in No-Factory
  * @retval None
  */
void DeleteAllAlgo(Frame *frame)
{
    frame = frame;
    TargetLibAlgoDeleteAll();
}

/**
  * @brief Get HiSparkTrace software version
  * @retval true or false
  */
bool GetFWVersion(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if (!buf) {
        return false;
    }
    rc = strncpy_s(buf, len, GetDevConfig()->swVersion, len);
    if (rc !=EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Get HiSparkTrace hardware version
  * @retval true or false
  */
bool GetHWVersion(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if (!buf) {
        return false;
    }
    rc = strncpy_s(buf, len, GetDevConfig()->hwVersion, len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Get HiSparkTrace Serial number
  * @retval true or false
  */
bool GetSN(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if (!buf) {
        return false;
    }
    rc = strncpy_s(buf, len, GetDevConfig()->sn, len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Multi copies enable prompt
  * @retval true or false
  */
bool GetMultiEnableSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((devConfig.enableMultiCopies == false) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Multi copies disable prompt
  * @retval true or false
  */
bool GetMultiDisableSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((devConfig.enableMultiCopies == true) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief High speed sampling enable prompt
  * @retval true or false
  */
bool GetHsSamplingEnableSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((devConfig.enableHsSampling != true) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief High speed sampling disable promptt
  * @retval true or false
  */
bool GetHsSamplingDisableSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((devConfig.enableHsSampling == true) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Power supply enable prompt
  * @retval true or false
  */
bool GetPowerSupplyEnableSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((devConfig.enablePowerSupply != true) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Power supply disable prompt
  * @retval true or false
  */
bool GetPowerSupplyDisableSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((devConfig.enablePowerSupply == true) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Factory Mode Enable prompt
  * @retval true or false
  */
bool GetFactoryModeEnableSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((devConfig.enableFactory != true) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief English selection prompt
  * @retval true or false
  */
bool LanguageEnglishSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    Language lang = GetWinLanguage();
    if ((lang != LANGUAGE_EN) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[lang], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Chinese selection prompt
  * @retval true or false
  */
bool LanguageChineseSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    Language lang = GetWinLanguage();
    if ((lang != LANGUAGE_CN) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[lang], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief SWD selection prompt
  * @retval true or false
  */
bool SwdInterfaceSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((SwdJtagDebugPortGet() != DAP_PORT_SWD) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief JTAG selection prompt
  * @retval true or false
  */
bool JtagInterfaceSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    if ((SwdJtagDebugPortGet() != DAP_PORT_JTAG) || !buf) {
        return false;
    }
    char *msg[] = MULTI_LANGUAGE(SELECTED);
    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Hook function for periodically refreshing the display.
  * @retval None
  */
void DisplayPeriodProcHook(void)
{
    if (IsProgramFailFrame()) {
        /* Periodically refreshes the upgrade failure information (blinking effect). */
        ImageUpgradeFailPeriodShow(GetCurFrame());
    }
}

/**
  * @brief Obtain the configuration of the display screen.
  * @retval Pointer to the configuration structure of the screen.
  */
DeviceConfig *GetDevConfig(void)
{
    return &devConfig;
}

/**
  * @brief Format the connection rate
  * @retval true or false
  */
static bool FormatConnectRate(unsigned int rate, char *const rateStr, unsigned int length)
{
    unsigned int formateRate;
    char buf[MSG_MAX_LEN] = {0};
    errno_t rc = EOK;

    if (rate > MHZ) {
        /* If the speed is greater than MHz, the speed is displayed in MHz. */
        formateRate = rate / MHZ;
        int2str(buf, formateRate);
        formateRate = (rate % MHZ) / KHZ;
        if (formateRate != 0) {
            rc = strcat_s(buf, MSG_MAX_LEN, ".");
            if (rc != EOK) {
                return false;
            }
            int2str(buf + strlen(buf), formateRate);
        }
        rc = strcat_s(buf, MSG_MAX_LEN, "MHz");
        if (rc != EOK) {
            return false;
        }
    } else {
        /* If the speed is less than MHz, the speed is displayed in KHz. */
        formateRate = rate / KHZ;
        int2str(buf, formateRate);
        formateRate = rate % KHZ;
        if (formateRate != 0) {
            rc = strcat_s(buf, MSG_MAX_LEN, ".");
            if (rc != EOK) {
                return false;
            }
            int2str(buf + strlen(buf), formateRate);
        }
        rc = strcat_s(buf, MSG_MAX_LEN, "KHz");
        if (rc != EOK) {
            return false;
        }
    }
    rc = strncpy_s(rateStr, length, buf, length);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Show connect status
  * @retval true or false
  */
static bool ConnectStatusShow(Frame *frame, TargetConnectInfo *connectInfo)
{
    errno_t rc = EOK;
    if (connectInfo->status == TARGET_DISCONNECT) {
        /* Show welcome screen when not connected */
        WelcomeMsg(frame);
        return false;
    }

    /* Show Connect Mode and rate */
    char buf[MSG_MAX_LEN + 1] = {0};
    char rateBuf[MSG_MAX_LEN + 1] = {0};
    char *msg[] = MULTI_LANGUAGE(CONNECTED);
    rc = strncpy_s(buf, sizeof(buf), msg[GetWinLanguage()], strlen(msg[GetWinLanguage()]));
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(buf, sizeof(buf), (SwdJtagDebugPortGet() == DAP_PORT_SWD) ? " SWD " : " JTAG ");
    if (rc != EOK) {
        return false;
    }
    FormatConnectRate(connectInfo->rate, rateBuf, MSG_MAX_LEN - strlen(buf));
    rc = strcat_s(buf, sizeof(buf), rateBuf);
    if (rc != EOK) {
        return false;
    }
    char *p = buf;
    p[MSG_MAX_LEN] = 0; /* make sure it's end with 0 */
    FramePopMsg(frame, p);  /* Popup Message */
    return true;
}

/**
  * @brief Show store status
  * @retval None
  */
static void ImageStoreStatusShow(Frame *frame, IMAGE_STORE_STATUS status)
{
    Language lang;

    /* All Store Status */
    char *msg[IMAGE_STORE_STATUS_NUM][LANGUAGE_NUM] = {
        MULTI_LANGUAGE(IMAGE_STORED),
        MULTI_LANGUAGE(IMAGE_FAIL),
        MULTI_LANGUAGE(MULTI_COPIES_FAIL),
        MULTI_LANGUAGE(TARGET_IMAGE_ERROR),
        MULTI_LANGUAGE(TARGET_IMAGE_ERROR),
        MULTI_LANGUAGE(TARGET_ALGO_ERROR),
        MULTI_LANGUAGE(WELCOME),
    };
    if (status >= IMAGE_STORE_STATUS_NUM) {
        /* Invalid state, return */
        return;
    }
    /* Popup the store status */
    lang = GetWinLanguage();
    FramePopMsg(frame, msg[status][lang]);
}

/**
  * @brief Get Connect rate and copy to buf
  * @retval true or false
  */
static bool GetConnectStatus(char *buf, int len)
{
    char *connRate[2] = MULTI_LANGUAGE(CONNECT_RATE); /* 2 languages */
    char *p = buf;
    char rateBuf[MSG_LINE_SIZE] = {0};
    errno_t rc = EOK;

    rc = strcpy_s, len, (p, (SwdJtagDebugPortGet() == DAP_PORT_SWD) ? "SWD" : "JTAG");
    if (rc != EOK) {
        return false;
    }
    rc = strcat_s(p, len, connRate[GetWinLanguage()]);
    if (rc != EOK) {
        return false;
    }
    FormatConnectRate(SwdJtagClockGet(), rateBuf, len - strlen(buf));
    rc = strcat_s(p, len, rateBuf);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Popup Upgrade success Message
  * @retval None
  */
static void ImageUpgradeSuccessShow(Frame *frame, const char *msg)
{
    /* Power ON LED show upgrade success */
    DownLoadLedOn(frame);

    /* The success message in big font is displayed on the screen. */
    SetGlobalDisaplyaMode(DISPLAY_REVERSE);
    OledClearFromStartYPixel(FONT_SIZE);
    ForceAllLineUsed();
    FramePopMsgWithFont(frame, msg, BIG_FONT_SIZE);
    SetGlobalDisaplyaMode(DISPLAY_NORMAL);
}

/**
  * @brief Popup Upgrade Fail Message
  * @retval None
  */
static void ImageUpgradeFailShow(const Frame *frame, const char *msg)
{
    /* The Fail message in big font is displayed on the screen. */
    OledClearFromStartYPixel(FONT_SIZE);
    ForceAllLineUsed();
    FramePopMsgWithFont(frame, msg, BIG_FONT_SIZE);
}

/**
  * @brief The upgrade failure information blinks periodically.
  * @retval None
  */
static void ImageUpgradeFailPeriodShow(const Frame *frame)
{
    static unsigned int cnt = 0;
    SetGlobalDisaplyaMode((cnt++ & 1) ? DISPLAY_REVERSE : DISPLAY_NORMAL);
    ImageUpgradeFailShow(frame, GetImageUpgradeFailMsg());
}

/**
  * @brief Display image upgrade status.
  * @retval None
  */
static void ImageUpgradeStatusShow(Frame *frame, IMAGE_UPGRADE_STATUS status)
{
    /* all image upgrade status */
    char *msg[IMAGE_UPGRADE_STATUS_NUM][LANGUAGE_NUM] = {
        MULTI_LANGUAGE(IMAGE_UPGRADEING),
        MULTI_LANGUAGE(TARGET_UPGRADE_SUCC),
        MULTI_LANGUAGE(TARGET_CONNECTED_FAIL),
        MULTI_LANGUAGE(TARGET_GET_ALGO_FAIL),
        MULTI_LANGUAGE(TARGET_PROGRAMMING),
        MULTI_LANGUAGE(TARGET_VERIFYING),
        MULTI_LANGUAGE(TARGET_WRITE_FAIL),
        MULTI_LANGUAGE(TARGET_GET_IMAGE_FAIL),
        MULTI_LANGUAGE(TARGET_VERIFY_FAIL),
        MULTI_LANGUAGE(TARGET_IMAGE_ERROR),
        MULTI_LANGUAGE(TARGET_ALGO_ERROR),
        MULTI_LANGUAGE(TARGET_RUN_ALGO_FAIL),
        MULTI_LANGUAGE(WELCOME),
        MULTI_LANGUAGE(TARGET_IMAGE_MAX_LOADS_ERROR),
    };

    if (status >= IMAGE_UPGRADE_STATUS_NUM) {
        return; /* return if status is invalid */
    }

    Language lang = GetWinLanguage();

    if ((status == IMAGE_UPGRADE_WRITE_FAIL) || (status == IMAGE_UPGRADE_VERIFY_FAIL)) {
        /* if upgrade write fail or verify fail, Show special Fail message */
        Frame *tmpFrame = frame;
        if (devConfig.enableFactory) {
            tmpFrame = GetFactoryImageProgramFailFrame();
        } else {
            tmpFrame = GetImageProgramFailFrame();
        }
        /* Switch to the dedicated fail frame for special processing. */
        UpdateActiveFrame(tmpFrame);
        SetImageUpgradeFailMsg(msg[status][lang]);
        return ImageUpgradeFailShow(tmpFrame, msg[status][lang]);
    }

    if (status == IMAGE_UPGRADE_SUCCESS) {
        /* if upgrade success, Show special success message */
        Frame *tmpFrame = frame;
        if (devConfig.enableFactory) {
            /* Switch to the dedicated success frame for special processing. */
            tmpFrame = GetFactoryImageProgramSuccFrame();
            UpdateActiveFrame(tmpFrame);
        }
        return ImageUpgradeSuccessShow(tmpFrame, msg[status][GetWinLanguage()]);
    }
    /* Other status, show as normal */
    FramePopMsg(frame, msg[status][GetWinLanguage()]);
    char msgBuf[MSG_MAX_LEN + 1] = {0};
    GetConnectStatus(msgBuf, sizeof(msgBuf) - 1);
    char *p = msgBuf;
    p[MSG_MAX_LEN] = 0; /* make sure end with 0 */
    FrameShowStatusBar(frame, p);
}

/**
  * @brief Display algo store status.
  * @retval None
  */
static void AlgoStoreStatusShow(Frame *frame, ALGORITHM_STORE_STATUS status)
{
    Language lang;
    /* all algo store status */
    char *msg[ALGORITHM_STORE_STATUS_NUM][LANGUAGE_NUM] = {
        MULTI_LANGUAGE(ALGO_STORED),
        MULTI_LANGUAGE(ALGO_STORED_FAIL),
        MULTI_LANGUAGE(ALGO_STORED_MULTI_COPIES_FAIL),
        MULTI_LANGUAGE(ALGO_CRC_FAIL),
        MULTI_LANGUAGE(WELCOME),
    };
    if (status >= ALGORITHM_STORE_STATUS_NUM) {
        return;
    }
    /* popup status */
    lang = GetWinLanguage();
    FramePopMsg(frame, msg[status][lang]);
}

/**
  * @brief Firmware upgrade status.
  * @retval None
  */
static void FirmwareUpgradeStatusShow(Frame *frame, FIRMWARE_UPGRADE_STATUS status)
{
    Language lang = GetWinLanguage();
    /* all firmware store status */
    char *msg[FIRMWARE_UPGRADE_STATUS_NUM][LANGUAGE_NUM] = {
        MULTI_LANGUAGE(FW_UPGRADEING),
        MULTI_LANGUAGE(FW_UPGRADE_SUCC),
        MULTI_LANGUAGE(FW_CRC_FAIL),
        MULTI_LANGUAGE(WELCOME),
    };
    if (status > FIRMWARE_UPGRADE_STATUS_NUM) {
        return;
    }
    /* popup status */
    FramePopMsg(frame, msg[status][lang]);
}

/**
  * @brief DAP sampline status.
  * @retval true or false
  */
static bool DapSamplingStatusShow(const Frame *frame, const DapDebugInfo *dbgInfo)
{
    Language lang = GetWinLanguage();
    /* all DAP store status */
    char *title[DAP_STATUS_NUM][LANGUAGE_NUM] = {
        MULTI_LANGUAGE(DAP_NOT_SAMPLING),
        MULTI_LANGUAGE(DAP_DEBGU),
        MULTI_LANGUAGE(DAP_SAMPLING),
        MULTI_LANGUAGE(DAP_PC_STOP),
        MULTI_LANGUAGE(DAP_TARGET_STOP),
        MULTI_LANGUAGE(WELCOME),
    };
    if (dbgInfo->status > DAP_STATUS_NUM) {
        return false;
    }

    char msgBuf[DAP_MSG_LINES][MSG_LINE_SIZE + 1] = {0};
    char digits[DAP_MSG_LINES][DAP_DATA_RATE_CHARS] = {0};
    uint8_t i = 0;
    /* Get Total Sampling, error, overflow count */
    util_write_uint64_with_delimiter(digits[i++], dbgInfo->totalSampleCount);
    util_write_uint64_with_delimiter(digits[i++], dbgInfo->totalErrorCount);
    util_write_uint32(digits[i++], dbgInfo->errorRate);
    util_write_uint64_with_delimiter(digits[i++], dbgInfo->totalOverflowCount);
    i = 0;
    /* Format the numbers */
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), GET_MULTILANGE_STR(DAP_SAMPLE_CNT, lang)) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), digits[i]) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), GET_MULTILANGE_STR(TIMES, lang)) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[++i], sizeof(msgBuf[i]), GET_MULTILANGE_STR(DAP_SAMPLE_ERROR, lang)) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), digits[i]) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), GET_MULTILANGE_STR(TIMES, lang)) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[++i], sizeof(msgBuf[i]), GET_MULTILANGE_STR(DAP_ERROR_RATE, lang)) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), digits[i]) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), "%") != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[++i], sizeof(msgBuf[i]), GET_MULTILANGE_STR(DAP_OVERFLOW_CNT, lang)) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), digits[i]) != EOK) {
        return false;
    }
    if (strcat_s(msgBuf[i], sizeof(msgBuf[i]), GET_MULTILANGE_STR(TIMES, lang)) != EOK) {
        return false;
    }
    char *msg[DAP_MSG_LINES];

    /* Each item of data is regarded as an item. */
    for (i = 0; i < sizeof(msg) / sizeof(msg[0]); ++i) {
        msg[i] = msgBuf[i];
    }
    /* Use the DAP status as the header and the count as the display item. */
    FramePopMsgWithTitle(frame, title[dbgInfo->status][lang], (const char **)msg, sizeof(msg) / sizeof(msg[0]),
        ALIGN_LEFT);
    return true;
}

/**
  * @brief Image delete status.
  * @retval None
  */
static void ImageDeleteStatusShow(const Frame *frame, IMAGE_DELETE_STATUS status)
{
    Language lang;
    /* all image delete status */
    char *msg[IMAGE_DELETE_STATUS_NUM][LANGUAGE_NUM] = {
        MULTI_LANGUAGE(DELETEING),
        MULTI_LANGUAGE(DELETE_SUCCESS),
        MULTI_LANGUAGE(DELETE_FAIL),
        MULTI_LANGUAGE(WELCOME),
    };
    if (status >= IMAGE_DELETE_STATUS_NUM) {
        return;
    }
    lang = GetWinLanguage();
    FramePopMsg(frame, msg[status][lang]);
}

/**
  * @brief Algo delete status.
  * @retval None
  */
static void AlgoDeleteStatusShow(const Frame *frame, ALGORITHM_DELETE_STATUS status)
{
    Language lang;
    char *msg[ALGORITHM_DELETE_STATUS_NUM][LANGUAGE_NUM] = {
        MULTI_LANGUAGE(DELETEING),
        MULTI_LANGUAGE(DELETE_SUCCESS),
        MULTI_LANGUAGE(DELETE_FAIL),
        MULTI_LANGUAGE(WELCOME),
    };
    if (status >= ALGORITHM_DELETE_STATUS_NUM) {
        return;
    }
    lang = GetWinLanguage();
    FramePopMsg(frame, msg[status][lang]);
}

/**
  * @brief Image selected process
  * @retval None
  */
static void ImageSelectedProc(Frame *frame)
{
    TargetLibImageSelect(frame->pointer - frame->staticItemNum);
    OfflineDownLoadStart();
}

/**
  * @brief Image delete process
  * @retval None
  */
static void ImageDeleteProc(Frame *frame)
{
    unsigned int imageIdx = frame->userData;
    TargetLibImageDelete(imageIdx);
}

/**
  * @brief Image dummy process, do nothing
  * @retval None
  */
static void FactoryImageListDummyProc(Frame *frame)
{
    frame = frame;
}

/**
  * @brief Factory Image delete process
  * @retval None
  */
static void FactoryImageDeleteProc(Frame *frame)
{
    unsigned int imageIdx = frame->userData;
    FactoryImageFree(imageIdx);
}

/**
  * @brief Factory Algo selected process
  * @retval None
  */
static void AlgoSelectedProc(Frame *frame)
{
    unsigned int algoId = frame->userData;
    TargetAlgoConfig(algoId);
    FrameShow(frame);
}

/**
  * @brief Algo selected process
  * @retval None
  */
static void AlgoDeleteProc(Frame *frame)
{
    unsigned int algoId = frame->userData;
    TargetLibAlgoDelete(algoId);
}

/**
 * @brief Check has valid image
  * @retval true or fail
  */
static bool HasValidImage(int imageCnt)
{
    return ((imageCnt > 0) && (imageCnt <= TARGET_LIB_FACTORY_IMAGE_MAX_NUM));
}

/**
  * @brief Check has valid algo
  * @retval true or fail
  */
static bool HasValidAlgo(int algoCnt)
{
    return ((algoCnt > 0) && (algoCnt <= TARGET_LIB_FACTORY_ALGO_MAX_NUM));
}

/**
  * @brief Stores image upgrade failure information. This information
  * is required by the function that periodically displays messages.
  * @retval true or fail
  */
static void SetImageUpgradeFailMsg(const char *msg)
{
    errno_t ret = EOK;
    /* Recording error information. */
    ret = strncpy_s(g_imageUpgradeFailMsg, IMAGE_UPGRADE_FAIL_MSG_MAX_LEN, msg, sizeof(g_imageUpgradeFailMsg));
    if (ret != EOK) {
        return;
    }
    g_imageUpgradeFailMsg[IMAGE_UPGRADE_FAIL_MSG_MAX_LEN] = 0;
}

/**
  * @brief Get stores image upgrade failure information.
  * @retval true or fail
  */
static char *GetImageUpgradeFailMsg(void)
{
    return g_imageUpgradeFailMsg; /* String of fail message */
}
