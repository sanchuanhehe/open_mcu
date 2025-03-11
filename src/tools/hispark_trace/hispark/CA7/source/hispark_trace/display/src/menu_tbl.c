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
  * @file    menu_tbl.c
  * @author  MCU Driver Team
  * @brief   daplink boot menu  display process
  */
#include <string.h>
#include "display_common.h"
#include "display.h"
#include "res_en.h"
#include "res_zh.h"
#include "proc.h"

static Frame mainMenuFrame;
static Frame dbgStatusMenuFrame;
static Frame flashMenuFrame;
static Frame flashEraseFrame;
static Frame flashProtectOffFrame;
static Frame settingMenuFrame;
static Frame selectImageMenuFrame;
static Frame deleteImageMenuFrame;
static Frame selectAlgoMenuFrame;
static Frame deleteAlgoMenuFrame;
static Frame enableMultiCopiesMenuFrame;
static Frame softwareUpgradeMenuFrame;
static Frame languageMenuFrame;
static Frame dbgInterfaceMenuFrame;
static Frame imageSelectedFrame;
static Frame imageGetMoreFrame;
static Frame algoSelectedFrame;
static Frame algoDeleteFrame;
static Frame algoDeleteAllFrame;
static Frame imageDeleteFrame;
static Frame imageDeleteAllFrame;
static Frame algoGetMoreFrame;
static Frame factoryResetMenuFrame;
static Frame hsSamplingSimuFrame;
static Frame powerSupplyFrame;
static Frame factoryModeFrame;
static Frame factoryImageSelectedFrame;
static Frame factoryCrcCheckFrame;
static Frame factoryShowImageMenuFrame;
static Frame factoryShowAlgoMenuFrame;
static Frame factoryViewFrame;
static Frame factoryDeleteFrame;
static Frame factoryDeleteImageAlgoMenuFrame;
static Frame factoryDeleteAlgoMenuFrame;
static Frame errImageSelectedFrame;
static Frame errAlgoSelectedFrame;
static Frame fatoryResetFrame;
static Frame factoryImageListFrame;
static Frame factoryImageSelectedSuccFrame;
static Frame factoryImageSelectedFailFrame;
static Frame imageSelectedFailFrame;
static Frame eraseAlgoSelectedFrame;
static Frame flashProtectOffDoneFrame;
static Frame factoryDeleteImageAlgoConfirmMenuFrame;

#define FRAME_INIT_WITH_POINTER(x, NAME, rollEnable, pointerIdx) \
    .pointer = (pointerIdx),            \
    .lineNumber = ARRAY_SIZE(x),        \
    .staticItemNum = ARRAY_SIZE(x),     \
    .begin = 0,                         \
    .pItem = (x),                       \
    .name = (NAME),                     \
    .rollEn = (rollEnable)

#define FRAME_INIT(x, NAME, rollEnable) FRAME_INIT_WITH_POINTER(x, NAME, rollEnable, 0)

#define BACK_ITEM_INIT(LINKFRAME)      \
{                                      \
    .text = MULTI_LANGUAGE(BACK),      \
    .itemSelectedProc = 0,             \
    .linkFrame = &(LINKFRAME),         \
    .getInfo = 0,                      \
    .windowsMenu = 1                   \
}

#define TITLE_ITEM_INIT(titleTxt)      \
{                                      \
    .text = titleTxt,                  \
    .itemSelectedProc = 0,             \
    .linkFrame = NULL,                 \
    .getInfo = 0,                      \
    .title = 1                         \
}

/* 菜单显示元素 */
/* TOP Menu */
static Item mainItems[] = {
    {MULTI_LANGUAGE(DBG_STATUS),     NULL,             &dbgStatusMenuFrame},
    {MULTI_LANGUAGE(FACTORY_MODE),   EntryFactoryMode, &factoryModeFrame},
    {MULTI_LANGUAGE(FLASHER_MODE),   NULL,             &flashMenuFrame  },
    {MULTI_LANGUAGE(FLASHER_ERASE),  NULL,             &flashEraseFrame },
    {MULTI_LANGUAGE(FLASHER_UNLOCK), NULL,             &flashProtectOffFrame},
    {MULTI_LANGUAGE(SETTING),        NULL,             &settingMenuFrame},
};

/* Level 2 */
static Item dbgStatusItems[] = {
    {MULTI_LANGUAGE(MENU),         NULL,          &mainMenuFrame       },
};

static Item flasherItems[] = {
    BACK_ITEM_INIT(mainMenuFrame),
    {MULTI_LANGUAGE(IMAGE_SELECT), NULL,          &selectImageMenuFrame},
    {MULTI_LANGUAGE(IMAGE_DELETE), NULL,          &deleteImageMenuFrame},
    {MULTI_LANGUAGE(ALGO_SELECT),  NULL,          &selectAlgoMenuFrame },
    {MULTI_LANGUAGE(ALGO_DELETE),  NULL,          &deleteAlgoMenuFrame },
};

static Item eraseItems[] = {
    BACK_ITEM_INIT(mainMenuFrame),
};

static Item flashProtectOffItems[] = {
    TITLE_ITEM_INIT(MULTI_LANGUAGE(FLASH_PROTECT_OFF)),
    {MULTI_LANGUAGE(YES),      NULL,        &flashProtectOffDoneFrame},
    {MULTI_LANGUAGE(NO),       NULL,        &mainMenuFrame},
};

static Item flashProtectOffDoneItems[] = {
    BACK_ITEM_INIT(mainMenuFrame),
};

static Item factoryModeItems[] = {
    TITLE_ITEM_INIT(MULTI_LANGUAGE(FACTORY_MODE_TITLE)),
    {MULTI_LANGUAGE(FACTORY_IMAGE_LOAD),  FactoryImageUpgrade,   &factoryImageSelectedFrame},
    {MULTI_LANGUAGE(FACTORY_CRC_CHECK),   NULL,        &factoryImageListFrame},
    {MULTI_LANGUAGE(FACTORY_VIEW),        NULL,        &factoryViewFrame},
    {MULTI_LANGUAGE(FACTORY_DELETE),      NULL,        &factoryDeleteFrame},
    {MULTI_LANGUAGE(FACTORY_EXIT),        ExitFactoryMode,    &mainMenuFrame},
};

static Item settingItems[] = {
    BACK_ITEM_INIT(mainMenuFrame),
    {MULTI_LANGUAGE(LANGUAGE_SELECT), NULL,       &languageMenuFrame},
    {MULTI_LANGUAGE(POWER_SUPPLY),    NULL,       &powerSupplyFrame},
    {MULTI_LANGUAGE(DBG_INTERFACE),   NULL,       &dbgInterfaceMenuFrame},
    {MULTI_LANGUAGE(FW_VER),          NULL,       NULL,  GetFWVersion},
    {MULTI_LANGUAGE(FW_UPGRADE),      NULL,       &softwareUpgradeMenuFrame},
    {MULTI_LANGUAGE(HW_VER),          NULL,       NULL,  GetHWVersion},
    {MULTI_LANGUAGE(SN),              NULL,       NULL,  GetSN},
    {MULTI_LANGUAGE(HS_SAMPLE_SIMU),  NULL,       &hsSamplingSimuFrame},
    {MULTI_LANGUAGE(RESTORE),         NULL,       &factoryResetMenuFrame},
};

static Item settingNoPowerSupplyItems[] = {
    BACK_ITEM_INIT(mainMenuFrame),
    {MULTI_LANGUAGE(LANGUAGE_SELECT),     NULL,       &languageMenuFrame},
    {MULTI_LANGUAGE(OLD_DBG_INTERFACE),   NULL,       &dbgInterfaceMenuFrame},
    {MULTI_LANGUAGE(OLD_FW_VER),          NULL,       NULL,  GetFWVersion},
    {MULTI_LANGUAGE(OLD_FW_UPGRADE),      NULL,       &softwareUpgradeMenuFrame},
    {MULTI_LANGUAGE(OLD_HW_VER),          NULL,       NULL,  GetHWVersion},
    {MULTI_LANGUAGE(OLD_SN),              NULL,       NULL,  GetSN},
    {MULTI_LANGUAGE(OLD_HS_SAMPLE_SIMU),  NULL,       &hsSamplingSimuFrame},
    {MULTI_LANGUAGE(OLD_RESTORE),         NULL,       &factoryResetMenuFrame},
};

/* Level 3 for flasher */
static Item selectImageItems[] = {
    BACK_ITEM_INIT(flashMenuFrame),
};

static Item deleteImageItems[] = {
    BACK_ITEM_INIT(flashMenuFrame),
    {MULTI_LANGUAGE(DELETE_ALL),   NULL, &imageDeleteAllFrame},
};

static Item selectAlgoItems[] = {
    BACK_ITEM_INIT(flashMenuFrame),
};

static Item deleteAlgoItems[] = {
    BACK_ITEM_INIT(flashMenuFrame),
    {MULTI_LANGUAGE(DELETE_ALL),   NULL, &algoDeleteAllFrame},
};

static Item enableMultiCopiesItems[] = {
    BACK_ITEM_INIT(flashMenuFrame),
    {MULTI_LANGUAGE(ON),           EnableMultiCopies,        NULL,  GetMultiEnableSelected},
    {MULTI_LANGUAGE(OFF),          DisableMultiCopies,       NULL,  GetMultiDisableSelected},
};

/* Level 3 for setting */
static Item languageItems[] = {
    BACK_ITEM_INIT(settingMenuFrame),
    {MULTI_LANGUAGE(CHINESE),      SetLanguageToCN, NULL, LanguageChineseSelected},
    {MULTI_LANGUAGE(ENGLISH),      SetLanguageToEN, NULL, LanguageEnglishSelected},
};

static Item dbgInterfaceItems[] = {
    BACK_ITEM_INIT(settingMenuFrame),
    {MULTI_LANGUAGE(SWD),       SetInterfaceToSwd,  NULL, SwdInterfaceSelected},
    {MULTI_LANGUAGE(JTAG),      SetInterfaceToJtag, NULL, JtagInterfaceSelected},
};

static Item factoryResetItems[] = {
    TITLE_ITEM_INIT(MULTI_LANGUAGE(FACTORY_RESET_TITLE)),
    {MULTI_LANGUAGE(YES),      RestoreFactory,        &fatoryResetFrame},
    {MULTI_LANGUAGE(NO),       NULL,  &mainMenuFrame},
};

static Item factoryResetDoneItems[] = {
    BACK_ITEM_INIT(settingMenuFrame),
};

static Item hsSamplingSimuItems[] = {
    BACK_ITEM_INIT(settingMenuFrame),
    {MULTI_LANGUAGE(ON),           EnableHsSampling,        NULL,  GetHsSamplingEnableSelected},
    {MULTI_LANGUAGE(OFF),          DisableHsSampling,       NULL,  GetHsSamplingDisableSelected},
};

static Item powerSupplyItems[] = {
    BACK_ITEM_INIT(settingMenuFrame),
    {MULTI_LANGUAGE(ON),           EnablePowerSupply,        NULL,  GetPowerSupplyEnableSelected},
    {MULTI_LANGUAGE(OFF),          DisablePowerSupply,       NULL,  GetPowerSupplyDisableSelected},
};

static Item softwareUpgradeItems[] = {
    BACK_ITEM_INIT(settingMenuFrame),
};

/* Level 4 */
static Item imageSelectedItems[] = {
    BACK_ITEM_INIT(selectImageMenuFrame),
};

static Item imageGetMoreItems[] = {
    BACK_ITEM_INIT(selectImageMenuFrame),
};

static Item uploadImageItmes[] = {
    BACK_ITEM_INIT(selectImageMenuFrame),
};

static Item imageDeleteItems[] = {
    BACK_ITEM_INIT(deleteImageMenuFrame),
};

static Item imageDeleteDialogItems[] = {
    TITLE_ITEM_INIT(MULTI_LANGUAGE(DELETE_ALL_TITLE)),
    {MULTI_LANGUAGE(YES),      DeleteAllImage,        &imageDeleteFrame},
    {MULTI_LANGUAGE(NO),       NULL,  &deleteImageMenuFrame},
};

static Item selectedAlgoItems[] = {
    BACK_ITEM_INIT(selectAlgoMenuFrame),
};

static Item algoGetMoreItems[] = {
    BACK_ITEM_INIT(selectAlgoMenuFrame),
};

static Item algoDeleteItems[] = {
    BACK_ITEM_INIT(deleteAlgoMenuFrame),
};

static Item algoDeleteDialogItems[] = {
    TITLE_ITEM_INIT(MULTI_LANGUAGE(DELETE_ALL_TITLE)),
    {MULTI_LANGUAGE(YES),      DeleteAllAlgo,        &algoDeleteFrame},
    {MULTI_LANGUAGE(NO),       NULL,  &deleteAlgoMenuFrame},
};

static Item eraseAlgoSelectedItems[] = {
    BACK_ITEM_INIT(flashEraseFrame),
};

static Item uploadAlgoItmes[] = {
    BACK_ITEM_INIT(selectAlgoMenuFrame),
};

/* 描述菜单链接关系和动作 */
static Frame mainMenuFrame = {
    FRAME_INIT(mainItems, "mainMenu", true),
};

static Frame dbgStatusMenuFrame = {
    FRAME_INIT(dbgStatusItems, "dbgMenu", true),
    .popupMsg = DbgFrameProc,
};

static Frame flashMenuFrame = {
    FRAME_INIT(flasherItems, "flashMenu", true),
};

static Frame flashEraseFrame = {
    FRAME_INIT(eraseItems, "eraseMenu", true),
    .genDynItems = CreateEraseListFrame,
    .dynItemSelectFrame = &eraseAlgoSelectedFrame,
    .popupMsg = NoFlashAlgoMsgPopup,
};

static Frame flashProtectOffFrame = {
    FRAME_INIT(flashProtectOffItems, "protectOffMenu", true),
};

static Frame flashProtectOffDoneFrame = {
    FRAME_INIT(flashProtectOffDoneItems, "protectOffDoneMenu", true),
    .popupMsg = FlashProtectOffMsgPopup,
};

static Frame settingMenuFrame = {
    FRAME_INIT(settingItems, "settingMenu", true),
};

/* Level 3 Menu */
/* Level 3 menu for flasher */
static Frame selectImageMenuFrame = {
    FRAME_INIT(selectImageItems, "selectImage", true),
    .genDynItems = CreateSelectImageListFrame,
    .dynItemSelectFrame = &imageSelectedFrame,
    .dynFailItemSelectFrame = &errImageSelectedFrame,
    .popupMsg = NoImageMsgPopup,
};

static Frame deleteImageMenuFrame = {
    FRAME_INIT(deleteImageItems, "deleteImage", true),
    .genDynItems = CreateDeleteImageListFrame,
    .dynItemSelectFrame = &imageDeleteFrame,
};

static Frame selectAlgoMenuFrame = {
    FRAME_INIT(selectAlgoItems, "selectAlgo", true),
    .genDynItems = AlgoSelectFrameProc,
    .dynItemSelectFrame = NULL,
    .dynFailItemSelectFrame = &errAlgoSelectedFrame,
    .popupMsg = NoAlgoMsgPopup,
};

static Frame deleteAlgoMenuFrame = {
    FRAME_INIT(deleteAlgoItems, "deleteAlgo", true),
    .genDynItems = CreateDeleteAlgoListFrame,
    .dynItemSelectFrame = &algoDeleteFrame,
};

static Frame enableMultiCopiesMenuFrame = {
    FRAME_INIT(enableMultiCopiesItems, "enableMulti", true),
};

/* Level 3 Menu */
/* Level 3 menu for factory */
static Item factoryShowImageItems[] = {
    BACK_ITEM_INIT(factoryImageListFrame),
};

static Frame factoryShowImageMenuFrame = {
    FRAME_INIT(factoryShowImageItems, "showImage", true),
    .genDynItems = CreateShowImageListFrame,
    .popupMsg = NoImageMsgPopup,
};

static Item factoryImageItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
};

static Frame factoryErrImageSelectedFrame = {
    FRAME_INIT(factoryImageItems, "ImageSelected", true),
    .popupMsg = ErrorImageUpgradeMsgPopup,
};

static Frame factoryImageListFrame = {
    FRAME_INIT(factoryImageItems, "selectImage", true),
    .genDynItems = CreateFactoryImageListFrame,
    .dynItemSelectFrame = &factoryCrcCheckFrame,
    .popupMsg = NoImageMsgPopup,
};

static Item factoryShowAlgoItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
};

static Frame factoryShowAlgoMenuFrame = {
    FRAME_INIT(factoryShowAlgoItems, "showAlgo", true),
    .genDynItems = CreateShowAlgoListFrame,
    .popupMsg = NoAlgoMsgPopup,
};

/* Level 3 menu for Setting */
static Frame languageMenuFrame = {
    FRAME_INIT(languageItems, "langSet", true),
};

static Frame dbgInterfaceMenuFrame = {
    FRAME_INIT(dbgInterfaceItems, "dbgInterface", true),
};

static Frame factoryResetMenuFrame = {
    FRAME_INIT(factoryResetItems, "recovery", true),
};

static Frame softwareUpgradeMenuFrame = {
    FRAME_INIT(softwareUpgradeItems, "swUpgrade", true),
    .popupMsg = FirmwareUpgradeMsgPopup,
};

static Frame hsSamplingSimuFrame = {
    FRAME_INIT(hsSamplingSimuItems, "HsSampling", true),
};

static Frame powerSupplyFrame = {
    FRAME_INIT(powerSupplyItems, "powerSupply", true),
};

static Frame factoryModeFrame = {
    FRAME_INIT_WITH_POINTER(factoryModeItems, "factoryMode", true, 1),
    .minPointer = 1,
};

static Frame imageSelectedFrame = {
    FRAME_INIT(imageSelectedItems, "ImageSelected", true),
    .popupMsg = ImageUpgradeMsgPopup,
};

static Frame errImageSelectedFrame = {
    FRAME_INIT(imageSelectedItems, "ImageSelected", true),
    .popupMsg = ErrorImageUpgradeMsgPopup,
};

static Frame imageGetMoreFrame = {
    FRAME_INIT(imageGetMoreItems, "GetMoreImage", true),
    .popupMsg = GetMoreImageMsgPopup,
};

static Frame imageDeleteAllFrame = {
    FRAME_INIT(imageDeleteDialogItems, "ImageDelete", true),
};

static Frame imageDeleteFrame = {
    FRAME_INIT(imageDeleteItems, "ImageDelete", true),
    .popupMsg = ImageDeleteMsgPopup,
};

static Frame algoSelectedFrame = {
    FRAME_INIT(selectedAlgoItems, "AlgoSelected", true),
};

static Frame errAlgoSelectedFrame = {
    FRAME_INIT(selectedAlgoItems, "ImageSelected", true),
    .popupMsg = ErrorAlgoSelectedMsgPopup,
};

static Frame algoDeleteFrame = {
    FRAME_INIT(algoDeleteItems, "AlgoDelte", true),
    .popupMsg = AlgoDeleteMsgPopup,
};

static Frame algoDeleteAllFrame = {
    FRAME_INIT(algoDeleteDialogItems, "AlgoDelte", true),
};

static Frame algoGetMoreFrame = {
    FRAME_INIT(algoGetMoreItems, "AlgoGetMore", true),
    .popupMsg = GetMoreAlgoMsgPopup,
};

static Frame fatoryResetFrame = {
    FRAME_INIT(factoryResetDoneItems, "FactoryResetDone", true),
    .popupMsg = RecoveryFactoryMsgPopup,
};

static Item factoryImageSelectedItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
};

static Frame factoryImageSelectedFrame = {
    FRAME_INIT(factoryImageSelectedItems, "factoryImageSelectedFrame", true),
    .popupMsg = FactoryImageUpgradeMsgPopup,
};

static Item factoryImageSelectedSuccItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
    {MULTI_LANGUAGE(CONTINUE_PROGRAM), FactoryImageUpgrade,   &factoryImageSelectedFrame},
};

static Frame factoryImageSelectedSuccFrame = {
    FRAME_INIT_WITH_POINTER(factoryImageSelectedSuccItems, "FactoryImageSelected", true, 1),
};

static Item factoryImageSelectedFailItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
};

static Frame factoryImageSelectedFailFrame = {
    FRAME_INIT_WITH_POINTER(factoryImageSelectedFailItems, "FactoryImageSelected", true, 1),
};

static Item imageSelectedFailItems[] = {
    BACK_ITEM_INIT(selectImageMenuFrame),
};

static Frame imageSelectedFailFrame = {
    FRAME_INIT_WITH_POINTER(imageSelectedFailItems, "ImageSelected", true, 1),
};


static Frame eraseAlgoSelectedFrame = {
    FRAME_INIT(eraseAlgoSelectedItems, "eraseAlgoSelected", true),
    .popupMsg = EraseChipMsgPopup,
};

static Item factoryCrcCheckItems[] = {
    BACK_ITEM_INIT(factoryImageListFrame),
};

static Frame factoryCrcCheckFrame = {
    FRAME_INIT(factoryCrcCheckItems, "FactoryCrcCheck", true),
    .popupMsg = CrcCheckMsgPopup,
    .onlySupportSelected = true,
};

static Item factoryViewItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
};

static Frame factoryViewFrame = {
    FRAME_INIT(factoryViewItems, "FactoryView", true),
    .genDynItems = CreateShowImageAndAlgoListFrame,
};

static Item factoryDeleteItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
    {MULTI_LANGUAGE(FACTORY_DEL_IMAGE_ALGO), NULL,   &factoryDeleteImageAlgoConfirmMenuFrame},
};

static Frame factoryDeleteFrame = {
    FRAME_INIT(factoryDeleteItems, "FactoryDelete", true),
    .genDynItems = CreateDeleteImageAndAlgoListFrame,
    .dynItemSelectFrame = &factoryDeleteImageAlgoMenuFrame,
};

static Item factoryDeleteImageAlgoItems[] = {
    BACK_ITEM_INIT(factoryDeleteFrame),
};

static Frame factoryDeleteImageAlgoMenuFrame = {
    FRAME_INIT(factoryDeleteImageAlgoItems, "ImageAlgoDel", true),
    .popupMsg = FactoryDeleteMsgPopup,
};

static Item factoryDeleteImageAlgoConfirmItems[] = {
    TITLE_ITEM_INIT(MULTI_LANGUAGE(DELETE_ALL_TITLE)),
    {MULTI_LANGUAGE(YES),      FactoryImageAlgoDelete,        &factoryDeleteImageAlgoMenuFrame},
    {MULTI_LANGUAGE(NO),       NULL,  &factoryDeleteFrame},
};

static Frame factoryDeleteImageAlgoConfirmMenuFrame = {
    FRAME_INIT(factoryDeleteImageAlgoConfirmItems, "factoryDelImageAlgoConfirm", true),
};

static Item factoryDeleteImageItems[] = {
    BACK_ITEM_INIT(factoryDeleteFrame),
};

static Item factoryImageAlgoNoMatchFrameItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
    {MULTI_LANGUAGE(IMAGE_ALGO_NO_MATCH_TITLE), NULL,   NULL},
};

static Frame factoryImageAlgoNoMatchFrame = {
    FRAME_INIT(factoryImageAlgoNoMatchFrameItems, "dynFrame", true),
    .genDynItems = CreateUnMatchAlgoListFrame,
};

static Item factoryImageRestrictedItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
    {MULTI_LANGUAGE(IMAGE_RESTRICTED_TITLE), NULL,   NULL},
};

static Frame factoryImageRestrictedFrame = {
    FRAME_INIT(factoryImageRestrictedItems, "restrictedFrame", true),
    .genDynItems = CreateImageRestrictedListFrame,
};

static Item factoryImageAlgoFrameItems[] = {
    BACK_ITEM_INIT(factoryModeFrame),
    {MULTI_LANGUAGE(IMAGE),      NULL,       NULL},
    {MULTI_LANGUAGE(ALGO),       NULL,       NULL},
};

static Frame factoryImageAlgoFrame = {
    FRAME_INIT(factoryImageAlgoFrameItems, "dynFrame", true),
};

/**
  * @brief Get default frame
  * @retval ptr to frame
  */
Frame *GetDefaultFrame(void)
{
    DeviceConfig *cfg = GetDevConfig();
    return cfg->enableFactory ? &factoryModeFrame : &dbgStatusMenuFrame;
}

/**
  * @brief Get Factory Image and Algo not match Frame
  * @retval ptr to frame
  */
Frame *GetFactoryImageAlgoNoMatchFrame(void)
{
    return &factoryImageAlgoNoMatchFrame;
}

/**
  * @brief Get Factory Image Restricte Frame
  * @retval ptr to frame
  */
Frame *GetFactoryImageRestrictedFrame(void)
{
    return &factoryImageRestrictedFrame;
}

/**
  * @brief Get Factory Image Program success Frame pointer
  * @retval ptr to frame
  */
Frame *GetFactoryImageProgramSuccFrame(void)
{
    return &factoryImageSelectedSuccFrame;
}

/**
  * @brief Get Factory Image Program Fail Frame pointer
  * @retval ptr to frame
  */
Frame *GetFactoryImageProgramFailFrame(void)
{
    return &factoryImageSelectedFailFrame;
}

/**
  * @brief Get Image Program Fail Frame pointer
  * @retval ptr to frame
  */
Frame *GetImageProgramFailFrame(void)
{
    return &imageSelectedFailFrame;
}

/**
  * @brief Is Debug Status Frame
  * @retval true： yes,  false: not
  */
bool IsDbgStatusFrame(void)
{
    return GetCurFrame() == &dbgStatusMenuFrame;
}

/**
  * @brief Is FActory Image Program success frame
  * @retval ptr to frame
  */
bool IsFactoryImageProgramSuccFrame(void)
{
    return GetCurFrame() == GetFactoryImageProgramSuccFrame();
}

/**
  * @brief Is Image Program fail frame
  * @retval ptr to frame
  */
bool IsProgramFailFrame(void)
{
    return ((GetCurFrame() == GetFactoryImageProgramFailFrame()) ||
            (GetCurFrame() == GetImageProgramFailFrame()));
}

/**
  * @brief Set Menu Frame without Power supply item
  * @retval No
  */
void SettingFrameItemsWithNoPowerSupply(void)
{
    settingMenuFrame.pItem = settingNoPowerSupplyItems;
    settingMenuFrame.lineNumber = ARRAY_SIZE(settingNoPowerSupplyItems);
    settingMenuFrame.staticItemNum = ARRAY_SIZE(settingNoPowerSupplyItems);
}