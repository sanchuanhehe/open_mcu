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
  * @file    proc.h
  * @author  MCU Driver Team
  * @brief   Display process Header
  */

#ifndef PROC_H
#define PROC_H

#include "target_lib_manager.h"

#define MSG_MAX_LEN 128
#define SN_MAX_LEN  24
#define VER_MAX_LEN 12

typedef struct {
    unsigned int enableMultiCopies;
    char swVersion[VER_MAX_LEN];
    char hwVersion[VER_MAX_LEN];
    char sn[SN_MAX_LEN];
    unsigned int enableHsSampling;
    unsigned int enablePowerSupply;
    unsigned int enableFactory;
} DeviceConfig;

void DbgFrameProc(Frame *frame);
void FirmwareUpgradeMsgPopup(Frame *frame);
void ImageUpgradeMsgPopup(Frame *frame);
void ErrorImageUpgradeMsgPopup(Frame *frame);
void FactoryImageUpgradeMsgPopup(Frame *frame);
void CrcCheckMsgPopup(Frame *frame);
void GetMoreImageMsgPopup(Frame *frame);
void RestoreFactory(Frame *frame);
void CreateSelectImageListFrame(Frame *frame);
void CreateDeleteImageListFrame(Frame *frame);
void CreateShowImageListFrame(Frame *frame);
void CreateFactoryImageListFrame(Frame *frame);
void AlgoSelectFrameProc(Frame *frame);
void ErrorAlgoSelectedMsgPopup(Frame *frame);
void CreateDeleteAlgoListFrame(Frame *frame);
void CreateShowAlgoListFrame(Frame *frame);
void CreateEraseListFrame(Frame *frame);
void DeleteAllImage(Frame *frame);
void DeleteAllAlgo(Frame *frame);
void SetLanguageToCN(Frame *frame);
void SetLanguageToEN(Frame *frame);
void SetInterfaceToSwd(Frame *frame);
void SetInterfaceToJtag(Frame *frame);
void EnableHsSampling(Frame *frame);
void DisableHsSampling(Frame *frame);
void EnablePowerSupply(Frame *frame);
void DisablePowerSupply(Frame *frame);
void EnableMultiCopies(Frame *frame);
void DisableMultiCopies(Frame *frame);
void GetMoreAlgoMsgPopup(Frame *frame);
void ImageDeleteMsgPopup(Frame *frame);
void AlgoDeleteMsgPopup(Frame *frame);
void NoImageMsgPopup(Frame *frame);
void NoAlgoMsgPopup(Frame *frame);
void NoFlashAlgoMsgPopup(Frame *frame);
bool LanguageEnglishSelected(char *buf, unsigned int len);
bool LanguageChineseSelected(char *buf, unsigned int len);
bool SwdInterfaceSelected(char *buf, unsigned int len);
bool JtagInterfaceSelected(char *buf, unsigned int len);
void RecoveryFactoryMsgPopup(Frame *frame);
bool GetMultiEnableSelected(char *buf, unsigned int len);
bool GetMultiDisableSelected(char *buf, unsigned int len);
bool GetHsSamplingEnableSelected(char *buf, unsigned int len);
bool GetHsSamplingDisableSelected(char *buf, unsigned int len);
bool GetPowerSupplyEnableSelected(char *buf, unsigned int len);
bool GetPowerSupplyDisableSelected(char *buf, unsigned int len);
void FactoryImageAlgoDelete(Frame *frame);
void ProcInit(void);
DeviceConfig *GetDevConfig(void);
void ExitFactoryMode(Frame *frame);
void EntryFactoryMode(Frame *frame);
void FactoryImageUpgrade(Frame *frame);
void FactoryDeleteMsgPopup(Frame *frame);
void CreateShowImageAndAlgoListFrame(Frame *frame);
void CreateDeleteImageAndAlgoListFrame(Frame *frame);
void CreateUnMatchAlgoListFrame(Frame *frame);
void EraseChipMsgPopup(Frame *frame);
void FlashProtectOffMsgPopup(Frame *frame);
void CreateImageRestrictedListFrame(Frame *frame);

#endif