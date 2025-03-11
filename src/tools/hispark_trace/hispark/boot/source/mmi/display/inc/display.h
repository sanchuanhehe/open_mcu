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
  * @file    display.h
  * @author  MCU Driver Team
  * @brief   daplink boot display header
  */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stddef.h>
#include "display_common.h"

#define SOFTWARE_VERSION DAPLINK_COMPILE_TIME
#define HARDWARE_VERSION "V1.00"

#define ITEM_LIST_MAX_LEN 128
#define EXT_INFO_MAX_LEN  72

typedef struct ItemST Item;
typedef struct FrameST Frame;

typedef void (*SelectedProc)(void);
typedef unsigned int (*GetExtText)(char *buf, unsigned int len);

typedef enum {
    FRAME_STATIC = 0,
    FRAME_SHOW_MSG,
    FRAME_DYN_ITEMS,
} FrameType;

typedef struct {
    unsigned int width;
    unsigned int high;
} WinSize;

struct ItemST {
    char *text[LANGUAGE_NUM];
    SelectedProc itemSelectedProc;
    GetExtText getInfo;
};

struct FrameST {
    unsigned int pointer;
    unsigned int begin;
    unsigned int winSize;
    size_t       lineNumber;
    size_t       staticItemNum;
    unsigned int rollEn;
    Item *pItem;
    char *name;
};

typedef struct {
    unsigned char name[FILENAME_MAX_LEN];
} FileName;

typedef struct {
    unsigned int width;
    unsigned int high;
    unsigned int showLineMask;
    Language     language;
    Frame        *frame;
} WinInfo;

void FrameShow(const Frame *frame);
Frame *GetCurFrame(void);
Language GetWinLanguage(void);
void FrameMoveUp(void);
void FrameMoveDown(void);
void FrameItemSelect(void);
void PopupMsgStartupFail(void);

#endif