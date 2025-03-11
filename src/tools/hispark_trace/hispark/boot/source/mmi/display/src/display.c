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
  * @file    display.c
  * @author  MCU Driver Team
  * @brief   daplink boot display process
  */

#include <stdint.h>
#include <string.h>
#include "securec.h"
#include "oled.h"
#include "menu_tbl.h"
#include "proc.h"
#include "res_en.h"
#include "res_zh.h"
#include "display.h"

static WinInfo g_WinInfo = {
    .width = WIN_WIDTH / (FONT_SIZE / 2),  /* 默认为显示英文字符 */
    .high  = WIN_HEIGHT / FONT_SIZE,
    .showLineMask = 0,
    .language = LANGUAGE_CN,
};

/**
  * @brief Merge Two String to buffer.
  */
static char *MergeStr(char *const buf, unsigned int bufLen, char *part1, const char *part2)
{
    char *p = buf;
    errno_t rc = EOK;
    if (part2 == NULL) { /* if part2 is empty, don't merge */
        return part1;
    }

    rc = strcpy_s(p, bufLen, part1);
    if (rc != EOK) {
        return part1;
    }
    rc = strncat_s(p, bufLen, part2, bufLen - strlen(p));
    if (rc != EOK) {
        return part1;
    }
    return p;
}

/**
  * @brief Get Language
  */
Language GetWinLanguage(void)
{
    return g_WinInfo.language;
}

/**
  * @brief Frame Move Up
  */
void FrameMoveUp(void)
{
    Frame *frame = GetDefaultFrame();

    if (frame->pointer > 0) {
        frame->pointer--;
        if (frame->pointer >= g_WinInfo.high - 1) {
            frame->begin--;  /* if pointer move out of window, frame move up one line */
        }
    } else {
        if (frame->rollEn) { /* roll process */
            unsigned int end = frame->lineNumber;
            frame->pointer = end - 1;
            if (end >= g_WinInfo.high) {
                frame->begin = end - g_WinInfo.high;
            } else {
                frame->begin = 0;
            }
        }
    }
    FrameShow(frame);  /* show frame */
}

/**
  * @brief Frame Move Down
  */
void FrameMoveDown(void)
{
    Frame *frame = GetDefaultFrame();

    if (frame->pointer < frame->lineNumber - 1) {
        frame->pointer++;    /* make pointer-- */
        if (frame->pointer >= g_WinInfo.high) {
            frame->begin++;  /* if pointer move out of window, frame move down one line */
        }
    } else {
        if (frame->rollEn) {  /* roll process */
            frame->pointer = 0;
            frame->begin = 0;
        }
    }
    FrameShow(frame);  /* show frame */
}

/**
  * @brief Frame Selected process
  */
void FrameItemSelect(void)
{
    Frame *frame = GetDefaultFrame();
    Item *item = &frame->pItem[frame->pointer];
    if (!item) {  /* item without connect frame, return */
        return;
    }
    if (item->itemSelectedProc) { /* item with selected process, do process */
        item->itemSelectedProc();
    }
    return;
}

/**
  * @brief Frame Show
  */
void FrameShow(const Frame *frame)
{
    char itemText[ITEM_LIST_MAX_LEN] = {0};
    for (unsigned int i = frame->begin; i < frame->lineNumber; ++i) {
        uint8_t reversed = (i == frame->pointer) ? 0 : 1;
        char *p = (char *)frame->pItem[i].text[g_WinInfo.language];
        char *dynStr;
        if (frame->pItem[i].getInfo) {   /* Get items ext info */
            char extInfo[EXT_INFO_MAX_LEN];
            memset_s(extInfo, sizeof(extInfo), 0, sizeof(extInfo));
            if (frame->pItem[i].getInfo(extInfo, sizeof(extInfo))) {
                /* Add extended information after items. */
                dynStr = extInfo;
                p = MergeStr(itemText, sizeof(itemText), p, dynStr);
            }
        }
        OledShowMenuItem(i, p, FONT_SIZE, reversed);
    }
    if (frame == GetDefaultFrame()) {
        /* Show boot version in boot main frame */
        OledShowMsg(g_WinInfo.high - 1, BOOT_VERSION_EN, strlen(BOOT_VERSION_EN));
    }
}

/**
  * @brief Popup Message if startup fail
  */
void PopupMsgStartupFail(void)
{
    for (int line = 0; line < g_WinInfo.high; ++line) {
        OledClearLine(line, 0, g_WinInfo.width);
    }
    char *title[] = MULTI_LANGUAGE(START_FAIL_TITLE);
    int i = 0;
    OledShowMsg(i++, title[GetWinLanguage()], FONT_SIZE);
    char *msg[] = MULTI_LANGUAGE(START_FAIL_MSG);
    i++;  /* Insert a blank line between title and message. */
    OledShowMsg(i, msg[GetWinLanguage()], FONT_SIZE);
}
