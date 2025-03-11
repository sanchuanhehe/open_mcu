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
  * @brief   Display process
 */

#include <string.h>
#include <stdbool.h>
#include "securec.h"
#include "oled.h"
#include "menu_tbl.h"
#include "status.h"
#include "offline_sys_config.h"
#include "proc.h"
#include "display.h"


static char *MergeStr(char *const buf, unsigned int bufLen, char *part1, const char *part2);
static void PopupMsgProc(void);
static void DownLoadLedOff(Frame *curFrame);
static Frame *g_ledOnFrame = NULL;
static bool g_ledOn = false;

WinInfo g_WinInfo = {
    .width = WIN_WIDTH / (FONT_SIZE / 2),  /* 默认为显示英文字符 */
    .high  = WIN_HEIGHT / FONT_SIZE,
    .showLineMask = 0,
    .language = LANGUAGE_CN,
};

/**
  * @brief Get Language
  * @retval None
  */
Language GetWinLanguage(void)
{
    return g_WinInfo.language;
}

/**
  * @brief Set Language
  * @retval None
  */
void SetWinLanguage(Language lang)
{
    g_WinInfo.language = lang;
    SysLanguageSet((lang == LANGUAGE_CN) ? SYS_LANGUAGE_CN : SYS_LANGUAGE_EN);
}

/**
  * @brief Force all Line used, then all line will be clear
  * @retval None
  */
void ForceAllLineUsed(void)
{
    g_WinInfo.showLineMask = (unsigned int)-1;
}

/**
  * @brief Is support power supply, base on board type id
  * @retval bool, true: support, false : don't support
  */
static bool IsSupportPowerSupply(void)
{
    unsigned int boardId = (unsigned int)SysBoardIdGet();
    return ((boardId == SYS_ALINK_BOARD_VB_10M) || (boardId == SYS_ALINK_BOARD_VB_16M));
}

/**
  * @brief Display Module Init
  * @retval None
  */
void DisplayInit(void)
{
    ProcInit();
    RegisterDaplinkStatusCallBackFunc(PopupMsgProc);
    if (!IsSupportPowerSupply()) {
        SettingFrameItemsWithNoPowerSupply();
    }
    SetWinLanguage((SysLanguageGet() == SYS_LANGUAGE_EN) ? LANGUAGE_EN : LANGUAGE_CN);
    g_WinInfo.frame = GetDefaultFrame();
    UpdateActiveFrame(GetCurFrame());
}

/**
  * @brief Need Move to end frame when it's popup message
  * @retval bool true: need do move to end, false: no
  */
static bool IsNeedPopMoveToEndFrame(const Frame * frame)
{
    return frame->lineNumber > frame->staticItemNum;
}

/**
  * @brief Frame Move up
  * @retval No
  */
void FrameMoveUp(void)
{
    Frame *frame = GetCurFrame();
    if (frame->onlySupportSelected) {
        /* Don't support move up/down, return */
        return;
    }

    if (frame->pointer > frame->minPointer) {
        /* select items move down */
        frame->pointer--;
        if (frame->pointer >= g_WinInfo.high - 1) {
            frame->begin--;
        }
    } else {
        /* Move to the top of frame */
        if (frame->rollEn) {
            /* if roll enable, goto the bottom items of frame */
            unsigned int end = frame->lineNumber;
            frame->pointer = end - 1;
            if (end >= g_WinInfo.high) {
                frame->begin = end - g_WinInfo.high;
            } else {
                frame->begin = 0;
            }
        }
    }
    FrameShow(frame);
}

/**
  * @brief Frame Move down
  * @retval No
  */
void FrameMoveDown(void)
{
    Frame *frame = GetCurFrame();
    bool rollDown = false;
    if (frame->onlySupportSelected) {
        /* don't support Move up/down action */
        return;
    }
    if (frame->pointer < frame->lineNumber - 1) {
        /* select items move up */
        frame->pointer++;
        if (frame->pointer >= g_WinInfo.high) {
            frame->begin++;
        }
    } else {
        /* Move to the bottom of frame */
        rollDown = true;
        if (frame->rollEn) {
            /* if roll enable, goto the top items of frame */
            frame->pointer = frame->pItem[0].title ? 1 : 0;
            frame->begin = 0;
        }
    }
    if (rollDown && IsNeedPopMoveToEndFrame(frame) && frame->moveToEndFrame) {
        /* if move to bottom line of frame, popup message */
        UpdateActiveFrame(frame->moveToEndFrame);
        return;
    }
    FrameShow(frame);
}

/**
  * @brief Frame Selected
  * @retval No
  */
void FrameItemSelect(void)
{
    Frame *frame = GetCurFrame();
    Item *item = &frame->pItem[frame->pointer];
    if (!item) { /* item don't exist */
        /* don't be herer */
        return;
    }
    if (item->itemSelectedProc) {
        /* Selected Process, transfer user data of line to frame, then Process can used the parameter */
        frame->userData = item->userData;
        item->itemSelectedProc(frame);
    }
    if (item->linkFrame) {
        if (GetFactoryImageProgramFailFrame() == frame) { /* Force to exit item if image upgrade fail in factory mode */
            item->linkFrame->pointer = item->linkFrame->lineNumber - 1;
        }
        /* transfer user data of line to link frame, then Process can used the parameter */
        item->linkFrame->userData = item->userData;
        UpdateActiveFrame(item->linkFrame);
    }
    return;
}

/**
  * @brief Get Line number of item text
  * @retval No
  */
static inline unsigned int GetItemTextLines(const char *p)
{
    unsigned int len = (unsigned int)strlen(p);
    return ((len + g_WinInfo.width - 1) / g_WinInfo.width);
}

/**
  * @brief 重新计算frames能显示的items数，因为当前选中的item可能占用多行
  * @retval No
  */
static void GetShowItemsRange(const Frame *frame, unsigned int *beginIdx)
{
    char *p = frame->pItem[frame->pointer].text[g_WinInfo.language];
    unsigned int occupyLines = GetItemTextLines(p);
    if (occupyLines <= 1) {  /* item occupy only one line, return */
        return;
    }
    unsigned int end = frame->pointer + occupyLines - 1;
    if ((end - frame->begin) >= (g_WinInfo.high - 1)) {
        *beginIdx += occupyLines - 1;  /* 1: all item already occupy one line */
    }
    if (*beginIdx > frame->pointer) {
        *beginIdx = frame->pointer;    /* force set beginIdx */
    }
}

/**
  * @brief Show windows Menu
  * @retval No
  */
static inline bool WindowsMenuShow(const Frame *frame, unsigned int line)
{
    if (frame->pItem[0].windowsMenu) { /* Is menu */
        (void)OledShowMenuItem(line,
            frame->pItem->text[GetWinLanguage()],
            FONT_SIZE,
            (frame->pointer == line) ? 0 : 1); /* highlight items when the pointer ptr to */
        return true;
    }
    return false;
}

/**
  * @brief Show windows Title
  * @retval No
  */
static inline bool TitleShow(const Frame *frame, unsigned int line)
{
    if (frame->pItem[0].title) {
        OledShowMsg(0, frame->pItem[0].text[GetWinLanguage()], FONT_SIZE);
        return true;
    }
    return false;
}

/**
  * @brief Clear Lines
  * @retval No
  */
static void ClearLines(unsigned int startLines)
{
    unsigned int line;
    for (line = startLines; line < g_WinInfo.high; ++line) {
        OledClearLine(line, 0, g_WinInfo.width);
    }
}

static unsigned int ShowItems(Frame *frame, unsigned int beginIdx, unsigned int startLine)
{
    char itemText[ITEM_LIST_MAX_LEN] = {0};
    unsigned int line = startLine;

    if (frame->pointer >= frame->lineNumber) {
        /* can't be here */
        frame->pointer = (frame->lineNumber >= 1) ? (frame->lineNumber - 1) : 0;
    }
    for (unsigned int i = beginIdx; i < frame->lineNumber && line < g_WinInfo.high; ++i) {
        uint8_t reversed = (i == frame->pointer) ? 0 : 1;
        char *p = frame->pItem[i].text[g_WinInfo.language];
        char *dynStr;
        uint8_t showLines;
        uint8_t calcLines;
        if (frame->pItem[i].getInfo) {
            char extInfo[EXT_INFO_MAX_LEN];
            memset_s(extInfo, sizeof(extInfo), 0, sizeof(extInfo));
            if (frame->pItem[i].getInfo(extInfo, sizeof(extInfo))) {
                /* Get ext info and append to the behind of item text */
                dynStr = extInfo;
                p = MergeStr(itemText, sizeof(itemText), p, dynStr);
            }
        }
        showLines = OledShowMenuItem(line, p, FONT_SIZE, reversed);
        calcLines = GetItemTextLines(p);
        if (showLines < calcLines) {
            showLines = calcLines;
        }
        /* The selected menu is displayed according to the line occupied by the actual character string. */
        line += (reversed == 0) ? showLines : 1;
    }
    return line;
}

/**
  * @brief Frame Show
  * @retval No
  */
void FrameShow(Frame *frame)
{
    unsigned int line = 0;
    unsigned int beginIdx;

    if (!frame) {
        return;
    }

    beginIdx = frame->begin;

    if (frame->genDynItems) {
        /* Generate dynamic items before show */
        frame->genDynItems(frame);
    }

    if (WindowsMenuShow(frame, line)) {
        beginIdx++; /* The menu always occupies one line. */
        line++;
    }
    if (TitleShow(frame, line)) {
        beginIdx++;
        line++;
    }
    GetShowItemsRange(frame, &beginIdx);
    line = ShowItems(frame, beginIdx, line);

    /* mask all show lines */
    for (unsigned int i = 1; i <= line; ++i) {
        g_WinInfo.showLineMask |= 1 << i;
    }
    ClearLines(line);
    if (frame->popupMsg) {
        frame->popupMsg(frame);
    }
}

/**
  * @brief Active Frame
  * @retval No
  */
void UpdateActiveFrame(Frame *frame)
{
    g_WinInfo.frame = frame;
    if (IsFactoryImageProgramSuccFrame()) {
        /* Indicates the first item of the clock execution menu for special frames. */
        frame->pointer = 1;
    }
    /* Reinit show config */
    SetGlobalDisaplyaMode(DISPLAY_NORMAL);
    DownLoadLedOff(frame);
    FrameShow(frame);
}

/**
  * @brief Get current frame
  * @retval current frame
  */
Frame *GetCurFrame(void)
{
    return g_WinInfo.frame;
}

/**
  * @brief Merge two string
  * @retval No
  */
static char *MergeStr(char *const buf, unsigned int bufLen, char *part1, const char *part2)
{
    char *p = buf;
    errno_t rc = EOK;
    if (part2 == NULL) {
        return part1;  /* only one string, don't merge */
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
  * @brief Popup Message
  * @retval No
  */
void FramePopMsg(const Frame *frame, const char *msg)
{
    unsigned int pointer = frame->lineNumber + 1;
    ClearLines(frame->lineNumber);
    OledShowMsg(pointer, msg, FONT_SIZE);
    g_WinInfo.showLineMask = 0xFF;
}

/**
  * @brief Popup Message with title
  * @retval No
  */
void FramePopMsgWithTitle(const Frame *frame, const char *title, const char **msg, int msgNum, DisplayAlignMode mode)
{
    unsigned int pointer = frame->lineNumber;
    unsigned int width = (WIN_WIDTH * 2) / FONT_SIZE;
    ClearLines(pointer);
    if (title) {
        OledShowMsg(pointer++, title, FONT_SIZE); /* Title in middle of line */
    }
    for (int i = 0; i < msgNum; ++i) {
        if (mode == ALIGN_MID) {
            /* items show in middle of line */
            OledShowMsg(pointer, msg[i], FONT_SIZE);
        } else {
            /* items show in begin of line */
            (void)OledShowString(0, pointer, msg[i], FONT_SIZE, 1);
        }
        /* pointer add line number of message */
        pointer += ((unsigned int)(strlen(msg[i])) + width - 1) / width;
    }
    g_WinInfo.showLineMask = 0xFF;
}

/**
  * @brief Popup Message with specified font
  * @retval No
  */
void FramePopMsgWithFont(const Frame *frame, const char *msg, int font)
{
    unsigned int pointer = frame->lineNumber;
    ClearLines(pointer);
    OledShowMsg(pointer, msg, font);
    g_WinInfo.showLineMask = 0xFF;
}

/**
  * @brief Show status on bottom of window
  * @retval No
  */
void FrameShowStatusBar(const Frame *frame, const char *status)
{
    uint8_t idx = g_WinInfo.high - 1;
    OledShowMsg(idx, status, FONT_SIZE);
    g_WinInfo.showLineMask |= (1 << idx);
}

/* Called when status changed */
void PopupMsgProc(void)
{
    Frame *frame = GetCurFrame();
    if (frame->popupMsg) {
        frame->popupMsg(frame);
    }
}

/**
  * @brief Led On
  * @retval No
  */
void DownLoadLedOn(Frame *frame)
{
    g_ledOnFrame = frame;
    g_ledOn = true;
    OfflineLoadLedSet(SYS_OFFLINE_LED_ON);
}

/**
  * @brief Led OFf
  * @retval No
  */
static void DownLoadLedOff(Frame *curFrame)
{
    if (g_ledOn && (curFrame != g_ledOnFrame)) {
        g_ledOn = false;
        OfflineLoadLedSet(SYS_OFFLINE_LED_OFF);
    }
}
