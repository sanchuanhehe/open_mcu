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
#include <stdbool.h>
#include "display_common.h"
#include "display.h"
#include "res_en.h"
#include "res_zh.h"
#include "boot.h"
#include "util.h"
#include "proc.h"

static Frame mainMenuFrame;

#define FRAME_INIT(x, NAME, rollEnable) \
    .pointer = 0,                       \
    .lineNumber = ARRAY_SIZE(x),        \
    .staticItemNum = ARRAY_SIZE(x),     \
    .begin = 0,                         \
    .pItem = (x),                       \
    .name = (NAME),                     \
    .rollEn = (rollEnable)

/* 菜单显示元素 */
/* TOP Menu */
static Item mainItems[] = {
    {MULTI_LANGUAGE(STARTUP_SELECT),         0,            0},
    {MULTI_LANGUAGE(STARTUP_FROM_BLOCK_A),   SelectedBlockA,  GetBlockASelected},
    {MULTI_LANGUAGE(STARTUP_FROM_BLOCK_B),   SelectedBlockB,  GetBlockBSelected},
};

/* 描述菜单链接关系和动作 */
static Frame mainMenuFrame = {
    FRAME_INIT(mainItems, "mainMenu", true),
};

/* Get Boot Main Frame */
Frame *GetDefaultFrame(void)
{
    return &mainMenuFrame;
}
