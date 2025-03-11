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
  * @file    display_common.h
  * @author  MCU Driver Team
  * @brief   Display Common Header
  */

#ifndef DISPLAY_COMMON_H
#define DISPLAY_COMMON_H

#include "util.h"

#ifndef NULL
#define NULL (void *)0
#endif

#define MULTI_LANGUAGE(x) {x##_EN, x##_ZH}
#define GET_MULTILANGE_STR(x, language) ((language) == LANGUAGE_EN ? x##_EN : x##_ZH)

#define WIN_HEIGHT (16 * 6)
#define WIN_WIDTH  (24 * 8)
#define FONT_SIZE  16
#define BIG_FONT_SIZE 32
#define WIN_MAX_LINE ((WIN_HEIGHT) / (FONT_SIZE))
#define ITEM_CONTEXT_MAX_LEN 48
#define FILENAME_MAX_LEN 128

typedef enum {
    LANGUAGE_EN = 0,
    LANGUAGE_CN = 1,
    LANGUAGE_NUM = 2,
} Language;

#endif