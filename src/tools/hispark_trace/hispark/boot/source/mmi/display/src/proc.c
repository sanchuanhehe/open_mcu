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
  * @brief   daplink boot menu item selectd connect process
  */

#include <string.h>
#include <stdbool.h>
#include "securec.h"
#include "res_en.h"
#include "res_zh.h"
#include "display.h"
#include "oled.h"
#include "config_storage_update.h"
#include "proc.h"

/**
  * @brief Boot Block A selected.
  */
unsigned int GetBlockASelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    char *msg[] = MULTI_LANGUAGE(SELECTED);

    if (ConfigStartFlagRead() != CONFIG_STRAT_FLAG_A) {
        /* Block A isn't selected */
        return false;
    }

    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Boot Block B selected process
  */
unsigned int GetBlockBSelected(char *buf, unsigned int len)
{
    errno_t rc = EOK;
    char *msg[] = MULTI_LANGUAGE(SELECTED);

    if (ConfigStartFlagRead() != CONFIG_STRAT_FLAG_B) {
        /* Block B isn't selected */
        return false;
    }

    rc = strncpy_s(buf, len, msg[GetWinLanguage()], len);
    if (rc != EOK) {
        return false;
    }
    return true;
}