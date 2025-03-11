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
  * @file    user_crc32.c
  * @author  MCU Driver Team
  * @brief   user crc32 module.
  */
#include <string.h>
#include "securec.h"
#include "flash_intf.h"
#include "FlashPrg.h"
#include "crc32.h"
#include "user_crc.h"

#define ONECE_CHECK_LEN        1024

static uint32_t g_currentCrcValue = 0;
static uint32_t g_currentCheckValue = 0;

void UserCrcInit(uint32_t initValue, uint32_t checkValue)
{
    g_currentCrcValue = initValue;
    g_currentCheckValue = checkValue;
}

void UserCrc32(const uint8_t *data, int nBytes)
{
    g_currentCrcValue = Crc32Continue(g_currentCrcValue, data, nBytes);
}

uint32_t UserCurrentCrc32Get(void)
{
    return g_currentCrcValue;
}

uint32_t IsCheckSuccess(void)
{
    if (g_currentCrcValue == g_currentCheckValue) {
        return 1;
    }
    return 0;
}

int32_t CheckCrcFromFlash(const flash_intf_t *flash_intf, uint32_t startAddr, uint32_t len)
{
    int status;
    uint8_t buf[ONECE_CHECK_LEN];
    uint32_t checkLen = len;
    uint32_t addr = startAddr;
    uint32_t currentCheckLen = 0;

    status = flash_intf->init();
    if (ERROR_SUCCESS != status) {
        flash_intf->uninit();
        return USER_CRC_CHECK_NOT_CONNECT;
    }

    if (flash_intf->flash_algo_set) {
        status = flash_intf->flash_algo_set(startAddr);
        if (ERROR_SUCCESS != status) {
            flash_intf->uninit();
            return USER_CRC_CHECK_ALGO_ERROR;
        }
    }

    while (checkLen > 0) {
        if (memset_s(buf, ONECE_CHECK_LEN, 0, ONECE_CHECK_LEN) != EOK) {
            flash_intf->uninit();
            return USER_CRC_CHECK_READ_FAIL;
        }

        if (checkLen >= ONECE_CHECK_LEN) {
            currentCheckLen = ONECE_CHECK_LEN;
            checkLen -= ONECE_CHECK_LEN;
        } else {
            currentCheckLen = checkLen;
            checkLen = 0;
        }
        status = flash_intf->read(addr, buf, currentCheckLen);
        if (ERROR_SUCCESS != status) {
            flash_intf->uninit();
            return USER_CRC_CHECK_READ_FAIL;
        }
        addr += currentCheckLen;
        g_currentCrcValue = Crc32Continue(g_currentCrcValue, buf, currentCheckLen);
    }
    flash_intf->uninit();
    if (g_currentCrcValue == g_currentCheckValue) {
        return USER_CRC_CHECK_SUCCESS;
    }
    return USER_CRC_CHECK_ERROR;
}
