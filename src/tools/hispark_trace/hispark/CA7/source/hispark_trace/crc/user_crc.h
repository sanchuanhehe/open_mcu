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
  * @file    user_crc.h
  * @author  MCU Driver Team
  * @brief   user crc32 module.
  */
#ifndef USER_CRC_H
#define USER_CRC_H

#include <stdint.h>
#include "flash_intf.h"

#define USER_CRC_CHECK_SUCCESS       1
#define USER_CRC_CHECK_FAIL          2
#define USER_CRC_CHECK_NOT_CONNECT   3
#define USER_CRC_CHECK_ERROR         4
#define USER_CRC_CHECK_READ_FAIL     5
#define USER_CRC_CHECK_BAD_FILE      6
#define USER_CRC_CHECK_ALGO_ERROR    7
#define USER_CRC_CHECK_NO_ALGO       8


void UserCrcInit(uint32_t initValue, uint32_t checkValue);
void UserCrc32(const uint8_t *data, int nBytes);
uint32_t UserCurrentCrc32Get(void);
uint32_t IsCheckSuccess(void);
int32_t CheckCrcFromFlash(const flash_intf_t *flash_intf, uint32_t startAddr, uint32_t len);

#endif