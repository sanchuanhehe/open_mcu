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
  * @file    target_algo_parse.h
  * @author  MCU Driver Team
  * @brief   target algo parse driver.
  */
#ifndef TARGET_ALGO_PARSE_H
#define TARGET_ALGO_PARSE_H

#include <stdint.h>

#define TARGET_ALGO_PARSE_OK             0
#define TARGET_ALGO_PARSE_ERROR          1

#define TARGET_ALGO_PARSE_BUFF_SIZE      8192

#define TARGET_ALGO_PARSE_MIN_SIZE                      0x48

#define TARGET_ALGO_PARSE_FLASH_START_OFFSET            0x00
#define TARGET_ALGO_PARSE_FLASH_SIZE_OFFSET             0x04
#define TARGET_ALGO_PARSE_FLASH_SECTOR_SIZE_OFFSET      0x08

#define TARGET_ALGO_PARSE_RAM_START_OFFSET              0x0C
#define TARGET_ALGO_PARSE_PRG_BUFF_OFFSET               0x10
#define TARGET_ALGO_PARSE_BK_ADDR_OFFSET                0x14
#define TARGET_ALGO_PARSE_STATIC_BASE_OFFSET            0x18
#define TARGET_ALGO_PARSE_STACK_POINTER_OFFSET          0x1C

#define TARGET_ALGO_PARSE_INIT_FUN_OFFSET               0x20
#define TARGET_ALGO_PARSE_UNINIT_FUN_OFFSET             0x24
#define TARGET_ALGO_PARSE_ERASE_CHIP_FUN_OFFSET         0x28
#define TARGET_ALGO_PARSE_ERASE_SECTOR_FUN_OFFSET       0x2C
#define TARGET_ALGO_PARSE_PRG_PAGE_FUN_OFFSET           0x30
#define TARGET_ALGO_PARSE_VERIFY_FUN_OFFSET             0x34
#define TARGET_ALGO_PARSE_READ_FUN_OFFSET               0x38
#define TARGET_ALGO_PARSE_ALGO_FLAGS_OFFSET             0x3c

#define TARGET_ALGO_PARSE_RESERVED_OFFSET               0x40

#define TARGET_ALGO_PARSE_FLASH_ALGO_LEN_OFFSET         0x44
#define TARGET_ALGO_PARSE_ALGO_BLOB_OFFSET              0x48

int32_t TargetAlgoConfig(int32_t algoIndex);


#endif /* #ifndef TARGET_ALGO_PARSE_H */