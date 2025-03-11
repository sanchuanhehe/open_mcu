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
  * @file    boot.h
  * @author  MCU Driver Team
  * @brief   daplink boot header
  */
#ifndef BOOT_H
#define BOOT_H

#include <stdint.h>

#define K_BYTE(size) ((size)*1024)
#define M_BYTE(size) (size) * 1024 * 1024)

#define FLASH_WRITE_SIZE 256
#define FLASH_READ_SIZE  4096
typedef enum {
    IMAGE_BLOCK_A = 0,
    IMAGE_BLOCK_B,
    IMAGE_BLOCK_NUM,
} IMAGE_BLOCK_ID;

typedef enum {
    IMAGE_CA7 = 0,
    IMAGE_CM4,
} IMAGE_TYPE;

typedef enum {
    LOAD_SUCCESS = 0,
    LOAD_FAIL = -1,
    LOAD_CRC_ERROR = -2,
} LOAD_RET_TYPE;

typedef struct {
    unsigned int  a7Start;
    unsigned int  a7Size;    /* image后为image size */
    unsigned int  m4Start;
    unsigned int  m4Size;
} ImageLoadST;

typedef struct {
    ImageLoadST  load[IMAGE_BLOCK_NUM];
    unsigned int  blockIdAddr;
    unsigned int  a7Addr;
    unsigned int  m4Addr;

    IMAGE_BLOCK_ID blockId;
    unsigned int readPageSize;
    unsigned int ca7ImageSize;
    unsigned int cm4ImageSize;
} ImageInfoST;

int BootMain(void);

void SelectedBlockA(void);

void SelectedBlockB(void);

extern uint32_t FlashRead(uint32_t adr, uint32_t sz, uint8_t *buf);

#endif