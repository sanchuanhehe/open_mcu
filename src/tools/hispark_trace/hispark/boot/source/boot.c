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
  * @file    boot.c
  * @author  MCU Driver Team
  * @brief   daplink boot code
  */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "securec.h"
#include "stm32mp1xx.h"
#include "daplink_addr.h"
#include "crc32.h"
#include "display.h"
#include "menu_tbl.h"
#include "key.h"
#include "sdk.h"
#include "FlashPrg.h"
#include "config_storage_update.h"
#include "boot.h"

#ifndef NULLPTR
#define NULLPTR (void *)0
#endif

#define SUCCESS     0
#define FAIL        0xFFFFFFFF
#define BOOT_SIZE   K_BYTE(32)
#define PARAM_SIZE  K_BYTE(16)
#define IMAGE_BASE_ADDR  (DAPLINK_ROM_BL_START + BOOT_SIZE + PARAM_SIZE)

#define BOOT_KEY_WAIT_TIMES    2000
#define BOOT_KEY_PRESS_TH      1000
#define BOOT_NO_PRESS_TIMEOUT  100

#define BOOK_KEY_PRESS_DETECTION_INTERVAL 50

#if (defined(DAPLINK_BASEADDR) && (DAPLINK_BASEADDR != 0))
#define DAPLINK_A7_CODE_START DAPLINK_BASEADDR
#else
#error "Please set a7 code start"
#endif
#define DAPLINK_M4_CODE_START 0x38000000

/* DaplinK Image Information */
static ImageInfoST g_imageInfo = {
    .blockIdAddr = DAPLINK_ROM_BL_START + BOOT_SIZE,
    .a7Addr = DAPLINK_A7_CODE_START,
    .m4Addr = DAPLINK_M4_CODE_START, /* 内存镜像，实际会写到RETRAM(0x0000000) */
    .readPageSize = FLASH_READ_SIZE,
    .load[0] = {
        .a7Start = IMAGE_BASE_ADDR,
        .a7Size  = IMAGE_BASE_ADDR + K_BYTE(700) - 4,
        .m4Start = IMAGE_BASE_ADDR + K_BYTE(700),
        .m4Size  = IMAGE_BASE_ADDR + K_BYTE(1024) - 4,
    },
    .load[1] = {
        .a7Start = IMAGE_BASE_ADDR + K_BYTE(1024),
        .a7Size  = IMAGE_BASE_ADDR + K_BYTE(700 + 1024) - 4,
        .m4Start = IMAGE_BASE_ADDR + K_BYTE(700 + 1024),
        .m4Size  = IMAGE_BASE_ADDR + K_BYTE(2048) - 4,
    }
};

typedef void (*PHookFunc)(void);

/**
  * @brief Get Block Index.
  * @param blockId [OUTPUT]
  * @retval success
  */
static uint32_t GetBlockIndex(IMAGE_BLOCK_ID *blockId)
{
    *blockId = (ConfigStartFlagRead() == CONFIG_STRAT_FLAG_A) ? IMAGE_BLOCK_A : IMAGE_BLOCK_B;
    return LOAD_SUCCESS;
}

/**
  * @brief Get DAPLink CA7 image size
  * @param blockId [INPUT]
  * @retval success/fail
  */
static uint32_t GetCA7ImageSize(IMAGE_BLOCK_ID blockId)
{
    uint32_t imageSize;
    if (FlashRead(g_imageInfo.load[blockId].a7Size, sizeof(uint32_t), (uint8_t *)&imageSize) != 0) {
        /* read image fail, return fail */
        return LOAD_FAIL;
    }
    if (imageSize == 0) {
        /* empty image, return fail */
        return LOAD_FAIL;
    }
    g_imageInfo.ca7ImageSize = imageSize;
    return LOAD_SUCCESS;
}

/**
  * @brief Get DAPLink CM4 image size
  * @param blockId [INPUT]
  * @retval success/fail
  */
static uint32_t GetCM4ImageSize(IMAGE_BLOCK_ID blockId)
{
    uint32_t imageSize;
    if (FlashRead(g_imageInfo.load[blockId].m4Size, sizeof(uint32_t), (uint8_t *)&imageSize) != 0) {
        /* read image fail, return fail */
        return LOAD_FAIL;
    }
    if (imageSize == 0) {
        /* empty image, return fail */
        return LOAD_FAIL;
    }
    g_imageInfo.cm4ImageSize = imageSize;
    return LOAD_SUCCESS;
}

/**
  * @brief Write Image from src to dest
  * @param dst destination address
  * @param src source address
  * @param len image length
  * @retval success/fail
  */
static uint32_t WriteImage(uint32_t dst, uint32_t src, uint32_t len)
{
    errno_t rc = EOK;
    uint32_t imageSize = len - 4;
    uint32_t a7LoadAddr = src;
    uint32_t dest = dst;
    uint8_t buf[FLASH_READ_SIZE];
    uint32_t ckSum = 0;
    uint32_t crc;

    if (FlashRead(a7LoadAddr + imageSize, sizeof(int), buf) != SUCCESS) {
        /* If get image size fail, uninit flash and return fail */
        UnInit(0);
        return FAIL;
    }
    /* Get expect crc */
    crc = *(unsigned int *)buf;
    while (imageSize > 0) {
        /* Copy Image from src to dest */
        uint32_t readLen = (imageSize >= g_imageInfo.readPageSize) ? g_imageInfo.readPageSize : imageSize;
        if (FlashRead(a7LoadAddr, readLen, buf) != SUCCESS) {
            UnInit(0);
            return FAIL;
        }
        /* calc crc checksum */
        ckSum = Crc32Continue(ckSum, buf, readLen);
        rc = memcpy_s((void *)dest, readLen, (void *)buf, readLen);
        if (rc != EOK) {
            UnInit(0);
            return FAIL;
        }
        imageSize -= readLen;
        a7LoadAddr += readLen;
        dest += readLen;
    }
    /* return success if calc crc equl expect crc, or return error */
    return (ckSum == crc) ? LOAD_SUCCESS : LOAD_CRC_ERROR;
}

/**
  * @brief Write DAPLink CA7 image
  * @param blockId [INPUT]
  * @retval success/fail
  */
static inline uint32_t WriteCA7Image(IMAGE_BLOCK_ID blockId)
{
    return WriteImage(g_imageInfo.a7Addr, g_imageInfo.load[blockId].a7Start, g_imageInfo.ca7ImageSize);
}

/**
  * @brief Write DAPLink CM4 image
  * @param blockId [INPUT]
  * @retval success/fail
  */
static inline uint32_t WriteCM4Image(IMAGE_BLOCK_ID blockId)
{
    return WriteImage(g_imageInfo.m4Addr, g_imageInfo.load[blockId].m4Start, g_imageInfo.cm4ImageSize);
}

/**
  * @brief Load Firmware
  * @param blockId [INPUT]
  * @retval success/fail
  */
static uint32_t FirmwareLoad(IMAGE_BLOCK_ID blockId)
{
    uint32_t ret;

    /* Get CA7 Image size */
    if (GetCA7ImageSize(blockId) != LOAD_SUCCESS) {
        return LOAD_FAIL;
    }

    /* Write CA7 Image */
    ret = WriteCA7Image(blockId);
    if (ret != LOAD_SUCCESS) {
        return ret;
    }

    /* Get CM4 Image size */
    if (GetCM4ImageSize(blockId) != LOAD_SUCCESS) {
        return LOAD_FAIL;
    }

    /* Write CM4 Image */
    ret = WriteCM4Image(blockId);
    if (ret != LOAD_SUCCESS) {
        return ret;
    }
    return SUCCESS;
}

/**
  * @brief Jump to address and run
  * @param addr next pc address
  * @retval None
  */
static void Jump(unsigned int addr)
{
    (*(void(*)(void))addr)();
}

/**
  * @brief Select a Startup block
  * @param None
  * @retval bool 0: no block selectd, 1: select a block
  */
static bool BlockSelect(void)
{
    unsigned int preTick = (unsigned int)HAL_GetTick();
    unsigned int curTick;
    unsigned char keyVal;
    unsigned int keyCnt = 0;

    for (unsigned int i = 0; i < BOOT_KEY_WAIT_TIMES;) {
        curTick = (unsigned int)HAL_GetTick();
        if (curTick == preTick) {
            /* The key event is counted only once at a fixed interval. */
            continue;
        }
        preTick = curTick;
        keyVal = KeyScan(LONG_KEY);
        if (keyVal == KEY_CONFIRM) {
            /* Only the confirmation key is counted. */
            keyCnt++;
        }
        ++i;
        if (i == BOOT_NO_PRESS_TIMEOUT) {
            /* Key not detected. */
            if (keyCnt == 0) {
                return 0;
            }
        }
    }
    /* If the number of key presses in a specified period exceeds the
       threshold, the confirm key is pressed. */
    return keyCnt >= BOOT_KEY_PRESS_TH;
}

/**
  * @brief Key processing
  * @param None
  * @retval None
  */
static void KeyProcess(void)
{
    uint8_t keyValue = 0;
    keyValue = KeyScan(SHORT_KEY);
    switch (keyValue) {
        case KEY_DOWN:
            FrameMoveDown();  /* Move Down */
            break;
        case KEY_UP:
            FrameMoveUp();    /* Move Up */
            break;
        case KEY_CONFIRM:
            FrameItemSelect(); /* Selected block */
            break;
        default:
            break;
    }
}

/**
  * @brief Menu processing
  * @param None
  * @retval None
  */
static void BootMenuProcess(void)
{
    Frame  *frame = GetDefaultFrame(); /* Get Boot Frame */
    unsigned int curTick = 0;
    unsigned int preTick = 0;

    FrameShow(frame);

    /* Wait Key Press */
    while (1) {
        curTick = (unsigned int)HAL_GetTick() / BOOK_KEY_PRESS_DETECTION_INTERVAL;
        if (curTick != preTick) {
            preTick = curTick;
            KeyProcess(); /* If startup block is selected, don't return here */
        }
    }
}

/**
  * @brief Get Start Image Block Id
  * @param None
  * @retval None
  */
static inline unsigned int GetStartBlock(void)
{
    return (ConfigStartFlagRead() == CONFIG_STRAT_FLAG_A) ? IMAGE_BLOCK_A : IMAGE_BLOCK_B;
}

/**
  * @brief Set Start Image Block Id
  * @param None
  * @retval None
  */
static inline void SetStartBlock(unsigned int blockId)
{
    ConfigStartFlagSave(blockId == IMAGE_BLOCK_A ? CONFIG_STRAT_FLAG_A : CONFIG_STRAT_FLAG_B);
}

/**
  * @brief Start frome specified Image Block
  * @param blockId Image block index
  * @param pfunc   Image Load fail hook function
  * @retval None
  */
static void StartupWithBlockId(unsigned int blockId, PHookFunc pfunc)
{
    unsigned int index = blockId;

    Init(0, 0, 0);  /* Flash Init */

    if (FirmwareLoad(index) != LOAD_SUCCESS) { /* Load Image from specified block */
        if (pfunc) {
            pfunc();  /* try hook process when load fail */
        }
        /* if load fail, try start from another block */
        index = (index == IMAGE_BLOCK_A) ? IMAGE_BLOCK_B : IMAGE_BLOCK_A;
        if (FirmwareLoad(index) != LOAD_SUCCESS) {
            return;
        }
    }
    /* If the startup block is not the same as the specified block (starting from the
       specified block fails), update the block */
    if (GetStartBlock() != index) {
        SetStartBlock(index);
    }
    UnInit(0); /* Flash DeInit */
    Jump(g_imageInfo.a7Addr); /* Jump to DAPLINK CA7 code first instruction */
}

/**
  * @brief Wait Confirm an popup fail message
  * @param void
  * @retval None
  */
static void WaitConfirm(void)
{
    unsigned int curTick = 0;
    unsigned int preTick = 0;

    while (1) {
        curTick = (unsigned int)HAL_GetTick() / BOOK_KEY_PRESS_DETECTION_INTERVAL;
        if (curTick != preTick) {
            preTick = curTick;
            if (KeyScan(SHORT_KEY) == KEY_CONFIRM) { /* Confirmed, End the infinite loop */
                break;
            }
        }
    }
}

/**
  * @brief Load Fail process, popup a messeage and wait
  * @param void
  * @retval None
  */
static void StartUpFailProc(void)
{
    PopupMsgStartupFail();
    WaitConfirm();
}

/**
  * @brief Select BlockA, a function connect frame items
  * @param void
  * @retval None
  */
void SelectedBlockA(void)
{
    StartupWithBlockId(IMAGE_BLOCK_A, StartUpFailProc);
}

/**
  * @brief Select BlockB, a function connect frame items
  * @param void
  * @retval None
  */
void SelectedBlockB(void)
{
    StartupWithBlockId(IMAGE_BLOCK_B, StartUpFailProc);
}

/**
  * @brief boot main function
  * @param void
  * @retval None
  */
int BootMain(void)
{
    IMAGE_BLOCK_ID blockId;

    if (BlockSelect()) {
        /* If you press and hold the OK key during startup, the boot menu
           is displayed. May Changed startup block id */
        BootMenuProcess();
    }
    /* Get startup block id */
    if (GetBlockIndex(&blockId) != LOAD_SUCCESS) {
        return LOAD_FAIL;
    }
    /* Start From block id */
    StartupWithBlockId(blockId, NULLPTR);
    return 0;
}
