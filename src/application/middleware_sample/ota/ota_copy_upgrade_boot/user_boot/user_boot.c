/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2024. All rights reserved.
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
  * @file    user_boot.c
  * @author  MCU Driver Team
  * @brief   This file provides the user boot function.
  */

/* Includes ------------------------------------------------------------------*/
#include "flash.h"
#include "crg.h"
#include "user_boot.h"

/* Macro definitions --------------------------------------------------------- */
#define USER_UPDATE_FLAG 0x01
#define USER_PARAMTER_FLASH_START       0x301FC00
#define USER_APP_RUN_INIT_ADDR          0x3004000

#define USER_LOADER_PARAMETER_SIZE    1024
#define USER_LOADER_PARAMETER_PAGE_NUM 1

#define UPDATA_APP_FLAGE_OFFSET   0
#define APP_RUN_START_ADDR_OFFSET 4
#define APP_RUN_SIZE_OFFSET       8
#define APP_BAK_START_ADDR_OFFSET 12

#define EFLASH_MAGIC_NUMBER       0xA37E95BD
#define EFLASH_MAGIC_OFFSET       0x04

#define BYTE_0            0
#define BYTE_0_OFFSET     24
#define BYTE_1            1
#define BYTE_1_OFFSET     16
#define BYTE_2            2
#define BYTE_2_OFFSET     8
#define BYTE_3            3

/**
  * @brief User boot function.
  * @param flash handle FLASH handle.
  * @retval None.
  */
static void FlashInit(FLASH_Handle *flash)
{
    HAL_CRG_IpEnableSet(EFC_BASE, BASE_CFG_ENABLE); /* Enable the CRG of EFC modular. */
    flash->baseAddress = EFC;
    flash->peMode = FLASH_PE_OP_BLOCK;
    HAL_FLASH_Init(flash); /* Init flash. */
}

/**
  * @brief Gets the value of a word in a char array.
  * @param buffer the data array.
  * @retval the value, unit : word.
  */
static unsigned int GetWordValue(unsigned char *buffer)
{
    unsigned int ret;
    unsigned int i = 0;
    if (buffer == NULL) { /* Null pointer check. */
        return 0;
    }

    ret = buffer[i++] << BYTE_0_OFFSET; /* Highest 8 bits. */
    ret += buffer[i++] << BYTE_1_OFFSET;
    ret += buffer[i++] << BYTE_2_OFFSET;
    ret += buffer[i++];
    return ret;
}

/**
  * @brief Jump function.
  * @param addr the target address of jump.
  * @retval None.
  */
static void UserLoaderJump(unsigned int addr)
{
    /* Judgement flash empty */
    unsigned int *magic = (unsigned int *)(void *)addr;
    if (*magic != EFLASH_MAGIC_NUMBER) {
        while (1) {
            ;
        }
    }
    /* Jump to app run start address. */
    (*(void(*)(void))(addr + EFLASH_MAGIC_OFFSET))();
}

/**
  * @brief User boot function.
  * @param None.
  * @retval None.
  */
void UserBoot(void)
{
    SystemInit(); /* Init peripheral module */
    unsigned char loaderParameter[USER_LOADER_PARAMETER_SIZE] = {0};
    BASE_StatusType ret;
    FLASH_Handle flash;

    unsigned int updateFlage;
    unsigned int appRunStartAddr;
    unsigned int appRunSize;
    unsigned int appBakStartAddr;

    /* Init flash */
    FlashInit(&flash);

    /* Get the flage of user loader. */
    HAL_FLASH_Read(&flash, USER_PARAMTER_FLASH_START - FLASH_READ_BASE, USER_LOADER_PARAMETER_SIZE,
                   loaderParameter, USER_LOADER_PARAMETER_SIZE);
    /* Get the address of app run. */
    appRunStartAddr = GetWordValue(&loaderParameter[APP_RUN_START_ADDR_OFFSET]);
    if (appRunStartAddr < USER_APP_RUN_INIT_ADDR || appRunStartAddr == 0xFFFFFFFF) { /* Check the address validity. */
        appRunStartAddr = USER_APP_RUN_INIT_ADDR;
    }
    /* Get the firmware update status. */
    updateFlage = GetWordValue(&loaderParameter[UPDATA_APP_FLAGE_OFFSET]);
    /* Need to update app run. */
    if (updateFlage == USER_UPDATE_FLAG) {
        appRunSize = GetWordValue(&loaderParameter[APP_RUN_SIZE_OFFSET]);
        appBakStartAddr = GetWordValue(&loaderParameter[APP_BAK_START_ADDR_OFFSET]);
        /* Erase original firmware and prepare to move new firmware. */
        unsigned int eraseNum = appRunSize / FLASH_ONE_PAGE_SIZE;
        eraseNum += ((appRunSize % FLASH_ONE_PAGE_SIZE) == 0) ? 0 : 1;
        ret = HAL_FLASH_EraseBlocking(&flash, FLASH_ERASE_MODE_PAGE, (appRunStartAddr - FLASH_READ_BASE), eraseNum);
        if (ret != BASE_STATUS_OK) {
            UserLoaderJump(appRunStartAddr);
        }
        /* Copy data from app bak flash to app run flash. */
        ret = HAL_FLASH_WriteBlocking(&flash, appBakStartAddr, appRunStartAddr, appRunSize);
        if (ret != BASE_STATUS_OK) {
            UserLoaderJump(appRunStartAddr);
        }

        /* Clear APP update flage */
        loaderParameter[0x03] = 0; /* 0x03 is offset of flage of update. */
        ret = HAL_FLASH_EraseBlocking(&flash, FLASH_ERASE_MODE_PAGE, (USER_PARAMTER_FLASH_START - FLASH_READ_BASE),
                                      USER_LOADER_PARAMETER_PAGE_NUM);
        if (ret != BASE_STATUS_OK) {
            UserLoaderJump(appRunStartAddr);
        }
        /* copy data from app bak flash to app run flash. */
        ret = HAL_FLASH_WriteBlocking(&flash, (unsigned int)(uintptr_t)loaderParameter, USER_PARAMTER_FLASH_START,
                                      USER_LOADER_PARAMETER_SIZE);
        if (ret != BASE_STATUS_OK) {
            UserLoaderJump(appRunStartAddr);
        }
    }

    /* Jump to app run start address. */
    UserLoaderJump(appRunStartAddr);
}