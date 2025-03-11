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

#define UASE_APP0_RUN_FLAG                 0x00
#define UASE_APP1_RUN_FLAG                 0x01

#define USER_PARAMTER_FLASH_START          0x301FC00
#define USER_APP_RUN_INIT_ADDR             0x3004000

#define USER_LOADER_PARAMETER_SIZE         1024
#define USER_LOADER_PARAMETER_PAGE_NUM     1

#define JUMP_APP_FLAGE_OFFSET              0
#define APP0_RUN_START_ADDR_OFFSET         4
#define APP1_RUN_START_ADDR_OFFSET         8

#define EFLASH_MAGIC_NUMBER                0xA37E95BD
#define EFLASH_MAGIC_OFFSET                0x04
/*
 * byte and bit offset in unsigned int
 */
#define BYTE_0            0
#define BYTE_0_OFFSET     24
#define BYTE_1            1
#define BYTE_1_OFFSET     16
#define BYTE_2            2
#define BYTE_2_OFFSET     8
#define BYTE_3            3

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
  * @brief User boot function.
  * @param None.
  * @retval None.
  */
void UserBoot(void)
{
    SystemInit(); /* Init peripheral module */
    unsigned char loaderParameter[USER_LOADER_PARAMETER_SIZE] = {0};
    FLASH_Handle flash;
    unsigned int jumpAppFlage;
    unsigned int app0RunStartAddr;
    unsigned int app1RunStartAddr;
    
    /* Init flash */
    FlashInit(&flash);
    /* Get the flage of user loader. */
    HAL_FLASH_Read(&flash, USER_PARAMTER_FLASH_START - FLASH_READ_BASE, USER_LOADER_PARAMETER_SIZE,
                   loaderParameter, USER_LOADER_PARAMETER_SIZE);

    jumpAppFlage = GetWordValue(&loaderParameter[JUMP_APP_FLAGE_OFFSET]);
    app0RunStartAddr = GetWordValue(&loaderParameter[APP0_RUN_START_ADDR_OFFSET]);
    if (app0RunStartAddr < USER_APP_RUN_INIT_ADDR || app0RunStartAddr == 0xFFFFFFFF) { /* Check the address validity. */
        app0RunStartAddr = USER_APP_RUN_INIT_ADDR;
    }
    app1RunStartAddr = GetWordValue(&loaderParameter[APP1_RUN_START_ADDR_OFFSET]);
    if (app1RunStartAddr < USER_APP_RUN_INIT_ADDR || app1RunStartAddr == 0xFFFFFFFF) { /* Check the address validity. */
        app1RunStartAddr = USER_APP_RUN_INIT_ADDR;
    }

    /* The app1 is new, and jump to the app1 address. */
    if (jumpAppFlage == UASE_APP1_RUN_FLAG) {
        UserLoaderJump(app1RunStartAddr);
    }
    /* Jump to app run start address. */
    UserLoaderJump(app0RunStartAddr);
}