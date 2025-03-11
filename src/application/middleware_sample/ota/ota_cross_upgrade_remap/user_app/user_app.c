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
#include "debug.h"
#include "user_app.h"

#define USER_PARAMTER_FLASH_ERASE_SIZE           1
#define USER_PARAMTER_FLASH_WRITE_SIZE           1024

#define USER_PARAMTER_FORMER_FLASH_ADDR          0x303FC00
#define USER_PARAMTER_BACK_FLASH_ADDR            0x307FC00

#define USER_LOADER_PARAMETER_SIZE               1024
#define REMAP_FLAGE_OFFSET                       0

#define EFLASH_MAGIC_NUMBER                      0xA37E95BD
#define EFLASH_MAGIC_OFFSET                      0x04

#define REMAP_ENABLE_MASK                        0x01

#define BYTE_0            0
#define BYTE_0_OFFSET     24
#define BYTE_1            1
#define BYTE_1_OFFSET     16
#define BYTE_2            2
#define BYTE_2_OFFSET     8
#define BYTE_3            3

#define USER_LOADER_HANDSHAKE_TIMEOUT 1000
#define DBG_PRINTF_FINISH_WAIT_TIME 100
#define LOOP_PRINTF_TIME 1000

static unsigned int g_otaFlage = 0;

void GPIO3_5_CallbackFunc(void *param)
{
    BASE_FUNC_UNUSED(param); /* Prevent compilation alarms. */
    g_otaFlage = 1;
}

/**
  * @brief Gets the value of a word in a char array.
  * @param buffer the data array.
  * @retval the value, unit : word.
  */
static unsigned int GetWordValue(unsigned char *buffer, unsigned int size)
{
    unsigned int ret;
    unsigned int index = 0;
    BASE_FUNC_UNUSED(size); /* Compiler prevention alarm. */
    if (buffer == NULL) { /* Null pointer check. */
        return 0;
    }

    ret = buffer[index++] << BYTE_0_OFFSET; /* Highest 8 bits. */
    ret += buffer[index++] << BYTE_1_OFFSET;
    ret += buffer[index++] << BYTE_2_OFFSET;
    ret += buffer[index++];
    return ret;
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
  * @brief Jump function.
  * @param addr the target address of jump.
  * @retval None.
  */
static void UserLoaderJump(unsigned int addr)
{
    /* Jump to ServiceProgress() address. */
    (*(void(*)(void))(addr))();
}

/**
  * @brief Start the OTA upgrade process.
  * @param None.
  * @retval None.
  */
static void StartOtaProgress(void)
{
    /* Initializes module resources required for the upgrade. */
    ULOADER_Handle uLoaderHandle;
    FLASH_Handle flashHandle;
    CRC_Handle crcHandle;
    UART_Handle uart0Handle;
    ULOADER_Cmd cmd;

    uLoaderHandle.flash = &flashHandle;
    uLoaderHandle.crc = &crcHandle;
    uLoaderHandle.comHandle = (void *)&uart0Handle;
    uLoaderHandle.mode = COMMUNICATION_UART;
    ULOADER_Init(&uLoaderHandle); /* Init the module resources of user loader. */
    /* User OTA progress */
    if (ULOADER_HandShake(&uLoaderHandle, USER_LOADER_HANDSHAKE_TIMEOUT) == BASE_STATUS_OK) {
        while (true) {
            if (ULOADER_ReceiveCmd(&uLoaderHandle, &cmd) != BASE_STATUS_OK) {
                continue;
            }
            if (ULOADER_CommandExec(&uLoaderHandle, &cmd) != BASE_STATUS_OK) {
                ULOADER_Ack(&uLoaderHandle, ACK_FAIL, 0);
            }
        }
    }

    ULOADER_DeInit(&uLoaderHandle);
    BASE_FUNC_SoftReset(); /* Hand shake fail, reset the chip. */
}

/**
  * @brief User app progress function.
  * @param None.
  * @retval None.
  */
static void ServiceProgress(void)
{
    while (true) {
        /* User Logic Program Example. */
        DBG_PRINTF("User app loop wait remap ota flage\r\n");
        BASE_FUNC_DELAY_MS(LOOP_PRINTF_TIME);

        /* The program enters the OTA phase and starts the firmware upgrade. */
        if (g_otaFlage == 1) {
            /* All interrupts used in the APP must be disabled to ensure that the upgrade process is not interrupted. */
            IRQ_DisableN(IRQ_GPIO3);
            DBG_PRINTF("User loader start.\r\n");
            BASE_FUNC_DELAY_MS(DBG_PRINTF_FINISH_WAIT_TIME);
            StartOtaProgress(); /* Start the user OTA progress. */
        }
    }
}

/**
  * @brief User boot function.
  * @param None.
  * @retval None.
  */
void UserAppProgress(void)
{
    SystemInit(); /* Init peripheral module */
    unsigned char loaderParameter[USER_LOADER_PARAMETER_SIZE] = {0};
    FLASH_Handle flash;
    unsigned int remapReg;

    /* Init flash */
    FlashInit(&flash);
    /* Get the flage of user loader. */
    HAL_FLASH_Read(&flash, USER_PARAMTER_FORMER_FLASH_ADDR - FLASH_READ_BASE, USER_LOADER_PARAMETER_SIZE,
                   loaderParameter, USER_LOADER_PARAMETER_SIZE);
    
    remapReg = GetWordValue(&loaderParameter[REMAP_FLAGE_OFFSET], 0x04); /* 0x04: one word equal 4 byte. */
    if (remapReg == REMAP_ENABLE_MASK) {
        /* Enable remap and jump. */
        DCL_SYSCTRL_SetRemapMode(SYSCTRL_REMAP_MODE3);
        DCL_SYSCTRL_EnableRemap();

        /* The target function address is related to the code. */
        unsigned int addr = (unsigned int)ServiceProgress;
        UserLoaderJump(addr); /* Jump to ServiceProgress() address. */
    }

    ServiceProgress(); /* Executing a Service Processing Program. */
}