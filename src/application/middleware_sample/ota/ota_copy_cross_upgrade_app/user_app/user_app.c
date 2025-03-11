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
  * @file    user_app.c
  * @author  MCU Driver Team
  * @brief   This file provides the user app function.
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "user_loader.h"
#include "debug.h"
#include "user_app.h"

/* Macro definitions --------------------------------------------------------- */
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
  * @brief Start the OTA upgrade process.
  * @param None.
  * @retval None.
  */
static void StartOtaProgress(void)
{
    /* Initializes module resources required for the upgrade. */
    ULOADER_Handle uloaderHandle;
    FLASH_Handle flashHandle;
    CRC_Handle crcHandle;
    UART_Handle uart0Handle;
    ULOADER_Cmd cmd;

    uloaderHandle.flash = &flashHandle;
    uloaderHandle.crc = &crcHandle;
    uloaderHandle.comHandle = (void *)&uart0Handle;
    uloaderHandle.mode = COMMUNICATION_UART;
    ULOADER_Init(&uloaderHandle); /* Init the module resources of user loader. */
    /* User OTA progress */
    if (ULOADER_HandShake(&uloaderHandle, USER_LOADER_HANDSHAKE_TIMEOUT) == BASE_STATUS_OK) {
        while (true) {
            if (ULOADER_ReceiveCmd(&uloaderHandle, &cmd) != BASE_STATUS_OK) {
                continue;
            }
            if (ULOADER_CommandExec(&uloaderHandle, &cmd) != BASE_STATUS_OK) {
                ULOADER_Ack(&uloaderHandle, ACK_FAIL, 0);
            }
        }
    }

    ULOADER_DeInit(&uloaderHandle);
    BASE_FUNC_SoftReset(); /* Hand shake fail, reset the chip. */
}

/**
  * @brief User app progress function.
  * @param None.
  * @retval None.
  */
void UserAppProgress(void)
{
    SystemInit(); /* Init peripheral module */
    while (true) {
        /* User Logic Program Example. */
        DBG_PRINTF("User loop wait ota flage\r\n");
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