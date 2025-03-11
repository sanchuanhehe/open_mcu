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
  * @file    user_loader.h
  * @author  MCU Driver Team
  * @brief   This file provides the user loader structure and functions to manage the following functionalities of
  *          the user loader.
  *          + Initialization and de-initialization functions.
  *          + Handshake, command receiving, and command execution functions.
  *          + Read and write functions.
  *          + Atomic command function.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_USER_LOADER_H
#define McuMagicTag_USER_LOADER_H

/* Includes ------------------------------------------------------------------*/
#include "flash.h"
#include "crc.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "can.h"
#include "crg.h"

/* Macro definitions --------------------------------------------------------- */
#define PARAMETER_ADDR    0x301FC00
#define CMD_PAYLOAD_MAX   30         /**< Command payload length */

/* Frame header flag parameter. */
#define XHDSHK 0xEB
#define XHEAD  0xFE
#define XDATA  0xDA
#define XTAIL  0xED
#define XACK   0xCB
#define XCMD   0xAB
#define XKEY   0xCD
#define XVER   0xCE

/* The length of different frame. */
#define LENGTH_XHDSHK_FRAME       9
#define LENGTH_XHEAD_FRAME        5
#define LENGTH_XCMD_FRAME         12
#define LENGTH_XHDATA_FRAME       11
#define LENGTH_XDATA_ADD_FRAME    5
#define LENGTH_XEND_FRAME         6
#define LENGTH_ACK_FRAME          6

/**
  * @brief Atomis commands enumeration definition.
  */
enum {
    ATOMIS_WRITE = 0xD2,          /**< 0xD2: Download Image */
    ATOMIS_ERASE = 0xE4,          /**< 0xE4: Get version information. */
    ATOMIS_RESET = 0x87,          /**< 0x87: Reset MCU. */
    ATOMIS_UPDATE = 0x90,         /**< 0x90: Update parameter settings. */
};

/**
  * @brief Communication mode enumeration definition.
  */
typedef enum {
    COMMUNICATION_UART = 0,
    COMMUNICATION_I2C = 1,
    COMMUNICATION_SPI = 2,
    COMMUNICATION_CAN = 3,
} COMMUNICATION_Mode;

/**
  * @brief Answering status enumeration definition.
  */
typedef enum {
    ACK_FAIL    = 0xA5,
    ACK_SUCCESS = 0x5A,
} ACK_Status;

/**
  * @brief User loader command frame structure definition.
  */
typedef struct {
    unsigned char type;                           /**< Command Type. */
    unsigned char rcvBuf[CMD_PAYLOAD_MAX];        /**< Command Buf All Command contents. */
    unsigned short rcvBufLength;                  /**<  Buffer length. */
} ULOADER_Cmd;

/**
 * @brief Module handle structure definition.
 */
typedef struct _ULOADER_Handle {
    void               *comHandle;  /**< Register base address. */
    FLASH_Handle       *flash;
    CRC_Handle         *crc;
    COMMUNICATION_Mode  mode;
} ULOADER_Handle;

void ULOADER_Init(ULOADER_Handle *handle);
void ULOADER_DeInit(ULOADER_Handle *handle);
BASE_StatusType ULOADER_HandShake(ULOADER_Handle *handle, unsigned int timeout);
BASE_StatusType ULOADER_ReceiveCmd(ULOADER_Handle *handle, ULOADER_Cmd *receiveCmd);
BASE_StatusType ULOADER_CommandExec(ULOADER_Handle *handle, ULOADER_Cmd *execCmd);

BASE_StatusType ULOADER_WriteDate(ULOADER_Handle *handle, unsigned char *wData, unsigned int dataSize,
                                  unsigned int timeout);
BASE_StatusType ULOADER_ReadDate(ULOADER_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                 unsigned short targetByte, unsigned int timeout);
BASE_StatusType ULOADER_Ack(ULOADER_Handle *handle, ACK_Status status, unsigned char index);

BASE_StatusType ULOADER_CMD_Write(ULOADER_Handle *handle, ULOADER_Cmd *command);
BASE_StatusType ULOADER_CMD_Erase(ULOADER_Handle *handle, ULOADER_Cmd *command);
BASE_StatusType ULOADER_CMD_Update(ULOADER_Handle *handle, ULOADER_Cmd *command);
BASE_StatusType ULOADER_CMD_Reset(ULOADER_Handle *handle);
#endif