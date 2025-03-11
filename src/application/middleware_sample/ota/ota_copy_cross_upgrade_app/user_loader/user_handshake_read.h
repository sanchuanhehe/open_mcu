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
  * @file    user_handshake_read.h
  * @author  MCU Driver Team
  * @brief   This file provides the user handshake and read functions to manage the following functionalities of
  *          the user loader.
  *          + Receiving handshake frames functions.
  *          + UART, I2C, SPI read blocking adapt functions.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_USER_HADNSHAKE_READ_H
#define McuMagicTag_USER_HADNSHAKE_READ_H

/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "uart_ex.h"
#include "i2c.h"
#include "i2c_ex.h"
#include "spi.h"
#include "spi_ex.h"
#include "can.h"
#include "iocmg.h"

BASE_StatusType UloaderReceiveHandShakeFrame(UART_Handle *handle, unsigned char *buf, unsigned int lenth,
                                             unsigned char targetByte);
BASE_StatusType ULOADER_UART_ReadBlocking(UART_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                          unsigned char targetByte, unsigned int timeout);
BASE_StatusType ULOADER_I2C_ReadBlocking(I2C_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                         unsigned int timeout);
BASE_StatusType ULOADER_SPI_ReadBlocking(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                         unsigned char targetByte, unsigned int timeout);

#endif