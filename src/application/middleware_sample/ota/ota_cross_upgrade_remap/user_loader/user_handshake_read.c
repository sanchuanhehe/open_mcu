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
  * @file    user_handshake_read.c
  * @author  MCU Driver Team
  * @brief   This file provides the user handshake and read functions to manage the following functionalities of
  *          the user loader.
  *          + Receiving handshake frames functions.
  *          + UART, I2C, SPI read blocking adapt functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "user_handshake_read.h"

/* Macro definitions --------------------------------------------------------- */
#define SYSTICK_MS_DIV 1000
#define ACK_SIZE 6
#define RECEIVE_ONE_BYTE_DELAY_TIME 40
#define RECEIVE_HAND_SHAKE_FRAME_DELAY_TIME 500

/**
  * @brief Check whether the automatic baud rate detection is complete.
  * @param handle UART handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType IsActiveUart(UART_Handle *handle)
{
    if (handle->baseAddress->UART_RIS.BIT.abdcris == 1 && handle->baseAddress->UART_RIS.BIT.abderis == 0 &&
        handle->baseAddress->UART_ABDEN.BIT.abdbusy == 0) {
        HAL_UART_DisableBaudDetectionEx(handle); /* Auto-baud detection complete. */
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief User receive one byte.
  * @param handle UART handle.
  * @param targetByte the target byte.
  * @param timeout Timeout period, unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType UartReceiveTargetByte(UART_Handle *handle, unsigned char targetData, unsigned int timeout)
{
    unsigned int preTick = DCL_SYSTICK_GetTick(); /* Get current tick. */
    unsigned int curTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / SYSTICK_MS_DIV * timeout;
    unsigned char headFrame;

    while (true) {
        /* Receives handshake frames sent by the host. */
        headFrame = DCL_UART_ReadData(handle->baseAddress);
        if (headFrame == targetData) {
            return BASE_STATUS_OK;
        }
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* Check timeout. */
            break;
        }
        preTick = curTick;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief User loader receiving handshake frame.
  * @param handle UART handle.
  * @param buf Address of the data buffer to be saved.
  * @param length the size of the data to be receiving.
  * @param targetByte the target byte.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType UloaderReceiveHandShakeFrame(UART_Handle *handle, unsigned char *buf, unsigned int length,
                                             unsigned char targetByte)
{
    BASE_StatusType ret;
    if (IsActiveUart(handle) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    /* Receives handshake frames sent ack frame to the host. */
    ret = HAL_UART_WriteBlocking(handle, buf, ACK_SIZE, RECEIVE_HAND_SHAKE_FRAME_DELAY_TIME);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    /* Receives handshake frames sent by the host. */
    ret = UartReceiveTargetByte(handle, targetByte, RECEIVE_HAND_SHAKE_FRAME_DELAY_TIME);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    buf[0] = targetByte; /* receive target byte, and then recive data. */
    ret = HAL_UART_ReadBlocking(handle, &buf[1], (length - 1), RECEIVE_HAND_SHAKE_FRAME_DELAY_TIME);
    return ret;
}

/**
  * @brief Blocking read data in UART mode.
  * @param handle UART handle.
  * @param rData Address of the data buffer to be saved.
  * @param dataSize the size of the data to be receiving.
  * @param targetByte the target byte.
  * @param timeout Timeout period, unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType ULOADER_UART_ReadBlocking(UART_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                          unsigned char targetByte, unsigned int timeout)
{
    BASE_StatusType ret;
    /* Receives handshake frames sent by the host. */
    ret = UartReceiveTargetByte(handle, targetByte, timeout);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    rData[0] = targetByte; /* receive target byte, and then recive data. */
    ret = HAL_UART_ReadBlocking(handle, &rData[1], (dataSize - 1), timeout);
    return ret;
}

/**
  * @brief Blocking read data in I2C mode.
  * @param handle I2C handle.
  * @param rData Address of the data buffer to be saved.
  * @param dataSize the size of the data to be receiving.
  * @param timeout Timeout period, unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType ULOADER_I2C_ReadBlocking(I2C_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                         unsigned int timeout)
{
    BASE_FUNC_UNUSED(handle); /* Compiler prevention alarm */
    BASE_FUNC_UNUSED(rData);
    BASE_FUNC_UNUSED(dataSize);
    BASE_FUNC_UNUSED(timeout);
    return BASE_STATUS_OK;
}

/**
  * @brief Blocking read data in SPI mode.
  * @param handle SPI handle.
  * @param rData Address of the data buffer to be saved.
  * @param dataSize the size of the data to be receiving.
  * @param targetByte the target byte.
  * @param timeout Timeout period, unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType ULOADER_SPI_ReadBlocking(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                         unsigned char targetByte, unsigned int timeout)
{
    BASE_FUNC_UNUSED(handle); /* Compiler prevention alarm */
    BASE_FUNC_UNUSED(rData);
    BASE_FUNC_UNUSED(dataSize);
    BASE_FUNC_UNUSED(targetByte);
    BASE_FUNC_UNUSED(timeout);
    return BASE_STATUS_OK;
}