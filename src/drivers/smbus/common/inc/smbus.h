/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file    smbus.h
  * @author  MCU Driver Team,
  * @brief   SMBUS module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the SMBUS.
  *          + Initialization and de-initialization functions.
  *          + Peripheral transmit and receiving functions.
  *          + SMBUS parameter handle definition.
  *          + Basic Configuration Parameter Enumeration Definition.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_SMBUS_H
#define McuMagicTag_SMBUS_H

/* Includes ------------------------------------------------------------------*/
#include "i2c_ip.h"
#include "smbus_ip.h"
#include "dma.h"

/**
  * @defgroup SMBUS SMBUS
  * @brief SMBUS module.
  * @{
  */

/**
  * @defgroup SMBUS_Common SMBUS Common
  * @brief SMBUS common external module.
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/

/**
  * @defgroup SMBUS_Handle_Definition SMBUS Handle Definition
  * @{
  */

/**
  * @brief Module Status Enumeration Definition
  */
typedef enum {
    SMBUS_STATE_RESET           = 0x00000000U,
    SMBUS_STATE_READY           = 0x00000001U,
    SMBUS_STATE_BUSY            = 0x00000002U,
    SMBUS_STATE_BUSY_MASTER_TX  = 0x00000004U,
    SMBUS_STATE_BUSY_MASTER_RX  = 0x00000008U,
    SMBUS_STATE_BUSY_SLAVE_TX   = 0x00000010U,
    SMBUS_STATE_BUSY_SLAVE_RX   = 0x00000020U,
    SMBUS_STATE_TIMEOUT         = 0x00000040U,
    SMBUS_STATE_ERROR           = 0x00000080U,
} SMBUS_StateType;

/**
 * @brief Module handle structure definition
 */
typedef struct _SMBUS_Handle {
    I2C_RegStruct           *baseAddress;      /**< Register base address. */
    SMBUS_ModeSelectType     functionMode;     /**< Set master or slave. */
    SMBUS_AddressMode        addrMode;         /**< 7bit or 10bit. */
    unsigned int             slaveOwnAddress;  /**< Own address as slave. */
    unsigned int             sdaHoldTime;      /**< SDA hold time. */
    unsigned int             freq;             /**< Operating Frequency. */
    unsigned int             ignoreAckFlag;    /**< Ignore the response flag bit. */
    unsigned int             generalCallMode;  /**< General call mode. */

    unsigned int             rxWaterMark;      /**< RX threshold configuration. */
    unsigned int             txWaterMark;      /**< TX threshold configuration. */
    unsigned int             rxDmaCh;          /**< RX DMA channel */
    unsigned int             txDmaCh;          /**< TX DMA channel */
    DMA_Handle              *dmaHandle;        /**< DMA handle */

    volatile unsigned int    deviceAddress;
    volatile unsigned char  *transferBuff;     /**< Transmission Data buffer. */
    volatile unsigned int    transferSize;     /**< Transmission Data Length. */
    volatile unsigned int    transferCount;    /**< Transferred Data Count. */
    volatile unsigned int    frameOpt;

    unsigned int             timeout;          /**< Timeout period. */
    volatile unsigned char   sendAddrStatus;
    volatile unsigned int    txReadCmdCount;
    volatile unsigned int    dmaTransferSize;
    volatile unsigned int    excuteCmdFlag;

    volatile SMBUS_StateType  state;            /**< Running Status. */
    BASE_StatusType           errorCode;        /**< Error Code. */
    SMBUS_UserCallBack        userCallBack;     /**< User-defined callback function. */
    SMBUS_ExtendHandle        handleEx;                 /**< Extend handle,  configuring some special parameters. */
} SMBUS_Handle;
/**
  * @}
  */

/**
  * @defgroup SMBUS_API_Declaration SMBUS HAL API
  * @{
  */
/**
 * @brief Callback Function Type Definition.
 */
typedef void (*SMBUS_CallbackFunType)(void *handle);

/* Function Interface Definition -------------------------------------------------------*/

BASE_StatusType HAL_SMBUS_Init(SMBUS_Handle *handle);
BASE_StatusType HAL_SMBUS_Deinit(SMBUS_Handle *handle);
BASE_StatusType HAL_SMBUS_RegisterCallback(SMBUS_Handle *handle, SMBUS_CallbackId callbackID,
                                           SMBUS_CallbackFunType pcallback);

BASE_StatusType HAL_SMBUS_MasterReadBlocking(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                             unsigned int timeout, unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_MasterWriteBlocking(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                              unsigned int timeout, unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_SlaveReadBlocking(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int timeout,
                                            unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_SlaveWriteBlocking(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int timeout,
                                             unsigned int frameOpt);

BASE_StatusType HAL_SMBUS_MasterReadIT(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                       unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_MasterWriteIT(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                        unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_SlaveReadIT(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_SlaveWriteIT(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt);

BASE_StatusType HAL_SMBUS_MasterReadDMA(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                        unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_MasterWriteDMA(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                         unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_SlaveReadDMA(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt);
BASE_StatusType HAL_SMBUS_SlaveWriteDMA(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt);

void HAL_SMBUS_IrqHandler(void *handle);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_SMBUS_H */