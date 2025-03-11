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
  * @file    smbus_ip.h
  * @author  MCU Driver Team
  * @brief   SMBUS module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the SMBUS.
  *          + Extended Configuration Parameter Struct Definition.
  *          + Register definition structure.
  *          + Timing command enumeration.
  *          + Direct configuration layer interface.
  *          + Basic parameter configuration macro.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_SMBUS_IP_H
#define McuMagicTag_SMBUS_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/* Macro definitions --------------------------------------------------------- */
#ifdef SMBUS_PARAM_CHECK
#define SMBUS_ASSERT_PARAM  BASE_FUNC_ASSERT_PARAM
#define SMBUS_PARAM_CHECK_NO_RET BASE_FUNC_PARAMCHECK_NO_RET
#define SMBUS_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define SMBUS_ASSERT_PARAM(para)  ((void)0U)
#define SMBUS_PARAM_CHECK_NO_RET(para) ((void)0U)
#define SMBUS_PARAM_CHECK_WITH_RET(para, ret) ((void)0U)
#endif

/**
  * @addtogroup SMBUS
  * @{
  */

/**
  * @defgroup SMBUS_IP SMBUS_IP
  * @brief SMBUS_IP: smbus_v0
  * @{
  */

#define SMBUS_IGNORE_NAK_ENABLE        BASE_CFG_ENABLE  /**< Ignore acknowledgment configuration enable. */
#define SMBUS_IGNORE_NAK_DISABLE       BASE_CFG_DISABLE /**< Ignore acknowledgment configuration disable. */

#define SMBUS_STANDARD_FREQ_TH     100000 /**< Standard mode,the frequency band is less than or equal to 100 kHz. */

#define SMBUS_INTR_RAW_ALL_ENABLE  0x00FFFFFFU /**< 1111111111111 */
#define SMBUS_INTR_RAW_ALL_DISABLE  0x00000000U /**< 0000000000000 */

#define SMBUS_INTR_EN_ALL_ENABLE   0x00FFFFFFU /**< 1111111111111 */
#define SMBUS_INTR_EN_ALL_DISABLE  0x00000000U /**< 0000000000000 */

#define SMBUS_ONCE_TRANS_MAX_NUM   0x400

#define XMBUS_OWN_ADDRESS_MASK     0x3FF
/**
  * @defgroup SMBUS_Param_Def SMBUS Parameters Definition.
  * @brief Definition of SMBUS configuration parameters.
  * @{
  */
/* Typedef definitions -------------------------------------------------------*/
/**
  * @brief Address Mode Selection Enumeration Definition
  */
typedef enum {
    SMBUS_7_BITS  = 0x00000000U,
    SMBUS_10_BITS = 0x00000001U,
} SMBUS_AddressMode;

/**
  * @brief SMBUS mode selection enumeration definition
  */
typedef enum {
    SMBUS_MODE_SELECT_NONE         = 0x00000000U,
    SMBUS_MODE_SELECT_MASTER_ONLY  = 0x00000001U,
    SMBUS_MODE_SELECT_SLAVE_ONLY   = 0x00000002U,
    SMBUS_MODE_SELECT_MASTER_SLAVE = 0x00000003U,
} SMBUS_ModeSelectType;

/**
  * @brief Callback Function ID Enumeration Definition
  */
typedef enum {
    SMBUS_MASTER_TX_COMPLETE_CB_ID           = 0x00000000U,
    SMBUS_MASTER_RX_COMPLETE_CB_ID           = 0x00000001U,
    SMBUS_SLAVE_TX_COMPLETE_CB_ID            = 0x00000002U,
    SMBUS_SLAVE_RX_COMPLETE_CB_ID            = 0x00000003U,
    SMBUS_SLAVE_WAKEUP_COMPLETE_CB_ID        = 0x00000004U,
    SMBUS_ERROR_CB_ID                        = 0x00000005U,
    SMBUS_ADDR_MATCH_COMPLETE_CB_ID          = 0x00000006U,
    SMBUS_MSTAER_SEND_STOP_COMPLETE_CB_ID    = 0x00000007U,
    SMBUS_SLAVE_DETECTED_STOP_COMPLETE_CB_ID = 0x00000008U,
    SMBUS_ALERT_COMPLETE_CB_ID               = 0x00000009U,
} SMBUS_CallbackId;

/**
  * @brief SMBUS data transfer sequence enumeration definition.
  */
typedef enum {
    SMBUS_MOST_BIT_FIRST        = 0x00000000U,
    SMBUS_LEAST_BIT_FIRST      = 0x00000001U,
} SMBUS_DataTransferSequenceType;

/**
  * @brief SMBUS clock stretching enumeration definition.
  */
typedef enum {
    SMBUS_CLOCK_STRETCH_ENABLE   = 0x00000000U,
    SMBUS_CLOCK_STRETCH_DISABLE  = 0x00000001U,
} SMBUS_ClockStretchType;

/**
  * @brief SMBUS transmission frame format enumeration definition.
  */
typedef enum {
    SMBUS_FRAME_NONE                = 0x00000000U,
    SMBUS_FRAME_FIRST               = 0x00000001U,
    SMBUS_FRAME_START               = 0x00000002U,
    SMBUS_FRAME_MIDDLE              = 0x00000004U,
    SMBUS_FRAME_PEC                 = 0x00000008U,
    SMBUS_FRAME_STOP                = 0x00000010U,
    SMBUS_FRAME_FULL                = 0x00000020U,
    SMBUS_FRAME_BLOCK_PROCESSING    = 0x00000040U,
    SMBUS_FRAME_ADDR_MATCH          = 0x00000080U,
    SMBUS_FRAME_EXCUTE_CMD_MATCH    = 0x00000100U,
} SMBUS_FrameFormat;

/**
  * @brief XMBus alert type enumeration definition.
  */
typedef enum {
    SMBUS_ALERT_DISABLE    = 0x00000000U,
    SMBUS_ALERT_ENABLE     = 0x00000001U,
} SMBUS_AlertType;

/**
  * @brief SMBUS extend handle, configuring some special parameters.
  */
typedef struct {
    unsigned char *data;
    unsigned int   dataSize;
} SMBUS_DataBuffer;

/**
  * @brief SMBUS extend handle, configuring some special parameters.
  */
typedef struct {
    unsigned int              spikeFilterTime;          /**< The SDA and SCL Glitch Filtering Configuration. */
    unsigned int              sdaDelayTime;             /**< The SDA delay sampling configuration. */
    unsigned int              slaveOwnXmbAddressEnable; /**< Enable the I2C second own address function. */
    unsigned int              slaveOwnXmbAddress;       /**< The second own address as slave. */
} SMBUS_ExtendHandle;

/**
  * @brief User-defined callback function.
  */
typedef struct {
    void (*MasterTxCplCallback)(void* handle);        /**< Master Sending completion callback function. */
    void (*MasterRxCplCallback)(void* handle);        /**< Slave Receive completion callback function. */
    void (*SlaveTxCplCallback)(void* handle);         /**< Master Sending completion callback function. */
    void (*SlaveRxCplCallback)(void* handle);         /**< Slave Receive completion callback function. */
    void (*ErrorCallback)(void* handle);              /**< Error callback function. */
    void (*SlaveWakeupCallback)(void* handle);        /**< Slave Wakeup Callback Function. */
    void (*ExecuteCmdCallback)(void *handle);         /**< Callback function for executing smbus commands. */
    void (*AddrMatchCplCallback)(void *handle);       /**< Address matching callback function. */
    void (*MasterSendStopCplCallback)(void *handle);  /**< The master has sent a stop message callback function. */
    void (*SlaveDetectedStopCplCallback)(void *handle); /**< The slave has received a stop message callback function. */
    void (*AlertCallback)(void *handle);              /**< The alert callback function. */
    void (*SclLowTimeoutCallback)(void *handle);      /**< The scl low timeout callback function. */
} SMBUS_UserCallBack;

/**
  * @}
  */

/* Parameter check definition-------------------------------------------*/
/**
  * @brief Check SMBUS function mode selection.
  * @param functionMode SMBUS function mode type.
  * @retval true
  * @retval false
  */
static inline bool IsSmbusFunctionMode(SMBUS_ModeSelectType functionMode)
{
    return (functionMode == SMBUS_MODE_SELECT_NONE ||
            functionMode == SMBUS_MODE_SELECT_MASTER_ONLY ||
            functionMode == SMBUS_MODE_SELECT_SLAVE_ONLY ||
            functionMode == SMBUS_MODE_SELECT_MASTER_SLAVE);
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_SMBUS_IP_H */