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
  * @file      smbus_ex.c
  * @author    MCU Driver Team
  * @brief     SMBUS module driver
  * @details   The header file contains the following declaration:
  *             + Setting the Special Function Configuration.
  */

/* Includes ------------------------------------------------------------------*/
#include "smbus_ex.h"

/* Macro definitions ---------------------------------------------------------*/

/**
  * @brief Set data transfer sequence.
  * @param handle: SMBUS handle.
  * @param sequence: data transfer sequence enumeration value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetDataTransferSequenceEx(SMBUS_Handle *handle, SMBUS_DataTransferSequenceType sequence)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cDataTransferSequence(sequence), BASE_STATUS_ERROR);
    /**< Data Transfer Sequence. 0:SMBUS_MOST_BIT_FIRST, 1:SMBUS_LEAST_BIT_FIRST. */
    handle->baseAddress->I2C_MODE.BIT.lit_end = sequence;
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS clock stretching function.
  * @param handle: SMBUS handle.
  * @param clkStretch: clock stretching enumeration value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetSclStretchModeEx(SMBUS_Handle *handle, SMBUS_ClockStretchType clkStretch)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cClockStretchValue(clkStretch), BASE_STATUS_ERROR);
    /**< Clock stretching enable. 0:enable, 1:disable. */
    handle->baseAddress->I2C_MODE.BIT.scl_stretch_disable = clkStretch;
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS SCL low-level timeout.
  * @param handle: SMBUS handle.
  * @param sclLowTimeout: SCL low-level timeout value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetSclLowTimeoutEx(SMBUS_Handle *handle, unsigned int sclLowTimeout)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cSclLowTimeout(sclLowTimeout), BASE_STATUS_ERROR);
    /* The unit of bus free time is SMBUS working clock cycle. */
    handle->baseAddress->I2C_SCL_TIMEOUT.BIT.scl_low_timeout = sclLowTimeout;
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS bus idle threshold value.
  * @param handle: SMBUS handle.
  * @param busFreeTime: bus idle threshold value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetBusFreeTimeEx(SMBUS_Handle *handle, unsigned int busFreeTime)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cBusFreeTime(busFreeTime), BASE_STATUS_ERROR);
    /* The unit of bus free time is SMBUS working clock cycle. */
    handle->baseAddress->I2C_BUS_FREE.BIT.bus_free_time = busFreeTime;
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS slave receive 10-bit slave addressing.
  * @param handle: SMBUS handle.
  * @param arg: slave special function set enumeration value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_Set10BitSlaveEnableEx(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /**< Enable the slave receives the 10bit addressing. */
    handle->baseAddress->I2C_OWN_ADDR.BIT.i2c_10bit_slave_en = BASE_CFG_SET;
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS slave receive device ID address.
  * @param handle: SMBUS handle.
  * @param arg: slave special function set enumeration value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetDeviceIdAddressEnableEx(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /**< Enable the function of receiving device ID addresses. */
    handle->baseAddress->I2C_OWN_ADDR.BIT.i2c_device_id_en = BASE_CFG_SET;
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS slave receive start byte address.
  * @param handle: SMBUS handle.
  * @param arg: slave special function set enumeration value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetStartByteEnableEx(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    handle->baseAddress->I2C_OWN_ADDR.BIT.i2c_start_byte_en = BASE_CFG_SET; /**< Enable receiving START Byte Address. */
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS slave own address mask.
  * @param handle: SMBUS handle.
  * @param addrMask: own address mask.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetOwnAddressMaskEx(SMBUS_Handle *handle,  unsigned int addrMask)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cOwnAddressOrMask(addrMask), BASE_STATUS_ERROR);
    handle->baseAddress->I2C_OWN_ADDR.BIT.own_address_mask = addrMask; /**< Slave's own address mask. */
    return BASE_STATUS_OK;
}

/**
  * @brief Set SMBUS slave XMBus address mask.
  * @param handle: SMBUS handle.
  * @param addrMask: XMBus address mask.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SetOwnXmbAddressMaskEx(SMBUS_Handle *handle, unsigned int addrMask)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(IsXMBusAddressOrMask(addrMask), BASE_STATUS_ERROR);
    handle->baseAddress->XMB_DEV_ADDR.BIT.xmb_address_mask = addrMask; /**< The second own address mask as slave. */
    return BASE_STATUS_OK;
}
