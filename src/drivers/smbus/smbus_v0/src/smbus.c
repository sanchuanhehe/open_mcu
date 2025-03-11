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
  * @file    smbus.c
  * @author  MCU Driver Team
  * @brief   SMBUS module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the SMBUS.
  *          + Initialization and de-initialization functions
  *          + Peripheral Control functions
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "smbus.h"

/* Macro definitions ---------------------------------------------------------*/
#define SMBUS_INTERFACE_INDEX_0         0
#define SMBUS_INTERFACE_INDEX_1         1
#define SMBUS_MAX_INDEX_NUM             2

#define SMBUS_MASTER_STATUS             0x00
#define SMBUS_SLAVE_STATUS              0x01

#define SMBUS_MAX_FIFO_SIZE             15
#define SMBUS_MAX_DEV_ADDR              0x3FF

#define SMBUS_OPERATION_WRITE           0
#define SMBUS_OPERATION_READ            1

#define SMBUS_SEND_ADDR_STATUS_NONE     0
#define SMBUS_SEND_ADDR_STATUS_WRITE    1
#define SMBUS_SEND_ADDR_STATUS_READ     2

/* Enable scl_low_timeout\mst_cmd_done\arb_lost\tx_fifo_not_full\rx_fifo_not_empty\mst_rx_ack_unmatch */
#define SMBUS_CFG_INTERRUPT_MASTER_RX   0x3843
/* Enable scl_low_timeout\mst_cmd_done\arb_lost\tx_fifo_not_full\mst_rx_ack_unmatch */
#define SMBUS_CFG_INTERRUPT_MASTER_TX   0x3841
/* Enable slv_addr_match_int\slv_rx_ack_unmatch_int\stop_det_int */
#define SMBUS_CFG_INTERRUPT_SLAVE       0x302200
#define SMBUS_TICK_MS_DIV               1000

#define SMBUS_INTR_RAW_SLAVE_ADDR_MATCH_MASK      (0x1 << 21)
#define SMBUS_INTR_RAW_SLAVE_ACK_UNMATCH_MASK     (0x1 << 20)
#define SMBUS_INTR_RAW_SLAVE_PEC_CHECK_FAIL_MASK  (0x1 << 19)
#define SMBUS_INTR_RAW_MASTER_PEC_CHECK_FAIL_MASK (0x1 << 18)
#define SMBUS_INTR_RAW_SLAVE_CMD_INT1_MASK        (0x1 << 17)
#define SMBUS_INTR_RAW_MASTER_CMD_INT1_MASK       (0x1 << 16)
#define SMBUS_INTR_RAW_SMB_SUSPEND_MASK           (0x1 << 15)
#define SMBUS_INTR_RAW_SMB_ALERT_MASK             (0x1 << 14)
#define SMBUS_INTR_RAW_SCL_LOW_TIMEOUT_MASK       (0x1 << 13)
#define SMBUS_INTR_RAW_ALL_CMD_DONE_MASK          (0x1 << 12)
#define SMBUS_INTR_RAW_ARB_LOST_MASK              (0x1 << 11)
#define SMBUS_INTR_RAW_START_DET_MASK             (0x1 << 10)
#define SMBUS_INTR_RAW_STOP_DET_MASK              (0x1 << 9)
#define SMBUS_INTR_RAW_TX_DATA_REQUEST_MASK       (0x1 << 8)
#define SMBUS_INTR_RAW_RX_DATA_READY_MASK         (0x1 << 7)
#define SMBUS_INTR_RAW_TX_FIFO_NOT_FULL_MASK      (0x1 << 6)
#define SMBUS_INTR_RAW_TX_FIFO_EMPTY_MASK         (0x1 << 5)
#define SMBUS_INTR_RAW_TX_LE_WATERMARK_MASK       (0x1 << 4)
#define SMBUS_INTR_RAW_RX_FIFO_FULL_MASK          (0x1 << 3)
#define SMBUS_INTR_RAW_RX_GE_WATERMARK_MASK       (0x1 << 2)
#define SMBUS_INTR_RAW_RX_FIFO_NOT_EMPTY_MASK     (0x1 << 1)
#define SMBUS_INTR_RAW_ACK_BIT_UNMATCH_MASK       (0x1 << 0)

#define SMBUS_10BIT_SLAVE_READ_ADDR_MASK          (0xFEFF0000)
#define SMBUS_10BIT_SLAVE_WRITE_ADDR_MASK         (0x0000FEFF)
#define SMBUS_10BIT_SLAVE_READ_OPT_MASK           (0x01000000)

#define SMBUS_7BIT_SLAVE_READ_ADDR_MASK           (0x00FE0000)
#define SMBUS_7BIT_SLAVE_WRITE_ADDR_MASK          (0x000000FE)
#define SMBUS_7BIT_SLAVE_READ_OPT_MASK            (0x00010000)

#define SMBUS_SLAVE_WRITE_ADDR_POS                 8
#define SMBUS_SLAVE_READ_FIX_ADDR_POS              24
#define SMBUS_SLAVE_READ_DEV_ADDR_POS              16
#define SMBUS_SLAVE_ADDR_MASK                      0xFF
#define SMBUS_10BIT_SLAVE_ADDR_POS                 16

#define HIGH_HOLD_TIME_POS                       16
#define HIGH_HOLD_TIME_MASK                      0xFFFF0000
#define LOW_HOLD_TIME_MASK                       0x0000FFFF

#define DMA_RX_CHANNEL_POS                       8
#define DMA_CHANNEL_MASK                         0x00FF

#define COMMAND_ALL_DONE                         0
#define BUS_IS_FREE                              1
#define SLAVE_ADDRESS_MATCH                      2
#define TX_FIFO_NOT_FULL                         3
#define RX_FIFO_NOT_EMPTY                        4
#define TX_FIFO_EMPTY                            5

#define SMBUS_FREQ_HIGH_PARAMTER                   8
#define SMBUS_FREQ_LOW_PARAMTER                    9
#define SMBUS_ERROR_BIT_MASK                       0x5C2801 /* slv_rx_ack_unmatch\arb_lost\mst_rx_ack_unmatch */
#define SMBUS_SCL_LOW_TIMEOUT_MASK                 0x2000
#define SMBUS_SLAVE_WAKEUP_RAW_MASK                0x800000
#define SMBUS_SLAVE_WAKEUP_RAW_POS                 23
#define SMBUS_ALERT_RAW_MASK                       0x4000
#define SMBUS_ALERT_RAW_POS                        14

static BASE_StatusType SMBusDmaMasterReadData(SMBUS_Handle *handle, unsigned int size);
static BASE_StatusType SMBusDmaMasterWriteData(SMBUS_Handle *handle, unsigned int size);
static BASE_StatusType SMBusDmaSlaveReadData(SMBUS_Handle *handle, unsigned int size);
static BASE_StatusType SMBusDmaSlaveWriteData(SMBUS_Handle *handle, unsigned int size);

/* Some global parameters used for module internal operations */
static volatile unsigned int g_smbusInternalTxBuffDMA[I2C_ONCE_TRANS_MAX_NUM] = {0};
/**
  * @brief Check all initial configuration parameters.
  * @param handle SMBUS handle.
  * @param clockFreq  SMBUS work clock freq;
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusCheckAllInitParameters(SMBUS_Handle *handle, unsigned int clockFreq)
{
    /* Check the configuration of basic function parameters. */
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cFunctionMode(handle->functionMode), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cAddressMode(handle->addrMode), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cSdaHoldTime(handle->sdaHoldTime), BASE_STATUS_ERROR);
    /* Check whether the I2C freq is valid. */
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cFreq(handle->freq), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET((clockFreq > 0), BASE_STATUS_ERROR);

    if (handle->freq > clockFreq) {
        return BASE_STATUS_ERROR;
    }
    /* Check the configuration of basic function parameters. */
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cIgnoreAckFlag(handle->ignoreAckFlag), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cTxWaterMark(handle->txWaterMark), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cRxWaterMark(handle->rxWaterMark), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cSpikeFilterTime(handle->handleEx.spikeFilterTime), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(IsI2cSdaDelayTime(handle->handleEx.sdaDelayTime), BASE_STATUS_ERROR);

    /* Checking the own address and generalCall parameter enable when is used as slave. */
    if (handle->functionMode == SMBUS_MODE_SELECT_SLAVE_ONLY ||
        handle->functionMode == SMBUS_MODE_SELECT_MASTER_SLAVE) {
        SMBUS_PARAM_CHECK_WITH_RET(IsI2cOwnAddressOrMask(handle->slaveOwnAddress), BASE_STATUS_ERROR);
        SMBUS_PARAM_CHECK_WITH_RET(IsI2cGeneralCallMode(handle->generalCallMode), BASE_STATUS_ERROR);
        SMBUS_PARAM_CHECK_WITH_RET(IsXMBusAddressEnable(handle->handleEx.slaveOwnXmbAddressEnable), BASE_STATUS_ERROR);
        SMBUS_PARAM_CHECK_WITH_RET(IsXMBusAddressOrMask(handle->handleEx.slaveOwnXmbAddress), BASE_STATUS_ERROR);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Configuring the SMBUS Slave Device Address.
  * @param handle SMBUS handle.
  * @param devAddr Slave device address
  * @retval None.
  */
static void SMBusSetSlaveDevAddr(SMBUS_Handle *handle, const unsigned int devAddr)
{
    unsigned int addr;

    if (handle->addrMode == SMBUS_10_BITS) {
        /* The upper 16 bits are the read operation address, and the lower 16 bits are the write operation address. */
        addr = (((devAddr << 16) & SMBUS_10BIT_SLAVE_READ_ADDR_MASK) | SMBUS_10BIT_SLAVE_READ_OPT_MASK) |
               (devAddr & SMBUS_10BIT_SLAVE_WRITE_ADDR_MASK);
    } else {
        /* The upper 16 bits are the read operation address, and the lower 16 bits are the write operation address. */
        addr = (((devAddr << 16) & SMBUS_7BIT_SLAVE_READ_ADDR_MASK) | SMBUS_7BIT_SLAVE_READ_OPT_MASK) |
               (devAddr & SMBUS_7BIT_SLAVE_WRITE_ADDR_MASK);
    }

    handle->deviceAddress = addr;
}

/**
  * @brief SMBUS Bus clear.
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusBusClear(SMBUS_Handle *handle)
{
    handle->state = SMBUS_STATE_READY;
    handle->baseAddress->I2C_MODE.BIT.mst_slv_function = SMBUS_STATE_RESET;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;

    /* Set the SCL and SDA pins of the SMBUS to GPIO mode. */
    handle->baseAddress->I2C_CTRL2.BIT.gpio_mode = BASE_CFG_ENABLE;
    handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_ENABLE;
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_ENABLE;
    /* The device that controls the bus to be pulled down needs to release the bus within the 9 clocks. */
    for (unsigned int index = 0; index < 9; index++) {
        handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_UNSET;
        BASE_FUNC_DELAY_US(5); /* The I2C timing is required. The delay is about 5 μs. */
        handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_SET;
        BASE_FUNC_DELAY_US(5); /* The I2C timing is required. The delay is about 5 μs. */
    }
    handle->baseAddress->I2C_CTRL2.BIT.force_scl = BASE_CFG_ENABLE;
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_ENABLE;
    /* I2C start */
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_UNSET;
    BASE_FUNC_DELAY_US(10); /* The I2C timing is required. The delay is about 10 μs. */
    /* I2C stop */
    handle->baseAddress->I2C_CTRL2.BIT.force_sda = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL2.BIT.gpio_mode = BASE_CFG_DISABLE; /* Exit the I2C GPIO mode. */
}

/**
  * @brief Setting Error Handling.
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusSetErrorHandling(SMBUS_Handle *handle)
{
    handle->state = SMBUS_STATE_READY;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;

    /* If the low level times out, the SMBUS bus is cleared and the bus is expected to be released. */
    if (handle->baseAddress->I2C_INTR_RAW.BIT.scl_low_timeout_raw == BASE_CFG_ENABLE) {
        SMBusBusClear(handle);
        handle->baseAddress->I2C_INTR_RAW.BIT.scl_low_timeout_raw = BASE_CFG_ENABLE;
    }

    if (handle->errorCode != BASE_STATUS_OK && handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
}

/**
  * @brief Item is checked for readiness.
  * @param handle SMBUS handle.
  * @param checkItem The item to be checked.
  * @param opt Read or write flag.
  * @retval false, item is not ready. true, item is ready.
  */
static unsigned int SMBusCheckItemStatus(SMBUS_Handle *handle, unsigned int checkItem, unsigned int opt)
{
    unsigned int ret = 0;
    unsigned int tempStatusValue = 0;
    switch (checkItem) {
        case COMMAND_ALL_DONE:
            /* The 0x1200 is the bit of mst_cmd_done_raw and stop_det_raw. */
            tempStatusValue = (handle->baseAddress->I2C_INTR_RAW.reg & 0x1200); /* Check the I2C is all command done. */
            ret = tempStatusValue;
            break;
        case BUS_IS_FREE:
            /* The SMBUS bus is free. */
            ret = handle->baseAddress->I2C_FSM_STAT.BIT.i2c_bus_free;
            break;
        case SLAVE_ADDRESS_MATCH:
            /* Slave servers are matched */
            tempStatusValue = (handle->baseAddress->I2C_RX_ADDR.BIT.rx_rw == opt);
            tempStatusValue |= handle->baseAddress->I2C_INTR_RAW.BIT.slv_addr_match_raw;
            ret = tempStatusValue;
            break;
        case TX_FIFO_NOT_FULL:
            /* Tx fifo is not full. */
            ret = (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < SMBUS_MAX_FIFO_SIZE);
            break;
        case RX_FIFO_NOT_EMPTY:
            /* Rx fifo is not empty. */
            ret = handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_vld_num;
            break;
        case TX_FIFO_EMPTY:
            /* Tx fifo is not full. */
            ret = (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num == 0);
            break;
        default:
            break;
    }
    return ret;
}

/**
  * @brief Wait for the item status to be ready.
  * @param handle SMBUS handle.
  * @param checkItem The item to be checked.
  * @param opt Read or write flag.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusWaitStatusReady(SMBUS_Handle *handle, unsigned int checkItem, unsigned int opt)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / SMBUS_TICK_MS_DIV * handle->timeout;

    while (true) {
        if (handle->baseAddress->I2C_INTR_RAW.reg & SMBUS_ERROR_BIT_MASK) {
            SMBusSetErrorHandling(handle);
            return BASE_STATUS_ERROR;
        }
        
        /* Check the status of the item is ready. */
        if (SMBusCheckItemStatus(handle, checkItem, opt)) {
            if (checkItem == SLAVE_ADDRESS_MATCH) {
                /* Clear slave address match raw interrupt */
                handle->baseAddress->I2C_INTR_RAW.BIT.slv_addr_match_raw = BASE_CFG_SET;
            }
            return BASE_STATUS_OK;
        }

        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* Check timeout. */
            break;
        }
        preTick = curTick;
    }
    return BASE_STATUS_TIMEOUT;
}

/**
  * @brief Set the sending data and operation commands.
  * @param handle SMBUS handle.
  * @param cmd Operation commands.
  * @param data Sending data.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusSetTxFIFODataAndCmd(SMBUS_Handle *handle, I2C_CmdType cmd, unsigned char data)
{
    BASE_StatusType ret;
    unsigned int temp;

    ret = SMBusWaitStatusReady(handle, TX_FIFO_NOT_FULL, SMBUS_OPERATION_WRITE);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }
    /* The 8 to 11 bits are the Timing Commands, and the 0 to 7 bits are the write data. */
    temp = (((unsigned int)cmd << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
    temp |= (((unsigned int)data << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
    handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
    return BASE_STATUS_OK;
}

/**
  * @brief Send a write command to the slave device.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusSendSlaveAddressWriteCmd(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    unsigned char addr;
    /* Write slave address */
    if (handle->addrMode == SMBUS_10_BITS) { /* 10bit address Configuration */
        if (handle->transferCount == 0) {
            /* The first address of a 10-bit address configuration */
            addr = (unsigned char)((handle->deviceAddress >> SMBUS_SLAVE_WRITE_ADDR_POS) &
                                   SMBUS_SLAVE_ADDR_MASK);
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
            /* The second address of the 10-bit address configuration */
            addr = (unsigned char)(handle->deviceAddress & SMBUS_SLAVE_ADDR_MASK);
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        } else {
            addr = (unsigned char)((handle->deviceAddress >> SMBUS_SLAVE_WRITE_ADDR_POS) &
                                   SMBUS_SLAVE_ADDR_MASK);
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        }
    } else { /* 7bit address Configuration */
        addr = (unsigned char)(handle->deviceAddress & SMBUS_SLAVE_ADDR_MASK);
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Send a read command to the slave device.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusSendSlaveAddressReadCmd(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    unsigned char addr;
    /* Write slave address */
    if (handle->addrMode == SMBUS_10_BITS) { /* 10bit address Configuration */
        if (handle->transferCount == 0) {
            /* The first address of a 10-bit address configuration */
            addr = (unsigned char)((handle->deviceAddress >> SMBUS_SLAVE_READ_FIX_ADDR_POS) &
                                   SMBUS_SLAVE_ADDR_MASK);
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
            /* The second address of the 10-bit address configuration */
            addr = (unsigned char)((handle->deviceAddress >> SMBUS_SLAVE_READ_DEV_ADDR_POS) &
                                   SMBUS_SLAVE_ADDR_MASK);
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        } else {
            addr = (unsigned char)((handle->deviceAddress >> SMBUS_SLAVE_READ_FIX_ADDR_POS) &
                                   SMBUS_SLAVE_ADDR_MASK);
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
        }
    } else { /* 7bit address Configuration */
        addr = (unsigned char)((handle->deviceAddress >> SMBUS_SLAVE_READ_DEV_ADDR_POS) &
                               SMBUS_SLAVE_ADDR_MASK);
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, addr);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Master send stop command in blocking.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusBlockingSendStopCommand(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_P, 0);
    if (ret != BASE_STATUS_OK) {
        SMBusSetErrorHandling(handle);
        return ret;
    }
    /* Wait until all commands are executed. */
    ret = SMBusWaitStatusReady(handle, COMMAND_ALL_DONE, SMBUS_OPERATION_WRITE);
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_EN.reg = I2C_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = I2C_INTR_RAW_ALL_ENABLE;
    handle->state = SMBUS_STATE_READY;
    return ret;
}

/**
  * @brief The step of receive normal data in blocking as master.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusBlockingMasterRxLastData(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    /* Reads the last frame of data without ack. */
    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        /* Reads the last frame of data without ack. */
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_RPEC_TNACK_S_TPEC_RNACK, 0);
    } else {
        if ((handle->frameOpt & SMBUS_FRAME_BLOCK_PROCESSING) == SMBUS_FRAME_BLOCK_PROCESSING) {
            /* Reads the last frame of data with ack. */
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TACK_S_TD_RACK, 0);
        } else {
            /* Reads the last frame of data without ack. */
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TNACK_S_TD_RNACK, 0);
        }
    }
    return ret;
}

/**
  * @brief The step of receive normal data in blocking as master.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusBlockingMasterRxDataOptStepNormal(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    if (handle->transferBuff == NULL || handle->transferSize == 0) {
        return BASE_STATUS_OK;
    }

    while (handle->transferCount < handle->transferSize) {
        if (handle->transferCount == handle->transferSize - 1) {
            /* Reads the last frame of data. */
            ret = SMBusBlockingMasterRxLastData(handle);
        } else {
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TACK_S_TD_RACK, 0);
        }
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
         /* Wait the RX FIFO is not empty. */
        ret = SMBusWaitStatusReady(handle, RX_FIFO_NOT_EMPTY, SMBUS_OPERATION_READ);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        /* Obtains the data received from the RX FIFO. */
        *handle->transferBuff = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferBuff++;
        handle->transferCount++;
    }
    return ret;
}

/**
  * @brief The step of transmit normal data in blocking as master.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusBlockingMasterTxDataOptStepNormal(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    /* In this case, the quick cmd or other input parameters are incorrect. */
    if (handle->transferBuff == NULL || handle->transferSize == 0) {
        return BASE_STATUS_OK;
    }

    /* Sets data to be sent cyclically. */
    while (handle->transferCount < handle->transferSize) {
        if ((handle->transferCount == handle->transferSize - 1) &&
            (handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TPEC_RACK_S_RPEC_TACK, 0);
        } else {
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, *handle->transferBuff);
        }
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        handle->transferBuff++;
        handle->transferCount++;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief The step of receive normal data in blocking as slave.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusBlockingSlaveRxDataOptStepNormal(SMBUS_Handle *handle)
{
    if (handle->transferBuff == NULL || handle->transferSize == 0) {
        return BASE_STATUS_OK;
    }

    while (handle->transferCount < handle->transferSize) {
        /* Sets the data to be received. */
        if (SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_TD_RACK_S_RD_TACK, 0) != BASE_STATUS_OK) {
            return BASE_STATUS_TIMEOUT;
        }
        if (SMBusWaitStatusReady(handle, RX_FIFO_NOT_EMPTY, SMBUS_OPERATION_READ) != BASE_STATUS_OK) {
            return BASE_STATUS_TIMEOUT;
        }
        /* Obtains the data received in the RX FIFO. */
        *handle->transferBuff = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferBuff++;
        handle->transferCount++;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Checking Interrupts Caused by SMBUS Timing Errors.
  * @param handle SMBUS handle.
  * @param status Status of the SMBUS.
  * @retval true or false
  */
static void SMBusIsInterruptErrorStatus(SMBUS_Handle *handle, unsigned int status)
{
    /* If the low level timesout, the SMBUS bus is cleared and the bus is expected to be released. */
    if (status & SMBUS_SCL_LOW_TIMEOUT_MASK) {
        SMBusBusClear(handle);
    }
    /* Disable */
    handle->errorCode = BASE_STATUS_ERROR;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_DISABLE;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    handle->state = SMBUS_STATE_READY;
    if (handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
}

/**
  * @brief Interrupt handle send start command.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusInterruptSendStart(SMBUS_Handle *handle)
{
    unsigned int temp;
    if (((handle->frameOpt & SMBUS_FRAME_START) == 0) ||
        (handle->sendAddrStatus <= SMBUS_SEND_ADDR_STATUS_NONE)) {
        return BASE_STATUS_OK;
    }

    if (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < SMBUS_MAX_FIFO_SIZE) {
        /* The 8 to 11 bits are the Timing Commands, and the 0 to 7 bits are the write data. */
        temp = (((unsigned int)I2C_CMD_S << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
    } else {
        return BASE_STATUS_ERROR;
    }
    switch (handle->sendAddrStatus) {
        case SMBUS_SEND_ADDR_STATUS_WRITE:
            /* Send a write command to the slave. */
            if (SMBusSendSlaveAddressWriteCmd(handle) != BASE_STATUS_OK) {
                return BASE_STATUS_ERROR;
            }
            break;
        case SMBUS_SEND_ADDR_STATUS_READ:
            /* Send a read command to the slave. */
            if (SMBusSendSlaveAddressReadCmd(handle) != BASE_STATUS_OK) {
                return BASE_STATUS_ERROR;
            }
            break;
        default:
            break;
    }
    handle->sendAddrStatus = SMBUS_SEND_ADDR_STATUS_NONE;
    return BASE_STATUS_OK;
}

/**
 * @brief SMBUS Interrupt done Handling without stop.
 * @param handle SMBUS handle.
 * @retval None.
 */
static void SMBusInterruptAllDoneHandleWithoutStop(SMBUS_Handle *handle)
{
    switch (handle->state) {
        case SMBUS_STATE_BUSY_MASTER_TX: /* SMBus is at master tx state. */
            handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
            handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
            handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
            handle->state = SMBUS_STATE_READY;
            if (handle->userCallBack.MasterTxCplCallback != NULL) {
                handle->userCallBack.MasterTxCplCallback(handle); /* Invoke the TX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_MASTER_RX: /* SMBus is at master rx state. */
            handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
            handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
            handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
            handle->state = SMBUS_STATE_READY;
            if (handle->userCallBack.MasterRxCplCallback != NULL) {
                handle->userCallBack.MasterRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_SLAVE_TX: /* SMBus is at slave tx state. */
            handle->state = SMBUS_STATE_READY;
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
            handle->baseAddress->I2C_INTR_EN.BIT.rx_fifo_not_empty_en = BASE_CFG_SET;
            handle->baseAddress->I2C_INTR_RAW.reg =
                (SMBUS_INTR_RAW_TX_FIFO_NOT_FULL_MASK | SMBUS_INTR_RAW_RX_FIFO_FULL_MASK);
            if (handle->userCallBack.SlaveTxCplCallback != NULL) {
                handle->userCallBack.SlaveTxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_SLAVE_RX: /* SMBus is at slave rx state. */
            handle->state = SMBUS_STATE_READY;
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
            handle->baseAddress->I2C_INTR_EN.BIT.rx_fifo_not_empty_en = BASE_CFG_SET;
            handle->baseAddress->I2C_INTR_RAW.reg =
                (SMBUS_INTR_RAW_TX_FIFO_NOT_FULL_MASK | SMBUS_INTR_RAW_RX_FIFO_FULL_MASK);
            if (handle->userCallBack.SlaveRxCplCallback != NULL) {
                handle->userCallBack.SlaveRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        default:
            break;
    }
}

/**
 * @brief SMBUS Interrupt done Handling with stop callback.
 * @param handle SMBUS handle.
 * @param status SMBUS interrupt raw status.
 * @retval None.
 */
static void SMBusInterruptStopCallBack(SMBUS_Handle *handle, unsigned int status)
{
    if (status & SMBUS_INTR_RAW_ALL_CMD_DONE_MASK) {
        /* Invoke the RX callback processing function. */
        if (handle->userCallBack.MasterSendStopCplCallback != NULL) {
            handle->userCallBack.MasterSendStopCplCallback(handle);
        }
    } else if (status & SMBUS_INTR_RAW_STOP_DET_MASK) {
        /* Invoke the RX callback processing function. */
        if (handle->userCallBack.SlaveDetectedStopCplCallback != NULL) {
            handle->userCallBack.SlaveDetectedStopCplCallback(handle);
        }
    }
}

/**
 * @brief SMBUS Interrupt done Handling with stop.
 * @param handle SMBUS handle.
 * @param status SMBUS interrupt raw status.
 * @retval None.
 */
static void SMBusInterruptAllDoneHandleWithStop(SMBUS_Handle *handle, unsigned int status)
{
    switch (handle->state) {
        case SMBUS_STATE_BUSY_MASTER_TX:
            handle->state = SMBUS_STATE_READY;
            if (handle->userCallBack.MasterTxCplCallback != NULL) {
                handle->userCallBack.MasterTxCplCallback(handle); /* Invoke the TX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_MASTER_RX:
            handle->state = SMBUS_STATE_READY;
            if (handle->userCallBack.MasterRxCplCallback != NULL) {
                handle->userCallBack.MasterRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_SLAVE_TX:
            handle->state = SMBUS_STATE_READY;
            if (handle->userCallBack.SlaveTxCplCallback != NULL) {
                handle->userCallBack.SlaveTxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_SLAVE_RX:
            handle->state = SMBUS_STATE_READY;
            if (handle->userCallBack.SlaveRxCplCallback != NULL) {
                handle->userCallBack.SlaveRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        default:
            break;
    }
    SMBusInterruptStopCallBack(handle, status); /* SMBUS Interrupt done Handling with stop callback. */
}

/**
 * @brief SMBUS Interrupt done Handling
 * @param handle SMBUS handle.
 * @param status SMBUS interrupt raw status.
 * @retval None.
 */
static void SMBusInterruptAllDoneHandle(SMBUS_Handle *handle, unsigned int status)
{
    /* After all data transmission is complete, call the user's callback function. */
    unsigned int masterAllDone = status & SMBUS_INTR_RAW_ALL_CMD_DONE_MASK;
    unsigned int slaveReceiveStop = status & SMBUS_INTR_RAW_STOP_DET_MASK;
    unsigned int allDoneItFlag = (masterAllDone || slaveReceiveStop);

    if (handle->transferCount >= handle->transferSize &&
        ((handle->frameOpt & SMBUS_FRAME_STOP) != SMBUS_FRAME_STOP) &&
        (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num == 0)) {
        /* SMBUS Interrupt done Handling without stop. */
        SMBusInterruptAllDoneHandleWithoutStop(handle);
        return;
    }

    if (handle->transferCount >= handle->transferSize &&
        ((handle->frameOpt & SMBUS_FRAME_STOP) && allDoneItFlag)) {
        /* Clears interrupts and disables interrupt reporting to
           facilitate switching between different working modes. */
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_DISABLE;
        handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
        handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
        /* SMBUS Interrupt done Handling with stop. */
        SMBusInterruptAllDoneHandleWithStop(handle, status);
        handle->frameOpt = SMBUS_FRAME_NONE;
        handle->state = SMBUS_STATE_READY;
    }
}

/**
  * @brief SMBUS interrupt TX handling
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusInterruptMasterTxHandle(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_ASSERT_PARAM(handle->transferBuff != NULL);
    unsigned int temp;
    /* Send a start command to the slave. */
    if (SMBusInterruptSendStart(handle) != BASE_STATUS_OK) {
        return;
    }

    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < SMBUS_MAX_FIFO_SIZE &&
           handle->transferCount < handle->transferSize) {
        if ((handle->transferCount == handle->transferSize - 1) &&
            (handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
            /* Sets the data to be sent. */
            temp = (((unsigned int)I2C_CMD_M_TPEC_RACK_S_RPEC_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        } else {
            /* Sets the data to be sent. */
            temp = (((unsigned int)I2C_CMD_M_TD_RACK_S_RD_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            temp |= ((unsigned int)(*handle->transferBuff) & I2C_TXFIFO_WDATA_MASK);
        }
        handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
        handle->transferBuff++;
        handle->transferCount++;
    }
}

/**
 * @brief SMBUS Interrupt RX last data Handling
 * @param handle SMBUS handle.
 * @retval None.
 */
static unsigned int SMBusInterruptMasterRxLastDataHandle(SMBUS_Handle *handle)
{
    unsigned int temp;
    if (handle->frameOpt & SMBUS_FRAME_PEC) {
        /* Reads the last frame of data without ack. */
        temp = (((unsigned int)I2C_CMD_M_RPEC_TNACK_S_TPEC_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
    } else {
        if (handle->frameOpt & SMBUS_FRAME_BLOCK_PROCESSING) {
            /* Reads the last frame of data without ack. */
            temp = (((unsigned int)I2C_CMD_M_RD_TACK_S_TD_RACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        } else {
            /* Reads the last frame of data without ack. */
            temp = (((unsigned int)I2C_CMD_M_RD_TNACK_S_TD_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        }
    }
    return temp;
}

/**
 * @brief SMBUS Interrupt RX Handling
 * @param handle SMBUS handle.
 * @retval None.
 */
static void SMBusInterruptMasterRxHandle(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_ASSERT_PARAM(handle->transferBuff != NULL);
    unsigned int temp;
            
    /* Send a start command to the slave. */
    if (SMBusInterruptSendStart(handle) != BASE_STATUS_OK) {
        return;
    }

    /* The SMBUS controller fills in the receive command and starts to receive data. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < SMBUS_MAX_FIFO_SIZE &&
           handle->txReadCmdCount < handle->transferSize) {
        if (handle->txReadCmdCount == handle->transferSize - 1) {
            /* SMBUS Interrupt RX last data Handling. */
            temp = SMBusInterruptMasterRxLastDataHandle(handle);
        } else { /* Normal data transmission. */
            temp = (((unsigned int)I2C_CMD_M_RD_TACK_S_TD_RACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        }
        handle->baseAddress->I2C_TX_FIFO.reg = temp;
        handle->txReadCmdCount++;
    }

    /* Obtains the data received in the RX FIFO. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_vld_num > 0 &&
           handle->transferCount < handle->transferSize) {
        *handle->transferBuff++ = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferCount++;
    }
}

/**
  * @brief SMBUS interrupt slave TX handling
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusInterruptSlaveTxHandle(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_ASSERT_PARAM(handle->transferBuff != NULL);
    unsigned int temp;
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < SMBUS_MAX_FIFO_SIZE &&
           handle->transferCount < handle->transferSize) {
        if (handle->transferCount == (handle->transferSize - 1)) { /* no need ack. */
            if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
                temp = (((unsigned int)I2C_CMD_M_RPEC_TNACK_S_TPEC_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            } else {
                temp = (((unsigned int)I2C_CMD_M_RD_TNACK_S_TD_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
                temp |= ((unsigned int)(*handle->transferBuff) & I2C_TXFIFO_WDATA_MASK);
            }
        } else { /* Normal data transmission. */
            temp = (((unsigned int)I2C_CMD_M_RD_TACK_S_TD_RACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            temp |= ((unsigned int)(*handle->transferBuff) & I2C_TXFIFO_WDATA_MASK);
        }
        handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Sets the data and commands to be sent. */
        handle->transferBuff++;
        handle->transferCount++;
    }
}

/**
  * @brief SMBUS interrupt slave RX handling
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusInterruptSlaveRxHandle(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_ASSERT_PARAM(handle->transferBuff != NULL);
    unsigned int temp;

    /* Set the data receiving command. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < SMBUS_MAX_FIFO_SIZE &&
           handle->txReadCmdCount < handle->transferSize) {
        if (handle->txReadCmdCount == (handle->transferSize - 1)) { /* receive the last data. */
            if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
                /* Need to receive PEC. */
                temp = (((unsigned int)I2C_CMD_M_TPEC_RACK_S_RPEC_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            } else {
                /* Don't need to receive PEC. */
                temp = (((unsigned int)I2C_CMD_M_TD_RACK_S_RD_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
            }
        } else {
            temp = (((unsigned int)I2C_CMD_M_TD_RACK_S_RD_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        }
        handle->baseAddress->I2C_TX_FIFO.reg = temp; /* Write data to register. */
        handle->txReadCmdCount++;
    }
    /* Obtained data from RX FIFO. */
    while (handle->baseAddress->I2C_FIFO_STAT.BIT.rx_fifo_vld_num > 0 &&
           handle->transferCount < handle->transferSize) {
        *handle->transferBuff = handle->baseAddress->I2C_RX_FIFO.BIT.rx_fifo_rdata;
        handle->transferBuff++;
        handle->transferCount++;
    }
}

/**
  * @brief Addr match Callback function corresponding to the interrupt processing function.
  * @param handle SMBUS handle.
  * @param status Status of the SMBUS.
  * @retval None.
  */
static void SMBusInterruptAddrMatchHandle(SMBUS_Handle *handle, unsigned int status)
{
    if (status & SMBUS_INTR_RAW_SLAVE_ADDR_MATCH_MASK) {
        if ((handle->frameOpt & SMBUS_FRAME_EXCUTE_CMD_MATCH) &&
            handle->userCallBack.ExecuteCmdCallback != NULL) {
            handle->userCallBack.ExecuteCmdCallback(handle); /* Callback execute cmd function. */
        } else if (handle->userCallBack.AddrMatchCplCallback != NULL) {
            handle->userCallBack.AddrMatchCplCallback(handle); /* Callback address match function. */
        }
        handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_SLAVE_ADDR_MATCH_MASK;
    }
}

/**
  * @brief ICallback function corresponding to the interrupt processing function.
  * @param handle SMBUS handle.
  * @param status Status of the SMBUS.
  * @retval None.
  */
static void SMBusInterruptHandle(SMBUS_Handle *handle, unsigned int status)
{
    if (handle->state == SMBUS_STATE_BUSY_MASTER_TX) {
        SMBusInterruptMasterTxHandle(handle); /* Transfer data as a host. */
        return;
    } else if (handle->state == SMBUS_STATE_BUSY_MASTER_RX) {
        SMBusInterruptMasterRxHandle(handle); /* Receive data as a host. */
        return;
    } else if (handle->state == SMBUS_STATE_BUSY_SLAVE_TX) {
        if (status & SMBUS_INTR_RAW_SLAVE_ADDR_MATCH_MASK) {
            /* Set TX FIFO the waterline. */
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
        }
        if (handle->baseAddress->I2C_RX_ADDR.BIT.rx_rw == SMBUS_OPERATION_READ) {
            SMBusInterruptSlaveTxHandle(handle); /* Transfer data as slave. */
        }
        return;
    } else if (handle->state == SMBUS_STATE_BUSY_SLAVE_RX) {
        if (status & SMBUS_INTR_RAW_SLAVE_ADDR_MATCH_MASK) {
            /* Set TX FIFO the waterline. */
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
            /* Set RX FIFO the waterline. */
            handle->baseAddress->I2C_INTR_EN.BIT.rx_fifo_not_empty_en = BASE_CFG_SET;
        }

        if (handle->baseAddress->I2C_RX_ADDR.BIT.rx_rw == SMBUS_OPERATION_WRITE) {
            SMBusInterruptSlaveRxHandle(handle); /* Receive data as slave. */
        }
        return;
    }
    handle->errorCode = BASE_STATUS_ERROR;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->state = SMBUS_STATE_READY; /* Changing the SMBUS Bus Status. */
    if (handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
}

/**
  * @brief Callback Function Registration in transfer mode.
  * @param handle SMBUS handle.
  * @param callbackID Callback function ID.
  * @param pcallback Pointer to the address of the registered callback function.
  * @retval None.
  */
static BASE_StatusType SMBusRegisterTransferCallback(SMBUS_Handle *handle, SMBUS_CallbackId callbackID,
                                                     SMBUS_CallbackFunType pcallback)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    switch (callbackID) {
        case SMBUS_MASTER_TX_COMPLETE_CB_ID:
            /* Invoke the transfer completion callback function. */
            handle->userCallBack.MasterTxCplCallback = pcallback;
            break;
        case SMBUS_MASTER_RX_COMPLETE_CB_ID:
            /* Invoke the transfer completion callback function. */
            handle->userCallBack.MasterRxCplCallback = pcallback;
            break;
        case SMBUS_SLAVE_TX_COMPLETE_CB_ID:
            /* Invoke the transfer completion callback function. */
            handle->userCallBack.SlaveTxCplCallback = pcallback;
            break;
        case SMBUS_SLAVE_RX_COMPLETE_CB_ID:
            /* Invoke the receive completion callback function. */
            handle->userCallBack.SlaveRxCplCallback = pcallback;
            break;
        default:
            ret = BASE_STATUS_ERROR;
            break;
    }
    return ret;
}

/**
  * @brief DMA Command Configuration last data pec.
  * @retval Value of the command.
  */
static unsigned int SMBusDmaConfigLastDataPec(SMBUS_Handle *handle)
{
    unsigned int temp = 0;
    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        temp = (((unsigned int)I2C_CMD_M_RPEC_TNACK_S_TPEC_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
    } else {
        if ((handle->frameOpt & SMBUS_FRAME_BLOCK_PROCESSING) == SMBUS_FRAME_BLOCK_PROCESSING) {
            /* Reads the last frame of data with ack. */
            temp = (((unsigned int)I2C_CMD_M_RD_TACK_S_TD_RACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        } else {
            /* Reads the last frame of data without ack. */
            temp = (((unsigned int)I2C_CMD_M_RD_TNACK_S_TD_RNACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        }
    }
    return temp;
}

/**
  * @brief DMA Command Configuration in master.
  * @param handle SMBUS handle.
  * @param cmd The command type of SMBUS.
  * @param size The number of the data to be receiving or sending.
  * @retval Value of the command.
  */
static unsigned int SMBusMasterDmaConfigCommandData(SMBUS_Handle *handle, I2C_CmdType cmd, unsigned int size)
{
    unsigned int temp = 0;
    if (handle->state == SMBUS_STATE_BUSY_MASTER_TX || handle->state == SMBUS_STATE_BUSY_SLAVE_RX) {
        /* Determines whether to execute the PEC command. */
        if ((size == 1) && (handle->frameOpt & SMBUS_FRAME_PEC) && (handle->transferCount >= handle->transferSize)) {
            temp = (((unsigned int)I2C_CMD_M_TPEC_RACK_S_RPEC_TACK << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        } else {
            temp = (((unsigned int)cmd << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        }
    }
    return temp;
}

/**
  * @brief DMA Command Configuration.
  * @param handle SMBUS handle.
  * @param cmd The command type of SMBUS.
  * @param size The number of the data to be receiving or sending.
  * @retval Value of the command.
  */
static unsigned int SMBusDmaConfigCommandData(SMBUS_Handle *handle, I2C_CmdType cmd, unsigned int size)
{
    unsigned int temp = 0;
    /* Sets the command data. */
    if (handle->state == SMBUS_STATE_BUSY_MASTER_RX || handle->state == SMBUS_STATE_BUSY_SLAVE_TX) {
        if ((size == 1) && (handle->transferCount >= handle->transferSize)) {
            temp = SMBusDmaConfigLastDataPec(handle); /* DMA Command Configuration last data pec. */
        } else {
            /* Reads the last frame of data without ack. */
            temp = (((unsigned int)cmd << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
        }
    } else {
        /* DMA Command Configuration in master. */
        temp = SMBusMasterDmaConfigCommandData(handle, cmd, size);
    }

    return temp;
}

/**
  * @brief Config commands and data in dma as master.
  * @param handle SMBUS handle.
  * @param txBuff Address of the data buff to be receiving or sending.
  * @param cmd The command type of SMBUS.
  * @param size The number of the data to be receiving or sending.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusDmaMasterConfigDataAndCmd(SMBUS_Handle *handle, unsigned int *txBuff, I2C_CmdType cmd,
                                                      unsigned int size)
{
    unsigned int temp;
    unsigned int *tempTxBuff = txBuff;
    unsigned char *tempsrcTxBuff = (unsigned char*)handle->transferBuff;
    unsigned int tempSize = size;
    while (tempSize) {
        /* Sets the command data. */
        temp = SMBusDmaConfigCommandData(handle, cmd, tempSize);

        /* Sets the normal data. */
        if (handle->state == SMBUS_STATE_BUSY_MASTER_RX) {
            temp |= ((0x0 << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
        } else if (handle->state == SMBUS_STATE_BUSY_MASTER_TX) {
            if ((tempSize == 1) && (handle->frameOpt & SMBUS_FRAME_PEC) &&
                (handle->transferCount >= handle->transferSize)) {
                temp |= (((unsigned int)0x0 << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
            } else {
                temp |= (((unsigned int)*tempsrcTxBuff << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
                tempsrcTxBuff++;
            }
        }
        *tempTxBuff = temp; /* Set the combined data. */
        tempTxBuff++;
        tempSize--;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Config commands and data in dma as slave.
  * @param handle SMBUS handle.
  * @param txBuff Address of the data buff to be receiving or sending.
  * @param cmd The command type of SMBUS.
  * @param size The number of the data to be receiving or sending.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusDmaSlaveConfigDataAndCmd(SMBUS_Handle *handle, unsigned int *txBuff, I2C_CmdType cmd,
                                                     unsigned int size)
{
    unsigned int temp;
    unsigned int *tempTxBuff = txBuff;
    unsigned char *tempsrcTxBuff = (unsigned char*)handle->transferBuff;
    unsigned int tempSize = size;
    while (tempSize) {
        /* Sets the command data. */
        temp = SMBusDmaConfigCommandData(handle, cmd, tempSize);

        /* Sets the normal data. */
        if (handle->state == SMBUS_STATE_BUSY_SLAVE_TX) {
            if (tempSize == 1 && (handle->frameOpt & SMBUS_FRAME_PEC) &&
                (handle->transferCount >= handle->transferSize)) {
                temp |= ((0x0 << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
            } else {
                temp |= (((unsigned int)*tempsrcTxBuff << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
                tempsrcTxBuff++;
            }
        } else if (handle->state == SMBUS_STATE_BUSY_SLAVE_RX) {
            temp |= ((0x0 << I2C_TXFIFO_WDATA_POS) & I2C_TXFIFO_WDATA_MASK);
        }
        *tempTxBuff = temp; /* Set the combined data. */
        tempTxBuff++;
        tempSize--;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief SMBUS DMA Error Handling.
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusDmaErrorHandle(SMBUS_Handle *handle)
{
    /* Some settings when an error occurs. */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    handle->errorCode = BASE_STATUS_ERROR;
    if (handle->userCallBack.ErrorCallback != NULL) {
        handle->userCallBack.ErrorCallback(handle);
    }
    handle->state = SMBUS_STATE_READY;
}

/**
  * @brief SMBUS DMA completes processing.
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusDmaDoneHandle(SMBUS_Handle *handle, unsigned int intrRwa)
{
    /* Disable the DMA operation and configure parameters. */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    /* Call the corresponding callback function. */
    switch (handle->state) {
        case SMBUS_STATE_BUSY_MASTER_TX:
            if (handle->userCallBack.MasterTxCplCallback != NULL) {
                handle->userCallBack.MasterTxCplCallback(handle); /* Invoke the TX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_MASTER_RX:
            if (handle->userCallBack.MasterRxCplCallback != NULL) {
                handle->userCallBack.MasterRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_SLAVE_TX:
            if (handle->userCallBack.SlaveTxCplCallback != NULL) {
                handle->userCallBack.SlaveTxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_SLAVE_RX:
            if (handle->userCallBack.SlaveRxCplCallback != NULL) {
                handle->userCallBack.SlaveRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        default:
            break;
    }
    SMBusInterruptStopCallBack(handle, intrRwa); /* SMBUS Interrupt done Handling with stop callback. */
    handle->state = SMBUS_STATE_READY;
}

/**
 * @brief Wait until all SMBUS timings are processed.
 * @param handle SMBUS handle.
 * @retval None.
 */
static void SMBusDmaWaitHandleFinish(SMBUS_Handle *handle)
{
    unsigned int intrRwa;
    unsigned int preTick;
    unsigned int curTick;
    unsigned long long delta;
    unsigned long long targetDelta;

    delta = 0;
    preTick = DCL_SYSTICK_GetTick();
    /* Set the timeout threshold to 10000ms. */
    targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / SMBUS_TICK_MS_DIV * handle->timeout;

    while (true) {
        /* Waiting for the last DMA transfer to complete. */
        intrRwa = handle->baseAddress->I2C_INTR_RAW.reg;
        /* Check for errors. */
        if ((intrRwa & (SMBUS_INTR_RAW_ARB_LOST_MASK | SMBUS_INTR_RAW_ACK_BIT_UNMATCH_MASK |
                        SMBUS_INTR_RAW_SLAVE_ACK_UNMATCH_MASK)) > 0) {
            SMBusDmaErrorHandle(handle);
            break;
        }
        /* DMA transfer completed normally. */
        if ((intrRwa & (SMBUS_INTR_RAW_ALL_CMD_DONE_MASK | SMBUS_INTR_RAW_STOP_DET_MASK)) > 0) {
            SMBusDmaDoneHandle(handle, intrRwa);
            break;
        }
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* Check timeout. */
            return;
        }
        preTick = curTick;
    }
}

/**
 * @brief SMBus stop is not sent after DMA transfer is complete.
 * @param handle SMBUS handle.
 * @retval None.
 */
static void SMBusDmaHandleWithoutStop(SMBUS_Handle *handle)
{
    SMBusWaitStatusReady(handle, TX_FIFO_EMPTY, SMBUS_SEND_ADDR_STATUS_WRITE);
    switch (handle->state) {
        case SMBUS_STATE_BUSY_SLAVE_TX:
            if (handle->userCallBack.SlaveTxCplCallback != NULL) {
                handle->userCallBack.SlaveTxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_MASTER_TX:
            if (handle->userCallBack.MasterTxCplCallback != NULL) {
                handle->userCallBack.MasterTxCplCallback(handle); /* Invoke the TX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_SLAVE_RX:
            if (handle->userCallBack.SlaveRxCplCallback != NULL) {
                handle->userCallBack.SlaveRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        case SMBUS_STATE_BUSY_MASTER_RX:
            if (handle->userCallBack.MasterRxCplCallback != NULL) {
                handle->userCallBack.MasterRxCplCallback(handle); /* Invoke the RX callback processing function. */
            }
            break;
        default:
            break;
    }
    handle->state = SMBUS_STATE_READY;
}

/**
  * @brief The SMBUS uses the DMA completion send stop.
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusDmaOptStepNormalFinishSendStop(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    if (handle->transferCount >= handle->transferSize) {
        if ((handle->frameOpt & SMBUS_FRAME_STOP) == SMBUS_FRAME_STOP) {
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_P, 0);
            if (ret != BASE_STATUS_OK) {
                SMBusSetErrorHandling(handle); /* Error handle. */
                return;
            }
            SMBusDmaWaitHandleFinish(handle); /* The frame with stop. */
        } else {
            SMBusDmaHandleWithoutStop(handle); /* The frame without stop. */
        }
    }
}

/**
  * @brief The SMBUS uses the DMA completion callback function registered by the DMA module.
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusDmaOptStepNormalFinishFun(void *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_Handle *smbusHandle = (SMBUS_Handle *)(handle);
    SMBUS_ASSERT_PARAM(IsI2CInstance(smbusHandle->baseAddress));
    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int tempOnceTransferSize; /* Used to record the amount of data transmitted this time. */
    unsigned int offset;

    offset = smbusHandle->transferCount % I2C_ONCE_TRANS_MAX_NUM;
    smbusHandle->transferBuff += (offset == 0) ? I2C_ONCE_TRANS_MAX_NUM : offset; /* Update Transferred Data. */

    if (smbusHandle->transferCount < smbusHandle->transferSize) {
        /* Determine the amount of data transmitted at a time. */
        tempOnceTransferSize = (smbusHandle->dmaTransferSize >= I2C_ONCE_TRANS_MAX_NUM) ? I2C_ONCE_TRANS_MAX_NUM :
                                smbusHandle->dmaTransferSize;
        smbusHandle->dmaTransferSize -= tempOnceTransferSize;
        smbusHandle->transferCount += tempOnceTransferSize;
        /* Configuring the I2C Timing */
        if (smbusHandle->state == SMBUS_STATE_BUSY_MASTER_RX) {
            ret = SMBusDmaMasterReadData(smbusHandle, tempOnceTransferSize); /* The master read data by DMA. */
        } else if (smbusHandle->state == SMBUS_STATE_BUSY_MASTER_TX) {
            ret = SMBusDmaMasterWriteData(smbusHandle, tempOnceTransferSize); /* The master write data by DMA. */
        } else if (smbusHandle->state == SMBUS_STATE_BUSY_SLAVE_RX) {
            ret = SMBusDmaSlaveReadData(smbusHandle, tempOnceTransferSize);  /* The slave read data by DMA. */
        } else if (smbusHandle->state == SMBUS_STATE_BUSY_SLAVE_TX) {
            ret = SMBusDmaSlaveWriteData(smbusHandle, tempOnceTransferSize);  /* The slave write data by DMA. */
        }
        /* Check whether errors occur. */
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return;
        }
        return;
    }
    SMBusDmaOptStepNormalFinishSendStop(handle); /* The SMBUS uses the DMA completion send stop. */
    smbusHandle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
    smbusHandle->frameOpt = SMBUS_FRAME_NONE;
}

/**
  * @brief The SMBUS uses the DMA error callback function registered by the DMA module.
  * @param handle SMBUS handle.
  * @retval None.
  */
static void SMBusDmaErrorHandlerFun(void *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_Handle *smbusHandle = (SMBUS_Handle *)(handle);
    SMBUS_ASSERT_PARAM(IsI2CInstance(smbusHandle->baseAddress));
    /* Disable the interrupt and call the error callback function. */
    smbusHandle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    smbusHandle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    smbusHandle->errorCode = BASE_STATUS_ERROR;
    if (smbusHandle->userCallBack.ErrorCallback != NULL) {
        smbusHandle->userCallBack.ErrorCallback(smbusHandle);
    }
    /* Stop DMA channel transfer. */
    HAL_DMA_StopChannel(smbusHandle->dmaHandle, smbusHandle->txDmaCh);
    if (smbusHandle->state == SMBUS_STATE_BUSY_MASTER_RX || smbusHandle->state == SMBUS_STATE_BUSY_SLAVE_RX) {
        HAL_DMA_StopChannel(smbusHandle->dmaHandle, smbusHandle->rxDmaCh);
    }
    smbusHandle->state = SMBUS_STATE_READY;
}

/**
  * @brief Receive data as master by the DMA module.
  * @param handle SMBUS handle.
  * @param size Number of the data to be transmitted.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusDmaMasterReadData(SMBUS_Handle *handle, unsigned int size)
{
    /* Combine commands and data. */
    SMBusDmaMasterConfigDataAndCmd(handle, (unsigned int *)g_smbusInternalTxBuffDMA, I2C_CMD_M_RD_TACK_S_TD_RACK, size);
    
    /* Configuring the DMA Callback Function. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = NULL;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = SMBusDmaErrorHandlerFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelFinishCallBack =
        SMBusDmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelErrorCallBack = SMBusDmaErrorHandlerFun;
    /* Start the DMA for data transmission. */
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_smbusInternalTxBuffDMA,
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO.reg),
                        size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = SMBUS_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->I2C_RX_FIFO),
                        (uintptr_t)handle->transferBuff, size,
                        handle->rxDmaCh) != BASE_STATUS_OK) {
        handle->state = SMBUS_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE_READ;
    return BASE_STATUS_OK;
}

/**
  * @brief Transmit data as master by the DMA module.
  * @param handle SMBUS handle.
  * @param size Number of the data to be transmitted.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusDmaMasterWriteData(SMBUS_Handle *handle, unsigned int size)
{
    /* Combine commands and data. */
    SMBusDmaMasterConfigDataAndCmd(handle, (unsigned int *)g_smbusInternalTxBuffDMA, I2C_CMD_M_TD_RACK_S_RD_TACK, size);

    /* Configuring the DMA Callback Function. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack =
        SMBusDmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = SMBusDmaErrorHandlerFun;
    /* Start the DMA for data transmission. */
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_smbusInternalTxBuffDMA,
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO.reg),
                        size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = SMBUS_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE;
    return BASE_STATUS_OK;
}

/**
  * @brief Receive data as slave by the DMA module.
  * @param handle SMBUS handle.
  * @param size Number of the data to be transmitted.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusDmaSlaveReadData(SMBUS_Handle *handle, unsigned int size)
{
    /* Combine commands and data. */
    SMBusDmaSlaveConfigDataAndCmd(handle, (unsigned int *)g_smbusInternalTxBuffDMA, I2C_CMD_M_TD_RACK_S_RD_TACK, size);

    /* Configuring the DMA Callback Function. */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = NULL;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = SMBusDmaErrorHandlerFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelFinishCallBack =
        SMBusDmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->rxDmaCh].ChannelErrorCallBack = SMBusDmaErrorHandlerFun;
    /* Start the DMA for data transmission. */
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->I2C_RX_FIFO),
                        (uintptr_t)handle->transferBuff, size,
                        handle->rxDmaCh) != BASE_STATUS_OK) {
        handle->state = SMBUS_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_smbusInternalTxBuffDMA,
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO),
                        size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = SMBUS_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE_READ;
    return BASE_STATUS_OK;
}

/**
  * @brief Transmit data as slave by the DMA module.
  * @param handle SMBUS handle.
  * @param size Number of the data to be transmitted.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusDmaSlaveWriteData(SMBUS_Handle *handle, unsigned int size)
{
    /* Combine commands and data. */
    SMBusDmaSlaveConfigDataAndCmd(handle, (unsigned int *)g_smbusInternalTxBuffDMA, I2C_CMD_M_RD_TACK_S_TD_RACK, size);
    /* Configure DMA Parameters */
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack =
        SMBusDmaOptStepNormalFinishFun;
    handle->dmaHandle->userCallBack.DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = SMBusDmaErrorHandlerFun;
    if (HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)g_smbusInternalTxBuffDMA,
                        (uintptr_t)&(handle->baseAddress->I2C_TX_FIFO.reg), size, handle->txDmaCh) != BASE_STATUS_OK) {
        handle->state = SMBUS_STATE_READY;
        return BASE_STATUS_ERROR;
    }
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_WRITE;
    return BASE_STATUS_OK;
}

/**
  * @brief Transmit data by the DMA module.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusTransferDataDma(SMBUS_Handle *handle)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int tempOnceTransferSize;
    handle->dmaTransferSize = handle->transferSize;

    /* Determine the amount of data transmitted at a time. */
    tempOnceTransferSize = (handle->dmaTransferSize >= I2C_ONCE_TRANS_MAX_NUM) ? I2C_ONCE_TRANS_MAX_NUM :
                            handle->dmaTransferSize;
    handle->dmaTransferSize -= tempOnceTransferSize;
    handle->transferCount += tempOnceTransferSize;
    /* Configuring the SMBUS Timing */
    if (handle->state == SMBUS_STATE_BUSY_MASTER_RX) {
        ret = SMBusDmaMasterReadData(handle, tempOnceTransferSize);
    } else if (handle->state == SMBUS_STATE_BUSY_MASTER_TX) {
        ret = SMBusDmaMasterWriteData(handle, tempOnceTransferSize);
    } else if (handle->state == SMBUS_STATE_BUSY_SLAVE_RX) {
        ret = SMBusDmaSlaveReadData(handle, tempOnceTransferSize);
    } else if (handle->state == SMBUS_STATE_BUSY_SLAVE_TX) {
        ret = SMBusDmaSlaveWriteData(handle, tempOnceTransferSize);
    }
    /* Check whether errors occur. */
    if (ret != BASE_STATUS_OK) {
        SMBusSetErrorHandling(handle);
        return ret;
    }

    return ret;
}

/**
  * @brief As Slave Multiplex Interrupt Write or Read.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusSlaveMultiplexIT(SMBUS_Handle *handle)
{
    /* Parameter Settings. */
    handle->txReadCmdCount = 0;
    handle->sendAddrStatus = SMBUS_SEND_ADDR_STATUS_NONE;
    /* Clean interrupt */
    if (handle->frameOpt & SMBUS_FRAME_FIRST) {
        handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_RAW_ALL_DISABLE;
        handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    }

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;

    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_CFG_INTERRUPT_SLAVE;
    /* Enable interrupt */
    if (((handle->frameOpt & SMBUS_FRAME_START) != SMBUS_FRAME_START)) {
        if (handle->state == SMBUS_STATE_BUSY_SLAVE_TX) {
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
        } else {
            handle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_SET;
            /* Set RX FIFO the waterline. */
            handle->baseAddress->I2C_INTR_EN.BIT.rx_fifo_not_empty_en = BASE_CFG_SET;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Confige the frame set in smbus master read blocking.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusMasterReadBlockingFrameSet(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    /* step1 : Parameter Settings and startup Control. */
    if ((handle->frameOpt & SMBUS_FRAME_START) == SMBUS_FRAME_START) {
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_S, 0); /* Sets the start command to be sent. */
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        /* Send slave address and read command. */
        ret = SMBusSendSlaveAddressReadCmd(handle);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
    }
    return BASE_STATUS_OK;
}


/**
  * @brief Confige the frame set in smbus master write blocking.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusMasterWriteBlockingFrameSet(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    /* step1 : Send I2C start and device address. */
    if ((handle->frameOpt & SMBUS_FRAME_START) == SMBUS_FRAME_START) {
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_S, 0); /* Sets the start command to be sent. */
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
        /* Send slave address and write command. */
        ret = SMBusSendSlaveAddressWriteCmd(handle);
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Confige the frame set in smbus slave write blocking.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static BASE_StatusType SMBusSlaveWriteBlockingFrameSet(SMBUS_Handle *handle)
{
    BASE_StatusType ret;
    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }
    /* step1 : Waiting for slave address match. */
    if ((handle->frameOpt & SMBUS_FRAME_START) == SMBUS_FRAME_START) {
        ret = SMBusWaitStatusReady(handle, SLAVE_ADDRESS_MATCH, SMBUS_OPERATION_READ);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        /* Execute Cmd Callback. */
        if (((handle->frameOpt & SMBUS_FRAME_EXCUTE_CMD_MATCH) == SMBUS_FRAME_EXCUTE_CMD_MATCH) &&
            handle->userCallBack.ExecuteCmdCallback != NULL) {
            handle->userCallBack.ExecuteCmdCallback(handle);
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Initializing the SMBUS Module.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_Init(SMBUS_Handle *handle)
{
    unsigned int clockFreq;
    unsigned int val;
    unsigned int tempReg;
    unsigned int temp;
    unsigned int tempSclLowTime;
    unsigned int tempSclHighTime;

    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));

    clockFreq = HAL_CRG_GetIpFreq((void *)handle->baseAddress);
    if (SMBusCheckAllInitParameters(handle, clockFreq) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    handle->state = SMBUS_STATE_BUSY;
    /* Clears interrupts and disables interrupt reporting to facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;

    /* Set SDA and SCL glitch filtering time. */
    handle->baseAddress->I2C_FILTER.BIT.spike_filter_time = handle->handleEx.spikeFilterTime;
    /* Set SCL high and low duratiom time */
    tempSclLowTime = SMBUS_FREQ_LOW_PARAMTER + handle->handleEx.spikeFilterTime;
    tempSclHighTime = SMBUS_FREQ_HIGH_PARAMTER + handle->handleEx.spikeFilterTime;
    if (handle->freq <= SMBUS_STANDARD_FREQ_TH) {
        /* scl_high_time = (fclk_i2c/fSCL) x 0.5 - 8 - spike_filter_time. */
        val = clockFreq / (handle->freq * 2) - tempSclHighTime; /* The clockFreq / (freq * 2) = cloclFreq/0.5/freq. */
        /* scl_low_time = (fclk_i2c/fSCL) x 0.5 - 9 - spike_filter_time. */
        val = ((val - 1) & LOW_HOLD_TIME_MASK) | ((val << HIGH_HOLD_TIME_POS) & HIGH_HOLD_TIME_MASK);
    } else {
        /* scl_high_time = (fclk_i2c/fSCL) x 0.36 - 8 - spike_filter_time. (n/100*36)=0.36n. */
        val = ((((clockFreq / 100) * 36) / handle->freq) - tempSclHighTime) << HIGH_HOLD_TIME_POS;
        /* scl_low_time = (fclk_i2c/fSCL) x 0.64 - 9 - spike_filter_time. (n/100*64)=0.64n. */
        val |= (((((clockFreq / 100) * 64) / handle->freq) - tempSclLowTime) & LOW_HOLD_TIME_MASK);
    }
    handle->baseAddress->I2C_SCL_CFG.reg = val;

    /* Set sda hold duration.The value is fixed to 0xa */
    temp = ((handle->sdaHoldTime & 0x0000FFFF) << I2C_SDA_HOLD_DURATION_POS);
    tempReg = (handle->handleEx.sdaDelayTime & 0x0000000F) | temp;
    handle->baseAddress->I2C_SDA_CFG.reg = tempReg;

    /* Set SMBUS TX FIFO watermark */
    handle->baseAddress->I2C_TX_WATERMARK.BIT.tx_watermark = handle->txWaterMark;
    /* Set SMBUS RX FIFO watermark */
    handle->baseAddress->I2C_RX_WATERMARK.BIT.rx_watermark = handle->rxWaterMark;
    handle->baseAddress->I2C_MODE.BIT.mst_slv_function = handle->functionMode;
    handle->baseAddress->I2C_MODE.BIT.rack_mode = handle->ignoreAckFlag;

    if (handle->functionMode == SMBUS_MODE_SELECT_SLAVE_ONLY ||
        handle->functionMode == SMBUS_MODE_SELECT_MASTER_SLAVE) {
        /* Sets the first own address of the slave. */
        handle->baseAddress->I2C_OWN_ADDR.BIT.own_address = handle->slaveOwnAddress;
        handle->baseAddress->I2C_OWN_ADDR.BIT.i2c_general_call_en = handle->generalCallMode;
        /* Sets the second own address of the slave. */
        if (handle->handleEx.slaveOwnXmbAddressEnable == BASE_CFG_ENABLE) {
            handle->baseAddress->XMB_DEV_ADDR.BIT.xmb_address_en = BASE_CFG_ENABLE;
            handle->baseAddress->XMB_DEV_ADDR.BIT.xmb_address = handle->handleEx.slaveOwnXmbAddress;
        }
    }
    handle->state = SMBUS_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize the SMBUS module.
  * @param handle SMBUS handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_Deinit(SMBUS_Handle *handle)
{
    SMBUS_ASSERT_PARAM(handle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));

    handle->state = SMBUS_STATE_BUSY;
    /* Disable */
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    handle->baseAddress->I2C_MODE.BIT.mst_slv_function = SMBUS_MODE_SELECT_NONE;
    /* Clears interrupts and disables interrupt reporting to
       facilitate switching between different working modes. */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    /* Clean interrupt callback functions. */
    handle->userCallBack.MasterRxCplCallback = NULL;
    handle->userCallBack.MasterTxCplCallback = NULL;
    handle->userCallBack.SlaveRxCplCallback = NULL;
    handle->userCallBack.SlaveTxCplCallback = NULL;
    handle->userCallBack.SlaveWakeupCallback = NULL;
    handle->userCallBack.AlertCallback = NULL;
    handle->userCallBack.AddrMatchCplCallback = NULL;
    handle->userCallBack.ErrorCallback = NULL;
    handle->state = SMBUS_STATE_RESET;

    return BASE_STATUS_OK;
}

/**
  * @brief Callback Function Registration.
  * @param handle SMBUS handle.
  * @param callbackID Callback function ID.
  * @param pcallback Pointer to the address of the registered callback function.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_RegisterCallback(SMBUS_Handle *handle, SMBUS_CallbackId callbackID,
                                           SMBUS_CallbackFunType pcallback)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    /* Check the parameter validity. */
    SMBUS_ASSERT_PARAM(handle != NULL && pcallback != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_ASSERT_PARAM(pcallback != NULL);

    /* Callback Function Registration in transfer mode. */
    ret = SMBusRegisterTransferCallback(handle, callbackID, pcallback);
    if (ret == BASE_STATUS_OK) {
        return ret;
    } else {
        ret = BASE_STATUS_OK;
    }

    switch (callbackID) {
        case SMBUS_SLAVE_WAKEUP_COMPLETE_CB_ID:
            /* Invoke the slave wakeup callback function. */
            handle->userCallBack.SlaveWakeupCallback = pcallback;
            break;
        case SMBUS_ADDR_MATCH_COMPLETE_CB_ID:
            /* Invoke the address match callback function. */
            handle->userCallBack.AddrMatchCplCallback = pcallback;
            break;
        case SMBUS_MSTAER_SEND_STOP_COMPLETE_CB_ID:
            /* Invoke the send stop callback function. */
            handle->userCallBack.MasterSendStopCplCallback = pcallback;
            break;
        case SMBUS_SLAVE_DETECTED_STOP_COMPLETE_CB_ID:
            /* Invoke the detect send stop callback function. */
            handle->userCallBack.SlaveDetectedStopCplCallback = pcallback;
            break;
        case SMBUS_ALERT_COMPLETE_CB_ID:
            /* Invoke the alert callback function. */
            handle->userCallBack.AlertCallback = pcallback;
            break;
        case SMBUS_ERROR_CB_ID:
            /* Invoke the error callback function. */
            handle->userCallBack.ErrorCallback = pcallback;
            break;
        default:
            ret = BASE_STATUS_ERROR;
            handle->errorCode = BASE_STATUS_ERROR;
            break;
    }

    return ret;
}

/**
  * @brief I2C Parameter Configuration in blocking.
  * @param handle I2C handle.
  * @param transferStatus The status is used to indicate read or write.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
static void SMBusConfigParameters(SMBUS_Handle *handle)
{
    /* Clears row interrupts and disables interrupt. */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
}

/**
  * @brief Receiving data in blocking mode.
  * @param handle SMBUS handle.
  * @param devAddr Slave Device Address.
  * @param buffer: buffer.rData Address of the data buff to be receiving.
  * @param buffer: buffer.dataSize Number of the data to be receiving.
  * @param timeout Timeout period,unit: ms.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_MasterReadBlocking(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                             unsigned int timeout, unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(devAddr <= SMBUS_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR); /* State check. */

    BASE_StatusType ret;
    /* Configuring SMBUS Parameters. */
    handle->state = SMBUS_STATE_BUSY_MASTER_RX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;
    handle->frameOpt = frameOpt;
    SMBusSetSlaveDevAddr(handle, devAddr); /* Set the SMBUS Slave Device Address. */
    SMBusConfigParameters(handle);

    /* Waiting for the SMBUS bus to be idle. */
    if ((handle->frameOpt & SMBUS_FRAME_FIRST) == SMBUS_FRAME_FIRST) {
        ret = SMBusWaitStatusReady(handle, BUS_IS_FREE, SMBUS_SEND_ADDR_STATUS_WRITE);
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    }

    /* Confige the frame set in smbus master read blocking. */
    ret = SMBusMasterReadBlockingFrameSet(handle);
    if (ret != BASE_STATUS_OK) {
        SMBusSetErrorHandling(handle);
        return ret;
    }

    /* step3 : start receive data. */
    ret = SMBusBlockingMasterRxDataOptStepNormal(handle);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SMBusSetErrorHandling(handle);
        return ret;
    }
    /* step4 ：send stop CMD. */
    if ((handle->frameOpt & SMBUS_FRAME_STOP) == SMBUS_FRAME_STOP) {
        ret = SMBusBlockingSendStopCommand(handle);
    }
    handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
    handle->frameOpt = SMBUS_FRAME_NONE;
    handle->state = SMBUS_STATE_READY;
    return ret;
}

/**
  * @brief Send data in blocking mode.
  * @param handle SMBUS handle.
  * @param devAddr Slave Device Address.
  * @param buffer: buffer.wData Address of the data buff to be sent.
  * @param buffer: buffer.dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_MasterWriteBlocking(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                              unsigned int timeout, unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(devAddr <= SMBUS_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR); /* State check. */

    BASE_StatusType ret;
    /* Configuring SMBUS Parameters. */
    handle->state = SMBUS_STATE_BUSY_MASTER_TX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;
    handle->frameOpt = frameOpt;
    SMBusSetSlaveDevAddr(handle, devAddr); /* Set slave address. */
    SMBusConfigParameters(handle);

    /* Waiting for the SMBUS bus to be idle. */
    if ((handle->frameOpt & SMBUS_FRAME_FIRST) == SMBUS_FRAME_FIRST) {
        ret = SMBusWaitStatusReady(handle, BUS_IS_FREE, SMBUS_SEND_ADDR_STATUS_READ);
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    }
    /* Confige the frame set in smbus master write blocking. */
    ret = SMBusMasterWriteBlockingFrameSet(handle);
    if (ret != BASE_STATUS_OK) {
        SMBusSetErrorHandling(handle);
        return ret;
    }

    /* step3 : Send to slave data */
    ret = SMBusBlockingMasterTxDataOptStepNormal(handle);
    if (ret != BASE_STATUS_OK) {
        SMBusSetErrorHandling(handle);
        return ret;
    }
    /* step4 : send stop CMD */
    if ((handle->frameOpt & SMBUS_FRAME_STOP) == SMBUS_FRAME_STOP) {
        ret = SMBusBlockingSendStopCommand(handle);
    }
    SMBusWaitStatusReady(handle, TX_FIFO_EMPTY, SMBUS_OPERATION_WRITE);
    handle->frameOpt = SMBUS_FRAME_NONE;
    handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
    handle->state = SMBUS_STATE_READY;
    return ret;
}

/**
  * @brief Receiving data in blocking mode as slave.
  * @param handle SMBUS handle.
  * @param buffer: buffer.rData Address of the data buff to be receiving.
  * @param buffer: buffer.dataSize Number of the data to be receiving.
  * @param timeout Timeout period,unit: ms.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SlaveReadBlocking(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int timeout,
                                            unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR); /* State check. */

    BASE_StatusType ret;

    /* Configuring SMBUS Parameters. */
    handle->state = SMBUS_STATE_BUSY_SLAVE_RX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->timeout = timeout;
    handle->frameOpt = frameOpt;
    SMBusConfigParameters(handle);     /* step1 : Parameter Settings. */

    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    if ((handle->frameOpt & SMBUS_FRAME_START) == SMBUS_FRAME_START) {
        /* step2 : Waiting for slave address match. */
        ret = SMBusWaitStatusReady(handle, SLAVE_ADDRESS_MATCH, SMBUS_OPERATION_WRITE);
        if (ret != BASE_STATUS_OK) {
            handle->errorCode = ret;
            SMBusSetErrorHandling(handle);
            return ret;
        }
    }
    /* step3 : Slave receives data from the master device. */
    ret = SMBusBlockingSlaveRxDataOptStepNormal(handle);
    if (ret != BASE_STATUS_OK) {
        handle->errorCode = ret;
        SMBusSetErrorHandling(handle);
        return ret;
    }

    /* step3 : Wait until all commands are executed. */
    if ((handle->frameOpt & SMBUS_FRAME_STOP) == SMBUS_FRAME_STOP) {
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_P, 0);
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
        /* Waite command all done. */
        ret = SMBusWaitStatusReady(handle, COMMAND_ALL_DONE, SMBUS_OPERATION_WRITE);
        if (ret != BASE_STATUS_OK) {
            handle->errorCode = ret;
            SMBusSetErrorHandling(handle);
            return ret;
        }
    }

    handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
    handle->frameOpt = SMBUS_FRAME_NONE;
    handle->state = SMBUS_STATE_READY;
    return ret;
}

/**
  * @brief Send data in blocking mode as slave.
  * @param handle SMBUS handle.
  * @param buffer: buffer.wData Address of the data buff to be sent.
  * @param buffer: buffer.dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SlaveWriteBlocking(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int timeout,
                                             unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(timeout > 0, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR); /* State check. */

    BASE_StatusType ret;
    /* Configuring Transmission Parameters of SMBUS. */
    handle->state = SMBUS_STATE_BUSY_SLAVE_TX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->timeout = timeout;
    handle->frameOpt = frameOpt;
    handle->transferCount = 0;
    SMBusConfigParameters(handle);     /* step1 : Parameter Settings. */
    /* Confige the frame set in smbus slave write blocking. */
    ret = SMBusSlaveWriteBlockingFrameSet(handle);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }

    /* step2 : Slave send data to the master device. */
    while (handle->transferCount < handle->transferSize) {
        if (handle->transferCount == handle->transferSize - 1) {
            /* Deal the last frame. */
            if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
                ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_RPEC_TNACK_S_TPEC_RNACK, 0);
            } else {
                ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TNACK_S_TD_RNACK, *handle->transferBuff);
            }
        } else {
            ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_M_RD_TACK_S_TD_RACK, *handle->transferBuff);
        }

        if (ret != BASE_STATUS_OK) { /* Error check. */
            SMBusSetErrorHandling(handle);
            return ret;
        }
        handle->transferBuff++;
        handle->transferCount++;
    }
    /* step4 : send stop CMD */
    if ((handle->frameOpt & SMBUS_FRAME_STOP) == SMBUS_FRAME_STOP) {
        ret = SMBusBlockingSendStopCommand(handle);
    }
    handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
    handle->state = SMBUS_STATE_READY;
    return ret;
}

/**
  * @brief Receiving data in interrupts mode.
  * @param handle SMBUS handle.
  * @param devAddr Slave Device Address.
  * @param buffer: buffer.rData Address of the data buff to be receiving.
  * @param buffer: buffer.dataSize Number of the data to be receiving.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_MasterReadIT(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                       unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(devAddr <= SMBUS_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring I2C Parameters. */
    handle->state = SMBUS_STATE_BUSY_MASTER_RX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    SMBusSetSlaveDevAddr(handle, devAddr);
    handle->txReadCmdCount = 0;
    handle->sendAddrStatus = SMBUS_SEND_ADDR_STATUS_READ;

    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_RAW_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;

    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }
    if ((handle->frameOpt & SMBUS_FRAME_FIRST) == SMBUS_FRAME_FIRST) {
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    }
    /* Enable interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = (SMBUS_CFG_INTERRUPT_MASTER_RX | 0x4000);
    return BASE_STATUS_OK;
}

/**
  * @brief Send data in interrupts mode.
  * @param handle SMBUS handle.
  * @param devAddr Slave Device Address.
  * @param buffer: buffer.wData Address of the data buff to be sent.
  * @param buffer: buffer.dataSize Number of the data to be sent.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_MasterWriteIT(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                        unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(devAddr <= SMBUS_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring I2C Parameters. */
    handle->state = SMBUS_STATE_BUSY_MASTER_TX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    SMBusSetSlaveDevAddr(handle, devAddr);
    handle->txReadCmdCount = 0;
    handle->sendAddrStatus = SMBUS_SEND_ADDR_STATUS_WRITE;

    /* Clean interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_RAW_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.dma_operation = I2C_DMA_OP_NONE;

    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    if ((handle->frameOpt & SMBUS_FRAME_FIRST) == SMBUS_FRAME_FIRST) {
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    }
    /* Enable interrupt */
    handle->baseAddress->I2C_INTR_EN.reg = (SMBUS_CFG_INTERRUPT_MASTER_TX | 0x4000);
    return BASE_STATUS_OK;
}

/**
  * @brief Receiving data in interrupts mode as slave.
  * @param handle SMBUS handle.
  * @param buffer: buffer.rData Address of the data buff to be receiving.
  * @param buffer: buffer.dataSize Number of the data to be receiving.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SlaveReadIT(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt)
{
    BASE_StatusType ret;

    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring Transmission Parameters of SMBUS. */
    handle->state = SMBUS_STATE_BUSY_SLAVE_RX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    /* Configuring the SMBUS Timing */
    ret = SMBusSlaveMultiplexIT(handle);

    return ret;
}

/**
  * @brief Send data in interrupts mode as slave.
  * @param handle SMBUS handle.
  * @param buffer: buffer.wData Address of the data buff to be sent.
  * @param buffer: buffer.dataSize Number of the data to be sent.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SlaveWriteIT(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt)
{
    BASE_StatusType ret = BASE_STATUS_OK;

    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR);

    /* Configuring Transmission Parameters of SMBUS. */
    handle->state = SMBUS_STATE_BUSY_SLAVE_TX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    /* Configuring the SMBUS Timing */
    ret = SMBusSlaveMultiplexIT(handle);
    return ret;
}

/**
  * @brief Receiving data in DMA mode.
  * @param handle SMBUS handle.
  * @param devAddr Slave Device Address.
  * @param buffer: buffer.rData Address of the data buff to be receiving.
  * @param buffer: buffer.dataSize Number of the data to be receiving.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_MasterReadDMA(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                        unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Check the DMA transfer handle and channel. */
    SMBUS_ASSERT_PARAM(handle->dmaHandle != NULL);
    SMBUS_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET((handle->rxDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET((handle->rxDmaCh != handle->txDmaCh), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(devAddr <= SMBUS_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;

    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;

    /* Configuring I2C Parameters. */
    handle->state = SMBUS_STATE_BUSY_MASTER_RX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    SMBusSetSlaveDevAddr(handle, devAddr);
   /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;

    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    /* Waiting for the SMBUS bus to be idle. */
    if ((handle->frameOpt & SMBUS_FRAME_FIRST) == SMBUS_FRAME_FIRST) {
        ret = SMBusWaitStatusReady(handle, BUS_IS_FREE, SMBUS_SEND_ADDR_STATUS_READ);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    } else if (handle->baseAddress->I2C_CTRL1.BIT.mst_start != BASE_CFG_SET) {
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    }

    /* Set the start command and address to be sent. */
    if (handle->frameOpt & SMBUS_FRAME_START) {
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_S, 0); /* Sets the start command to be sent. */
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        ret = SMBusSendSlaveAddressReadCmd(handle); /* Send Address to Slave. */
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
    }
    ret = SMBusTransferDataDma(handle);
    return ret;
}

/**
  * @brief Send data in DMA mode.
  * @param handle SMBUS handle.
  * @param devAddr Slave Device Address.
  * @param buffer: buffer.wData Address of the data buff to be sent.
  * @param buffer: buffer.dataSize Number of the data to be sent.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_MasterWriteDMA(SMBUS_Handle *handle, unsigned short devAddr, SMBUS_DataBuffer buffer,
                                         unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Check the DMA transfer handle and channel. */
    SMBUS_ASSERT_PARAM(handle->dmaHandle != NULL);
    SMBUS_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(devAddr <= SMBUS_MAX_DEV_ADDR, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret;
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;

    /* Configuring SMBUS Parameters. */
    handle->state = SMBUS_STATE_BUSY_MASTER_TX;
    handle->transferBuff = buffer.data;
    handle->transferSize = buffer.dataSize;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    SMBusSetSlaveDevAddr(handle, devAddr);

    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;

    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    /* Waiting for the SMBUS bus to be idle. */
    if ((handle->frameOpt & SMBUS_FRAME_FIRST) == SMBUS_FRAME_FIRST) {
        ret = SMBusWaitStatusReady(handle, BUS_IS_FREE, SMBUS_SEND_ADDR_STATUS_READ);
        if (ret != BASE_STATUS_OK) {
            handle->errorCode = ret;
            SMBusSetErrorHandling(handle);
            return ret;
        }
        handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_SET;
    }

    /* Send SMBUS start */
    if ((handle->frameOpt & SMBUS_FRAME_START) == SMBUS_FRAME_START) {
        ret = SMBusSetTxFIFODataAndCmd(handle, I2C_CMD_S, 0); /* Sets the start command to be sent. */
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
        /* send slave addr */
        ret = SMBusSendSlaveAddressWriteCmd(handle);
    }
    if (ret != BASE_STATUS_OK) {
        SMBusSetErrorHandling(handle);
        return ret;
    }
    ret = SMBusTransferDataDma(handle);

    return ret;
}

/**
  * @brief Receiving data in DMA mode as slave.
  * @param handle SMBUS handle.
  * @param buffer: buffer.rData Address of the data buff to be receiving.
  * @param buffer: buffer.dataSize Number of the data to be receiving.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SlaveReadDMA(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Check the DMA transfer handle and channel. */
    SMBUS_ASSERT_PARAM(handle->dmaHandle != NULL);
    SMBUS_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET((handle->rxDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET((handle->rxDmaCh != handle->txDmaCh), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR); /* State Check. */

    BASE_StatusType ret = BASE_STATUS_OK;

    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    /* Configuring Transmission Parameters of SMBUS. */
    handle->state = SMBUS_STATE_BUSY_SLAVE_RX;
    handle->transferSize = buffer.dataSize;
    handle->transferBuff = buffer.data;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;

    if ((handle->frameOpt & SMBUS_FRAME_PEC) == SMBUS_FRAME_PEC) {
        handle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_SET;
    }

    /* Waiting for the address to be matched. */
    if ((handle->frameOpt & SMBUS_FRAME_START) == SMBUS_FRAME_START) {
        ret = SMBusWaitStatusReady(handle, SLAVE_ADDRESS_MATCH, SMBUS_OPERATION_WRITE);
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
    }
    SMBusTransferDataDma(handle);
    return ret;
}

/**
  * @brief Send data in DMA mode as slave.
  * @param handle SMBUS handle.
  * @param buffer: buffer.wData, Address of the data buff to be sent.
  * @param buffer: buffer.dataSize Number of the data to be sent.
  * @param frameOpt: frame format @ref SMBUS_FrameFormat.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT, NOT SUPPORT.
  */
BASE_StatusType HAL_SMBUS_SlaveWriteDMA(SMBUS_Handle *handle, SMBUS_DataBuffer buffer, unsigned int frameOpt)
{
    SMBUS_ASSERT_PARAM(handle != NULL && buffer.data != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(handle->baseAddress));
    /* Check the DMA transfer handle and channel. */
    SMBUS_ASSERT_PARAM(handle->dmaHandle != NULL);
    SMBUS_PARAM_CHECK_WITH_RET((handle->txDmaCh < CHANNEL_MAX_NUM), BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(buffer.dataSize > 0, BASE_STATUS_ERROR);
    SMBUS_PARAM_CHECK_WITH_RET(handle->state == SMBUS_STATE_READY, BASE_STATUS_ERROR);

    BASE_StatusType ret = BASE_STATUS_OK;
    handle->baseAddress->I2C_INTR_EN.reg = SMBUS_INTR_EN_ALL_DISABLE;
    handle->baseAddress->I2C_INTR_RAW.reg = SMBUS_INTR_RAW_ALL_ENABLE;
    /* Configuring Transmission Parameters of SMBUS. */
    handle->state = SMBUS_STATE_BUSY_SLAVE_TX;
    handle->transferSize = buffer.dataSize;
    handle->transferBuff = buffer.data;
    handle->transferCount = 0;
    handle->frameOpt = frameOpt;
    /* Startup Control */
    handle->baseAddress->I2C_CTRL1.BIT.rst_rx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.rst_tx_fifo = BASE_CFG_SET;
    handle->baseAddress->I2C_CTRL1.BIT.mst_start = BASE_CFG_UNSET;
    /* Waiting for the address to be matched. */
    if (handle->frameOpt & SMBUS_FRAME_START) {
        ret = SMBusWaitStatusReady(handle, SLAVE_ADDRESS_MATCH, SMBUS_OPERATION_WRITE);
        if (ret != BASE_STATUS_OK) {
            SMBusSetErrorHandling(handle);
            return ret;
        }
    }
    SMBusTransferDataDma(handle);
    return ret;
}

/**
  * @brief Interrupt Handling Function.
  * @param handle Handle pointers
  * @retval None
  */
void HAL_SMBUS_IrqHandler(void *handle)
{
    SMBUS_Handle *smbusHandle = (SMBUS_Handle *)handle;
    SMBUS_ASSERT_PARAM(smbusHandle != NULL);
    SMBUS_ASSERT_PARAM(IsI2CInstance(smbusHandle->baseAddress));

    unsigned int status;
    status = smbusHandle->baseAddress->I2C_INTR_STAT.reg;

    if (status & SMBUS_ERROR_BIT_MASK) { /* Check error interrupt mask. */
        SMBusIsInterruptErrorStatus(smbusHandle, status);
        return;
    }

    if ((status & SMBUS_ALERT_RAW_MASK) && (smbusHandle->userCallBack.AlertCallback != NULL)) {  /* smbus alert */
        smbusHandle->userCallBack.AlertCallback(smbusHandle);
        smbusHandle->baseAddress->I2C_INTR_RAW.reg = (1 << SMBUS_ALERT_RAW_POS);
        return;
    }
    /* Address match handler. */
    SMBusInterruptAddrMatchHandle(handle, status);

    /* Callback interrupt handler function. */
    SMBusInterruptHandle(smbusHandle, status);
    smbusHandle->baseAddress->I2C_INTR_RAW.reg =
        (SMBUS_INTR_RAW_TX_FIFO_NOT_FULL_MASK | SMBUS_INTR_RAW_RX_FIFO_FULL_MASK);
    if ((smbusHandle->transferCount >= smbusHandle->transferSize) &&
        (!(status & (SMBUS_INTR_RAW_ALL_CMD_DONE_MASK | SMBUS_INTR_RAW_STOP_DET_MASK)))) {
        if (smbusHandle->baseAddress->I2C_FIFO_STAT.BIT.tx_fifo_vld_num < SMBUS_MAX_FIFO_SIZE) {
            if ((smbusHandle->frameOpt & SMBUS_FRAME_STOP) == SMBUS_FRAME_STOP) {
                smbusHandle->baseAddress->I2C_TX_FIFO.reg =
                    (((unsigned int)I2C_CMD_P << I2C_TXFIFO_CMD_POS) & I2C_TXFIFO_CMD_MASK);
                smbusHandle->baseAddress->I2C_INTR_EN.BIT.tx_fifo_not_full_en = BASE_CFG_DISABLE;
                smbusHandle->baseAddress->I2C_MODE.BIT.xmb_pec_en = BASE_CFG_UNSET;
            }
        }
    }
    
    /* After all data transmission is complete, call the user's callback function. */
    SMBusInterruptAllDoneHandle(smbusHandle, status);
}