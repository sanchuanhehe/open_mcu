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
  * @file    user_loader.c
  * @author  MCU Driver Team
  * @brief   This file provides the user loader structure and functions to manage the following functionalities of
  *          the user loader.
  *          + Initialization and de-initialization functions.
  *          + Handshake, command receiving, and command execution functions.
  *          + Read and write functions.
  *          + Atomic command function.
  */

/* Includes ------------------------------------------------------------------*/
#include "user_handshake_read.h"
#include "user_loader.h"

/* Macro definitions --------------------------------------------------------- */
#define HANDSHAKE_FRAME_SIZE 9

#define DELAY_TIME 1000
#define SYSTICK_MS_DIV 1000
#define FRAME_CMD_OFFSET 0
#define HEADER_COMMAND_HIGH 1
#define HEADER_COMMAND_LOW 2
#define COMMDNA_BASE_LENGHT 3

#define CRC_LAST_WIDTH 0x2
#define ACK_FRAME_SIZE 6
#define ACK_FRAME_TYPE_OFFSET          0
#define ACK_FRAME_SEQ_OFFSET           1
#define ACK_FRAME_INV_SEQ_OFFSET       2
#define ACK_FRAME_RESULT_OFFSET        3

#define MAX_FRAME_DATA_SIZE 1024     /**< Max size of Frame Data Payload */
#define MAX_FRAME_INDEX 254

#define FRAME_HEAD_TAIL_SIZE 5
#define IMAGE_ADDR_OFFSET  1    /**< Image address offset field in header frame */
#define IMAGE_SIZE_OFFSET  5    /**< Image size offset field in header frame */
#define WRITE_ADDRESS_MUILYP 8

#define TRY_TIMES_WAIT  10
#define DATA_HEAD_FRAME_SIZE  14

#define NEGOTIATE_FRAME_NUMS_POS 4
#define NEGOTIATE_FRAME_SIZE_POS 8

#define ADDR_REMOVE_HIGH_POS 0xFFFFFF
#define OFFSET_LENGHT_THIRD  3
#define OFFSET_LENGHT_SEVEN  7
#define OFTHIRD   3

#define FLASH_ERASE_PAGE 1024
#define ERASE_ADDRESS_MUILTYP 1024

#define BYTE_0            0
#define BYTE_0_OFFSET     24
#define BYTE_1            1
#define BYTE_1_OFFSET     16
#define BYTE_2            2
#define BYTE_2_OFFSET     8
#define BYTE_3            3

#define WAIT_ACK_FINISH_TIME 2   /* Wait until the ACK message is sent. */

#define PARAMETER_OFFSET_FLAG  1
#define PARAMETER_OFFSET_COPY_START_ADDR 5
#define PARAMETER_OFFSET_COPY_SIZE 9
#define PARAMETER_OFFSET_BACKUP_ADDR 13

#define ULOADER_UART_BAND_RATE 115200

/**
  * @brief Convert big-endian data to unsigned short.
  * @param buf the buffer to store conversion data.
  * @retval the value, unsigned short.
  */
static unsigned short ShiftToShort(unsigned char *buffer)
{
    if (buffer == NULL) {
        return 0;
    }
    return (((buffer[0] << 0x08) & 0xff00) + (buffer[1] & 0xff));
}

/**
  * @brief Convert big-endian data to unsigned short.
  * @param data the data of be converted.
  * @param buf the buffer to store conversion data.
  * @retval None.
  */
static void UShortToBigEndian(unsigned short data, unsigned char *const buf)
{
    unsigned int i = 0;
    buf[i++] = (unsigned char)((data >> 0x8) & 0xFF);   /* get the higher 8 bit of unsigned short */
    buf[i] = (unsigned char)(data & 0xFF);
}

/**
  * @brief Convert an unsigned int into big-endian data of the char type for storage.
  * @param data the data of be converted.
  * @param buf the buffer to store conversion data.
  * @retval None.
  */
static void UInitToBigEndian(unsigned int data, unsigned char *const buf)
{
    unsigned int i = 0;
    buf[i++] = (unsigned char)((data >> BYTE_0_OFFSET) & 0xFF);   /* get the higher 8 bit of unsigned short */
    buf[i++] = (unsigned char)((data >> BYTE_1_OFFSET) & 0xFF);
    buf[i++] = (unsigned char)((data >> BYTE_2_OFFSET) & 0xFF);
    buf[i] = (unsigned char)(data & 0xFF);
}

/**
  * @brief Get the value of min.
  * @param a the compare of value.
  * @param b the compare of value.
  * @retval the data of min.
  */
static inline unsigned int GetMinValue(unsigned int a, unsigned int b)
{
    return a < b ? a : b;
}

/**
  * @brief Combines two pieces of data of the char type into one piece of data of the short type.
  * @param highByte the high byte.
  * @param lowByte the low byte.
  * @retval the data, unsigned short.
  */
static inline unsigned short MergeToUshort(unsigned char highByte, unsigned char lowByte)
{
    return (highByte << 0x8) + lowByte;  /* Make highByte in the high 8 bit */
}

/**
  * @brief Convert big-endian data to unsigned int.
  * @param buf the buffer to be converted.
  * @retval the data, unsigned int.
  */
static inline unsigned int BigEndianToUint(const unsigned char *buf)
{
    unsigned int i = 0;
    unsigned int sum;
    if (buf == NULL) { /* Null pointer check. */
        return 0;
    }
    sum = buf[i++] << BYTE_0_OFFSET; /* Highest 8 bits. */
    sum += buf[i++] << BYTE_1_OFFSET;
    sum += buf[i++] << BYTE_2_OFFSET;
    sum += buf[i++];
    return sum;
}

/**
  * @brief Uart0 pin int.
  * @param None.
  * @retval None.
  */
static void UloaderUart0PinInit(void)
{
    /* Config PIN52 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_3_AS_UART0_TXD);                           /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_3_AS_UART0_TXD, PULL_NONE);                   /* Pull-up and Pull-down */
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_3_AS_UART0_TXD, SCHMIDT_DISABLE);          /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_3_AS_UART0_TXD, LEVEL_SHIFT_RATE_SLOW); /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_3_AS_UART0_TXD, DRIVER_RATE_2);              /* Output signal edge fast/slow */
    /* Config PIN53 */
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_4_AS_UART0_RXD); /* Check function selection */
    HAL_IOCMG_SetPinPullMode(GPIO0_4_AS_UART0_RXD, PULL_UP);
    HAL_IOCMG_SetPinSchmidtMode(GPIO0_4_AS_UART0_RXD, SCHMIDT_DISABLE);          /* Schmitt input on/off */
    HAL_IOCMG_SetPinLevelShiftRate(GPIO0_4_AS_UART0_RXD, LEVEL_SHIFT_RATE_SLOW); /* Output drive capability */
    HAL_IOCMG_SetPinDriveRate(GPIO0_4_AS_UART0_RXD, DRIVER_RATE_2);              /* Output signal edge fast/slow */
}

/**
  * @brief Uart0 pin deint.
  * @param None.
  * @retval None.
  */
static void UloaderUart0PinDeInit(void)
{
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_3_AS_GPIO0_3);
    HAL_IOCMG_SetPinAltFuncMode(GPIO0_4_AS_GPIO0_4);
}

/**
  * @brief Init the UART0 resources used by the user loader.
  * @param handle User loader handle.
  * @retval None.
  */
static void UloaderUart0Init(UART_Handle *handle)
{
    UloaderUart0PinInit();

    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);  /* UART0 clock enable. */
    handle->baseAddress = UART0;

    handle->baudRate = ULOADER_UART_BAND_RATE; /* The baudrate. */
    handle->dataLength = UART_DATALENGTH_8BIT;
    handle->stopBits = UART_STOPBITS_ONE;
    handle->parity = UART_PARITY_NONE;
    handle->txMode = UART_MODE_BLOCKING;
    handle->rxMode = UART_MODE_BLOCKING;
    handle->fifoMode = BASE_CFG_ENABLE;
    handle->fifoTxThr = UART_FIFODEPTH_SIZE6;
    handle->fifoRxThr = UART_FIFODEPTH_SIZE6;
    handle->hwFlowCtr = BASE_CFG_DISABLE;
    handle->handleEx.overSampleMultiple = UART_OVERSAMPLING_16X;
    handle->handleEx.msbFirst = BASE_CFG_DISABLE;
    HAL_UART_Init(handle);
    HAL_UART_EnableBaudDetectionEx(handle); /* Enable baud auto detect. */
}

/**
  * @brief Init the CRC resources used by the user loader.
  * @param handle User loader handle.
  * @retval None.
  */
static void UloaderCrcInit(CRC_Handle *handle)
{
    HAL_CRG_IpEnableSet(CRC_BASE, IP_CLK_ENABLE); /* Enable the CRG of CRC modular. */

    handle->baseAddress = CRC;
    handle->inputDataFormat = CRC_MODE_BIT8; /* Input data length is 8 byte. */
    handle->polyMode = CRC16_1021_POLY_MODE;
    handle->initValueType = TYPE_CRC_INIT_VALUE_00;
    handle->resultXorValueType = TYPE_CRC_XOR_VALUE_00;
    handle->reverseEnableType = REVERSE_INPUT_FALSE_OUTPUT_FALSE;
    handle->xorEndianEnbaleType = DISABLE_XOR_ENABLE_LSB;
    HAL_CRC_Init(handle);  /* Init CRC. */
}

/**
  * @brief Init the flash resources used by the user loader.
  * @param handle User loader handle.
  * @retval None.
  */
static void UloaderFlashInit(FLASH_Handle *handle)
{
    HAL_CRG_IpEnableSet(EFC_BASE, BASE_CFG_SET); /* Enable the CRG of EFC modular. */
    handle->baseAddress = EFC;
    handle->peMode = FLASH_PE_OP_BLOCK;
    HAL_FLASH_Init(handle); /* Init flash. */
}

/**
  * @brief Init the module resources of user loader.
  * @param handle User loader handle.
  * @retval BASE status type: OK, ERROR.
  */
void ULOADER_Init(ULOADER_Handle *handle)
{
    /* Reset crc\uart\flash */
    HAL_CRG_IpClkResetSet(CRC_BASE, BASE_CFG_SET);
    HAL_CRG_IpClkResetSet(UART0_BASE, BASE_CFG_SET);
    HAL_CRG_IpClkResetSet(EFC_BASE, BASE_CFG_SET);
    /* Init crc\uart\flash */
    UloaderFlashInit(handle->flash);
    UloaderUart0Init(handle->comHandle); /* If the communication protocols are different, change the value here. */
    UloaderCrcInit(handle->crc);
}

/**
  * @brief Deinit the module resources of user loader.
  * @param handle User loader handle.
  * @retval BASE status type: OK, ERROR.
  */
void ULOADER_DeInit(ULOADER_Handle *handle)
{
    /* Reset crc\uart\flash */
    HAL_CRG_IpClkResetSet(handle->comHandle, BASE_CFG_SET);
    HAL_CRG_IpClkResetSet(handle->crc, BASE_CFG_SET);
    HAL_CRG_IpClkResetSet(handle->flash, BASE_CFG_SET);
    UloaderUart0PinDeInit();
}

/**
  * @brief Handshake negotiation function.
  * @param handle User loader handle.
  * @param timeout the value of timeout.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_HandShake(ULOADER_Handle *handle, unsigned int timeout)
{
    unsigned int preTick = DCL_SYSTICK_GetTick(); /* Get the current tick. */
    unsigned int curTick;
    unsigned long long delta = 0;
    /* Get the timeout of tick */
    unsigned long long targetDelta = HAL_CRG_GetIpFreq(SYSTICK_BASE) / SYSTICK_MS_DIV * timeout;

    unsigned char buf[HANDSHAKE_FRAME_SIZE] = {0};
    BASE_StatusType ret;
    unsigned short receiveCrc;
    unsigned short calculateCrc;

    while (true) { /* Receive data cyclically until timeout. */
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) { /* Check timeout. */
            return BASE_STATUS_TIMEOUT;
        }
        preTick = curTick;
        
        /* Response data in the configuration success state. */
        buf[ACK_FRAME_TYPE_OFFSET] = XACK;
        buf[ACK_FRAME_SEQ_OFFSET] = 0x00;
        buf[ACK_FRAME_INV_SEQ_OFFSET] = 0xff;
        buf[ACK_FRAME_RESULT_OFFSET] = ACK_SUCCESS;
        calculateCrc = HAL_CRC_Calculate(handle->crc, buf, ACK_FRAME_SIZE - 0x02); /* 2: crc length. */
        UShortToBigEndian(calculateCrc, &buf[ACK_FRAME_SIZE - 0x02]);
        /* Receive handshake frame data. */
        ret = UloaderReceiveHandShakeFrame(handle->comHandle, buf, LENGTH_XHDSHK_FRAME, XHDSHK);
        if (ret != BASE_STATUS_OK) {
            continue;
        }
        /* Data Crc check. */
        receiveCrc = ShiftToShort(&buf[ACK_FRAME_SIZE + 0x01]); /* The offset of crc data. */
        calculateCrc = HAL_CRC_Calculate(handle->crc, buf, ACK_FRAME_SIZE + 0x01);
        if (receiveCrc != calculateCrc) {
            continue;
        }
        ULOADER_Ack(handle, ACK_SUCCESS, 0);
        return BASE_STATUS_OK;
    }
}

/**
  * @brief Receiving commands function.
  * @param handle User loader handle.
  * @param receiveCmd User loader command frame.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_ReceiveCmd(ULOADER_Handle *handle, ULOADER_Cmd *receiveCmd)
{
    /* receive command header and command. */
    unsigned char recvBuffer[LENGTH_XHEAD_FRAME] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  /* command size. */
    unsigned short cmdLength;
    while (1) {
        /* receive command header. */
        if (ULOADER_ReadDate(handle, recvBuffer, LENGTH_XHEAD_FRAME, XCMD, DELAY_TIME) != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }
        if (recvBuffer[FRAME_CMD_OFFSET] != XCMD) {
            ULOADER_Ack(handle, ACK_FAIL, 0);
            return BASE_STATUS_ERROR;
        }
        if (recvBuffer[HEADER_COMMAND_HIGH] == 0xFF) {    /* 0xFF: Judgement trail frame */
            ULOADER_Ack(handle, ACK_SUCCESS, XTAIL);
            return BASE_STATUS_ERROR;
        }
        /* Command Header Check */
        ULOADER_Ack(handle, ACK_SUCCESS, 0);
        /* Receive the boot cmd */
        cmdLength = MergeToUshort(recvBuffer[HEADER_COMMAND_HIGH], recvBuffer[HEADER_COMMAND_LOW]);
        cmdLength = cmdLength + COMMDNA_BASE_LENGHT; /* 3:head + 2 crc */
        if (ULOADER_ReadDate(handle, (unsigned char *)receiveCmd, cmdLength, XKEY, DELAY_TIME) != BASE_STATUS_OK) {
            ULOADER_Ack(handle, ACK_FAIL, receiveCmd->type);
            return BASE_STATUS_ERROR;
        }
        if (receiveCmd->type != XKEY) { /* Received data is not destination bytes. */
            ULOADER_Ack(handle, ACK_FAIL, 0);
            return BASE_STATUS_ERROR;
        }
        receiveCmd->type = receiveCmd->rcvBuf[0];
        receiveCmd->rcvBufLength = cmdLength;
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Command execution function.
  * @param handle User loader handle.
  * @param execCmd User loader command frame.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_CommandExec(ULOADER_Handle *handle, ULOADER_Cmd *execCmd)
{
    /* Funciton transfer table. */
    switch (execCmd->type) {
        case ATOMIS_WRITE:  /* Write command */
            if (ULOADER_CMD_Write(handle, execCmd) == BASE_STATUS_OK) {
                return BASE_STATUS_OK;
            }
            break;
        case ATOMIS_ERASE:  /* Erase command */
            if (ULOADER_CMD_Erase(handle, execCmd) == BASE_STATUS_OK) {
                return BASE_STATUS_OK;
            }
            break;
        case ATOMIS_UPDATE:  /* Updata command */
            if (ULOADER_CMD_Update(handle, execCmd) == BASE_STATUS_OK) {
                return BASE_STATUS_OK;
            }
            break;
        case ATOMIS_RESET:  /* Reset command */
            if (ULOADER_CMD_Reset(handle) == BASE_STATUS_OK) {
                return BASE_STATUS_OK;
            }
            break;
        default:
            break;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Normalized write data.
  * @param handle User loader handle.
  * @param wData the buffer of receive data.
  * @param dataSize the size of need receive.
  * @param timeout the value of timeout.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_WriteDate(ULOADER_Handle *handle, unsigned char *wData, unsigned int dataSize,
                                  unsigned int timeout)
{
    /* CRC produce. */
    unsigned int crc;
    crc = HAL_CRC_Calculate(handle->crc, wData, dataSize - CRC_LAST_WIDTH);
    UShortToBigEndian(crc, &wData[dataSize - CRC_LAST_WIDTH]);

    /* Send Data. */
    if (handle->mode == COMMUNICATION_UART) {
        return HAL_UART_WriteBlocking((UART_Handle *)handle->comHandle, wData, dataSize, timeout);
    } else if (handle->mode == COMMUNICATION_I2C) {
        return HAL_I2C_SlaveWriteBlocking((I2C_Handle *)handle->comHandle, wData, dataSize, timeout);
    } else if (handle->mode == COMMUNICATION_SPI) {
        return HAL_SPI_WriteBlocking((SPI_Handle *)handle->comHandle, wData, dataSize, timeout);
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Normalized read data.
  * @param handle User loader handle.
  * @param rData the buffer of receive data.
  * @param dataSize the size of need receive.
  * @param targetByte the target byte.
  * @param timeout the value of timeout.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_ReadDate(ULOADER_Handle *handle, unsigned char *rData, unsigned int dataSize,
                                 unsigned short targetByte, unsigned int timeout)
{
    BASE_StatusType ret;
    unsigned int crcReceive = 0;
    unsigned int crcCalculate = 0;

    /* Check the communication mode. */
    if (handle->mode == COMMUNICATION_UART) {
        ret = ULOADER_UART_ReadBlocking((UART_Handle *)handle->comHandle, rData, dataSize, targetByte, timeout);
    } else if (handle->mode == COMMUNICATION_I2C) {
        ret = ULOADER_I2C_ReadBlocking((I2C_Handle *)handle->comHandle, rData, dataSize, timeout);
    } else if (handle->mode == COMMUNICATION_SPI) {
        ret = ULOADER_SPI_ReadBlocking((SPI_Handle *)handle->comHandle, rData, dataSize, targetByte, timeout);
    } else {
        return BASE_STATUS_ERROR; /* Illegal communication mode. */
    }

    /* CRC Check */
    if (ret == BASE_STATUS_OK) {
        crcReceive = MergeToUshort(rData[dataSize - 0x2], rData[dataSize - 1]);
        crcCalculate = HAL_CRC_Calculate(handle->crc, rData, dataSize - CRC_LAST_WIDTH);
        if (crcReceive == crcCalculate) {
            return BASE_STATUS_OK;
        }
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Answering host.
  * @param handle User loader handle.
  * @param status answering status.
  * @param index the index of frame to answer.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_Ack(ULOADER_Handle *handle, ACK_Status status, unsigned char index)

{
    unsigned char frame[ACK_FRAME_SIZE];
    unsigned int sequence = index;

    /* Construct the Frame */
    frame[ACK_FRAME_TYPE_OFFSET] = XACK;
    frame[ACK_FRAME_SEQ_OFFSET] = index;
    frame[ACK_FRAME_INV_SEQ_OFFSET] = (unsigned char)(~sequence & 0xFF);
    frame[ACK_FRAME_RESULT_OFFSET] = status;

    /* Write frame to host. */
    return ULOADER_WriteDate(handle, frame, ACK_FRAME_SIZE, DELAY_TIME);
}

/**
  * @brief Receives file head date from the host.
  * @param handle User loader handle.
  * @param buf the buffer of receive date.
  * @param size the size of receive.
  * @param perFrameSize Size of each frame to be transmitted.
  * @param tryTimes retry Times After Failure.
  * @retval BASE status type: OK, ERROR.
  */
static BASE_StatusType ReceiveFileHead(ULOADER_Handle *handle, unsigned char *buf, unsigned int size,
                                       unsigned int *perFrameSize, unsigned char *tryTimes)
{
    unsigned int frameNums = 0;
    unsigned char answer = 0;
    unsigned int tempTryTime = 0;

    /* Number of retransmission times. */
    while (tempTryTime < TRY_TIMES_WAIT) {
        /* Read file heade data. */
        if (ULOADER_ReadDate(handle, buf, DATA_HEAD_FRAME_SIZE, XHEAD, DELAY_TIME) != BASE_STATUS_OK) {
            tempTryTime++;
            continue;
        }

        /* Negotiate frame size */
        frameNums = BigEndianToUint(&buf[NEGOTIATE_FRAME_NUMS_POS]);
        if (frameNums > MAX_FRAME_INDEX) {
            tempTryTime++;
            ULOADER_Ack(handle, ACK_FAIL, 0);
            continue;
        }

        *perFrameSize = BigEndianToUint(&buf[NEGOTIATE_FRAME_SIZE_POS]);
        if (*perFrameSize > MAX_FRAME_DATA_SIZE || *perFrameSize == 0) {
            tempTryTime++;
            ULOADER_Ack(handle, ACK_FAIL, 0);
            continue;
        }

        answer = size % *perFrameSize == 0 ? 0 : 1;
        if (((size / *perFrameSize) + answer) != frameNums) { /* Data length integrity check. */
            /* Negotiate fail */
            tempTryTime++;
            ULOADER_Ack(handle, ACK_FAIL, 0);
            continue;
        }
        ULOADER_Ack(handle, ACK_SUCCESS, 0);
        *tryTimes = tempTryTime;
        return BASE_STATUS_OK;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Receives data from the host.
  * @param handle User loader handle.
  * @param buf the buffer of receive date.
  * @param remainLen Remaining transmission data volume.
  * @param perFrameSize Size of each frame to be transmitted.
  * @param addr the target of flash address.
  * @retval BASE status type: OK, ERROR.
  */
static BASE_StatusType ReceiveFileDate(ULOADER_Handle *handle, unsigned char *buf, unsigned int remainLen,
                                       unsigned int perFrameSize, unsigned int addr)
{
    BASE_StatusType ret;
    unsigned char oldIdIndex = 0;
    unsigned char tryTimes = 0;
    unsigned int rcvLen;
    unsigned int targetAddr = addr;
    unsigned int targetremainLen = remainLen;

    /* Number of retransmission times. */
    while (tryTimes < TRY_TIMES_WAIT) {
        /* Gets the minimum number of transfers for the last receive processing. */
        rcvLen = GetMinValue(targetremainLen, perFrameSize);
        buf[1] = 0;
        if (targetremainLen > 0) { /* There is still data to be received and continue to receive data. */
            if (ULOADER_ReadDate(handle, buf, rcvLen + FRAME_HEAD_TAIL_SIZE, XDATA, DELAY_TIME) != BASE_STATUS_OK) {
                ULOADER_Ack(handle, ACK_FAIL, buf[1]);
                tryTimes++;
                continue;
            }
        } else {
            /* There is no data to be received, and received file tail frame. */
            if (ULOADER_ReadDate(handle, buf,  rcvLen + FRAME_HEAD_TAIL_SIZE, XTAIL, DELAY_TIME) != BASE_STATUS_OK) {
                ULOADER_Ack(handle, ACK_FAIL, buf[1]);
                tryTimes++;
                continue;
            }
        }
        /* Check the Sequence of Received Data Packets. */
        if ((buf[1] - oldIdIndex) != 1) {
            ULOADER_Ack(handle, ACK_SUCCESS, oldIdIndex);
            continue;
        }

        /* Data info and trail judgement. */
        if (buf[0] == XTAIL && targetremainLen <= 0) {
            /* Ack file trail frame. */
            ULOADER_Ack(handle, ACK_SUCCESS, buf[1]);
            return BASE_STATUS_OK;
        }

        /* Writes the received data to the flash memory. */
        ret = HAL_FLASH_WriteBlocking(handle->flash, (unsigned int)(buf + OFFSET_LENGHT_THIRD),
                                      targetAddr & ADDR_REMOVE_HIGH_POS,
                                      ((rcvLen + OFFSET_LENGHT_SEVEN) >> OFFSET_LENGHT_THIRD) << OFFSET_LENGHT_THIRD);
        if (ret != BASE_STATUS_OK) {
            tryTimes++;
            ULOADER_Ack(handle, ACK_FAIL, buf[1]); /* Flash write error */
            continue;
        }

        tryTimes = 0; /* Reset times. */
        ULOADER_Ack(handle, ACK_SUCCESS, buf[1]);
        /* ID update and check. */
        oldIdIndex = buf[1];
        targetAddr += rcvLen;
        targetremainLen -= rcvLen;
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief Write the flash page by atomic write command.
  * @param handle User loader handle.
  * @param command User loader command frame.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_CMD_Write(ULOADER_Handle *handle, ULOADER_Cmd *command)
{
    BASE_StatusType ret;
    unsigned int addr;
    unsigned int size;
    unsigned char tryTimes = 0;
    unsigned int remainLen;
    unsigned int perFrameSize = 0;
    unsigned char buf[MAX_FRAME_DATA_SIZE + FRAME_HEAD_TAIL_SIZE];

    /* Step1: Check the address validity. */
    addr = BigEndianToUint(&command->rcvBuf[IMAGE_ADDR_OFFSET]);  /* Get the address of erase. */
    size = BigEndianToUint(&command->rcvBuf[IMAGE_SIZE_OFFSET]);  /* Get the size of erase. */
    /* Address must be an integer multiple of 8, and size cannot be 0.  */
    if (((addr % WRITE_ADDRESS_MUILYP) != 0) || size == 0) {
        return BASE_STATUS_ERROR;
    }
    ULOADER_Ack(handle, ACK_SUCCESS, 0);
    
    /* Step2:Receive the file header frame. */
    remainLen = size;
    ret = ReceiveFileHead(handle, buf, size, &perFrameSize, &tryTimes);
    if (tryTimes == TRY_TIMES_WAIT || ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    /* Step3:Received data is written into the flash memory. */
    ret = ReceiveFileDate(handle, buf, remainLen, perFrameSize, addr);
    return ret;
}

/**
  * @brief Erase the flash page by atomic erase command.
  * @param handle User loader handle.
  * @param command User loader command frame.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_CMD_Erase(ULOADER_Handle *handle, ULOADER_Cmd *command)
{
    BASE_StatusType ret;
    unsigned int  addr;
    unsigned int  size;

    addr = BigEndianToUint(&command->rcvBuf[IMAGE_ADDR_OFFSET]);  /* Get the address of erase. */
    size = BigEndianToUint(&command->rcvBuf[IMAGE_SIZE_OFFSET]);  /* Get the size of erase. */
    /* Address must be an integer multiple of 1024. */
    if (((addr % ERASE_ADDRESS_MUILTYP) != 0)) {
        return BASE_STATUS_ERROR;
    }
    /* The size must be an integer multiple of 1024 and not equal to 0. */
    if ((size == 0) || ((size % ERASE_ADDRESS_MUILTYP) != 0)) {
        return BASE_STATUS_ERROR;
    }

    /* Erase flash page. */
    ret = HAL_FLASH_EraseBlocking(handle->flash, FLASH_ERASE_MODE_PAGE, addr & ADDR_REMOVE_HIGH_POS,
                                  size / FLASH_ERASE_PAGE);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    ULOADER_Ack(handle, ACK_SUCCESS, 0);
    return BASE_STATUS_OK;
}

/**
  * @brief Updating OTA Parameters by atomic update command.
  * @param handle User loader handle.
  * @param command User loader command frame.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_CMD_Update(ULOADER_Handle *handle, ULOADER_Cmd *command)
{
    BASE_StatusType ret;
    unsigned char buf[FLASH_ERASE_PAGE];
    unsigned int upDateFlag;
    unsigned int copyStartAddr;
    unsigned int copySize;
    unsigned int appBakAddr;

    upDateFlag = BigEndianToUint(&command->rcvBuf[PARAMETER_OFFSET_FLAG]); /* Get the firmware update flag. */
    copyStartAddr = BigEndianToUint(&command->rcvBuf[PARAMETER_OFFSET_COPY_START_ADDR]); /* Get the copy address. */
    copySize = BigEndianToUint(&command->rcvBuf[PARAMETER_OFFSET_COPY_SIZE]); /* Get the size of new firmware. */
    appBakAddr = BigEndianToUint(&command->rcvBuf[PARAMETER_OFFSET_BACKUP_ADDR]); /* Get the firmware backup address. */
    /* Read the flash page of upgrade parameters. */
    ret = HAL_FLASH_Read(handle->flash, (PARAMETER_ADDR - FLASH_READ_BASE), FLASH_ERASE_PAGE, buf, FLASH_ERASE_PAGE);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    /* Erase the flash page of upgrade parameters. */
    ret = HAL_FLASH_EraseBlocking(handle->flash, FLASH_ERASE_MODE_PAGE, PARAMETER_ADDR & ADDR_REMOVE_HIGH_POS, 1);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    /* Modify the parameter and assign a value. */
    UInitToBigEndian(upDateFlag, &buf[PARAMETER_OFFSET_FLAG - 0x01]);
    UInitToBigEndian(copyStartAddr, &buf[PARAMETER_OFFSET_COPY_START_ADDR - 0x01]);
    UInitToBigEndian(copySize, &buf[PARAMETER_OFFSET_COPY_SIZE - 0x01]);
    UInitToBigEndian(appBakAddr, &buf[PARAMETER_OFFSET_BACKUP_ADDR - 0x01]);
    /* Write the flash page of upgrade parameters. */
    ret = HAL_FLASH_WriteBlocking(handle->flash, (unsigned int)buf, PARAMETER_ADDR & ADDR_REMOVE_HIGH_POS,
                                  FLASH_ERASE_PAGE);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    ULOADER_Ack(handle, ACK_SUCCESS, 0);
    return BASE_STATUS_OK;
}

/**
  * @brief MCU reset by atomic reset command.
  * @param handle User loader handle.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType ULOADER_CMD_Reset(ULOADER_Handle *handle)
{
    ULOADER_Ack(handle, ACK_SUCCESS, 0);
    BASE_FUNC_DELAY_MS(WAIT_ACK_FINISH_TIME); /* Wait until the ACK message is sent. */
    BASE_FUNC_SoftReset();
    return BASE_STATUS_OK;
}