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
  * @file    sample_smbus_slave_dma.c
  * @author  MCU Driver Team
  * @brief   Sample for SMBUS module dma as slave.
  * @details This sample demonstrates how to use the SMBUS slave DMA interface to read and write with SMBUS slave.
  *          To use this sample, the SMBUS interface must be connected to the SMBUS slave.
  */
#include "main.h"
#include "debug.h"
#include "sample_smbus_slave_dma.h"

#define SMBUS_SAMPLE_SLAVE_OPT_LEN           8
#define SMBUS_TEST_START_FLAG                0x3A

#define SMBUS_SAMPLE_SLAVE_TEST_NUM          500

static volatile int g_txDoneFlag = BASE_CFG_UNSET;
static volatile int g_rxDoneFlag = BASE_CFG_UNSET;
static volatile int g_stopDoneFlag = BASE_CFG_UNSET;
static volatile int g_errorFlag = BASE_CFG_UNSET;

/**
  * @brief SMBUS interrupt sample tx callback handle.
  * @param handle SMBUS handle.
  * @retval None.
  */
void SMBusTxCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("Tx callback\r\n");
    g_txDoneFlag = BASE_CFG_SET;
}

/**
  * @brief SMBUS interrupt sample rx callback handle.
  * @param handle SMBUS handle.
  * @retval None.
  */
void SMBusRxCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("Rx callback\r\n");
    g_rxDoneFlag = BASE_CFG_SET;
}

/**
  * @brief SMBUS interrupt sample send stop frame callback handle.
  * @param handle SMBUS handle.
  * @retval None.
  */
void SMBusStopCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("stop callback\r\n");
    g_stopDoneFlag = BASE_CFG_SET;
}

/**
  * @brief SMBUS interrupt sample Error callback handle.
  * @param handle SMBUS handle.
  * @retval None.
  */
void SMBusErrorCallback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    DBG_PRINTF("Error callback\r\n");
    g_errorFlag = BASE_CFG_SET;
}

/**
  * @brief Read data from master in dma.
  * @param buffer Address of buff to be receive data.
  * @param len Number of the data to be read.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType SampleSlaveDmaReadData(unsigned char *buffer, unsigned int len)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    unsigned int currentLen = len;
    unsigned char *tempBuffer = buffer;
    SMBUS_DataBuffer dataBuffer;
    unsigned int frameOpt;

    if (currentLen == 0) { /* Number of illegal transmissions. */
        return ret;
    }

    /* Read data from the master. */
    dataBuffer.data = tempBuffer;
    dataBuffer.dataSize = currentLen;
    /* Configuring the SMBUS Transmission Frame Format. */
    frameOpt = SMBUS_FRAME_START | SMBUS_FRAME_MIDDLE | SMBUS_FRAME_STOP;
    ret = HAL_SMBUS_SlaveReadDMA(&g_smbus, dataBuffer, frameOpt);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
        return ret;
    }
    /* Waiting for read completion or failure */
    while (!(g_stopDoneFlag || g_errorFlag)) {
        ;
    }
    g_rxDoneFlag = BASE_CFG_UNSET;
        g_stopDoneFlag = BASE_CFG_UNSET;
    if (g_errorFlag == BASE_CFG_SET) {
        DBG_PRINTF("LINE:%d,Read Data Fail\r\n", __LINE__);
        g_errorFlag = BASE_CFG_UNSET;
        return BASE_STATUS_ERROR;
    }

    return ret;
}

/**
  * @brief Send data to the master in dma.
  * @param buffer Address of buff to be send.
  * @param len Number of the data to be send.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType SampleSlaveDmaWriteData(unsigned char *buffer, unsigned int len)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    SMBUS_DataBuffer dataBuffer;
    unsigned int frameOpt;
    unsigned int currentLen = len;
    unsigned char *tempBuffer = buffer;

    if (currentLen == 0) { /* Number of illegal transmissions. */
        return ret;
    }

    /* Send data to the master. */
    dataBuffer.data = tempBuffer;
    dataBuffer.dataSize = currentLen;
    /* Configuring the SMBUS Transmission Frame Format. */
    frameOpt = SMBUS_FRAME_START | SMBUS_FRAME_MIDDLE | SMBUS_FRAME_STOP;
    ret = HAL_SMBUS_SlaveWriteDMA(&g_smbus, dataBuffer, frameOpt);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("LINE:%d,Write Data Fail!,ret:%d\r\n", __LINE__, ret);
        return ret;
    }
    /* Waiting for read completion or failure */
    while (!(g_stopDoneFlag || g_errorFlag)) {
        ;
    }
    g_txDoneFlag = BASE_CFG_UNSET;
        g_stopDoneFlag = BASE_CFG_UNSET;
    if (g_errorFlag == BASE_CFG_SET) {
        g_errorFlag = BASE_CFG_UNSET;
        DBG_PRINTF("LINE:%d,Read Data Fail\r\n", __LINE__);
        return BASE_STATUS_ERROR;
    }

    return ret;
}

/**
  * @brief Send and receive data in dma mode as slave.
  * @retval None.
  */
void SMBusSlaveDmaProcessing(void)
{
    BASE_StatusType ret;
    unsigned int i;
    unsigned int dataFlag;
    unsigned char tempWriteBuff[SMBUS_SAMPLE_SLAVE_OPT_LEN] = {0};
    unsigned int successCnt = 0;
    unsigned char testStart = 0;

    SystemInit();
    DBG_PRINTF("SMBUS Slave Dma Sample Start\r\n");
    for (i = 0; i < SMBUS_SAMPLE_SLAVE_OPT_LEN; i++) {
        tempWriteBuff[i] = 0x4A; /* The written data. */
    }
    /* Waiting for the start signal from the master */
    DBG_PRINTF("Waiting Master Start......\r\n");
    while (true) {
        SampleSlaveDmaReadData(&testStart, 1);
        if (testStart == SMBUS_TEST_START_FLAG) {
            DBG_PRINTF("Master wait done\r\n");
            break;
        }
    }

    /* Start to send and receive test data. */
    for (int j = 0; j < SMBUS_SAMPLE_SLAVE_TEST_NUM; j++) {
        unsigned char tempReadBuff[SMBUS_SAMPLE_SLAVE_OPT_LEN] = {0};

        /* Write data to master */
        ret = SampleSlaveDmaWriteData(tempWriteBuff, SMBUS_SAMPLE_SLAVE_OPT_LEN);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Write Data Fail,ret:%d\r\n", __LINE__, ret);
        }

        /* Read data from master */
        ret = SampleSlaveDmaReadData(tempReadBuff, SMBUS_SAMPLE_SLAVE_OPT_LEN);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
        }

        BASE_FUNC_DELAY_MS(10);  /* Delay 10 ms. */
        /* Compare read and write data */
        dataFlag = 0;
        for (i = 0; i < SMBUS_SAMPLE_SLAVE_OPT_LEN; i++) {
            if (tempReadBuff[i] != tempWriteBuff[i]) {
                DBG_PRINTF("SMBUS Data error! offset[%d]\r\nReadData:0x%x\r\nWriteData:0x%x\r\n", i,
                    tempReadBuff[i], tempWriteBuff[i]);
                dataFlag = 1;
                break;
            }
        }

        /* The read data is exactly the same as the written data. */
        if (dataFlag == 0) {
            successCnt++;
            DBG_PRINTF("SMBUS Data Success\r\n");
        }
    }
    DBG_PRINTF("SMBUS sample End!\r\nsuccessCnt:%d\r\n", successCnt);
}