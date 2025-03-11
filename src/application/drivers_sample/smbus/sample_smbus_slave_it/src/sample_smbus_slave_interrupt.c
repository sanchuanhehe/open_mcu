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
  * @file    sample_smbus_slave_interrupt.c
  * @author  MCU Driver Team
  * @brief   Sample for SMBUS module interrupt as slave.
  * @details This sample demonstrates how to use the SMBUS slave interrupt interface to read and write with SMBUS slave.
  *          To use this sample, the SMBUS interface must be connected to the SMBUS slave.
  */
#include "main.h"
#include "debug.h"
#include "sample_smbus_slave_interrupt.h"

#define SMBUS_SAMPLE_SLAVE_OPT_LEN           8
#define SMBUS_TEST_START_FLAG                0x3A

#define SMBUS_SAMPLE_SLAVE_TEST_NUM          500

static volatile int g_txDoneFlag = BASE_CFG_UNSET;
static volatile int g_rxDoneFlag = BASE_CFG_UNSET;
static volatile int g_stopDoneFlag = BASE_CFG_UNSET;
static volatile int g_errorFlag = BASE_CFG_UNSET;

/**
  * @brief I2c interrupt sample tx callback handle.
  * @param handle I2c handle.
  * @retval None.
  */
void SMBusTxCallback(void *handle)
{
    DBG_PRINTF("SampleSMBusTxDone\r\n");
    BASE_FUNC_UNUSED(handle);
    g_txDoneFlag = BASE_CFG_SET;
}

/**
  * @brief I2c interrupt sample rx callback handle.
  * @param handle I2c handle.
  * @retval None.
  */
void SMBusRxCallback(void *handle)
{
    DBG_PRINTF("SampleSMBusRxDone\r\n");
    BASE_FUNC_UNUSED(handle);
    g_rxDoneFlag = BASE_CFG_SET;
}

/**
  * @brief I2c interrupt sample rx callback handle.
  * @param handle I2c handle.
  * @retval None.
  */
void SMBusStopCallback(void *handle)
{
    DBG_PRINTF("SMBusStopCallback\r\n");
    BASE_FUNC_UNUSED(handle);
    g_stopDoneFlag = BASE_CFG_SET;
}

/**
  * @brief I2c interrupt sample Error callback handle.
  * @param handle I2c handle.
  * @retval None.
  */
void SMBusErrorCallback(void *handle)
{
    DBG_PRINTF("SampleSMBusError\r\n");
    BASE_FUNC_UNUSED(handle);
    g_errorFlag = BASE_CFG_SET;
}

/**
  * @brief Read data from master in interrupt.
  * @param buffer Address of buff to be receive data.
  * @param len Number of the data to be read.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType SampleSlaveInterruptReadData(unsigned char *buffer, unsigned int len)
{
    BASE_StatusType ret = BASE_STATUS_OK; /* Transfer success and failure status. */
    unsigned int currentLen = len;
    unsigned char *tempBuffer = buffer;

    if (currentLen == 0) { /* Number of illegal transmissions. */
        return ret;
    }
    /* Read data from the master. */
    SMBUS_DataBuffer dataBuffer;
    unsigned int frameOpt;
            dataBuffer.data = tempBuffer;
        dataBuffer.dataSize = currentLen;
        frameOpt = SMBUS_FRAME_FIRST | SMBUS_FRAME_START | SMBUS_FRAME_MIDDLE | SMBUS_FRAME_STOP;
        ret = HAL_SMBUS_SlaveReadIT(&g_smbus, dataBuffer, frameOpt);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
        return ret;
    }
    /* Waiting for read completion or failure */
    while (!(g_rxDoneFlag || g_errorFlag)) {
        ;
    }
    g_rxDoneFlag = BASE_CFG_UNSET;
    if (g_errorFlag == BASE_CFG_SET) {
        g_errorFlag = BASE_CFG_UNSET;
        DBG_PRINTF("LINE:%d,Read Data Fail\r\n", __LINE__);
        return BASE_STATUS_ERROR;
    }

    return ret;
}

/**
  * @brief Send data to the master in interrupt.
  * @param buffer Address of buff to be send.
  * @param len Number of the data to be send.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType SampleSlaveInterruptWriteData(unsigned char *buffer, unsigned int len)
{
    BASE_StatusType ret = BASE_STATUS_OK; /* Transfer success and failure status. */
    unsigned int currentLen = len;
    unsigned char *tempBuffer = buffer;
    SMBUS_DataBuffer dataBuffer;
    unsigned int frameOpt; /* Frame structure for controlling SMBUS transmission. */
    if (currentLen == 0) {
        return ret;
    }
    /* Send data to the master. */
        dataBuffer.data = tempBuffer;
        dataBuffer.dataSize = currentLen;
        frameOpt = SMBUS_FRAME_FIRST | SMBUS_FRAME_START | SMBUS_FRAME_MIDDLE | SMBUS_FRAME_STOP;
        ret = HAL_SMBUS_SlaveWriteIT(&g_smbus, dataBuffer, frameOpt);
    if (ret != BASE_STATUS_OK) {
        DBG_PRINTF("LINE:%d,Write Data Fail!,ret:%d\r\n", __LINE__, ret);
        return ret;
    }
    /* Waiting for write completion or failure */
    while (!(g_txDoneFlag || g_errorFlag)) {
        ;
    }
    g_txDoneFlag = BASE_CFG_UNSET;
    if (g_errorFlag == BASE_CFG_SET) {
        g_errorFlag = BASE_CFG_UNSET;
        DBG_PRINTF("LINE:%d,Write Data Fail\r\n", __LINE__);
        return BASE_STATUS_ERROR;
    }

    return ret;
}

/**
  * @brief Send and receive data in interrupt mode as slave.
  * @retval None.
  */
void SMBusSlaveInterruptProcessing(void)
{
    BASE_StatusType ret;
    unsigned int i;
    unsigned int dataFlag;
    unsigned char tempWriteBuff[SMBUS_SAMPLE_SLAVE_OPT_LEN] = {0};
    unsigned int successCnt = 0;
    unsigned char testStart = 0;

    SystemInit();
    DBG_PRINTF("SMBUS Slave Interrupt Sample Start\r\n");
    for (i = 0; i < SMBUS_SAMPLE_SLAVE_OPT_LEN; i++) {
        tempWriteBuff[i] = 0x4A; /* The written data. */
    }
    /* Waiting for the start signal from the master */
    DBG_PRINTF("Waiting Master Start......\r\n");
    while (true) {
        SampleSlaveInterruptReadData(&testStart, 1);
        if (testStart == SMBUS_TEST_START_FLAG) {
            DBG_PRINTF("Master wait done\r\n");
            break;
        }
    }
    /* Start to send and receive test data. */
    for (int j = 0; j < SMBUS_SAMPLE_SLAVE_TEST_NUM; j++) {
        unsigned char tempReadBuff[SMBUS_SAMPLE_SLAVE_OPT_LEN] = {0};

        /* Write data to master */
        ret = SampleSlaveInterruptWriteData(tempWriteBuff, SMBUS_SAMPLE_SLAVE_OPT_LEN);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Write Data Fail,ret:%d\r\n", __LINE__, ret);
        }

        /* Read data from master */
        ret = SampleSlaveInterruptReadData(tempReadBuff, SMBUS_SAMPLE_SLAVE_OPT_LEN);
        if (ret != BASE_STATUS_OK) {
            DBG_PRINTF("LINE:%d,Read Data Fail,ret:%d\r\n", __LINE__, ret);
        }

        BASE_FUNC_DELAY_MS(5);  /* Delay 5 ms. */
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