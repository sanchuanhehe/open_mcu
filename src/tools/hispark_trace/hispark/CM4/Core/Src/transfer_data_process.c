/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
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
  * @file    transfer_data_process.c
  * @author  MCU Driver Team
  * @brief   Data Processing of Heteronuclear Communication.
  */
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "securec.h"
#include "DAP_config.h"
#include "DAP.h"
#include "msg_queue.h"
#include "debug.h"
#include "gpio_config.h"
#include "swd_jtag_config.h"
#include "var_monitor_process.h"
#include "transfer_data_process.h"

#define MAX_SWD_RETRY                           100

#define TRANSFER_DATA_PROCESS_IDLE              0
#define TRANSFER_DATA_PROCESS_DEBUG             1
#define TRANSFER_DATA_PROCESS_SAMPLEINIT        2
#define TRANSFER_DATA_PROCESS_WRITEVAR          3
#define TRANSFER_DATA_PROCESS_CLOCK_CONFIG      4
#define TRANSFER_DATA_PROCESS_PERFORMANCE_TEST  5
#define TRANSFER_DATA_PROCESS_DEBUG_PORT_CONFIG 6
#define TRANSFER_DATA_PROCESS_PAUSESAMPLEINIT   7

typedef struct {
    uint32_t request;
    uint32_t data;
}TransferDebugInfo;

static uint32_t g_transferDataProcessState = TRANSFER_DATA_PROCESS_IDLE;
static uint32_t g_transferDataProcessPreState = TRANSFER_DATA_PROCESS_IDLE;
static TransferDebugInfo g_transfetDebugInfo;
static uint32_t g_configClock = 0;
static uint32_t g_debugPortConfig = DAP_PORT_DISABLED;


/**
  * @brief Saves the variable list information delivered by core A.
  * @param data Pointer to the data delivered by core A.
  * @param len Length of data delivered by core A.
  * @retval None.
  */
static void SaveSamplingMsgData(uint8_t *data, uint32_t len)
{
    uint32_t bufLen1 = sizeof(DapVarMonitor) - sizeof(DapVarList *);
    bufLen1 = bufLen1 > len ? len : bufLen1;
    memcpy_s(&g_dapValMonitor, sizeof(DapVarMonitor), data, bufLen1);

    if (g_dapValMonitor.isSample != 0) {
        if (g_dapValMonitor.varList != NULL) {
            free(g_dapValMonitor.varList);
            g_dapValMonitor.varList = NULL;
        }
        g_dapValMonitor.varList = malloc(g_dapValMonitor.varNum * sizeof(DapVarList));
        if (g_dapValMonitor.varList == NULL) {
            DBG_PRINTF("g_dapValMonitor.varList malloc fail!!\r\n");
            FreeMemory();
            return;
        }
        memcpy_s(g_dapValMonitor.varList, (g_dapValMonitor.varNum * sizeof(DapVarList)),
                 (data + bufLen1), g_dapValMonitor.varNum * sizeof(DapVarList));
    }
}

/**
  * @brief Saves the write variable information delivered by core A.
  * @param data Pointer to the data delivered by core A.
  * @param len Length of data delivered by core A.
  * @retval None.
  */
static void SaveWriteVarMsgData(uint8_t *data, uint32_t len)
{
    memcpy_s(&g_dapWriteVar, sizeof(g_dapWriteVar), data, len);
}

static bool SaveClockConfig(uint8_t *data, uint32_t len)
{
    errno_t rc = EOK;
    rc = memcpy_s(&g_configClock, sizeof(g_configClock), data, len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Saves the performance test information delivered by core A.
  * @param data Pointer to the data delivered by core A.
  * @param len Length of data delivered by core A.
  * @retval true or false.
  */
static bool SavePerformanceTestMsgData(uint8_t *data, uint32_t len)
{
    errno_t rc = EOK;
    rc = memcpy_s(&g_isPerformanceTestFlag, sizeof(g_isPerformanceTestFlag), data, len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

static bool SaveDebugPortConfig(uint8_t *data, uint32_t len)
{
    errno_t rc = EOK;
    rc = memcpy_s(&g_debugPortConfig, sizeof(g_debugPortConfig), data, len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

/**
  * @brief Saves the pause sampling information delivered by core A.
  * @param data Pointer to the data delivered by core A.
  * @param len Length of data delivered by core A.
  * @retval true or false.
  */
static bool SavePauseSamplingMsgData(uint8_t *data, uint32_t len)
{
    errno_t rc = EOK;
    rc = memcpy_s(&g_pauseSampleState, sizeof(g_pauseSampleState), data, len);
    if (rc != EOK) {
        return false;
    }
    return true;
}

static void MsgProcess(void)
{
    MsgBuf msg;
    if (MSGQ_RxIsEmpty() != true) {
        MSGQ_ReceiveMsg(&msg);
        switch (msg.type) {
            case MSG_TYPE_SAMPLING:
                SaveSamplingMsgData(msg.buf, msg.bufLen);
                g_transferDataProcessPreState = g_transferDataProcessState;
                g_transferDataProcessState = TRANSFER_DATA_PROCESS_SAMPLEINIT;
                break;
            case MSG_TYPE_WRITEVAR:
                SaveWriteVarMsgData(msg.buf, msg.bufLen);
                g_transferDataProcessPreState = g_transferDataProcessState;
                g_transferDataProcessState = TRANSFER_DATA_PROCESS_WRITEVAR;
                break;
            case MSG_TYPE_CLK_CFG:
                SaveClockConfig(msg.buf, msg.bufLen);
                g_transferDataProcessPreState = g_transferDataProcessState;
                g_transferDataProcessState = TRANSFER_DATA_PROCESS_CLOCK_CONFIG;
                break;
            case MSG_TYPE_PERFORMANCE_TEST:
                SavePerformanceTestMsgData(msg.buf, msg.bufLen);
                g_transferDataProcessPreState = g_transferDataProcessState;
                g_transferDataProcessState = TRANSFER_DATA_PROCESS_PERFORMANCE_TEST;
                break;
            case MSG_TYPE_DEBUG_PORT_SET:
                SaveDebugPortConfig(msg.buf, msg.bufLen);
                g_transferDataProcessPreState = g_transferDataProcessState;
                g_transferDataProcessState = TRANSFER_DATA_PROCESS_DEBUG_PORT_CONFIG;
                break;
            case MSG_TYPE_PAUSESAMPLING:
                SavePauseSamplingMsgData(msg.buf, msg.bufLen);
                g_transferDataProcessPreState = g_transferDataProcessState;
                g_transferDataProcessState = TRANSFER_DATA_PROCESS_PAUSESAMPLEINIT;
                break;
            default:
                break;
        }
    }
}

static void IdleProcess(void)
{
    MsgProcess();
}

/**
  * @brief Start/Stop sampling processing.
  * @param None.
  * @retval None.
  */
static void SamplingProcess(void)
{
    MsgBuf msg;
    int retValue;

    msg.type = MSG_TYPE_SAMPLING;
    msg.bufLen = sizeof(unsigned char);
    msg.buf[0] = Q_TRUE;
    if (g_dapValMonitor.isSample != 0) {
        DBG_PRINTF("M start\r\n");
        retValue = StartSampling();
        if (retValue != RET_SUCCESS) {
            StopSampling();
            msg.buf[0] = Q_FALSE;
        }
    } else {
        DBG_PRINTF("M STOP\r\n");
        StopSampling();
    }
    MSGQ_SendMsg(&msg);
    g_transferDataProcessState = g_transferDataProcessPreState;
    MsgProcess();
}

/**
  * @brief Write variable processing.
  * @param None.
  * @retval None.
  */
static void WriteVarProcess(void)
{
    uint8_t ret;
    MsgBuf msg;

    ret = WriteOneVal(g_dapWriteVar.address, g_dapWriteVar.len, g_dapWriteVar.value);
    if (ret == DAP_TRANSFER_OK) {
        msg.buf[0] = Q_TRUE;
    } else {
        msg.buf[0] = Q_FALSE;
    }
    msg.type = MSG_TYPE_WRITEVAR;
    msg.bufLen = sizeof(unsigned char);
    MSGQ_SendMsg(&msg);
    g_transferDataProcessState = g_transferDataProcessPreState;
    MsgProcess();
}

/**
  * @brief pause sampling processing.
  * @param None.
  * @retval None.
  */
static void PauseSamplingProcess(void)
{
    MsgBuf msg;

    PauseSampling(g_pauseSampleState);
    msg.type = MSG_TYPE_PAUSESAMPLING;
    msg.bufLen = sizeof(unsigned char);
    msg.buf[0] = Q_TRUE;
    MSGQ_SendMsg(&msg);
    g_transferDataProcessState = g_transferDataProcessPreState;
    MsgProcess();
}

static void ClockConfigProcess(void)
{
    SwdJtagClockLevelSet(g_configClock);
    g_transferDataProcessState = g_transferDataProcessPreState;
    MsgProcess();
}

/**
  * @brief Performance test processing.
  * @param None.
  * @retval None.
  */
static void PerformanceTestProcess(void)
{
    MsgBuf msg;

    DBG_PRINTF("g_isPerformanceTestFlag %d\r\n", g_isPerformanceTestFlag);
    msg.type = MSG_TYPE_PERFORMANCE_TEST;
    msg.bufLen = sizeof(unsigned char);
    msg.buf[0] = Q_TRUE;
    MSGQ_SendMsg(&msg);
    g_transferDataProcessState = g_transferDataProcessPreState;
    MsgProcess();
}

static void DebugPortConfigProcess(void)
{
    SwdJtagDebugPortSet((uint8_t)g_debugPortConfig);
    g_transferDataProcessState = g_transferDataProcessPreState;
    MsgProcess();
}

int32_t TransferDataProcessInit(void)
{
    g_transferDataProcessState = TRANSFER_DATA_PROCESS_IDLE;
    g_transferDataProcessPreState = TRANSFER_DATA_PROCESS_IDLE;
    g_transfetDebugInfo.data = 0;
    g_transfetDebugInfo.request = 0;
    return 0;
}

void TransferDataProcess(void)
{
    switch (g_transferDataProcessState) {
        case TRANSFER_DATA_PROCESS_IDLE:
            IdleProcess();
            break;
        case TRANSFER_DATA_PROCESS_SAMPLEINIT:
            SamplingProcess();
            break;
        case TRANSFER_DATA_PROCESS_WRITEVAR:
            WriteVarProcess();
            break;
        case TRANSFER_DATA_PROCESS_CLOCK_CONFIG:
            ClockConfigProcess();
            break;
        case TRANSFER_DATA_PROCESS_PERFORMANCE_TEST:
            PerformanceTestProcess();
            break;
        case TRANSFER_DATA_PROCESS_DEBUG_PORT_CONFIG:
            DebugPortConfigProcess();
            break;
        case TRANSFER_DATA_PROCESS_PAUSESAMPLEINIT:
            PauseSamplingProcess();
            break;
        default:
            break;
    }
}
