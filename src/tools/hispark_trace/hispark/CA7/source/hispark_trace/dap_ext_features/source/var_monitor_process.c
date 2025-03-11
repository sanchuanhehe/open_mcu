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
  * @file    var_monitor_process.c
  * @author  MCU Driver Team
  * @brief   Variable monitoring protocol, The functions are as follows:
  *           + starting variable monitoring.
  *           + stopping variable monitoring.
  *           + suspending variable monitoring.
  *           + uploading variable monitoring data.
  *           + modifying variable values...
  */
#include <string.h>
#include "securec.h"
#include "cmsis_os2.h"
#include "var_monitor_queue.h"
#include "debug.h"
#include "alloc.h"
#include "msg_queue.h"
#include "status.h"
#include "var_monitor_process.h"


struct DapStatistics {
    unsigned long long totalSampleCount;
    unsigned long long totalOverflowCount;
    unsigned long long totalErrorCount;
    unsigned int errorRate;
} g_dapStat = {0, 0, 0, 0};
extern unsigned int g_vargStart;
static VarQueue *g_dapVarMonitorQueue = (VarQueue *)&g_vargStart;
static DapVarMonitor g_dapVarMonitor;
static DapWriteVar g_dapWriteVar;
static unsigned int g_dapVarMonitorQueueSize;
static uint8_t *g_dapVarFrameBuffer = NULL;
static uint32_t g_iSPerformanceTest = 0;
static uint32_t g_timeoutTimer = 0;
static int g_sendEndFlag = 1;
static unsigned int g_sendNum = 0;
static int g_framingCompatibleFlag;

/**
  * @brief Sends the variable list to the M core.
  * @param varMonitorHandle Variable list information.
  * @retval type: DAP_OK, DAP_ERROR.
  */
static int StartVariableSampling(DapVarMonitor *varMonitorHandle)
{
    DapLinkStatus daplinkStatus;
    MsgBuf msgBuf;
    unsigned int timeout = 10;
    errno_t ret;

    DapLinkStatusGet(&daplinkStatus);
    daplinkStatus.dapDbgInfo.status = DAP_STATUS_SAMPLING;
    DapLinkStatusSet(&daplinkStatus);
    varMonitorHandle->isSample = true;

    /* Start Variable Sampling */
    msgBuf.type = MSG_TYPE_SAMPLING;
    unsigned int bufLen1 = sizeof(unsigned int) + sizeof(unsigned int) + sizeof(unsigned int);
    ret = memcpy_s((void *)&msgBuf.buf, bufLen1, (void *)varMonitorHandle, bufLen1);
    if (ret != EOK) {
        return DAP_ERROR;
    }
    unsigned int bufLen2 = varMonitorHandle->varNum * sizeof(DapVarList);
    ret = memcpy_s((void *)&msgBuf.buf[bufLen1], bufLen2, (void *)varMonitorHandle->varList, bufLen2);
    if (ret != EOK) {
        return DAP_ERROR;
    }
    msgBuf.bufLen = bufLen1 + bufLen2;
    MSGQ_SendMsg(&msgBuf);
    memset_s(&msgBuf, sizeof(MsgBuf), 0, sizeof(MsgBuf));

    while (MSGQ_ReceiveMsg(&msgBuf) && (timeout > 0)) {
        timeout--;
        osDelay(1);
    }
    if ((timeout == 0) || msgBuf.type != MSG_TYPE_SAMPLING || (msgBuf.buf[0] != MSG_OK)) {
        return DAP_ERROR;
    }

    /* Timestamp 4-byte. */
    g_dapVarMonitorQueueSize = 4;
    for (unsigned int i = 0; i < varMonitorHandle->varNum; i++) {
        g_dapVarMonitorQueueSize += varMonitorHandle->varList[i].varNumBytes;
    }

    MemFree(varMonitorHandle->varList);
    varMonitorHandle->varList = NULL;

    g_dapVarFrameBuffer = (uint8_t *)MemAlloc(DAP_TRANSFER_MAX_PACKET_SIZE * sizeof(uint8_t));
    if (g_dapVarFrameBuffer == NULL) {
        return DAP_ERROR;
    }
    /* Start Timer */
    g_timeoutTimer = HAL_GetTickUs();

    return DAP_OK;
}

/**
  * @brief Batch Receive Variable List.
  * @param varMonitorHandle Variable list information.
  * @param request request buffer pointer.
  * @param sendLength Length of the message sent to core M.
  * @retval type: DAP_OK, DAP_ERROR.
  */
static int BatchVarTransfer(DapVarMonitor *varMonitorHandle, const uint8_t *request, unsigned int *sendLength)
{
    unsigned int lastSendNum;
    int ret = DAP_OK;
    unsigned int maskPos0;
    unsigned int maskPos8;
    unsigned int maskPos16;
    unsigned int maskPos24;

    if (g_sendNum == 0) {
        /* The first eight bytes are the sampling frequency and the number of variables. */
        request += VAR_LIST_PROTOCOL_POS;
    }
    lastSendNum = g_sendNum;
    maskPos0 = (*(request++) & MASK_POS_0);
    maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
    maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
    maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
    g_sendNum = maskPos0 | maskPos8 | maskPos16 | maskPos24;

    /* The sent length is 4 + g_sendNum * 8. */
    *sendLength = (4 + g_sendNum * 8);
    g_sendNum += lastSendNum;
    if (g_sendNum == varMonitorHandle->varNum) {
        g_sendEndFlag = 1;
    }

    for (unsigned int i = lastSendNum; i < g_sendNum; i++) {
        maskPos0 = (*(request++) & MASK_POS_0);
        maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
        maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
        maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
        varMonitorHandle->varList[i].varAddress = maskPos0 | maskPos8 | maskPos16 | maskPos24;
        maskPos0 = (*(request++) & MASK_POS_0);
        maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
        maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
        maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
        varMonitorHandle->varList[i].varNumBytes = maskPos0 | maskPos8 | maskPos16 | maskPos24;
    }

    if (g_sendEndFlag != 0) {
        ret = StartVariableSampling(varMonitorHandle);
    }

    return ret;
}

/**
  * @brief One-time Receive Variable List.
  * @param varMonitorHandle Variable list information.
  * @param request request buffer pointer.
  * @retval type: DAP_OK, DAP_ERROR.
  */
static int OneTimeVarTransfer(DapVarMonitor *varMonitorHandle, const uint8_t *request)
{
    int ret;
    unsigned int maskPosition0;
    unsigned int maskPosition8;
    unsigned int maskPosition16;
    unsigned int maskPosition24;

    /* The first eight bytes are the sampling frequency and the number of variables. */
    request += VAR_LIST_PROTOCOL_POS;
    for (unsigned int i = 0; i < varMonitorHandle->varNum; i++) {
        maskPosition0 = (*(request++) & MASK_POS_0);
        maskPosition8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
        maskPosition16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
        maskPosition24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
        varMonitorHandle->varList[i].varAddress = maskPosition0 | maskPosition8 | maskPosition16 | maskPosition24;
        maskPosition0 = (*(request++) & MASK_POS_0);
        maskPosition8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
        maskPosition16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
        maskPosition24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
        varMonitorHandle->varList[i].varNumBytes = maskPosition0 | maskPosition8 | maskPosition16 | maskPosition24;
    }

    ret = StartVariableSampling(varMonitorHandle);

    return ret;
}

/**
  * @brief Obtains the parameters of variable sampling.
  * @param varMonitorHandle Variable list information.
  * @param request request buffer pointer.
  * @retval type: DAP_OK, DAP_ERROR.
  */
static int GetVarMonitorParam(DapVarMonitor *varMonitorHandle, const uint8_t *request)
{
    unsigned int maskPos0;
    unsigned int maskPos8;
    unsigned int maskPos16;
    unsigned int maskPos24;

    maskPos0 = (*(request++) & MASK_POS_0);
    maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
    maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
    maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
    varMonitorHandle->periodUs = maskPos0 | maskPos8 | maskPos16 | maskPos24;
    maskPos0 = (*(request++) & MASK_POS_0);
    maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
    maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
    maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
    varMonitorHandle->varNum = maskPos0 | maskPos8 | maskPos16 | maskPos24;
    if (varMonitorHandle->varNum <= MAX_VAR_NUM) {
        g_framingCompatibleFlag = 1;
    } else {
        g_framingCompatibleFlag = 0;
    }
    g_sendNum = 0;

    varMonitorHandle->varList = (DapVarList *)MemAlloc(varMonitorHandle->varNum * sizeof(DapVarList));
    if (varMonitorHandle->varList == NULL) {
        DBG_PRINTF("varList malloc fail!\r\n");
        return DAP_ERROR;
    }

    return DAP_OK;
}

/**
  * @brief The release interface is used to read variable data in the shared memory.
  * @param maxTimes Maximum number of reads.
  * @param response response buffer pointer.
  * @retval Number of bytes actually read.
  */
static unsigned int ReadVarRelease(unsigned int maxTimes, uint8_t *response)
{
    unsigned long long readableSampleTimes;
    unsigned long long maxTimesLong = maxTimes;
    unsigned long long readFrameCountLong;
    unsigned int readFrameCount;
    unsigned int readQueueSize;
    unsigned int retLen;

    if (g_framingCompatibleFlag != 0) {
        readableSampleTimes = QueueReadableLength(g_dapVarMonitorQueue) / g_dapVarMonitorQueueSize;
    } else {
        readableSampleTimes = QueueReadableLength(g_dapVarMonitorQueue);
    }
    readFrameCountLong = readableSampleTimes > maxTimesLong ? maxTimesLong : readableSampleTimes;
    readFrameCount = readFrameCountLong & MASK_LOW_32;
    *response++ = (readFrameCount) & MASK_POS_0;
    *response++ = (readFrameCount >> SHIFTS_8_BIT) & MASK_POS_0;
    *response++ = (readFrameCount >> SHIFTS_16_BIT) & MASK_POS_0;
    *response++ = (readFrameCount >> SHIFTS_24_BIT) & MASK_POS_0;
    if (g_framingCompatibleFlag != 0) {
        readQueueSize = readFrameCount * g_dapVarMonitorQueueSize;
    } else {
        readQueueSize = readFrameCount;
    }
    retLen = ReadQueue(g_dapVarMonitorQueue, response, readQueueSize);
    /* No data in the queue exits the loop. */
    if (retLen != readQueueSize) {
        readQueueSize = 0;
    }

    return readQueueSize;
}

/**
  * @brief Frees the memory of the variable list.
  * @param None.
  * @retval None.
  */
static void FreeVarList(void)
{
    if (g_dapVarMonitor.varList != NULL) {
        MemFree(g_dapVarMonitor.varList);
        g_dapVarMonitor.varList = NULL;
    }
}

/**
  * @brief Receives variable monitoring information and sends it to the M core.
  * @param request request buffer pointer.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_VarMonitor(const uint8_t *request, uint8_t *response)
{
    unsigned int requestLength = 0;
    unsigned int batchSendLength = 0;
    int ret;
    *response = DAP_OK;

    if (g_sendEndFlag != 0) {
        g_sendEndFlag = 0;
        ret = GetVarMonitorParam(&g_dapVarMonitor, request);
        if (ret == DAP_ERROR) {
            FreeVarList();
            *response = DAP_ERROR;
            return (requestLength << 16U) | 1U;
        }

        /* sizeof(periodUs + varNum) = 8 */
        requestLength = 8;
        if (g_framingCompatibleFlag != 0) {
            g_sendEndFlag = 1;

            ret = OneTimeVarTransfer(&g_dapVarMonitor, request);
            if (ret == DAP_ERROR) {
                FreeVarList();
                *response = DAP_ERROR;
                return (requestLength << 16U) | 1U;
            }
            /* sizeof(DapVarList) = 8 */
            requestLength += g_dapVarMonitor.varNum * 8;
            return (requestLength << 16U) | 1U;
        }
    }

    ret = BatchVarTransfer(&g_dapVarMonitor, request, &batchSendLength);
    requestLength += batchSendLength;
    if (ret == DAP_ERROR) {
        FreeVarList();
        *response = DAP_ERROR;
        return (requestLength << 16U) | 1U;
    }
    return (requestLength << 16U) | 1U;
}

/**
  * @brief Read variable data and upload it.
  * @param request request buffer pointer.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_ReadVar(const uint8_t *request, uint8_t *response)
{
    unsigned int maxSampleTimes;
    unsigned int readQueueSize;
    unsigned int maskPos0;
    unsigned int maskPos8;
    unsigned int maskPos16;
    unsigned int maskPos24;
    errno_t ret = EOK;

    if (!g_dapVarMonitor.isSample) {
        *response++ = 0;
        *response++ = 0;
        *response++ = 0;
        *response++ = 0;
        return (0U << 16U) | 4U;
    }

    /* Clear the timer. */
    g_timeoutTimer = HAL_GetTickUs();

    if (g_framingCompatibleFlag != 0) {
        maxSampleTimes = DAP_TRANSFER_MAX_PACKET_SIZE / g_dapVarMonitorQueueSize;
        ret = memset_s(g_dapVarFrameBuffer, g_dapVarMonitorQueueSize * maxSampleTimes,
                       0, g_dapVarMonitorQueueSize * maxSampleTimes);
    } else {
        maskPos0 = (*(request++) & MASK_POS_0);
        maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
        maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
        maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
        maxSampleTimes = maskPos0 | maskPos8 | maskPos16 | maskPos24;
        ret = memset_s(g_dapVarFrameBuffer, maxSampleTimes, 0, maxSampleTimes);
    }
    if (ret != EOK) {
        return (0U << 16U) | 4U;
    }
    readQueueSize = ReadVarRelease(maxSampleTimes, response);
    if (readQueueSize == 0) {
        *response++ = 0;
        *response++ = 0;
        *response++ = 0;
        *response++ = 0;
    }

    return (0U << 16U) | (readQueueSize + 4U);
}

/**
  * @brief Sending a sampling stop instruction to the M core.
  * @param None.
  * @retval type: DAP_OK, DAP_ERROR.
  */
static uint32_t StopVarMonitor(void)
{
    uint32_t ret = DAP_OK;
    MsgBuf msgBuf;
    unsigned int timeout = 10;
    errno_t rc = EOK;

    /* Stop Variable Sampling */
    g_dapVarMonitor.isSample = false;
    msgBuf.type = MSG_TYPE_SAMPLING;
    msgBuf.bufLen = sizeof(unsigned int);
    rc = memcpy_s((void *)&msgBuf.buf, msgBuf.bufLen, (void *)&g_dapVarMonitor, msgBuf.bufLen);
    if (rc != EOK) {
        ret = DAP_ERROR;
        goto STOP_ERR;
    }
    MSGQ_SendMsg(&msgBuf);
    rc = memset_s(&msgBuf, sizeof(MsgBuf), 0, sizeof(MsgBuf));
    if (rc != EOK) {
        ret = DAP_ERROR;
        goto STOP_ERR;
    }
    while (MSGQ_ReceiveMsg(&msgBuf) && (timeout > 0)) {
        timeout--;
        osDelay(1);
    }
    if ((timeout <= 0) || msgBuf.type != MSG_TYPE_SAMPLING || (msgBuf.buf[0] != MSG_OK)) {
        ret = DAP_ERROR;
    }

STOP_ERR:
    if (g_dapVarFrameBuffer != NULL) {
        MemFree(g_dapVarFrameBuffer);
        g_dapVarFrameBuffer = NULL;
    }
    return ret;
}

/**
  * @brief End sampling.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_StopVarMonitor(uint8_t *response)
{
    uint32_t ret;
    DapLinkStatus daplinkStatus;
    ret = StopVarMonitor();

    DapLinkStatusGet(&daplinkStatus);
    daplinkStatus.dapDbgInfo.status = DAP_STATUS_NOTWORKING;
    DapLinkStatusSet(&daplinkStatus);
    if (ret) {
        *response = DAP_ERROR;
        return (1U);
    }

    *response = DAP_OK;
    return (1U);
}

/**
  * @brief Pause/resume sampling.
  * @param request request buffer pointer.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_PauseVarMonitor(const uint8_t *request, uint8_t *response)
{
    MsgBuf msgBuf;
    unsigned int timeout = 10;
    errno_t rc = EOK;

    msgBuf.type = MSG_TYPE_PAUSESAMPLING;
    msgBuf.bufLen = sizeof(uint8_t);
    rc= memcpy_s((void *)&msgBuf.buf, msgBuf.bufLen, request, msgBuf.bufLen);
    if (rc != EOK) {
        *response = DAP_ERROR;
        return (sizeof(uint8_t) << 16U) | (1U);
    }
    MSGQ_SendMsg(&msgBuf);
    rc = memset_s(&msgBuf, sizeof(MsgBuf), 0, sizeof(MsgBuf));
    if (rc != EOK) {
        *response = DAP_ERROR;
        return (sizeof(uint8_t) << 16U) | (1U);
    }
    while (MSGQ_ReceiveMsg(&msgBuf) && (timeout > 0)) {
        timeout--;
        osDelay(1);
    }
    if ((timeout <= 0) || msgBuf.type != MSG_TYPE_PAUSESAMPLING || (msgBuf.buf[0] != MSG_OK)) {
        *response = DAP_ERROR;
        return (sizeof(uint8_t) << 16U) | (1U);
    }

    *response = DAP_OK;
    return (sizeof(uint8_t) << 16U) | (1U);
}

/**
  * @brief Status detection.
  * @param None.
  * @retval None.
  */
void DAP_StatusDetection(void)
{
    uint32_t curTicks;
    uint32_t delta;
    DapLinkStatus daplinkStatus;

    if (g_dapVarMonitor.isSample) {
        curTicks = HAL_GetTickUs();
        delta = GetTickDelta(curTicks, g_timeoutTimer);
        if (delta >= DISCONNECT_SESSION_TIMEOUT) {
            StopVarMonitor();
            DBG_PRINTF("SAMPLEOFFEXCEPTION\r\n");
            DapLinkStatusGet(&daplinkStatus);
            daplinkStatus.dapDbgInfo.status = DAP_STATUS_SAMPLEOFFEXCEPTION;
            DapLinkStatusSet(&daplinkStatus);
        }
        if (g_dapVarMonitorQueue->exceptionExitFlag) {
            StopVarMonitor();
            DBG_PRINTF("GDBEXCEPTIONEXIT\r\n");
            DapLinkStatusGet(&daplinkStatus);
            daplinkStatus.dapDbgInfo.status = DAP_STATUS_GDBEXCEPTIONEXIT;
            daplinkStatus.dapDbgInfo.errorRate = g_dapVarMonitorQueue->errorRate;
            DapLinkStatusSet(&daplinkStatus);
        }
    }
}

/**
  * @brief Update DAP statistics.
  * @param None.
  * @retval None.
  */
void UpdateDapStatistics(void)
{
    uint32_t flag = 0;
    if (!g_dapVarMonitor.isSample) {
        return;
    }
    if (g_dapVarMonitorQueue->totalSampleCount != g_dapStat.totalSampleCount) {
        g_dapStat.totalSampleCount = g_dapVarMonitorQueue->totalSampleCount;
        flag = 1;
    }
    if (g_dapVarMonitorQueue->totalOverflowCount != g_dapStat.totalOverflowCount) {
        g_dapStat.totalOverflowCount = g_dapVarMonitorQueue->totalOverflowCount;
        flag = 1;
    }
    if (g_dapVarMonitorQueue->totalErrorCount != g_dapStat.totalErrorCount) {
        g_dapStat.totalErrorCount = g_dapVarMonitorQueue->totalErrorCount;
        flag = 1;
    }
    if (g_dapVarMonitorQueue->errorRate != g_dapStat.errorRate) {
        g_dapStat.errorRate = g_dapVarMonitorQueue->errorRate;
        flag = 1;
    }
    if (flag) {
        DapLinkStatus daplinkStatus;
        DapLinkStatusGet(&daplinkStatus);
        daplinkStatus.dapDbgInfo.totalSampleCount = g_dapStat.totalSampleCount;
        daplinkStatus.dapDbgInfo.totalOverflowCount = g_dapStat.totalOverflowCount;
        daplinkStatus.dapDbgInfo.totalErrorCount = g_dapStat.totalErrorCount;
        daplinkStatus.dapDbgInfo.errorRate = g_dapStat.errorRate;
        DapLinkStatusSet(&daplinkStatus);
    }
}

/**
  * @brief Writing Variables.
  * @param request request buffer pointer.
  * @param response response buffer pointer.
  * @retval high 16-bits is request length, low 16-bits is responce length.
  */
uint32_t DAP_WriteVar(const uint8_t *request, uint8_t *response)
{
    MsgBuf msgBuf;
    unsigned int timeout = 10;
    unsigned int maskPos0;
    unsigned int maskPos8;
    unsigned int maskPos16;
    unsigned int maskPos24;
    errno_t rc = EOK;

    maskPos0 = (*(request++) & MASK_POS_0);
    maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
    maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
    maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
    g_dapWriteVar.address = maskPos0 | maskPos8 | maskPos16 | maskPos24;
    maskPos0 = (*(request++) & MASK_POS_0);
    maskPos8 = ((*(request++) << SHIFTS_8_BIT) & MASK_POS_8);
    maskPos16 = ((*(request++) << SHIFTS_16_BIT) & MASK_POS_16);
    maskPos24 = ((*(request++) << SHIFTS_24_BIT) & MASK_POS_24);
    g_dapWriteVar.len = maskPos0 | maskPos8 | maskPos16 | maskPos24;
    /* sizeof(value) = 8 */
    rc = memcpy_s(&g_dapWriteVar.value, MAX_WRITE_BYTE_VALUE, request, MAX_WRITE_BYTE_VALUE);
    if (rc != EOK) {
        *response = DAP_ERROR;
        return (sizeof(DapWriteVar) << 16U) | (1U);
    }

    msgBuf.type = MSG_TYPE_WRITEVAR;
    msgBuf.bufLen = sizeof(DapWriteVar);
    rc = memcpy_s((void *)&msgBuf.buf, msgBuf.bufLen, (void *)&g_dapWriteVar, msgBuf.bufLen);
    if (rc != EOK) {
        *response = DAP_ERROR;
        return (sizeof(DapWriteVar) << 16U) | (1U);
    }
    MSGQ_SendMsg(&msgBuf);
    rc = memset_s(&msgBuf, sizeof(MsgBuf), 0, sizeof(MsgBuf));
    if (rc != EOK) {
        *response = DAP_ERROR;
        return (sizeof(DapWriteVar) << 16U) | (1U);
    }
    while (MSGQ_ReceiveMsg(&msgBuf) && (timeout > 0)) {
        timeout--;
        osDelay(1);
    }
    if ((timeout <= 0) || msgBuf.type != MSG_TYPE_WRITEVAR || (msgBuf.buf[0] != MSG_OK)) {
        *response = DAP_ERROR;
        return (sizeof(DapWriteVar) << 16U) | (1U);
    }

    *response = DAP_OK;
    return (sizeof(DapWriteVar) << 16U) | (1U);
}

/**
  * @brief Performance test interface.
  * @param flag performance test flag.
  * @retval None.
  */
void SetPerformanceTest(unsigned int flag)
{
    MsgBuf msgBuf;
    unsigned int timeout = 10;
    errno_t rc = EOK;

    msgBuf.type = MSG_TYPE_PERFORMANCE_TEST;
    msgBuf.bufLen = sizeof(unsigned int);
    rc = memcpy_s((void *)&msgBuf.buf, msgBuf.bufLen, (void *)&flag, msgBuf.bufLen);
    if (rc != EOK) {
        return;
    }
    MSGQ_SendMsg(&msgBuf);

    rc = memset_s(&msgBuf, sizeof(MsgBuf), 0, sizeof(MsgBuf));
    if (rc != EOK) {
        return;
    }
    while (MSGQ_ReceiveMsg(&msgBuf) && (timeout > 0)) {
        timeout--;
        osDelay(1);
    }
    if ((timeout <= 0) || msgBuf.type != MSG_TYPE_PERFORMANCE_TEST || (msgBuf.buf[0] != MSG_OK)) {
        DBG_PRINTF("SetPerformanceTest fail!!");
    } else {
        g_iSPerformanceTest = flag;
    }
}

/**
  * @brief get Performance test flag.
  * @param None.
  * @retval Performance test flag.
  */
uint32_t GetPerformanceTest(void)
{
    return g_iSPerformanceTest;
}