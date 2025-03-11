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
  * @file    var_monitor_queue.c
  * @author  MCU Driver Team
  * @brief   Shared queue for storing variable monitoring data.
  */
#include <stdbool.h>
#include <string.h>
#include "securec.h"
#include "debug.h"
#include "var_monitor_queue.h"

/**
  * @brief init a queue.
  * @param q pointer of queue.
  * @retval None.
  */
void QueueInit(VarQueue *q)
{
    q->read = 0;
    q->write = 0;
    q->size = VARQ_SIZE;
    q->totalReadSize = 0;
    q->totalWriteSize = 0;
    q->totalSampleCount = 0;
    q->totalOverflowCount = 0;
    q->totalErrorCount = 0;
    q->exceptionExitFlag = 0;
    q->errorRate = 0;
}

/**
  * @brief destroy a queue.
  * @param q pointer of queue.
  * @retval None.
  */
void QueueDestroy(VarQueue *q)
{
    QueueInit(q);
}

/**
  * @brief whether a queue is full.
  * @param q pointer of queue.
  * @retval Whether the value is full.
  */
static bool QueueFull(VarQueue *q)
{
    return ((q->totalWriteSize - q->totalReadSize) == q->size);
}

/**
  * @brief write buffers to queue.
  * @param q pointer of queue.
  * @param buf pointer of write buffer.
  * @param len length of write buffer.
  * @retval Length of a successful write.
  */
unsigned int WriteQueue(VarQueue *q, const unsigned char *buf, unsigned int len)
{
    unsigned int ret = 0;
    errno_t rc = EOK;
    unsigned long long rest = q->size - q->write;
    unsigned long long space = q->size - (q->totalWriteSize - q->totalReadSize);

    if ((q->totalWriteSize - q->totalReadSize) >= q->size) {
        return ret;
    }
    if (QueueFull(q)) {
        return ret;
    }
    if (space >= len) {
        ret = len;
        if (rest >= len) {
            rc = memcpy_s((void *)(q->buf + q->write), rest, (void *)buf, len);
            if (rc != EOK) {
                return 0;
            }
            q->write = (q->write + len) % q->size;
            q->totalWriteSize += len;
        } else {
            rc = memcpy_s((void *)(q->buf + q->write), rest, (void *)buf, rest);
            if (rc != EOK) {
                return 0;
            }
            q->write = 0;
            rc = memcpy_s((void *)q->buf, q->size, (void *)(buf + rest), len - rest);
            if (rc != EOK) {
                return 0;
            }
            q->write = len - rest;
            q->totalWriteSize += len;
        }
    }
    return ret;
}
