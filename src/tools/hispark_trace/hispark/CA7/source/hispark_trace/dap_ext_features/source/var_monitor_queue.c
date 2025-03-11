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
  * @file    var_monitor_queue.h
  * @author  MCU Driver Team
  * @brief   Shared queue for storing variable monitoring data.
  */
#include <stdbool.h>
#include <string.h>
#include "securec.h"
#include "var_monitor_queue.h"


/**
  * @brief whether a queue is empty.
  * @param q pointer of queue.
  * @retval Whether the value is empty.
  */
static bool QueueEmpty(VarQueue *q)
{
    return  (q->totalReadSize == q->totalWriteSize);
}

/**
  * @brief read buffers from queue.
  * @param q pointer of queue.
  * @param buf pointer of read buffer.
  * @param len read length.
  * @retval Length of a successful read.
  */
unsigned int ReadQueue(VarQueue *q, unsigned char *buf, unsigned int len)
{
    unsigned long long rest = q->size - q->read;
    unsigned int ret = 0;
    errno_t rc = EOK;
    unsigned long long length = q->totalWriteSize - q->totalReadSize;

    if (q->totalReadSize >= q->totalWriteSize) {
        return ret;
    }
    if (QueueEmpty(q)) {
        return ret;
    }

    if (length >= len) {
        ret = len;
        if (rest >= len) {
            rc = memcpy_s((void *)buf, len, (void *)(q->buf + q->read), len);
            if (rc != EOK) {
                return 0;
            }
            q->read = (q->read + len) % q->size;
            q->totalReadSize += len;
        } else {
            rc = memcpy_s((void *)buf, len, (void *)(q->buf + q->read), rest);
            if (rc != EOK) {
                return 0;
            }
            q->read = 0;
            rc = memcpy_s((void *)(buf + rest), len, (void *)q->buf, len - rest);
            if (rc != EOK) {
                return 0;
            }
            q->read = len - rest;
            q->totalReadSize += len;
        }
    }
    return ret;
}

/**
  * @brief the readable length in the queue.
  * @param q pointer of queue.
  * @retval the readable length in the queue.
  */
unsigned long long QueueReadableLength(VarQueue *q)
{
    return (q->totalWriteSize - q->totalReadSize);
}