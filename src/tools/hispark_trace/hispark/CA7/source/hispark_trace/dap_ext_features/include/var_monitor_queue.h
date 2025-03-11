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
#ifndef VAR_MONITOR_QUEUE_H
#define VAR_MONITOR_QUEUE_H


#define VARQ_SECTION __attribute__((section(".varq_section")))
#define VARQ_SIZE    (0x1E000 - 18 * sizeof(unsigned int))   /* (120k - 72)bytes */

/**
 * VarQueue - queue structure
 * @read: position of read
 * @write: position of write
 * @size: buffer size
 * @totalReadSize: Read the total size.
 * @totalWriteSize: Write the total size.
 */
typedef struct {
    unsigned long long read;
    unsigned long long write;
    unsigned long long size;
    unsigned long long totalReadSize;
    unsigned long long totalWriteSize;
    unsigned long long totalSampleCount;
    unsigned long long totalOverflowCount;
    unsigned long long totalErrorCount;
    unsigned int exceptionExitFlag;
    unsigned int errorRate;
    char buf[VARQ_SIZE];
} volatile VarQueue;

unsigned int ReadQueue(VarQueue *q, unsigned char *buf, unsigned int len);
unsigned long long QueueReadableLength(VarQueue *q);

#endif /* #ifndef VAR_MONITOR_QUEUE_H */