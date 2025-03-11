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
  * @file    msg_queue.h
  * @author  MCU Driver Team
  * @brief   Message Queue Process between Cortex-A and Cortex-M
  */

#ifndef MSG_QUEUE_H
#define MSG_QUEUE_H

#include <stdbool.h>
#include <stddef.h>

/* A核与M核共享的消息队列, 由A核初始化 */
#ifdef ROLE_MASTER
#define MSGQ_SECTION __attribute__((section(".msgq_section")))
#endif

#define Q_TRUE    1
#define Q_FALSE   0

#define WRITE_Q_START_ADDR 0x10050000
#define WRITE_Q_SIZE       0x8000
#define READ_Q_START_ADDR  0x10058000
#define READ_Q_SIZE 0x8000

#define MSG_SIZE 3072
#define MSG_NUM 2

typedef enum {
    MSG_ERROR,
    MSG_OK
} MSG_ACK;

typedef enum {
    MSG_TYPE_DEBUG,
    MSG_TYPE_SAMPLING,
    MSG_TYPE_WRITEVAR,
    MSG_TYPE_CLK_CFG,
    MSG_TYPE_PERFORMANCE_TEST,
    MSG_TYPE_DEBUG_PORT_SET,
    MSG_TYPE_PAUSESAMPLING,
} MSG_TYPE;

typedef enum {
#ifdef ROLE_MASTER
    MSG_QUEUE_TX,
    MSG_QUEUE_RX,
#else
    MSG_QUEUE_RX,
    MSG_QUEUE_TX,
#endif
    MSG_QUEUE_NUN
} MSG_QUEUE_EN;

typedef struct {
    unsigned int type;
    unsigned int bufLen;
    unsigned char buf[MSG_SIZE - (sizeof(unsigned int) + sizeof(int))];
} MsgBuf;

typedef enum {
    QUEUE_ERROR = -1,
    QUEUE_OK = 0,
    QUEUE_FULL = 1,
    QUEUE_EMPTY = 2,
} QUEUE_RETVAL;

typedef struct {
    size_t num;
    unsigned int send;
    unsigned int rcv;
    MsgBuf msg[MSG_NUM];
} MsgQ;

typedef struct {
    unsigned int initFlg;
    MsgQ msgQ[MSG_QUEUE_NUN];
} volatile MsgQCtrl;

void MSGQ_Init(void);
unsigned int MSGQ_SendMsg(MsgBuf *msg);
unsigned int MSGQ_ReceiveMsg(MsgBuf *msg);
bool MSGQ_RxIsEmpty(void);

#endif /* MSG_QUEUE_H */