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
  * @file    msg_queue.c
  * @author  MCU Driver Team
  * @brief   Message Queue Process between Cortex-A and Cortex-M
  */
#include <stdlib.h>
#include <string.h>
#include "securec.h"
#include "debug.h"
#include "msg_queue.h"

extern int g_msgQStart;  /* defined by linker script */
MsgQCtrl *g_msgQCtrl = (MsgQCtrl *)&g_msgQStart;

/**
  * @brief Message Queue Init
  * @retval None
  */
void MSGQ_Init(void)
{
#ifdef ROLE_MASTER
    volatile MsgQ *p;
    for (unsigned int i = 0; i < sizeof(g_msgQCtrl->msgQ) / sizeof(g_msgQCtrl->msgQ[0]); ++i) {
        p = &g_msgQCtrl->msgQ[i];
        p->num = sizeof(g_msgQCtrl->msgQ[i].msg) / sizeof(g_msgQCtrl->msgQ[i].msg[0]);
        p->send = 0;
        p->rcv = 0;
    }
    g_msgQCtrl->initFlg = 1;
#else
    while (g_msgQCtrl->initFlg == 0) {
        ; /* Wait Master Init done */
    }
#endif
}

/**
  * @brief Check whether the message queue is full.
  * @retval true： full,  false: not full
  */
static inline bool MSGQ_TxIsFull(void)
{
    volatile MsgQ *p = &g_msgQCtrl->msgQ[MSG_QUEUE_TX];
    return (((p->send + 1) % p->num) == p->rcv);
}

/**
  * @brief Check whether the message queue is empty.
  * @retval true： empty,  false: not empty
  */
bool MSGQ_RxIsEmpty(void)
{
    volatile MsgQ *p = &g_msgQCtrl->msgQ[MSG_QUEUE_RX];
    return (p->rcv == p->send);
}

/**
  * @brief Send Message to queue
  * @retval result
  */
unsigned int MSGQ_SendMsg(MsgBuf *msg)
{
    errno_t rc = EOK;
    volatile MsgQ *p = &g_msgQCtrl->msgQ[MSG_QUEUE_TX];
    if (msg == NULL) {
        return QUEUE_ERROR;
    }
    if (MSGQ_TxIsFull() == true) {
        return QUEUE_FULL;
    }
    rc = memcpy_s((void *)&p->msg[p->send], sizeof(*msg), (void *)msg, sizeof(*msg));
    if (rc != EOK) {
        return QUEUE_ERROR;
    }
    p->send = (p->send + 1) % p->num; /* update send ptr */
    return QUEUE_OK;
}

/**
  * @brief Receive Message from queue
  * @retval result
  */
unsigned int MSGQ_ReceiveMsg(MsgBuf *msg)
{
    errno_t rc = EOK;
    volatile MsgQ *p = &g_msgQCtrl->msgQ[MSG_QUEUE_RX];
    if (msg == NULL) {
        return QUEUE_ERROR;
    }
    if (MSGQ_RxIsEmpty() == true) {
        return QUEUE_EMPTY;
    }
    rc = memcpy_s((void *)msg, sizeof(*msg), (void *)&p->msg[p->rcv], sizeof(*msg));
    if (rc != EOK) {
        return QUEUE_ERROR;
    }
    p->rcv = (p->rcv + 1) % p->num;  /* update receive ptr */
    return QUEUE_OK;
}