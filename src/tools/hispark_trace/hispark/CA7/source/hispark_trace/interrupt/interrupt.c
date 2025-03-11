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
  * @file    interrupt.c
  * @author  MCU Driver Team
  * @brief   gic interrupt process
  */
#include "stm32mp1xx.h"
#include "util.h"
#include "interrupt.h"

/**
  * Interrupt vector table, supports up to IRQ_MAX interrupts, except for IRQ_VECTOR_CNT internal
  * standard interrupts, which can be configured according to actual conditions.
  */
IRQ_PROC_FUNC g_irqCallbackFunc[MAX_IRQ_n];

/**
  * Default handler for GIC
  */
static void IRQ_DefaultHandler(void)
{
    ;
}

/**
  * Init IRQ table
  */
static void IRQ_TableInit(void)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(g_irqCallbackFunc); ++i) {
        g_irqCallbackFunc[i] = IRQ_DefaultHandler;
    }
}

/**
  * Interrupt entry, called by start.S
  */
void InterruptEntry(unsigned int irqNum)
{
    if (irqNum >= MAX_IRQ_n) {
        return;
    }
    g_irqCallbackFunc[irqNum]();
}

/**
  * IRQ Module Init
  */
void IRQ_Init(void)
{
    __enable_irq();
    GIC_Enable();
    IRQ_TableInit();
}

/**
  * Register callback function for IRQ
  */
unsigned int IRQ_Register(unsigned int irqNum, IRQ_PROC_FUNC func)
{
    if (irqNum >= MAX_IRQ_n) {
        return -1;
    }

    g_irqCallbackFunc[irqNum] = func;
    return 0;
}

/**
  * UnRegister callback function for IRQ
  */
void IRQ_UnRegister(unsigned int irqNum, IRQ_PROC_FUNC func)
{
    if (irqNum >= MAX_IRQ_n) {
        return;
    }
    g_irqCallbackFunc[irqNum] = IRQ_DefaultHandler;
}

/**
  * Disable interrupt
  */
void IRQ_Disable(IRQn_Type irqNum)
{
    GIC_DisableIRQ(irqNum);
    GIC_EndInterrupt(irqNum);
}

/**
  * Enable interrupt
  */
void IRQ_Enable(IRQn_Type irqNum, unsigned int priority, IRQ_PROC_FUNC irqHandler)
{
    GIC_SetPriority(irqNum, priority);
    GIC_EnableIRQ(irqNum);
    IRQ_Register(irqNum, irqHandler);
}
