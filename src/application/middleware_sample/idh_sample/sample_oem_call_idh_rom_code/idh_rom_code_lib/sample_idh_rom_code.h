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
  * @file   sample_idh_rom_code.h
  * @author MCU Driver Team
  * @brief  This file provides sample code for IDH provide api for oem.
  */

#ifndef McuMagicTag_SAMPLE_IDH_ROM_CODE_H
#define McuMagicTag_SAMPLE_IDH_ROM_CODE_H

#include <stdint.h>
#include "flash.h"
#include "timer.h"

#define FIXED_OPERATOR_SRAM __attribute__((section(".fixoperator_sram")))
#define RAM_CODE __attribute__((section(".text.sram")))
typedef void (* IRQ_PROC_FUNC)(void *arg);

/* Definition of the interrupt HAL function pointer. */
typedef unsigned int (*IRQ_ENABLEN)(unsigned int irqNum);
typedef unsigned int (*IRQ_REGISTER)(unsigned int irqNum, IRQ_PROC_FUNC func, void *arg);
typedef unsigned int (*IRQ_SETPRIORITY)(unsigned int irqNum, unsigned int priority);

/* Definition of the flash HAL function pointer. */
typedef BASE_StatusType (*FLASH_INIT)(FLASH_Handle *handle);
typedef BASE_StatusType (*FLASH_DEINIT)(FLASH_Handle *handle);
typedef BASE_StatusType (*FLASH_READ)(FLASH_Handle *handle, unsigned int srcAddr, unsigned int readLen,
                                      unsigned char *dataBuff, unsigned int buffLen);
typedef BASE_StatusType (*FLASH_WRITE_BLOCKING)(FLASH_Handle *handle, unsigned int srcAddr, unsigned int destAddr,
                                                unsigned int srcLen);
typedef BASE_StatusType (*FLASH_ERASE_BLOCKING)(FLASH_Handle *handle, FLASH_EraseMode eraseMode,
                                                FLASH_SectorAddr startAddr, unsigned int eraseNum);

/* Definition of the Timer HAL function pointer. */
typedef BASE_StatusType (*TIMER_INIT)(TIMER_Handle *handle);
typedef void (*TIMER_DEINIT)(TIMER_Handle *handle);
typedef void (*TIMER_START)(TIMER_Handle *handle);
typedef void (*TIMER_STOP)(TIMER_Handle *handle);
typedef void (*TIMER_IRQHANDLER)(void *handle);
typedef BASE_StatusType (*TIMER_CALLBACK)(TIMER_Handle *handle, TIMER_InterruptType typeID,
                                          TIMER_CallBackFunc callBackFunc);

/* Macro definitions ---------------------------------------------------------*/
typedef enum {
    CALACULATE_ADD = 0,
    CALACULATE_SUB = 1,
    CALACULATE_MUL = 2,
    CALACULATE_DIV = 3,
} CalculateType;

typedef struct {
    IRQ_ENABLEN irqEnableCb;
    IRQ_REGISTER irqRegisterCb;
    IRQ_SETPRIORITY irqSetpriorityCb;
} FIXED_OPERATOR_IrqCallback;

typedef struct {
    FLASH_INIT flashInitCb;
    FLASH_DEINIT flashDeinitCb;
    FLASH_READ flashReadCb;
    FLASH_WRITE_BLOCKING flashWriteBlockingCb;
    FLASH_ERASE_BLOCKING flashEraseBlockingCb;
} FIXED_OPERATOR_FlashCallback;

typedef struct {
    TIMER_INIT timerInitCb;
    TIMER_DEINIT timerDeinitCb;
    TIMER_START timerStartCb;
    TIMER_STOP timerStopCb;
    TIMER_IRQHANDLER timerIrqHandlerCb;
    TIMER_CALLBACK timerCallbackCb;
} FIXED_OPERATOR_TimerCallback;

/* Register API structure -----------------------------------------------------------------*/
void REGISTER_IRQ_CALLBACK(FIXED_OPERATOR_IrqCallback *callback);
void REGISTER_FLASH_CALLBACK(FIXED_OPERATOR_FlashCallback *callback);
void REGISTER_TIMER_CALLBACK(FIXED_OPERATOR_TimerCallback *callback);

/* Fixed operator API structure -----------------------------------------------------------------*/
uint32_t FIXED_OPERATOR_FlashInit(FLASH_Handle *handle);
uint32_t FIXED_OPERATOR_FlashDeInit(FLASH_Handle *handle);

int32_t FIXED_OPERATOR_TimerInit(TIMER_Handle *handle, uint32_t irqNum, uint32_t priority, TIMER_InterruptType typeID,
                                 TIMER_CallBackFunc userCallback);
int32_t FIXED_OPERATOR_TimerDeInit(TIMER_Handle *handle);

int32_t FIXED_OPERATOR_Calculate(int32_t data1, int32_t data2, CalculateType type);
int32_t RAM_CODE FIXED_OPERATOR_CalculateSRAM(int32_t data1, int32_t data2, CalculateType type);
int32_t FIXED_OPERATOR_StoreStart(TIMER_Handle *handle);
int32_t FIXED_OPERATOR_StoreStop(TIMER_Handle *handle);
int32_t FIXED_OPERATOR_Store(FLASH_Handle *handle, uint32_t data, uint32_t optPageAddr, uint32_t len);
int32_t RAM_CODE FIXED_OPERATOR_StoreSRAM(FLASH_Handle *handle, uint32_t data, uint32_t optPageAddr, uint32_t len);

#endif /* McuMagicTag_SAMPLE_IDH_ROM_CODE_H */