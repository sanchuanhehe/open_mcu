/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      diagnose_mcu_connect.h
  * @author    MCU Driver Team
  * @brief     This file contains the handle and function declaration for connect diagnose.
  */

#ifndef DIAGNOSE_MCU_CONNECT_H
#define DIAGNOSE_MCU_CONNECT_H

#include "function_safety_common.h"
#ifdef I2C
#include "i2c.h"
#endif
#ifdef UART0
#include "uart.h"
#endif
#ifdef SPI
#include "spi.h"
#endif
#ifdef CAN
#include "can.h"
#endif

#define MODULE_I2C          0x01
#define MODULE_UART         0x02
#define MODULE_SPI          0x03
#define MODULE_CAN          0x04

#define FEATURE_CONNECT_DATA_CORRECT    0x01

#define FAULT_CONNECT_DATA_UNCORRECT    0x01

typedef struct {
#ifdef I2C
    I2C_Handle*   i2cHandle;
#endif
#ifdef UART0
    UART_Handle*  uartHandle;
#endif
#ifdef SPI
    SPI_Handle*   spiHandle;
#endif
#ifdef CAN
    CAN_Handle*   canHandle;
#endif
} CONNECT_DiagnoseHandle;


#endif