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
  * @file    uart.c
  * @author  MCU Driver Team
  * @brief   file uart4 for printf
  */
#include "uart.h"

int UART_Out(int ch)
{
    /* 串口4输出 */
    while ((USART_UX->ISR & 0x40) == 0) {
        ; // waiting for tx done
    }
    USART_UX->TDR = (uint8_t)ch;
    return ch;
}

void UART_Init(uint32_t baudrate)
{
    /* 串口4初始化 */
    UART_HandleTypeDef uart4Handle;
    uart4Handle.Instance = USART_UX;
    uart4Handle.Init.BaudRate = baudrate;
    /* Use fixed parameters. */
    uart4Handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart4Handle.Init.StopBits = UART_STOPBITS_1;
    uart4Handle.Init.Parity = UART_PARITY_NONE;
    uart4Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart4Handle.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&uart4Handle);
}
