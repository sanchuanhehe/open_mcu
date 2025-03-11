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
  * @file    uart.h
  * @author  MCU Driver Team
  * @brief   Header file uart4 for printf
  */

/* Define to prevent recursive inclusion --------------------------------------------*/
#ifndef UART_H
#define UART_H

#include "stm32mp1xx.h"

/*******************************************************************************************************/
/* 配置UART的GPIO管脚 */

#define USART_TX_GPIO_PORT                  GPIOG
#define USART_TX_GPIO_PIN                   GPIO_PIN_11
#define USART_TX_GPIO_AF                    GPIO_AF6_UART4

#define USART_RX_GPIO_PORT                  GPIOB
#define USART_RX_GPIO_PIN                   GPIO_PIN_2
#define USART_RX_GPIO_AF                    GPIO_AF8_UART4

#define USART_UX                            UART4
#define USART_UX_IRQ_N                       UART4_IRQn
#define USART_UX_IRQ_HANDLER                 UART4_IRQHandler

/*******************************************************************************************************/

#define USART_EN_RX     1                       /* USART_EN使能 */

void UART_Init(uint32_t baudrate);
int UART_Out(int ch);

#endif
