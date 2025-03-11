 /**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2024. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided
  * that the following conditions are met:
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
  * @file    sample_uart_single_wire_communication.c
  * @author  MCU Driver Team
  * @brief   uart sample module.
  * @details In single-line communication, one line is used to achieve the TX and RX of the UART.
  */
#include "sample_uart_single_hardwire_communication.h"
#define BLOCKING_TIME 2000

/**
 * @brief UART Tx and Rx simultaneously.
 * @param None.
 * @retval None.
 */
void UART_SingleHardWireCommunication(void)
{
    SystemInit();
    HAL_UART_SetLineModeEx(&g_uart, 1); /* 0:Full Duplex; 1:Hardwire half Duplex */
    unsigned char rxStr[20] = {0}; /* rxStr[20], Receive memory address */
    DBG_PRINTF(" UART single hardwire mode Init finish\r\n");
    unsigned int retRx;
    while (1) {
        BASE_FUNC_DELAY_MS(100); /* Wait 100 ms */
        retRx = HAL_UART_ReadBlocking(&g_uart, rxStr, 5, BLOCKING_TIME); /* 5 is data length, 2000 is timeout limit */
        if (retRx == BASE_STATUS_OK) {
            DBG_PRINTF("Receive success: %s \r\n", rxStr);
        } else if (retRx == BASE_STATUS_TIMEOUT) {
            DBG_PRINTF("Receive time out!\r\n");
        } else {
            DBG_PRINTF("Receive verification error!\r\n");
        }
    }
}