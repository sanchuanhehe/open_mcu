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
  * @file      main.c
  * @author    MCU Driver Team
  * @brief     Main program body.
  */

#include "typedefs.h"
#include "feature.h"
#include "sample_smbus_slave_blocking.h"
#include "main.h"
/* USER CODE BEGIN 0 */
/* USER CODE 区域内代码不会被覆盖，区域外会被生成的默认代码覆盖（其余USER CODE 区域同理） */
/* 建议用户放置头文件 */
/* USER CODE END 0 */
UART_Handle g_uart0;
SMBUS_Handle g_smbus;
/* USER CODE BEGIN 1 */
/* 建议用户定义全局变量、结构体、宏定义或函数声明等 */
/* USER CODE END 1 */

int main(void)
{
    /* USER CODE BEGIN 2 */
    /* 建议用户放置初始化代码或启动代码等 */
    /* USER CODE END 2 */
    SMBusSlaveBlockingProcessing();
    /* USER CODE BEGIN 3 */

    /* 建议用户放置初始配置代码 */
    /* USER CODE END 3 */
    while (1) {
        /* USER CODE BEGIN 4 */
        /* 建议用户放置周期性执行代码 */
        /* USER CODE END 4 */
    }
    /* USER CODE BEGIN 5 */
    /* 建议用户放置代码流程 */
    /* USER CODE END 5 */
    return BASE_STATUS_OK;
}

/* USER CODE BEGIN 6 */
/* 建议用户放置自定义函数 */
/* USER CODE END 6 */