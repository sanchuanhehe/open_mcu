/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file    sample_dma_mem_to_per.c
  * @author  MCU Driver Team
  * @brief   dma sample module, memory-to-peripheral transfer.
  * @details This file provides sample code for users to help use
  *          the data transfer function of the dma.
  */
#include "sample_dma_mem_to_per.h"

#define NUM 10

static unsigned char g_str1[NUM] = "123456789";

/**
  * @brief User-defined callback function for completing the transfer of memory to the peripheral.
  * @param handle callback handle.
  * @retval None.
  */
static void DMA_MemToPeriFinish(void *handle)
{
    DBG_PRINTF("Interrupt Finish!\r\n");
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief DMA sample code for the transfer of memory to the peripheral.
  * @param None.
  * @retval None.
  */
int DMA_MemoryToPeriphIT(void)
{
    SystemInit();
    DBG_PRINTF("MemoryToPeriph Begin: \r\n");
    DBG_PRINTF("src_memory g_str1: %s\r\n", g_str1);
    unsigned int channel = 3;  /* select transfer channel 3 */

    HAL_DMA_RegisterCallback(&g_dmac, DMA_CHANNEL_FINISH, channel, DMA_MemToPeriFinish);

    unsigned int ret;
    ret = HAL_DMA_StartIT(&g_dmac, (uintptr_t)(void *)g_str1, (uintptr_t)(void *)&(g_uart0.baseAddress->UART_DR),
                          6, channel);      /* The transmission length is defined as 6 */
    if (ret == BASE_STATUS_ERROR) {
        DBG_PRINTF("HAL_DMA_StartIT: BASE_STATUS_ERROR\r\n");
    } else {
        g_uart0.baseAddress->UART_DMACR.BIT.txdmae = BASE_CFG_ENABLE;
        DBG_PRINTF("HAL_DMA_StartIT: BASE_STATUS_OK\r\n");
    }
    DBG_PRINTF("End:\r\n");
    return 0;
}