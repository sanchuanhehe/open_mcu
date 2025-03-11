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
  * @file    sample_spi_master_3wires.c
  * @author  MCU Driver Team
  * @brief   Sample for SPI Module Master.
  * @details This example demonstrates using the HAL interface in 3-wire master mode. This example uses blocking mode.
  *          This example must be connected to a three-wire slave device.
  *          This example sends 0x1105, 0x1c20, 0x1183, 0x1285, 0x1240, 0x12c0, and 0x1340 in polling mode.
  *          Print the received data through the serial port.
  */
#include "main.h"
#include "spi.h"
#include "debug.h"
#include "sample_spi_master_3wires.h"

#define USER_TIMEOUT    0x400

#define MASTER_READ_TESE
#define MASTER_WRITE_TESE

#define MASTER_3WIRES_MODE_SET_CH0 0x01
#define MASTER_3WIRES_MODE_SET_CH1 0x02
#define MASTER_3WIRES_MODE_SET_CH2 0x03
#define MASTER_3WIRES_MODE_SET_CH3 0x04
#define MASTER_3WIRES_MODE_SET_CH4 0x05
#define MASTER_3WIRES_MODE_SET_CH5 0x06
#define MASTER_3WIRES_MODE_SET_CH6 0x07

#define MAX_TIMEOUT_VAL     5000

/**
  * @brief Spi master three wires sample processing.
  * @param None.
  * @retval None.
  */
void MasterThreeWiresTestSampleProcessing(void)
{
    unsigned short tempWdata[] = {
        MASTER_3WIRES_MODE_SET_CH0,
        MASTER_3WIRES_MODE_SET_CH1,
        MASTER_3WIRES_MODE_SET_CH2,
        MASTER_3WIRES_MODE_SET_CH3,
        MASTER_3WIRES_MODE_SET_CH4,
        MASTER_3WIRES_MODE_SET_CH5,
        MASTER_3WIRES_MODE_SET_CH6,
        MASTER_3WIRES_MODE_SET_CH2,
        MASTER_3WIRES_MODE_SET_CH3,
        MASTER_3WIRES_MODE_SET_CH4
    };
    unsigned short tempRdata[10] = {0};
    SystemInit();
    while (1) {
#ifdef MASTER_WRITE_TESE
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("Master 3wires tempWdata[%d] = 0x%x \r\n", i, tempWdata[i]);
            BASE_FUNC_DELAY_MS(10); /* Delay: 10 ms */
        }
        HAL_SPI_WriteBlocking(&g_spiSampleHandle, (unsigned char *)tempWdata, sizeof(tempWdata), MAX_TIMEOUT_VAL);
        BASE_FUNC_DELAY_MS(300); /* Delay: 300 ms */
#endif
#ifdef MASTER_READ_TESE
        HAL_SPI_ReadBlocking(&g_spiSampleHandle, (unsigned char *)tempRdata, sizeof(tempRdata), MAX_TIMEOUT_VAL);
        for (int i = 0; i < 10; i++) { /* Print the test data for 10 times. */
            DBG_PRINTF("Master 3wires tempRdata[%d] = 0x%x \r\n", i, tempRdata[i]);
            BASE_FUNC_DELAY_MS(10); /* Delay: 10 ms */
        }
#endif
    }
}
