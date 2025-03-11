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
  * @file   idh_ram_data_copy.h
  * @author MCU Driver Team
  * @brief  This file provides sample code for OEM help IDH move data from flash to sram.
  */

#ifndef McuMagicTag_IDH_RAM_DATA_COPY_H
#define McuMagicTag_IDH_RAM_DATA_COPY_H

#define LOAD_ARRAY_LENGTH   1

extern unsigned char g_idhSramDataStartAddr[LOAD_ARRAY_LENGTH];
extern unsigned char g_idhSramDataSrcAddr[LOAD_ARRAY_LENGTH];
extern unsigned char g_idhSramDataEndAddr[LOAD_ARRAY_LENGTH];

/**
  * @brief Copy idh data to sram.
  * @param None.
  * @retval None.
  */
static inline void CopyIdhDataToSram(void)
{
    unsigned char *targetAddr;
    unsigned char *srcAddr;

    targetAddr = g_idhSramDataStartAddr; /* Get the address of sram */
    srcAddr = g_idhSramDataSrcAddr;

    while (targetAddr < g_idhSramDataEndAddr) {
        *targetAddr = *srcAddr;
        targetAddr++;
        srcAddr++;
    }
}

#endif /* McuMagicTag_IDH_RAM_DATA_COPY_H */