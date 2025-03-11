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
  * @file    var_monitor_process.h
  * @author  MCU Driver Team
  * @brief   Variable monitoring protocol, The functions are as follows:
  *           + starting variable monitoring.
  *           + stopping variable monitoring.
  *           + suspending variable monitoring.
  *           + uploading variable monitoring data.
  *           + modifying variable values...
  */
#ifndef VAR_MONITOR_PROCESS_H
#define VAR_MONITOR_PROCESS_H
#include "DAP.h"
#include "DAP_config.h"


#define DAP_TRANSFER_MAX_PACKET_SIZE   (DAP_PACKET_SIZE - 8)
#define MAX_VAR_NUM                     60
#define DISCONNECT_SESSION_TIMEOUT      5000000
#define VAR_LIST_PROTOCOL_POS           8

#define MAGICNUMBE_BYTES                sizeof(unsigned int)
#define SAMPLECOUNT_BYTES               sizeof(unsigned int)

#define SHIFTS_8_BIT                    8
#define SHIFTS_16_BIT                   16
#define SHIFTS_24_BIT                   24

#define MASK_POS_0                      0x000000ff
#define MASK_POS_8                      0x0000ff00
#define MASK_POS_16                     0x00ff0000
#define MASK_POS_24                     0xff000000

#define MASK_LOW_32                     0xffffffff

#define MAGIC_NUMBER                    0x44332211

#define MAX_WRITE_BYTE_VALUE            8

typedef struct {
    unsigned int varAddress;
    unsigned int varNumBytes;
} DapVarList;

typedef struct {
    unsigned int isSample;
    unsigned int periodUs;
    unsigned int varNum;
    DapVarList *varList;
} DapVarMonitor;

typedef struct {
    unsigned int address;
    unsigned int len;
    unsigned char value[MAX_WRITE_BYTE_VALUE];
} DapWriteVar;

extern uint32_t GetTickDelta(uint32_t curTicks, uint32_t preTicks);
extern uint32_t HAL_GetTickUs(void);

uint32_t DAP_VarMonitor(const uint8_t *request, uint8_t *response);
uint32_t DAP_ReadVar(const uint8_t *request, uint8_t *response);
uint32_t DAP_StopVarMonitor(uint8_t *response);
uint32_t DAP_PauseVarMonitor(const uint8_t *request, uint8_t *response);
uint32_t DAP_WriteVar(const uint8_t *request, uint8_t *response);
void SetPerformanceTest(unsigned int flag);
uint32_t GetPerformanceTest(void);
void DAP_StatusDetection(void);
void UpdateDapStatistics(void);
#endif /* #ifndef VAR_MONITOR_PROCESS_H */