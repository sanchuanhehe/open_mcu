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
  * @brief   Processes variable monitoring data. The functions are as follows:
  *           + Variable monitoring.
  *           + Continuous Address Variable Monitoring Acceleration.
  *           + Variable monitoring performance test.
  *           + modifying variable values...
  */
#ifndef VAR_MONITOR_PROCESS_H
#define VAR_MONITOR_PROCESS_H

#include <stdint.h>

#define MAX_VAR_NUM                     300
#define DISCONNECT_SESSION_TIMEOUT      5000000
#define FLASH_BOUNDARY                  0x400

#define BUS_CHANNEL_AHB                 0x1000000

#define ACCESS_MEMORY_LEN_JTAG          0x0
#define ACCESS_MEMORY_LEN_SWD           0x1

#define ACCESS_MEMORY_ADDR_JTAG         0x4
#define ACCESS_MEMORY_ADDR_SWD          0x5

#define WRITE_MEMORY_JTAG               0xC
#define WRITE_MEMORY_SWD                0xD

#define READ_MEMORY_PRE_JTAG            0xE
#define READ_MEMORY_PRE_SWD             0xF

#define READ_MEMORY_END                 0xE

#define ACCESS_DISCONTINUOUS_ONE_BYTE   0x0
#define ACCESS_DISCONTINUOUS_TWO_BYTE   0x1
#define ACCESS_DISCONTINUOUS_FOUR_BYTE  0x2
#define ACCESS_CONTINUOUS_FOUR_BYTE     0x22

#define ALIGN_FOUR_BYTE                 0x3

#define LOW_32_BIT                      0x00000000ffffffff
#define HIGH_32_BIT                     0xffffffff00000000


#define VARTYPE_CHAR_LEN                sizeof(uint8_t)
#define VARTYPE_SHORT_LEN               sizeof(uint16_t)
#define VARTYPE_INT_LEN                 sizeof(uint32_t)
#define VARTYPE_LONG_LEN                sizeof(uint64_t)

#define ADDR_INTERVAL_LEN_ONE           1
#define ADDR_INTERVAL_LEN_TWO           2
#define ADDR_INTERVAL_LEN_THREE         3
#define ADDR_INTERVAL_LEN_FOUR          4

#define PARSING_CONTINUOUS              0
#define PARSING_ENDING                  1

#define RET_SUCCESS                      0
#define MALLOC_FAIL                     (-1)
#define RET_FAIL                        (-2)


typedef enum {
    TYPE_CHAR  = 0x1,
    TYPE_SHORT = 0x2,
    TYPE_INT   = 0x4,
    TYPE_LONG  = 0x8,
    TYPE_MIX   = 0xff
} VAR_TYPE;

typedef struct {
    unsigned int address;
    unsigned int len;
    unsigned long long value;
} DapWriteVar;

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

/* Used to record the address sequence before sorting. */
typedef struct {
    unsigned int map;
    unsigned char sampleValue[8];
} DapAddressMap;

typedef struct {
    unsigned int position[MAX_VAR_NUM];
    unsigned int len[MAX_VAR_NUM];
} DapPositionInfo;

/* Used to group incremental addresses */
typedef struct {
    unsigned int startAddress;
    unsigned int length;
    VAR_TYPE type;
    unsigned int mixPosition;
} DapAddressGroup;

typedef struct {
    unsigned int isAddressMapFlag;
    unsigned int isAddressIncrementFlag;
    unsigned int count;
    DapAddressGroup *addressGroup;
    unsigned int *addressMap;       /* Save the position of the sorted variable. */
} DapAddressGroupList;

extern DapVarMonitor g_dapValMonitor;
extern DapWriteVar g_dapWriteVar;
extern uint32_t g_isPerformanceTestFlag;
extern uint32_t g_pauseSampleState;
extern uint8_t JTAG_RESET(void);

uint8_t WriteOneVal(uint32_t addr, uint32_t length, uint64_t value);
void ReadMemoryToQueue(void);
int StartSampling(void);
void StopSampling(void);
void PauseSampling(uint32_t state);
void FreeMemory(void);

#endif /* #ifndef VAR_MONITOR_PROCESS_H */
