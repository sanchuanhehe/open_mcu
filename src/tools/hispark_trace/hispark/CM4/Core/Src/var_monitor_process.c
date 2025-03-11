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
  * @file    var_monitor_process.c
  * @author  MCU Driver Team
  * @brief   Processes variable monitoring data. The functions are as follows:
  *           + Variable monitoring.
  *           + Continuous Address Variable Monitoring Acceleration.
  *           + Variable monitoring performance test.
  *           + modifying variable values...
  */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "securec.h"
#include "DAP.h"
#include "var_monitor_queue.h"
#include "swd_jtag_config.h"
#include "debug.h"
#include "timer.h"
#include "var_monitor_process.h"


#define MAX_SWD_RETRY                   100
#define ERROR_DETECTION_INTERVAL_US     30000UL
#define ERROR_RATE_1                    1
#define ERROR_MAX_ALLOWED               5

// SWD register access
#define SWD_REG_AP        (1)
#define SWD_REG_DP        (0)
#define SWD_REG_R         (1<<1)
#define SWD_REG_W         (0<<1)
#define SWD_REG_ADR(a)    ((a) & 0x0c)

#define DP_CTRL_STAT      0x04U   // Control & Status

// Debug Control and Status definitions
#define ORUNDETECT     0x00000001  // Overrun Detect
#define STICKYORUN     0x00000002  // Sticky Overrun
#define TRNMODE        0x0000000C  // Transfer Mode Mask
#define TRNNORMAL      0x00000000  // Transfer Mode: Normal
#define TRNVERIFY      0x00000004  // Transfer Mode: Pushed Verify
#define TRNCOMPARE     0x00000008  // Transfer Mode: Pushed Compare
#define STICKYCMP      0x00000010  // Sticky Compare
#define STICKYERR      0x00000020  // Sticky Error
#define READOK         0x00000040  // Read OK (SW Only)
#define WDATAERR       0x00000080  // Write Data Error (SW Only)
#define MASKLANE       0x00000F00  // Mask Lane Mask
#define MASKLANE0      0x00000100  // Mask Lane 0
#define MASKLANE1      0x00000200  // Mask Lane 1
#define MASKLANE2      0x00000400  // Mask Lane 2
#define MASKLANE3      0x00000800  // Mask Lane 3
#define TRNCNT         0x001FF000  // Transaction Counter Mask
#define CDBGRSTREQ     0x04000000  // Debug Reset Request
#define CDBGRSTACK     0x08000000  // Debug Reset Acknowledge
#define CDBGPWRUPREQ   0x10000000  // Debug Power-up Request
#define CDBGPWRUPACK   0x20000000  // Debug Power-up Acknowledge
#define CSYSPWRUPREQ   0x40000000  // System Power-up Request
#define CSYSPWRUPACK   0x80000000  // System Power-up Acknowledge

// Abort Register definitions
#define DAPABORT       0x00000001  // DAP Abort
#define STKCMPCLR      0x00000002  // Clear STICKYCMP Flag (SW Only)
#define STKERRCLR      0x00000004  // Clear STICKYERR Flag (SW Only)
#define WDERRCLR       0x00000008  // Clear WDATAERR Flag (SW Only)
#define ORUNERRCLR     0x00000010  // Clear STICKYORUN Flag (SW Only)

/* Save the initialization information of the variable list. */
DapVarMonitor g_dapValMonitor;
/* Save the modified variable information. */
DapWriteVar g_dapWriteVar;
/* flag of enabling the performance test. */
uint32_t g_isPerformanceTestFlag = 0;
/* Sampling pause status. */
uint32_t g_pauseSampleState;
extern char g_varQStart;
/* Shared memory queue for storing sampled data */
static VarQueue *g_varMonitorQueue = (VarQueue *)&g_varQStart;
/* Save the grouping information of the variable list optimization sampling. */
static DapAddressGroupList g_dapAddressGroupList;
/* Stores the position and length information of the mixed variable list. */
static DapPositionInfo *g_mixPositionInfo;
/* Virtual variables used in the performance test. */
static unsigned long long int *g_performanceTestVar;
/* Record the length of the variable list in a frame. */
static uint32_t g_varMonitorFrameSize = 0;
/* Relative Time. */
static uint32_t g_preTicks = 0;
/* time stamp. */
static uint32_t g_timeStamp = 0;
/* Flag for enabling variable sampling. */
static uint32_t g_isReadMemoryFlag = 0;
/* Flag for enabling the configuration of the sampling sequence. */
static uint32_t g_isValMonitorSetFlag = 0;
/* Timer for calculating error rate */
static uint32_t g_errorDetectionTimer = 0;
/* Total number of samples used to calculate the error rate. */
static uint64_t g_errorDetectionTotalCounter = 0;
/* Number of sampling errors used to calculate the error rate. */
static uint64_t g_errorDetectionErrorCounter = 0;
/* Alarm value of the error rate. */
static uint32_t g_errorRate = ERROR_RATE_1;
/* Variable counter for parsing optimized sampled data. */
static uint32_t g_parsingVarCount = 0;

/**
  * @brief SWD/JTAG Interface Transfer Function.
  * @param request Indicates the address of the DP/AP register.
  * @param data Data to be written or read back.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SWDJTAG_Transfer(uint32_t request, uint32_t *data)
{
    if (g_DAP_Data.debug_port == DAP_PORT_SWD) {
        return SWD_Transfer(request, data);
    } else {
        return JTAG_Transfer(request, data);
    }
}

static void Int2Array(uint8_t *res, uint32_t data, uint8_t len)
{
    uint8_t i = 0;

    for (i = 0; i < len; i++) {
        res[i] = (data >> (8 * i)) & 0xff; /* 8-bit processing */
    }
}

static uint8_t SwdWriteDP(uint8_t adr, uint32_t val)
{
    uint32_t req;
    uint8_t data[4];
    uint8_t ack;

    req = SWD_REG_DP | SWD_REG_W | SWD_REG_ADR(adr);
    Int2Array(data, val, 4); /* 4 byte */
    ack = SWDJTAG_Transfer(req, (uint32_t *)data);
    return ack;
}

static uint8_t SwdReadDP(uint8_t adr, uint32_t *val)
{
    uint32_t tmp_in;
    uint8_t tmp_out[4];
    uint8_t ack;
    uint32_t tmp;
    tmp_in = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(adr);
    ack = SWDJTAG_Transfer(tmp_in, (uint32_t *)tmp_out);
    *val = 0;
    tmp = tmp_out[3]; /* Byte 3 */
    *val |= (tmp << 24); /* Moved leftwards by 24 bits and placed in the most significant byte. */
    tmp = tmp_out[2]; /* Byte 2 */
    *val |= (tmp << 16); /* Moved leftwards by 16 bits */
    tmp = tmp_out[1]; /* Byte 1 */
    *val |= (tmp << 8); /* Moved leftwards by 8 bits */
    tmp = tmp_out[0];
    *val |= (tmp << 0);

    return ack;
}

/**
  * @brief SWD/JTAG interface reset function.
  * @retval None.
  */
static void SwdJtagReset(void)
{
    uint8_t tmp_in[8];
    uint8_t i = 0;
    /* Transmitted 8-byte 0xFF */
    for (i = 0; i < 8; i++) {
        tmp_in[i] = 0xff;
    }
    /* Transmit 51 timings */
    SWJ_Sequence(51, tmp_in);
    return;
}

static void SwitchSwdJtag(uint16_t val)
{
    uint8_t tmp_in[2];
    tmp_in[0] = val & 0xff;
    tmp_in[1] = (val >> 8) & 0xff; /* Shift left by 8 bits */
    SWJ_Sequence(16, tmp_in); /* Transmit 16 timings */
    return;
}

static void SwdPowerSet(void)
{
    uint32_t timeout = 200;
    uint32_t tmp = 0;

    SwdWriteDP(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ);
    for (int i = 0; i < timeout; i++) {
        SwdReadDP(DP_CTRL_STAT, &tmp);
        if ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK)) {
            // Break from loop if powerup is complete
            break;
        }
    }
    SwdWriteDP(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE);
}

/**
  * @brief SWD/JTAG interface read idcode function.
  * @param id Idcode value.
  * @retval None.
  */
static void SwdJtagReadIdcode(uint32_t *id)
{
    uint32_t request;
    uint8_t tmp_in[1];
    uint8_t tmp_out[4];
    tmp_in[0] = 0x00;
    SWJ_Sequence(8, tmp_in); /* Transmit 8 timings */

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        *id = JTAG_ReadIDCode();
    } else {
        request = 0x2;
        SWDJTAG_Transfer(request, id);
        /* The 3 byte is shifted left by 24 bits, the 2 byte is shifted left by 16 bits,
           and the 1byte is shifted left by 8 bits. */
        *id = (tmp_out[3] << 24) | (tmp_out[2] << 16) | (tmp_out[1] << 8) | tmp_out[0];
    }
    return;
}

/**
  * @brief SWD/JTAG Interface Transfer Retry Function.
  * @param request Indicates the address of the DP/AP register.
  * @param data Data to be written or read back.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR.
  */
static uint8_t SWDJTAG_TransferRetry(uint32_t request, uint32_t *data)
{
    uint8_t ack;

    for (uint32_t i = 0; i < MAX_SWD_RETRY; i++) {
        ack = SWDJTAG_Transfer(request, data);
        if (ack != DAP_TRANSFER_WAIT) {
            break;
        }
    }
    return ack;
}

/**
  * @brief SWD/JTAG Interface clean error Function.
  * @retval None.
  */
static void SwdJtagCleanError(void)
{
    uint32_t request;
    uint32_t data;

    data = STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR;
    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        JTAG_WriteAbort(data);
        return;
    }

    request = DP_ABORT;
    SWDJTAG_TransferRetry(request, &data);
    return;
}

/**
  * @brief Switching the AHB bus channel.
  * @param None.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR.
  */
static uint8_t SwitchBusChannel(void)
{
    uint32_t request;
    uint32_t data;

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        /* WriteDP 2 */
        uint32_t ir = JTAG_DPACC;
        JTAG_IR(ir);
    }
    request = DP_SELECT;
    data = BUS_CHANNEL_AHB;

    return SWDJTAG_TransferRetry(request, &data);
}

/**
  * @brief Sets the length of the memory to be accessed.
  * @param accessLength length of the memory to be accessed.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SetAccessMemoryLength(uint32_t accessLength)
{
    uint32_t request;
    uint32_t data;

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        /* WriteAP 0 */
        uint32_t ir = JTAG_APACC;
        JTAG_IR(ir);
        request = ACCESS_MEMORY_LEN_JTAG;
    } else {
        request = ACCESS_MEMORY_LEN_SWD;
    }
    data = accessLength;

    return SWDJTAG_TransferRetry(request, &data);
}

/**
  * @brief Sets the address of the memory to be accessed.
  * @param addr address of the memory to be accessed.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SetAccessMemoryAddr(uint32_t addr)
{
    uint32_t request;
    uint32_t data;

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        /* WriteAP 1 */
        uint32_t ir = JTAG_APACC;
        JTAG_IR(ir);
        request = ACCESS_MEMORY_ADDR_JTAG;
    } else {
        request = ACCESS_MEMORY_ADDR_SWD;
    }
    data = addr;

    return SWDJTAG_TransferRetry(request, &data);
}

/**
  * @brief Modifying the Memory.
  * @param value Modified memory value.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t WriteMemory(uint32_t value)
{
    uint32_t request;
    uint32_t data;

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        /* WriteAP 3 */
        uint32_t ir = JTAG_APACC;
        JTAG_IR(ir);
        request = WRITE_MEMORY_JTAG;
    } else {
        request = WRITE_MEMORY_SWD;
    }
    data = value;

    return SWDJTAG_TransferRetry(request, &data);
}

/**
  * @brief Preparation for reading the memory.
  * @param None.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t ReadMemoryPrepare(void)
{
    uint32_t request;

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        /* Read AP3 */
        uint32_t ir = JTAG_APACC;
        JTAG_IR(ir);
        request = READ_MEMORY_PRE_JTAG;
    } else {
        request = READ_MEMORY_PRE_SWD;
    }

    return SWDJTAG_TransferRetry(request, NULL);
}

/**
  * @brief Read the memory continuously.
  * @param outData Read back memory data.
  * @param len  Data len.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR.
  */
static uint8_t ReadMemoryContinuous(uint8_t *outData, uint32_t len)
{
    uint32_t request;

    if (len > sizeof(uint32_t)) {  /* Length verification */
        return DAP_TRANSFER_FAULT;
    }

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        /* Read AP3 */
        uint32_t ir = JTAG_APACC;
        JTAG_IR(ir);
        request = READ_MEMORY_PRE_JTAG;  /* Jtag request */
    } else {
        request = READ_MEMORY_PRE_SWD;   /* Swd request */
    }

    return SWDJTAG_TransferRetry(request, (uint32_t *)outData);
}

/**
  * @brief Last readback memory.
  * @param outData Read back memory data.
  * @param len data len
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t ReadMemoryEnd(uint8_t *outData, uint32_t len)
{
    uint32_t request;

    if (len > sizeof(uint32_t)) {
        return DAP_TRANSFER_FAULT;
    }

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        /* Read DP3 */
        uint32_t ir = JTAG_DPACC;
        JTAG_IR(ir);
    }
    request = READ_MEMORY_END;

    return SWDJTAG_TransferRetry(request, (uint32_t *)outData);
}

/**
  * @brief Sample a frame of complete data.
  * @param addr sample addr.
  * @param length sample length.
  * @param outData sample data.
  * @param maxLen  Max data len.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SamplingOneVal(uint32_t addr, uint32_t length, uint8_t *outData, uint32_t maxLen)
{
    uint32_t data;
    uint8_t ack;
    int32_t count = length / VARTYPE_INT_LEN;
    uint8_t *outDataPtr = outData;
    
    if (maxLen > VARTYPE_LONG_LEN) {
        return DAP_TRANSFER_FAULT;   /* Error check */
    }
    /* req = 8, data = 1000000
       req = 5, data = 4001800
       req = F, data = EA00000D
       req = E, data = 0       */
    if (g_isValMonitorSetFlag == 0) {
        g_isValMonitorSetFlag = 1;

        if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
            JTAG_RESET();      /* reset jtag */
        }
        ack = SwitchBusChannel();    /* Switching Channels */
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }
        /* Sets the length of the memory to be accessed. */
        ack = SetAccessMemoryLength(ACCESS_DISCONTINUOUS_FOUR_BYTE);
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }
    }

    data = addr;
    do {
        ack = SetAccessMemoryAddr(data);  /* Sets the address for accessing the memory. */
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }
        /* Read four bytes back */
        data += VARTYPE_INT_LEN;

        ack = ReadMemoryPrepare();   /* Read Memory Value. */
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }

        ack = ReadMemoryEnd(outDataPtr, sizeof(uint32_t));  /* Read Memory Value. */
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }
        outDataPtr += VARTYPE_INT_LEN;
        count--;
    } while (count > 0);

    return ack;
}

/**
  * @brief Parsing of single-byte and double-byte variable data.
  * @param addressGroup Variable grouping information.
  * @param destAddr Parsed data.
  * @param srcAddr Data before parsing.
  * @param valLen Val len.
  * @param valType variable type.
  * @retval true or false.
  */
static bool ShortDataParsing(DapAddressGroup addressGroup, uint8_t *destAddr, uint8_t *srcAddr,
                             uint32_t valLen, VAR_TYPE valType)
{
    uint8_t *outDataPtr = destAddr;
    unsigned int startAddr = addressGroup.startAddress;
    unsigned int groupLen = addressGroup.length;
    unsigned int varLength = valType;
    errno_t rc = EOK;

    if (valLen > sizeof(uint32_t)) {
        return false;
    }

    do {
        /* Parse the content of each variable from the read four bytes of data. */
        rc = memcpy_s(outDataPtr,
                      varLength,
                      (char *)&srcAddr[(startAddr + g_parsingVarCount * varLength) & ALIGN_FOUR_BYTE],
                      varLength);
        if (rc != EOK) {
            return false;
        }
        outDataPtr += VARTYPE_LONG_LEN;
        g_parsingVarCount++;
        if (g_parsingVarCount == groupLen) {
            break;
        }
    } while (((startAddr + g_parsingVarCount * varLength) & ALIGN_FOUR_BYTE) != 0);
    return true;
}

/**
  * @brief Parse the sampled data to the value of the corresponding variable.
  * @param addressGroup Variable grouping information.
  * @param destAddr Parsed data.
  * @param valType variable type.
  * @param parsingPhase Data Parsing phase.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t DataParsing(DapAddressGroup addressGroup, uint8_t *destAddr, VAR_TYPE valType, uint8_t parsingPhase)
{
    uint8_t ack;
    errno_t rc = EOK;
    uint8_t value[4];
    uint8_t *outDataPtr = destAddr;

    if (valType == TYPE_LONG) {
        ack = ReadMemoryContinuous(outDataPtr, sizeof(uint32_t));
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }
        outDataPtr += VARTYPE_INT_LEN;
    }
    if (parsingPhase == PARSING_CONTINUOUS) {
        ack = ReadMemoryContinuous(value, sizeof(value));
    } else if (parsingPhase == PARSING_ENDING) {
        ack = ReadMemoryEnd(value, sizeof(value));
    }
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }
    if (valType <= TYPE_SHORT) {
        ShortDataParsing(addressGroup, outDataPtr, value, sizeof(value), valType);
    } else if (valType == TYPE_INT) {
        rc = memcpy_s(outDataPtr, VARTYPE_INT_LEN, (char *)value, VARTYPE_INT_LEN);
    } else if (valType == TYPE_LONG) {
        rc = memcpy_s(outDataPtr, VARTYPE_INT_LEN, (char *)value, VARTYPE_INT_LEN);
    }
    if (rc != EOK) {
        return DAP_TRANSFER_ERROR;
    }

    return ack;
}

/**
  * @brief Sampling and parsing data.
  * @param addressGroup Variable grouping information.
  * @param sampleCount Number of sampling times.
  * @param valType variable type.
  * @param outData Data Parsing phase.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SampleAndParsing(DapAddressGroup addressGroup, uint32_t sampleCount, VAR_TYPE valType, uint8_t *outData)
{
    uint8_t ack = DAP_TRANSFER_ERROR;
    uint32_t recordVarCount = 0;
    uint8_t *outDataPtr = outData;

    g_parsingVarCount = 0;
    for (uint32_t i = 0; i < sampleCount; i++) {
        if (i == 0) {
            ack = ReadMemoryPrepare();
        } else {
            ack = DataParsing(addressGroup, outDataPtr, valType, PARSING_CONTINUOUS);
            /* Sampling buffer pointer offset. */
            if (valType == TYPE_CHAR || valType == TYPE_SHORT) {
                /* 8 bytes in each step */
                outDataPtr += (g_parsingVarCount - recordVarCount) * VARTYPE_LONG_LEN;
                recordVarCount = g_parsingVarCount;
            } else if (valType == TYPE_INT || valType == TYPE_LONG) {
                /* 8 bytes in each step */
                outDataPtr += VARTYPE_LONG_LEN;
            }
        }
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }

        if (i == sampleCount - 1) {
            ack = DataParsing(addressGroup, outDataPtr, valType, PARSING_ENDING);
        }
    }

    return ack;
}

/**
  * @brief Pre-sampled for mixed-type variable groups.
  * @param mixPositionInfo Variable information of a mixed type variable group.
  * @param outData Read back memory data.
  * @param parsingPhase Data Parsing phase.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t DataPrepareMix(const DapPositionInfo *mixPositionInfo, uint8_t *outData, uint8_t parsingPhase)
{
    uint8_t ack;

    if (parsingPhase == PARSING_CONTINUOUS) {
        ack = ReadMemoryContinuous(outData, sizeof(uint32_t));
    } else if (parsingPhase == PARSING_ENDING) {
        if (mixPositionInfo->len[g_parsingVarCount] != VARTYPE_LONG_LEN) {
            ack = ReadMemoryEnd(outData, sizeof(uint32_t));
        } else {
            ack = ReadMemoryContinuous(outData, sizeof(uint32_t));
        }
    }

    return ack;
}

/**
  * @brief Parsing sampled data for mixed-type variable groups.
  * @param mixPositionInfo Variable information of a mixed type variable group.
  * @param destAddr Parsed data.
  * @param groupLen mixed type variable group length.
  * @param parsingPhase Data Parsing phase.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t DataParsingMix(DapPositionInfo *mixPositionInfo, uint8_t *destAddr, unsigned int groupLen,
                              uint8_t parsingPhase)
{
    uint8_t ack;
    errno_t rc = EOK;
    uint8_t value[4];
    uint8_t *outDataPtr = destAddr;

    ack = DataPrepareMix(mixPositionInfo, value, parsingPhase);
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }
    do {
        if (mixPositionInfo->len[g_parsingVarCount] == VARTYPE_LONG_LEN) {
            rc = memcpy_s(outDataPtr, VARTYPE_INT_LEN, (char *)value, VARTYPE_INT_LEN);
            if (rc != EOK) {
                return DAP_TRANSFER_ERROR;
            }
            outDataPtr += VARTYPE_INT_LEN;

            if (parsingPhase == PARSING_CONTINUOUS) {
                ack = ReadMemoryContinuous(outDataPtr, sizeof(uint32_t));
            } else if (parsingPhase == PARSING_ENDING) {
                ack = ReadMemoryEnd(outDataPtr, sizeof(uint32_t));
            }
            if (ack != DAP_TRANSFER_OK) {
                return ack;
            }
            outDataPtr += VARTYPE_INT_LEN;
            g_parsingVarCount++;
        } else {
            rc = memcpy_s(outDataPtr, mixPositionInfo->len[g_parsingVarCount],
                          (char *)&value[mixPositionInfo->position[g_parsingVarCount]],
                          mixPositionInfo->len[g_parsingVarCount]);
            if (rc != EOK) {
                return DAP_TRANSFER_ERROR;
            }
            outDataPtr += VARTYPE_LONG_LEN;
            g_parsingVarCount++;
        }
        if (g_parsingVarCount == groupLen) {
            break;
        }
    } while (mixPositionInfo->position[g_parsingVarCount] != 0);

    return ack;
}

/**
  * @brief char data sampling.
  * @param addressGroup Variable grouping information.
  * @param outData Parsed data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t GroupChar(DapAddressGroup addressGroup, uint8_t *outData)
{
    uint8_t ack;
    uint32_t sampleCount;

    /* Number of sampling times for determining the number of char variables */
    sampleCount = ((addressGroup.startAddress & ALIGN_FOUR_BYTE) + addressGroup.length * TYPE_CHAR) / VARTYPE_INT_LEN;
    if ((((addressGroup.startAddress & ALIGN_FOUR_BYTE) + addressGroup.length * TYPE_CHAR) % VARTYPE_INT_LEN) != 0) {
        sampleCount++;
    }

    ack = SampleAndParsing(addressGroup, sampleCount, TYPE_CHAR, outData);
    return ack;
}

/**
  * @brief short data sampling.
  * @param addressGroup Variable grouping information.
  * @param outData Parsed data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t GroupShort(DapAddressGroup addressGroup, uint8_t *outData)
{
    uint8_t ack;
    uint32_t sampleCount;

    /* Number of sampling times for determining the number of char variables */
    sampleCount = ((addressGroup.startAddress & ALIGN_FOUR_BYTE) + addressGroup.length * TYPE_SHORT) / VARTYPE_INT_LEN;
    if ((((addressGroup.startAddress & ALIGN_FOUR_BYTE) + addressGroup.length * TYPE_SHORT) % VARTYPE_INT_LEN) != 0) {
        sampleCount++;
    }

    ack = SampleAndParsing(addressGroup, sampleCount, TYPE_SHORT, outData);
    return ack;
}

/**
  * @brief int data sampling.
  * @param addressGroup Variable grouping information.
  * @param outData Parsed data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t GroupInt(DapAddressGroup addressGroup, uint8_t *outData)
{
    uint8_t ack;

    ack = SampleAndParsing(addressGroup, addressGroup.length, TYPE_INT, outData);
    return ack;
}

/**
  * @brief long data sampling.
  * @param addressGroup Variable grouping information.
  * @param outData Parsed data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t GroupLong(DapAddressGroup addressGroup, uint8_t *outData)
{
    uint8_t ack;

    ack = SampleAndParsing(addressGroup, addressGroup.length, TYPE_LONG, outData);
    return ack;
}

/**
  * @brief mix-type data sampling.
  * @param addressGroup Variable grouping information.
  * @param outData Parsed data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t GroupMix(DapAddressGroup addressGroup, uint8_t *outData)
{
    uint8_t ack = DAP_TRANSFER_ERROR;
    uint32_t sampleCount = 0;
    uint32_t recordVarCount = 0;
    uint8_t *outDataPtr = outData;

    /* Whether the position of the first variable is 0 or not, a sample sequence is required. */
    for (uint32_t i = 0; i < addressGroup.length; i++) {
        if (g_mixPositionInfo[addressGroup.mixPosition].position[i] == 0 && i != 0) {
            sampleCount++;
        }
    }
    sampleCount++;

    g_parsingVarCount = 0;
    for (uint32_t i = 0; i < sampleCount; i++) {
        if (i == 0) {
            ack = ReadMemoryPrepare();
        } else {
            ack = DataParsingMix(&g_mixPositionInfo[addressGroup.mixPosition], outDataPtr, addressGroup.length,
                                 PARSING_CONTINUOUS);
            /* 8 bytes in each step */
            outDataPtr += (g_parsingVarCount - recordVarCount) * VARTYPE_LONG_LEN;
            recordVarCount = g_parsingVarCount;
        }
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }

        if (i == sampleCount - 1) {
            ack = DataParsingMix(&g_mixPositionInfo[addressGroup.mixPosition], outDataPtr, addressGroup.length,
                                 PARSING_ENDING);
        }
    }
    return ack;
}

/**
  * @brief Selecting a group type.
  * @param addressGroup Variable grouping information.
  * @param outData Parsed data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SelectGroupType(DapAddressGroup addressGroup, uint8_t *outData)
{
    uint8_t ack = DAP_TRANSFER_ERROR;

    switch (addressGroup.type) {
        case TYPE_CHAR:
            ack = GroupChar(addressGroup, outData);
            break;
        case TYPE_SHORT:
            ack = GroupShort(addressGroup, outData);
            break;
        case TYPE_INT:
            ack = GroupInt(addressGroup, outData);
            break;
        case TYPE_LONG:
            ack = GroupLong(addressGroup, outData);
            break;
        case TYPE_MIX:
            ack = GroupMix(addressGroup, outData);
            break;
        default:
            break;
    }

    return ack;
}

/**
  * @brief address incremental continuous sampling.
  * @param addressGroup Variable grouping information.
  * @param outData Parsed data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SampleAddessIncreVal(DapAddressGroup addressGroup, uint8_t *outData)
{
    uint32_t data;
    uint8_t ack;

    if (g_isValMonitorSetFlag == 0) {
        g_isValMonitorSetFlag = 1;

        if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
            JTAG_RESET();
        }
        ack = SwitchBusChannel();
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }

        ack = SetAccessMemoryLength(ACCESS_CONTINUOUS_FOUR_BYTE);
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }
    }

    data = addressGroup.startAddress - addressGroup.startAddress % VARTYPE_INT_LEN;
    ack = SetAccessMemoryAddr(data);
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }

    ack = SelectGroupType(addressGroup, outData);

    return ack;
}

/**
  * @brief address map comparison function.
  * @param aa Comparison parameter aa.
  * @param bb Comparison parameter bb.
  * @retval Compare Results.
  */
static int MapCmp(const void *aa, const void *bb)
{
    DapAddressMap *a = (DapAddressMap *)aa;
    DapAddressMap *b = (DapAddressMap *)bb;

    return (a->map - b->map);
}

/**
  * @brief Recovery Address Sequence.
  * @param str Sampled data to be restored.
  * @param length Length of the variable list to be restored.
  * @retval type: MALLOC_FAIL, RET_FAIL, RET_SUCCESS.
  */
static int RestoreAddressSequence(uint8_t *str, uint32_t length)
{
    DapAddressMap *tempAddressMap;
    errno_t rc = EOK;
    tempAddressMap = (DapAddressMap *)malloc(length * sizeof(DapAddressMap));
    if (tempAddressMap == NULL) {
        DBG_PRINTF("tempAddressMap malloc fail!!\r\n");
        StopSampling();
        return MALLOC_FAIL;
    }
    for (int i = 0; i < length; i++) {
        tempAddressMap[i].map = g_dapAddressGroupList.addressMap[i];
        rc = memcpy_s(tempAddressMap[i].sampleValue, VARTYPE_LONG_LEN, &str[i * VARTYPE_LONG_LEN], VARTYPE_LONG_LEN);
        if (rc != EOK) {
            free(tempAddressMap);
            tempAddressMap = NULL;
            return RET_FAIL;
        }
    }

    qsort(tempAddressMap, length, sizeof(tempAddressMap[0]), MapCmp);

    for (int i = 0; i < length; i++) {
        rc = memcpy_s(&str[i * VARTYPE_LONG_LEN], VARTYPE_LONG_LEN, tempAddressMap[i].sampleValue, VARTYPE_LONG_LEN);
        if (rc != EOK) {
            free(tempAddressMap);
            tempAddressMap = NULL;
            return RET_FAIL;
        }
    }
    if (tempAddressMap != NULL) {
        free(tempAddressMap);
        tempAddressMap = NULL;
    }

    return RET_SUCCESS;
}

/**
  * @brief Check whether the variable list is continuous.
  * @param addrA Variable A address.
  * @param addrB Variable B address.
  * @param lenB Variable B length.
  * @retval true or false.
  */
static bool IsCharContinuous(unsigned int addrA, unsigned int addrB, unsigned int lenB)
{
    bool isIncrementalFlag = false;

    if (lenB == VARTYPE_SHORT_LEN && (addrB - addrA) == ADDR_INTERVAL_LEN_TWO) {
        /* char-short: If the address difference is 2, it is also continuous. */
        isIncrementalFlag = true;
    } else if (((lenB % VARTYPE_INT_LEN) == 0) &&
                ((addrB - addrA) == ADDR_INTERVAL_LEN_TWO  ||
                (addrB - addrA) == ADDR_INTERVAL_LEN_THREE ||
                (addrB - addrA) == ADDR_INTERVAL_LEN_FOUR)) {
        /* char-int: If the address difference is 2,3,4, it is also continuous. */
        isIncrementalFlag = true;
    }
}

/**
  * @brief Check whether the variable list is continuous.
  * @param addrA Variable A address.
  * @param addrB Variable B address.
  * @param lenA Variable A length.
  * @param lenB Variable B length.
  * @retval continuous or not.
  */
static int JudgeVarContinuous(unsigned int addrA, unsigned int addrB, unsigned int lenA, unsigned int lenB)
{
    int isIncrementalFlag = 0;

    /* Regroup at flash boundary. */
    if ((addrB % FLASH_BOUNDARY) == 0) {
        isIncrementalFlag = 0;
    } else if ((addrB - addrA) == lenA) {
        isIncrementalFlag = 1;
    } else if (lenA == VARTYPE_CHAR_LEN) {
        if (IsCharContinuous(addrA, addrB, lenB)) {
            isIncrementalFlag = 1;
        }
    } else if (lenA == VARTYPE_SHORT_LEN) {
        if (((lenB % VARTYPE_INT_LEN) == 0) &&
            ((addrB - addrA) == ADDR_INTERVAL_LEN_FOUR)) {
            /* short-int/long long: If the address difference is 4, it is also continuous. */
            isIncrementalFlag = 1;
        }
    }

    return isIncrementalFlag;
}

/**
  * @brief Grouping a list of consecutive variables.
  * @param dapValMonitor Variable list information.
  * @param dapAddressGroupList Variable grouping information.
  * @param address Variable Address list.
  * @param varLengthMap Variable length list.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int ContinuousVarGroup(const DapVarMonitor *dapValMonitor, DapAddressGroupList *dapAddressGroupList,
                              const unsigned int *address, const unsigned int *varLengthMap)
{
    int isIncrementalFlag = 0;

    dapAddressGroupList->addressGroup = (DapAddressGroup *)malloc(dapValMonitor->varNum * sizeof(DapAddressGroup));
    if (dapAddressGroupList->addressGroup == NULL) {
        DBG_PRINTF("addressGroup malloc fail!!\r\n");
        return MALLOC_FAIL;
    }

    /* Incremental Address Group Initialization. */
    dapAddressGroupList->isAddressIncrementFlag = 1;
    dapAddressGroupList->count = 1;
    dapAddressGroupList->addressGroup[dapAddressGroupList->count - 1].startAddress = address[0];
    dapAddressGroupList->addressGroup[dapAddressGroupList->count - 1].length = 1;
    /* Grouping Continuous and Non-Continuous Variables. */
    for (int i = 1; i < dapValMonitor->varNum; i++) {
        isIncrementalFlag = JudgeVarContinuous(address[i - 1], address[i], varLengthMap[i - 1], varLengthMap[i]);
        if (isIncrementalFlag != 0) {
            isIncrementalFlag = 0;
            dapAddressGroupList->addressGroup[dapAddressGroupList->count - 1].length++;
            continue;
        }
        dapAddressGroupList->count++;
        dapAddressGroupList->addressGroup[dapAddressGroupList->count - 1].startAddress = address[i];
        dapAddressGroupList->addressGroup[dapAddressGroupList->count - 1].length = 1;
    }

    return RET_SUCCESS;
}

/**
  * @brief Check whether the type is mixed.
  * @param varLengthMap Variable length list.
  * @param groupLen Length of the variable group
  * @param count Position counter of the variable in the variable length list.
  * @retval mix-type or not.
  */
static unsigned int JudgeIsMix(const unsigned int *varLengthMap, unsigned int groupLen, unsigned int count)
{
    unsigned int j;

    for (j = 0; j < groupLen - 1; j++) {
        if (varLengthMap[count + j] != varLengthMap[count + j + 1]) {
            break;
        }
    }
    if (j == (groupLen - 1)) {
        return 0;
    }

    return 1;
}

/**
  * @brief Determine the group type.
  * @param dapAddressGroupList Variable grouping information.
  * @param varLengthMap Variable length list.
  * @retval Number of variable groups of mixed types.
  */
static unsigned int JudgeType(DapAddressGroupList *dapAddressGroupList,
                              const unsigned int *varLengthMap, unsigned int mapLen)
{
    int i;
    unsigned int count = 0;
    unsigned int mixPosition = 0;
    /* Checking Data Validity */
    if (mapLen < dapAddressGroupList->count) {
        return 0;
    }
    /* Traverse the address group list. */
    for (i = 0; i < dapAddressGroupList->count; i++) {
        if (dapAddressGroupList->addressGroup[i].length > 1) {
            /* if each type of continuous variable is consistent */
            if (JudgeIsMix(varLengthMap, dapAddressGroupList->addressGroup[i].length, count) == 0) {
                dapAddressGroupList->addressGroup[i].type = varLengthMap[count];
            } else {
                dapAddressGroupList->addressGroup[i].type = TYPE_MIX;
                /* Obtains the position and length of each variable in the sampled four bytes of the mixed group. */
                dapAddressGroupList->addressGroup[i].mixPosition = mixPosition;
                mixPosition++;
            }
        } else {
            dapAddressGroupList->addressGroup[i].type = varLengthMap[count];
        }
        count += dapAddressGroupList->addressGroup[i].length;
    }

    return mixPosition;
}

/**
  * @brief Records the postion information of variable groups of mixed types.
  * @param dapAddressGroupList Variable grouping information.
  * @param address Variable Address list.
  * @param varLengthMap Variable length list.
  * @param mixGruopNum Number of variable groups of mixed types.
  * @retval type: MALLOC_FAIL, RET_FAIL, RET_SUCCESS.
  */
static int RecordMixPosInfo(DapAddressGroupList *dapAddressGroupList, const unsigned int *address,
                            const unsigned int *varLengthMap, unsigned int mixGruopNum)
{
    int i;
    int j;
    unsigned int count = 0;
    errno_t rc = EOK;

    g_mixPositionInfo = (DapPositionInfo *)malloc(sizeof(DapPositionInfo) * mixGruopNum);
    if (g_mixPositionInfo == NULL) {
        DBG_PRINTF("mixPositionInfo malloc fail!!\r\n");
        return MALLOC_FAIL;
    }
    /* clear mix group position infomation. */
    rc = memset_s(g_mixPositionInfo, sizeof(DapPositionInfo) * mixGruopNum, 0, sizeof(DapPositionInfo) * mixGruopNum);
    if (rc != EOK) {
        return RET_FAIL;
    }

    for (i = 0; i < dapAddressGroupList->count; i++) {
        if (dapAddressGroupList->addressGroup[i].type == TYPE_MIX) {
            unsigned int mixPos = dapAddressGroupList->addressGroup[i].mixPosition;
            for (j = 0; j < dapAddressGroupList->addressGroup[i].length; j++) {
                g_mixPositionInfo[mixPos].position[j] = address[count + j] & ALIGN_FOUR_BYTE;
                g_mixPositionInfo[mixPos].len[j] = varLengthMap[count + j];
            }
        }
        count += dapAddressGroupList->addressGroup[i].length;
    }

    return RET_SUCCESS;
}

/**
  * @brief Categorizing Variable Groups.
  * @param dapAddressGroupList Variable grouping information.
  * @param address Variable Address list.
  * @param varLengthMap Variable length list.
  * @param varLengthMapLen var Length Map len.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int ClassifyVarGroup(DapAddressGroupList *dapAddressGroupList, const unsigned int *address,
                            const unsigned int *varLengthMap, unsigned int varLengthMapLen)
{
    unsigned int mixPosition = 0;
    int retValue;

    /* Determine whether the type of the address continuous variable is single or mixed. */
    mixPosition = JudgeType(dapAddressGroupList, varLengthMap, varLengthMapLen);
    if (mixPosition != 0) {
        retValue = RecordMixPosInfo(dapAddressGroupList, address, varLengthMap, mixPosition);
        if (retValue != RET_SUCCESS) {
            return retValue;
        }
    }

    return RET_SUCCESS;
}

/**
  * @brief Record variable length mapping.
  * @param dapValMonitor Variable list information.
  * @param address Variable Address list.
  * @param varLenMap Variable length list.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int RecordVarLenMap(DapVarMonitor *dapValMonitor, const unsigned int *address, unsigned int *varLenMap)
{
    int i;
    int j;
    unsigned int *recordVar;
    unsigned int *varLengthMap = varLenMap;

    recordVar = (unsigned int *)malloc(dapValMonitor->varNum * sizeof(unsigned int));
    if (recordVar == NULL) {
        DBG_PRINTF("recordVar malloc fail!!\r\n");
        return MALLOC_FAIL;
    }

    for (i = 0; i < dapValMonitor->varNum; i++) {
        recordVar[i] = dapValMonitor->varList[i].varAddress;
    }

    for (i = 0; i < dapValMonitor->varNum; i++) {
        for (j = 0; j < dapValMonitor->varNum; j++) {
            if (address[i] == recordVar[j]) {
                recordVar[j] = 0;
                /* Save the length of the sorted variable. */
                varLengthMap[i] = dapValMonitor->varList[j].varNumBytes;
                break;
            }
        }
    }
    if (recordVar != NULL) {
        free(recordVar);
        recordVar = NULL;
    }
    return RET_SUCCESS;
}

/**
  * @brief Check whether the variable list is increasing and continuous.
  * @param dapValMonitor Variable list information.
  * @param dapAddressGroupList Variable grouping information.
  * @param address Variable Address list.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int IsIncrementalAddress(DapVarMonitor *dapValMonitor, DapAddressGroupList *dapAddressGroupList,
                                const unsigned int *address)
{
    /* Save the length of the sorted variable. */
    unsigned int *varLengthMap;
    int retValue;

    varLengthMap = (unsigned int *)malloc(dapValMonitor->varNum * sizeof(unsigned int));
    if (varLengthMap == NULL) {
        DBG_PRINTF("varLengthMap malloc fail!!\r\n");
        return MALLOC_FAIL;
    }
    retValue = RecordVarLenMap(dapValMonitor, address, varLengthMap);
    if (retValue != RET_SUCCESS) {
        free(varLengthMap);
        varLengthMap = NULL;
        return retValue;
    }

    /* Grouping Continuity of Variables. */
    retValue = ContinuousVarGroup(dapValMonitor, dapAddressGroupList, address, varLengthMap);
    if (retValue != RET_SUCCESS) {
        free(varLengthMap);
        varLengthMap = NULL;
        return retValue;
    }

    /* if there is no continuity for each variable. */
    if (dapAddressGroupList->count == dapValMonitor->varNum) {
        dapAddressGroupList->isAddressIncrementFlag = 0;
        if (dapAddressGroupList->addressGroup != NULL) {
            free(dapAddressGroupList->addressGroup);
            dapAddressGroupList->addressGroup = NULL;
        }
        if (varLengthMap != NULL) {
            free(varLengthMap);
            varLengthMap = NULL;
        }
        return RET_SUCCESS;
    }

    /* Categorizing Continuous Variables. */
    retValue = ClassifyVarGroup(dapAddressGroupList, address, varLengthMap,
                                dapValMonitor->varNum * sizeof(unsigned int));
    if (retValue != RET_SUCCESS) {
        free(varLengthMap);
        varLengthMap = NULL;
        return retValue;
    }
    if (varLengthMap != NULL) {
        free(varLengthMap);
        varLengthMap = NULL;
    }

    return RET_SUCCESS;
}

/**
  * @brief address comparison function.
  * @param aa Comparison parameter aa.
  * @param bb Comparison parameter bb.
  * @retval Compare Results.
  */
static int AddressCmp(const void *aa, const void *bb)
{
    long long int difference;
    difference = (long long int)(*(unsigned long long *)aa) - (long long int)(*(unsigned long long *)bb);

    if (difference > 0) {
        return 1;
    } else if (difference == 0) {
        return 0;
    } else {
        return -1;
    }
}

/**
  * @brief Record variable address mapping.
  * @param dapValMonitor Variable list information.
  * @param dapAddressGroupList Variable grouping information.
  * @param address Variable Address list.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int RecordVarAddrMap(DapVarMonitor *dapValMonitor, DapAddressGroupList *dapAddressGroupList,
                            const unsigned int *address)
{
    int i;
    int j;
    unsigned int *recordVar;

    recordVar = (unsigned int *)malloc(dapValMonitor->varNum * sizeof(unsigned int));
    if (recordVar == NULL) {
        DBG_PRINTF("recordVar malloc fail!!\r\n");
        return MALLOC_FAIL;
    }
    dapAddressGroupList->isAddressMapFlag = 1;
    for (i = 0; i < dapValMonitor->varNum; i++) {
        recordVar[i] = dapValMonitor->varList[i].varAddress;
    }
    /* Record the address sequence before sorting. */
    for (i = 0; i < dapValMonitor->varNum; i++) {
        for (j = 0; j < dapValMonitor->varNum; j++) {
            if (address[i] == recordVar[j]) {
                recordVar[j] = 0;
                /* Save the position of the sorted variable. */
                dapAddressGroupList->addressMap[i] = j;
                break;
            }
        }
    }
    /* Release resources. */
    if (recordVar != NULL) {
        free(recordVar);
        recordVar = NULL;
    }
    return RET_SUCCESS;
}

/**
  * @brief variable grouping algorithm.
  * @param dapValMonitor Variable list information.
  * @param dapAddressGroupList Variable grouping information.
  * @param address Variable Address list.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int VarGroupingAlgorithm(DapVarMonitor *dapValMonitor, DapAddressGroupList *dapAddressGroupList,
                                const unsigned int *address)
{
    int i;
    int isAddressMapFlag = 0;
    int retValue;

    for (i = 0; i < dapValMonitor->varNum; i++) {
        if (address[i] != dapValMonitor->varList[i].varAddress) {
            isAddressMapFlag = 1;
            break;
        }
    }

    /* If address mapping required */
    if (isAddressMapFlag != 0) {
        retValue = RecordVarAddrMap(dapValMonitor, dapAddressGroupList, address);
        if (retValue != RET_SUCCESS) {
            return retValue;
        }
    } else {
        dapAddressGroupList->isAddressMapFlag = 0;
        if (dapAddressGroupList->addressMap != NULL) {
            free(dapAddressGroupList->addressMap);
            dapAddressGroupList->addressMap = NULL;
        }
    }
    DBG_PRINTF("isAddressMapFlag %d\r\n", dapAddressGroupList->isAddressMapFlag);

    retValue = IsIncrementalAddress(dapValMonitor, dapAddressGroupList, address);
    if (retValue != RET_SUCCESS) {
        return retValue;
    }

    return RET_SUCCESS;
}

/**
  * @brief Variable incremental sampling initialization.
  * @param None.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int VarIncrementalInit(void)
{
    int i;
    errno_t rc = EOK;
    unsigned int *addressList;
    int retValue;

    addressList = (unsigned int *)malloc(g_dapValMonitor.varNum * sizeof(unsigned int));
    if (addressList == NULL) {
        DBG_PRINTF("addressList malloc fail!!\r\n");
        return MALLOC_FAIL;
    }

    for (i = 0; i < g_dapValMonitor.varNum; i++) {
        addressList[i] = g_dapValMonitor.varList[i].varAddress;
    }
    qsort(addressList, g_dapValMonitor.varNum, sizeof(unsigned int), AddressCmp);

    /* Clearing DapAddressGroupList Information. */
    rc = memset_s(&g_dapAddressGroupList, sizeof(DapAddressGroupList),
                  0, sizeof(DapAddressGroupList) - sizeof(DapAddressGroup *) - sizeof(unsigned int *));
    if (rc != EOK) {
        free(addressList);
        addressList = NULL;
        return RET_FAIL;
    }
    g_dapAddressGroupList.addressMap = (unsigned int *)malloc(sizeof(unsigned int) * g_dapValMonitor.varNum);
    if (g_dapAddressGroupList.addressMap == NULL) {
        DBG_PRINTF("addressMap malloc fail!!\r\n");
        free(addressList);
        addressList = NULL;
        return MALLOC_FAIL;
    }

    retValue = VarGroupingAlgorithm(&g_dapValMonitor, &g_dapAddressGroupList, addressList);
    if (retValue != RET_SUCCESS) {
        free(addressList);
        addressList = NULL;
        return retValue;
    }

    if (addressList != NULL) {
        free(addressList);
        addressList = NULL;
    }

    return RET_SUCCESS;
}

/**
  * @brief Non-incremental sampling.
  * @param dapValMonitor Variable list information.
  * @param frameData Sampled data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SamplingNonIncremental(DapVarMonitor *dapValMonitor, uint8_t *frameData)
{
    uint8_t *frameDataPtr = frameData;
    uint8_t ret;
    errno_t rc = EOK;

    for (uint32_t i = 0; i < dapValMonitor->varNum; i++) {
        uint8_t value[VARTYPE_LONG_LEN];

        ret = SamplingOneVal(dapValMonitor->varList[i].varAddress, dapValMonitor->varList[i].varNumBytes,
                             value, VARTYPE_LONG_LEN);
        if (ret != DAP_TRANSFER_OK) {
            break;
        }
        rc = memcpy_s(frameDataPtr, dapValMonitor->varList[i].varNumBytes,
                      (char *)&value[dapValMonitor->varList[i].varAddress & ALIGN_FOUR_BYTE],
                      dapValMonitor->varList[i].varNumBytes);
        if (rc != EOK) {
            return DAP_TRANSFER_ERROR;
        }
        frameDataPtr += dapValMonitor->varList[i].varNumBytes;
    }

    return ret;
}

/**
  * @brief incremental sampling.
  * @param dapValMonitor Variable list information.
  * @param frameData Sampled data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SamplingIncremental(DapVarMonitor *dapValMonitor, uint8_t *frameData)
{
    uint8_t *frameDataPtr = frameData;
    uint8_t ret = DAP_TRANSFER_ERROR;
    uint8_t *sampleValue;
    int retValue;
    errno_t rc = EOK;

    sampleValue = (uint8_t *)malloc(dapValMonitor->varNum * VARTYPE_LONG_LEN * sizeof(uint8_t));
    if (sampleValue == NULL) {
        DBG_PRINTF("sampleValue malloc fail!!\r\n");
        StopSampling();
        return DAP_TRANSFER_ERROR;
    }
    uint8_t *valuePtr = sampleValue;

    for (uint32_t i = 0; i < g_dapAddressGroupList.count; i++) {
        ret = SampleAddessIncreVal(g_dapAddressGroupList.addressGroup[i], valuePtr);
        if (ret != DAP_TRANSFER_OK) {
            break;
        }
        valuePtr += g_dapAddressGroupList.addressGroup[i].length * VARTYPE_LONG_LEN;
    }

    if (ret == DAP_TRANSFER_OK) {
        if (g_dapAddressGroupList.isAddressMapFlag != 0) {
            retValue = RestoreAddressSequence(sampleValue, dapValMonitor->varNum);
            if (retValue != RET_SUCCESS) {
                free(sampleValue);
                sampleValue = NULL;
                StopSampling();
                return DAP_TRANSFER_ERROR;
            }
        }

        for (uint32_t i = 0; i < dapValMonitor->varNum; i++) {
            rc = memcpy_s(frameDataPtr, dapValMonitor->varList[i].varNumBytes,
                          &sampleValue[i * VARTYPE_LONG_LEN], dapValMonitor->varList[i].varNumBytes);
            if (rc != EOK) {
                free(sampleValue);
                sampleValue = NULL;
                StopSampling();
                return DAP_TRANSFER_ERROR;
            }
            frameDataPtr += dapValMonitor->varList[i].varNumBytes;
        }
    }
    if (sampleValue != NULL) {
        free(sampleValue);
        sampleValue = NULL;
    }

    return ret;
}

/**
  * @brief Performance test sampling.
  * @param dapValMonitor Variable list information.
  * @param frameData Sampled data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t SamplingPerformance(DapVarMonitor *dapValMonitor, uint8_t *frameData)
{
    uint8_t *frameDataPtr = frameData;
    errno_t rc = EOK;

    for (uint32_t i = 0; i < dapValMonitor->varNum; i++) {
        /* The initialization step is 100 when the number of virtual variables exceeds 1000. */
        if (g_performanceTestVar[i]++ >= (i * 100 + 1000)) {
            /* Dummy variable steps every 100. */
            g_performanceTestVar[i] = i * 100;
        }
        rc = memcpy_s(frameDataPtr, dapValMonitor->varList[i].varNumBytes,
                      (char *)&g_performanceTestVar[i], dapValMonitor->varList[i].varNumBytes);
        if (rc != EOK) {
            return DAP_TRANSFER_ERROR;
        }
        frameDataPtr += dapValMonitor->varList[i].varNumBytes;
    }

    return DAP_TRANSFER_OK;
}

/**
  * @brief sampling.
  * @param dapValMonitor Variable list information.
  * @param frameData Sampled data.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t Sampling(DapVarMonitor *dapValMonitor, uint8_t *frameData)
{
    uint8_t ret = DAP_TRANSFER_OK;

    if (g_isPerformanceTestFlag == 0) {
        if (g_dapAddressGroupList.isAddressIncrementFlag != 0) {
            ret = SamplingIncremental(dapValMonitor, frameData);
        } else {
            ret = SamplingNonIncremental(dapValMonitor, frameData);
        }
    } else {
        ret = SamplingPerformance(dapValMonitor, frameData);
    }

    return ret;
}

/**
  * @brief Sample a complete frame once.
  * @param varMonitorQueue Sampling shared memory queue.
  * @param time time stamp.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
static int SampleOneFrame(VarQueue *varMonitorQueue, uint32_t time)
{
    uint32_t retLen;
    uint8_t ret;
    uint8_t *outData;
    uint32_t outDataLen;
    errno_t rc = EOK;
    /* (timeStamp)4-byte */
    outDataLen = g_dapValMonitor.varNum * VARTYPE_LONG_LEN + VARTYPE_INT_LEN;
    outData = (uint8_t *)malloc(outDataLen * sizeof(uint8_t));
    if (outData == NULL) {
        StopSampling();
        return MALLOC_FAIL;
    }
    uint8_t *frameDataPtr = outData;

    /* The livewatch function uses the full sequence for slow sampling(>100000us). */
    if (g_dapValMonitor.periodUs >= 100000) {
        g_isValMonitorSetFlag = 0;
    }
    /* Total sampling times. */
    varMonitorQueue->totalSampleCount++;
    /* Record timestamp. */
    g_timeStamp += time;
    rc = memcpy_s(frameDataPtr, VARTYPE_INT_LEN, (char *)&g_timeStamp, VARTYPE_INT_LEN);
    if (rc != EOK) {
        free(outData);
        outData = NULL;
        return RET_FAIL;
    }
    frameDataPtr += VARTYPE_INT_LEN;

    ret = Sampling(&g_dapValMonitor, frameDataPtr);
    if (ret != DAP_TRANSFER_OK) {
        /* Number of error samples for calculating the error rate. */
        g_errorDetectionErrorCounter++;
        /* Total number of sampling errors. */
        varMonitorQueue->totalErrorCount++;
    } else {
        retLen = WriteQueue(varMonitorQueue, outData, g_varMonitorFrameSize);
        if (retLen != g_varMonitorFrameSize) {
            /* Total number of sampling overflows. */
            varMonitorQueue->totalOverflowCount++;
        }
    }
    if (outData != NULL) {
        free(outData);
        outData = NULL;
    }

    return RET_SUCCESS;
}

/**
  * @brief Checking the sampling error rate.
  * @param varMonitorQueue Sampling shared memory queue.
  * @param currentTime current time.
  * @retval None.
  */
static void CheckErrorRate(VarQueue *varMonitorQueue, uint32_t currentTime)
{
    uint32_t idCode = 0;
    /* Total number of sampling times for calculating the error rate. */
    g_errorDetectionTotalCounter++;
    if (GetDelta(currentTime, g_errorDetectionTimer) >= ERROR_DETECTION_INTERVAL_US) {
        /* Multiply the error rate by 100. */
        varMonitorQueue->errorRate = g_errorDetectionErrorCounter * 100 / g_errorDetectionTotalCounter;
        if (varMonitorQueue->errorRate >= g_errorRate) {
            SwdJtagReset();
            SwitchSwdJtag(0xE79E);
            SwdJtagReset();
            SwdJtagReadIdcode(&idCode);
            SwdJtagCleanError();
            SwdPowerSet();
            g_isReadMemoryFlag = 1;
            g_isValMonitorSetFlag = 0;
        }
        g_errorDetectionErrorCounter = 0;
        g_errorDetectionTotalCounter = 0;
        g_errorDetectionTimer = currentTime;
    } else {
        if (g_errorDetectionErrorCounter >= ERROR_MAX_ALLOWED) {
            SwdJtagReset();
            SwitchSwdJtag(0xE79E);
            SwdJtagReset();
            SwdJtagReadIdcode(&idCode);
            SwdJtagCleanError();
            SwdPowerSet();
            g_isReadMemoryFlag = 1;
            g_isValMonitorSetFlag = 0;
            g_errorDetectionErrorCounter = 0;
            g_errorDetectionTotalCounter = 0;
        }
    }
}

/**
  * @brief Preprocessing Before Modifying Variables.
  * @param addr sample addr.
  * @param length sample length.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
static uint8_t WriteValPrepare(uint32_t addr, uint32_t length)
{
    uint8_t ack;

    /* Switching Bus Channels. */
    ack = SwitchBusChannel();
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }

    /* Sets the length of the variable to be modified. */
    if (length == VARTYPE_CHAR_LEN) {
        ack = SetAccessMemoryLength(ACCESS_DISCONTINUOUS_ONE_BYTE);
    } else if (length == VARTYPE_SHORT_LEN) {
        ack = SetAccessMemoryLength(ACCESS_DISCONTINUOUS_TWO_BYTE);
    } else if (length >= VARTYPE_INT_LEN) {
        ack = SetAccessMemoryLength(ACCESS_CONTINUOUS_FOUR_BYTE);
    }
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }

    /* Sets the address of a variable. */
    ack = SetAccessMemoryAddr(addr);

    return ack;
}

/**
  * @brief Write a single variable.
  * @param addr sample addr.
  * @param length sample length.
  * @param value write value.
  * @retval type: DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR..
  */
uint8_t WriteOneVal(uint32_t addr, uint32_t length, uint64_t value)
{
    uint8_t ack;
    uint32_t lowValue;
    uint32_t highValue;

    lowValue = (value & LOW_32_BIT);
    /* Obtains the value of the upper 32 bits. */
    highValue = (value & HIGH_32_BIT) >> 32;

    /* WriteDP 2 0x1000000
       WriteAP 0 0x000002
       WriteAP 1 0x040018B0
       WriteAP 3 0x12345678
    */
    ack = WriteValPrepare(addr, length);
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }

    /* left shifting ((addr % VARTYPE_INT_LEN) * 8)-bit. */
    lowValue <<= ((addr % VARTYPE_INT_LEN) * 8);
    ack = WriteMemory(lowValue);
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }
    if (length == VARTYPE_LONG_LEN) {
        ack = WriteMemory(highValue);
        if (ack != DAP_TRANSFER_OK) {
            return ack;
        }
    }

    /* Restore the read variable length. */
    if (g_dapAddressGroupList.isAddressIncrementFlag != 0) {
        ack = SetAccessMemoryLength(ACCESS_CONTINUOUS_FOUR_BYTE);
    } else {
        ack = SetAccessMemoryLength(ACCESS_DISCONTINUOUS_FOUR_BYTE);
    }
    if (ack != DAP_TRANSFER_OK) {
        return ack;
    }

    return ack;
}

/**
  * @brief Store sampled variable values in the queue.
  * @param None.
  * @retval None.
  */
void ReadMemoryToQueue(void)
{
    uint32_t curTicks;
    uint32_t delta;

    if (g_isReadMemoryFlag != 0) {
        curTicks = GetTimerTickUs();
        delta = GetDelta(curTicks, g_preTicks);
        /* Sampling frequency */
        if (delta > g_dapValMonitor.periodUs) {
            SampleOneFrame(g_varMonitorQueue, delta);
            CheckErrorRate(g_varMonitorQueue, curTicks);
            g_preTicks = curTicks;
        }
    }
}

/**
  * @brief Preparation to start sampling.
  * @param None.
  * @retval type: MALLOC_FAIL, RET_SUCCESS.
  */
int StartSampling(void)
{
    int retValue;

    QueueInit(g_varMonitorQueue);
    /* Timestamp 4-byte. */
    g_varMonitorFrameSize = 4;
    for (unsigned int i = 0; i < g_dapValMonitor.varNum; i++) {
        g_varMonitorFrameSize += g_dapValMonitor.varList[i].varNumBytes;
    }

    if (g_isPerformanceTestFlag != 0) {
        g_performanceTestVar = (unsigned long long int *)malloc(sizeof(unsigned long long int) *
                                                                g_dapValMonitor.varNum);
        if (g_performanceTestVar == NULL) {
            return MALLOC_FAIL;
        }
        for (unsigned int i = 0; i < g_dapValMonitor.varNum; i++) {
            /* Dummy variable steps every 100. */
            g_performanceTestVar[i] = i * 100;
        }
    } else {
        /* When sampling, the address increment needs to be determined. */
        retValue = VarIncrementalInit();
        if (retValue != RET_SUCCESS) {
            return retValue;
        }
    }
    /* Clear the timestamp. */
    g_timeStamp = 0;
    /* set the read memory flag */
    g_isReadMemoryFlag = 1;
    g_isValMonitorSetFlag = 0;
    g_preTicks = GetTimerTickUs();
    /* Error detection initialization. */
    g_errorDetectionTimer = g_preTicks;
    g_errorDetectionTotalCounter = 0;
    g_errorDetectionErrorCounter = 0;

    return RET_SUCCESS;
}

/**
  * @brief Release all memory requested for sampling.
  * @param None.
  * @retval None.
  */
void FreeMemory(void)
{
    if ((g_isPerformanceTestFlag != 0) && (g_performanceTestVar != NULL)) {
        free(g_performanceTestVar);
        g_performanceTestVar = NULL;
    }

    /* Clearing Sampling Information. */
    memset_s(&g_dapValMonitor, sizeof(DapVarMonitor) - sizeof(DapVarList *),
             0, sizeof(DapVarMonitor) - sizeof(DapVarList *));
    if (g_dapValMonitor.varList != NULL) {
        free(g_dapValMonitor.varList);
        g_dapValMonitor.varList = NULL;
    }

    if ((g_dapAddressGroupList.isAddressIncrementFlag != 0) && (g_mixPositionInfo != NULL)) {
        free(g_mixPositionInfo);
        g_mixPositionInfo = NULL;
    }

    if ((g_dapAddressGroupList.isAddressMapFlag != 0) && (g_dapAddressGroupList.addressMap != NULL)) {
        free(g_dapAddressGroupList.addressMap);
        g_dapAddressGroupList.addressMap = NULL;
    }
    if ((g_dapAddressGroupList.isAddressIncrementFlag != 0) && (g_dapAddressGroupList.addressGroup != NULL)) {
        free(g_dapAddressGroupList.addressGroup);
        g_dapAddressGroupList.addressGroup = NULL;
    }
    /* Clearing DapAddressGroupList Information. */
    memset_s(&g_dapAddressGroupList, sizeof(DapAddressGroupList) - sizeof(DapAddressGroup *) - sizeof(unsigned int *),
             0, sizeof(DapAddressGroupList) - sizeof(DapAddressGroup *) - sizeof(unsigned int *));
}

/**
  * @brief Stop sampling processing.
  * @param None.
  * @retval None.
  */
void StopSampling(void)
{
    QueueDestroy(g_varMonitorQueue);
    FreeMemory();

    /* Clear the timestamp. */
    g_timeStamp = 0;
    /* clear the read memory flag */
    g_isReadMemoryFlag = 0;
    /* clear Error detection counter */
    g_errorDetectionTotalCounter = 0;
    g_errorDetectionErrorCounter = 0;

    if (g_DAP_Data.debug_port == DAP_PORT_JTAG) {
        JTAG_RESET();
    }
}

/**
  * @brief Pause sampling processing.
  * @param state Pause or resume.
  * @retval None.
  */
void PauseSampling(uint32_t state)
{
    g_isReadMemoryFlag = state;
}
