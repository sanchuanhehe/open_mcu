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
  * @file    swd_jtag_config.c
  * @author  MCU Driver Team
  * @brief   swd and jtag config.
  */
#include "swd_jtag_config.h"
#include "debug.h"

#define SWD_JTAG_MIN_CLOCK_LEVEL_0                   12000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_1                   10000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_2                   9000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_3                   8000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_4                   7000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_5                   6000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_6                   5000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_7                   4000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_8                   3000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_9                   2000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_10                  1000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_11                  800000
#define SWD_JTAG_MIN_CLOCK_LEVEL_12                  600000
#define SWD_JTAG_MIN_CLOCK_LEVEL_13                  500000
#define SWD_JTAG_MIN_CLOCK_LEVEL_14                  400000
#define SWD_JTAG_MIN_CLOCK_LEVEL_15                  300000
#define SWD_JTAG_MIN_CLOCK_LEVEL_16                  250000
#define SWD_JTAG_MIN_CLOCK_LEVEL_17                  200000
#define SWD_JTAG_MIN_CLOCK_LEVEL_18                  180000
#define SWD_JTAG_MIN_CLOCK_LEVEL_19                  150000
#define SWD_JTAG_MIN_CLOCK_LEVEL_20                  130000
#define SWD_JTAG_MIN_CLOCK_LEVEL_21                  100000
#define SWD_JTAG_MIN_CLOCK_LEVEL_22                  85000
#define SWD_JTAG_MIN_CLOCK_LEVEL_23                  60000
#define SWD_JTAG_MIN_CLOCK_LEVEL_24                  50000
#define SWD_JTAG_MIN_CLOCK_LEVEL_25                  30000
#define SWD_JTAG_MIN_CLOCK_LEVEL_26                  20000
#define SWD_JTAG_MIN_CLOCK_LEVEL_27                  10000

#define SWD_JTAG_MAX_DELAY_LEVEL_CNT                 19
#define SWD_JTAG_CLOCK_NORMAL_LEVEL_CNT              11

typedef struct _DelayInfo {
    uint32_t delayLevel;
    uint32_t baseValue;
    uint32_t average;
} DelayInfo;

typedef struct _ClockLevelInfo {
    uint32_t minClock;
    uint32_t level;
} ClockLevelInfo;

/* All data in the following tables are based on actual measurements. */
const DelayInfo g_delayInfo[SWD_JTAG_MAX_DELAY_LEVEL_CNT] = {
    {SWD_JTAG_MIN_CLOCK_LEVEL_10, 7, 200000},  /* base value is 7, average is 200000 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_11, 9, 100000},  /* base value is 9, average is 100000 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_12, 12, 66000},  /* base value is 12, average is 66000 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_13, 15, 33000},  /* base value is 15, average is 33000 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_14, 20, 20000},  /* base value is 20, average is 20000 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_15, 27, 14285},  /* base value is 27, average is 14285 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_16, 33, 8333},   /* base value is 33, average is 8333 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_17, 41, 6250},   /* base value is 41, average is 6250 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_18, 46, 4000},   /* base value is 46, average is 4000 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_19, 55, 3333},   /* base value is 55, average is 3333 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_20, 65, 2200},   /* base value is 65, average is 2200 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_21, 85, 1200},   /* base value is 85, average is 1200 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_22, 100, 1000},  /* base value is 100, average is 1000 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_23, 140, 625},   /* base value is 140, average is 625 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_24, 170, 333},   /* base value is 170, average is 333 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_25, 280, 182},   /* base value is 280, average is 182 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_26, 400, 83},    /* base value is 400, average is 83 */
    {SWD_JTAG_MIN_CLOCK_LEVEL_27, 900, 20},    /* base value is 900, average is 20 */
    {0, 1500, 20}                              /* base value is 1500, average is 20 */
};

const ClockLevelInfo g_clockLevelInfo[SWD_JTAG_CLOCK_NORMAL_LEVEL_CNT] = {
    {SWD_JTAG_MIN_CLOCK_LEVEL_0, SWD_JTAG_CLOCK_LEVEL_1},
    {SWD_JTAG_MIN_CLOCK_LEVEL_1, SWD_JTAG_CLOCK_LEVEL_1},
    {SWD_JTAG_MIN_CLOCK_LEVEL_2, SWD_JTAG_CLOCK_LEVEL_2},
    {SWD_JTAG_MIN_CLOCK_LEVEL_3, SWD_JTAG_CLOCK_LEVEL_3},
    {SWD_JTAG_MIN_CLOCK_LEVEL_4, SWD_JTAG_CLOCK_LEVEL_4},
    {SWD_JTAG_MIN_CLOCK_LEVEL_5, SWD_JTAG_CLOCK_LEVEL_5},
    {SWD_JTAG_MIN_CLOCK_LEVEL_6, SWD_JTAG_CLOCK_LEVEL_6},
    {SWD_JTAG_MIN_CLOCK_LEVEL_7, SWD_JTAG_CLOCK_LEVEL_7},
    {SWD_JTAG_MIN_CLOCK_LEVEL_8, SWD_JTAG_CLOCK_LEVEL_8},
    {SWD_JTAG_MIN_CLOCK_LEVEL_9, SWD_JTAG_CLOCK_LEVEL_9},
    {SWD_JTAG_MIN_CLOCK_LEVEL_10, SWD_JTAG_CLOCK_LEVEL_LOW}
};

DAP_Data_t g_DAP_Data = {
    .swd_conf.turnaround = 1,
    .transfer.idle_cycles = 0,
    .swd_conf.data_phase  = 0,
    .clock_level = SWD_JTAG_CLOCK_LEVEL_6,
    .clock_delay = 1,
    .jtag_dev.index = 0,
    .jtag_dev.count = 1,
    .jtag_dev.ir_length[0] = 4, /* ir length is 4 */
    .jtag_dev.ir_before[0] = 0,
    .jtag_dev.ir_after[0] = 0,
};

static void SwdJtagDelaySet(uint32_t clock)
{
    uint32_t baseValue;
    uint32_t offsetValue;

    for (int i = 0; i < SWD_JTAG_MAX_DELAY_LEVEL_CNT; i++) {
        if (clock >= g_delayInfo[i].delayLevel) {
            baseValue = g_delayInfo[i].baseValue;
            offsetValue = (clock - g_delayInfo[i].delayLevel) / g_delayInfo[i].average;
            g_DAP_Data.clock_delay = baseValue - offsetValue;
            break;
        }
    }
}

int SwdTurnaroundSet(uint8_t turnaround)
{
    g_DAP_Data.swd_conf.turnaround = turnaround;
    return 0;
}

uint8_t SwdTurnaroundGet(void)
{
    return g_DAP_Data.swd_conf.turnaround;
}

int SwdDataPhaseSet(uint8_t phase)
{
    g_DAP_Data.swd_conf.data_phase = phase;
    return 0;
}

uint8_t SwdDataPhaseGet(void)
{
    return g_DAP_Data.swd_conf.data_phase;
}

int SwdJtagIdleCyclesSet(uint8_t cycles)
{
    g_DAP_Data.transfer.idle_cycles = cycles;
    return 0;
}

uint8_t SwdJtagIdleCyclesGet(void)
{
    return g_DAP_Data.transfer.idle_cycles;
}

int SwdJtagClockLevelSet(uint32_t clock)
{
    for (int i = 0; i < SWD_JTAG_CLOCK_NORMAL_LEVEL_CNT; i++) {
        if (clock >= g_clockLevelInfo[i].minClock) {
            g_DAP_Data.clock_level = g_clockLevelInfo[i].level;
            break;
        }
    }
    if (g_DAP_Data.clock_level == SWD_JTAG_CLOCK_LEVEL_LOW) {
        SwdJtagDelaySet(clock);
    }
    return 0;
}

uint32_t SwdJtagClockLevelGet(void)
{
    return g_DAP_Data.clock_level;
}

int SwdJtagClockDelaySet(uint32_t delay)
{
    g_DAP_Data.clock_delay = delay;
    return 0;
}

uint32_t SwdJtagClockDelayGet(void)
{
    return g_DAP_Data.clock_delay;
}

int SwdJtagDebugPortSet(uint8_t port)
{
    g_DAP_Data.debug_port = port;
    return 0;
}

uint8_t SwdJtagDebugPortGet(void)
{
    return g_DAP_Data.debug_port;
}
