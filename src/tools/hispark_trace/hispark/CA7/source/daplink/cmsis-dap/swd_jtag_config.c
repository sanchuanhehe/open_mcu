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
#include "DAP_config.h"
#include "DAP.h"
#include "config_storage_update.h"
#include "swd_jtag_config.h"

#define SWD_JTAG_MAX_LEVEL_CNT                       11

#define SWD_JTAG_MIN_CLOCK_LEVEL_0                   6000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_1                   5000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_2                   4000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_3                   3000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_4                   2000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_5                   1000000
#define SWD_JTAG_MIN_CLOCK_LEVEL_6                   800000
#define SWD_JTAG_MIN_CLOCK_LEVEL_7                   500000
#define SWD_JTAG_MIN_CLOCK_LEVEL_8                   300000
#define SWD_JTAG_MIN_CLOCK_LEVEL_9                   100000

#define SWD_JTAG_CLOCK_VAL_0                         6000000
#define SWD_JTAG_CLOCK_VAL_1                         5000000
#define SWD_JTAG_CLOCK_VAL_2                         4000000
#define SWD_JTAG_CLOCK_VAL_3                         3000000
#define SWD_JTAG_CLOCK_VAL_4                         2000000
#define SWD_JTAG_CLOCK_VAL_5                         1000000
#define SWD_JTAG_CLOCK_VAL_6                         800000
#define SWD_JTAG_CLOCK_VAL_7                         500000
#define SWD_JTAG_CLOCK_VAL_8                         300000
#define SWD_JTAG_CLOCK_VAL_9                         100000

typedef struct {
    uint32_t clockLevel;
    uint32_t minClockLevel;
    uint32_t clockVal;
} SwdJtagClockCfg;

const SwdJtagClockCfg g_swdJtagClockCfgList[SWD_JTAG_MAX_LEVEL_CNT] = {
    {SWD_JTAG_CLOCK_LEVEL_0, SWD_JTAG_MIN_CLOCK_LEVEL_0, SWD_JTAG_CLOCK_VAL_0},
    {SWD_JTAG_CLOCK_LEVEL_1, SWD_JTAG_MIN_CLOCK_LEVEL_1, SWD_JTAG_CLOCK_VAL_1},
    {SWD_JTAG_CLOCK_LEVEL_2, SWD_JTAG_MIN_CLOCK_LEVEL_2, SWD_JTAG_CLOCK_VAL_2},
    {SWD_JTAG_CLOCK_LEVEL_3, SWD_JTAG_MIN_CLOCK_LEVEL_3, SWD_JTAG_CLOCK_VAL_3},
    {SWD_JTAG_CLOCK_LEVEL_4, SWD_JTAG_MIN_CLOCK_LEVEL_4, SWD_JTAG_CLOCK_VAL_4},
    {SWD_JTAG_CLOCK_LEVEL_5, SWD_JTAG_MIN_CLOCK_LEVEL_5, SWD_JTAG_CLOCK_VAL_5},
    {SWD_JTAG_CLOCK_LEVEL_6, SWD_JTAG_MIN_CLOCK_LEVEL_6, SWD_JTAG_CLOCK_VAL_6},
    {SWD_JTAG_CLOCK_LEVEL_7, SWD_JTAG_MIN_CLOCK_LEVEL_7, SWD_JTAG_CLOCK_VAL_7},
    {SWD_JTAG_CLOCK_LEVEL_8, SWD_JTAG_MIN_CLOCK_LEVEL_8, SWD_JTAG_CLOCK_VAL_8},
    {SWD_JTAG_CLOCK_LEVEL_9, SWD_JTAG_MIN_CLOCK_LEVEL_9, SWD_JTAG_CLOCK_VAL_9},
    {SWD_JTAG_CLOCK_LEVEL_LOW, 0, SWD_JTAG_CLOCK_VAL_9}
};

int SwdTurnaroundSet(int turnaround)
{
    DAP_Data.swd_conf.turnaround = turnaround;
    return 0;
}

int SwdTurnaroundGet(void)
{
    return DAP_Data.swd_conf.turnaround;
}

int SwdDataPhaseSet(int phase)
{
    DAP_Data.swd_conf.data_phase = phase;
    return 0;
}

int SwdDataPhaseGet(void)
{
    return DAP_Data.swd_conf.data_phase;
}

int SwdIdleCyclesSet(int cycles)
{
    DAP_Data.transfer.idle_cycles = cycles;
    return 0;
}

int SwdIdleCyclesGet(void)
{
    return DAP_Data.transfer.idle_cycles;
}

int SwdJtagClockLevelSet(int clock)
{
    DAP_Data.clock_delay = 1;
    for (int i = 0; i < SWD_JTAG_MAX_LEVEL_CNT; i++) {
        if (clock >= g_swdJtagClockCfgList[i].minClockLevel) {
            DAP_Data.clock_level = g_swdJtagClockCfgList[i].clockLevel;
            break;
        }
    }
    return 0;
}

uint32_t SwdJtagClockGet(void)
{
    uint32_t currentClock;
    for (int i = 0; i < SWD_JTAG_MAX_LEVEL_CNT; i++) {
        if (g_swdJtagClockCfgList[i].clockLevel == DAP_Data.clock_level) {
            currentClock = g_swdJtagClockCfgList[i].clockVal;
            break;
        }
    }
    return currentClock;
}

int SwJtagClockLevelLevelSet(int level)
{
    DAP_Data.clock_level = level;
    return 0;
}

int SwdJtagClockLevelGet(void)
{
    return DAP_Data.clock_level;
}

int SwdJtagClockDelaySet(uint32_t clock)
{
    DAP_Data.clock_delay = clock;
    return 0;
}

int SwdJtagClockDelayGet(void)
{
    return DAP_Data.clock_delay;
}

uint8_t SwdJtagDebugPortGet(void)
{
    uint32_t port;

    port = ConfigDebugPorFlagRead();
    switch (port) {
        case DAP_PORT_JTAG:
            DAP_Data.debug_port = DAP_PORT_JTAG;
            break;
        case DAP_PORT_SWD:
            DAP_Data.debug_port = DAP_PORT_SWD;
            break;
        default:
            DAP_Data.debug_port = DAP_PORT_SWD;
            port = DAP_PORT_SWD;
            ConfigDebugPortFlagSave(port);
            break;
    }
    return DAP_Data.debug_port;
}

int SwdJtagDebugPortSet(uint8_t port)
{
    uint32_t tmpPort = port;
    DAP_Data.debug_port = port;
    ConfigDebugPortFlagSave(tmpPort);
    return 0;
}
