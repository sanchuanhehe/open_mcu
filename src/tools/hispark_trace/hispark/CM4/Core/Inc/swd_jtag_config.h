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
  * @file    swd_jtag_config.h
  * @author  MCU Driver Team
  * @brief   swd and jtag config.
  */
#ifndef SWD_JTAG_CONFIG_H
#define SWD_JTAG_CONFIG_H

#include <stdint.h>
#include "DAP_config.h"
#include "DAP.h"

#define SWD_JTAG_CLOCK_LEVEL_0                       0
#define SWD_JTAG_CLOCK_LEVEL_1                       1
#define SWD_JTAG_CLOCK_LEVEL_2                       2
#define SWD_JTAG_CLOCK_LEVEL_3                       3
#define SWD_JTAG_CLOCK_LEVEL_4                       4
#define SWD_JTAG_CLOCK_LEVEL_5                       5
#define SWD_JTAG_CLOCK_LEVEL_6                       6
#define SWD_JTAG_CLOCK_LEVEL_7                       7
#define SWD_JTAG_CLOCK_LEVEL_8                       8
#define SWD_JTAG_CLOCK_LEVEL_9                       9
#define SWD_JTAG_CLOCK_LEVEL_LOW                     300

extern DAP_Data_t g_DAP_Data;

int SwdTurnaroundSet(uint8_t turnaround);
uint8_t SwdTurnaroundGet(void);
int SwdDataPhaseSet(uint8_t phase);
uint8_t SwdDataPhaseGet(void);

int SwdJtagIdleCyclesSet(uint8_t cycles);
uint8_t SwdJtagIdleCyclesGet(void);
int SwdJtagClockLevelSet(uint32_t clock);
uint32_t SwdJtagClockLevelGet(void);
int SwdJtagClockDelaySet(uint32_t clock);
uint32_t SwdJtagClockDelayGet(void);
int SwdJtagDebugPortSet(uint8_t port);
uint8_t SwdJtagDebugPortGet(void);
#endif
