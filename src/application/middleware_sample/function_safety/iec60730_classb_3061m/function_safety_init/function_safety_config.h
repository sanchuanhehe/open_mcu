/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      function_safety_config.h
  * @author    MCU Driver Team
  * @brief     This file contains the config of function safety
  */

#ifndef FUNCTION_SAFETY_CONFIG_H
#define FUNCTION_SAFETY_CONFIG_H
/* ---------------------------------------- INCLUDE ----------------------------------- */
/* ----------------------------------------PROCESS------------------------------------ */
#define USE_FUNCTION_SAFETY_BENCHMARK
#define USE_FUNCTION_SAFETY_REPORT
/* ----------------------------------------RAM Test------------------------------------ */
/* Parameters are used during the RAM test, performed using March-C Algorithm */
#define BCKGRND_PATTERN                     0x00000000U
#define INV_BCKGRND_PATTERN                 0xFFFFFFFFU

/* Constants necessary for execution initial March test */
#define STARTUP_RAM_START                   ((unsigned int *)0x04000000U)
#define STARTUP_RAM_END                     ((unsigned int *)0x04008000U)

/* Constants necessary for execution of transparent run time March tests */
#define RUNTIME_RAM_START                   ((unsigned int *)0x04000000U)
#define RUNTIME_RAM_END                     ((unsigned int *)0x04008000U)

/* ------------------------------------------ROM Test------------------------------------ */
/* CRC checks the ROM region result */
#define ROM_START                           ((unsigned int *)0x03000000U)
#define FLASH_BLOCK_SIZE_IN_WORD            4
#define EFLASH_ECC_INT_ENABLE_VALUE         0x000C0000
/* ------------------------------------------AIO Test------------------------------------ */

#endif