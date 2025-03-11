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
  * @file      file_id_defs.h
  * @author    MCU Driver Team
  * @brief     file id module driver
  * @details   The header file contains the following declaration:
  *             +Definition of miniaturized log event IDs
  */
#ifndef FILE_ID_DEFS_H
#define FILE_ID_DEFS_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/**
  * @addtogroup DEBUG_Log
  * @brief DEBUG external module.
  * @{
  */

 /**
  * @defgroup FILE_ID_DEFS_Def FILE_ID_DEFS_Def
  * @brief Define source files and ID.
  * @{
  */

typedef enum {
    FILE_ID_LOG_C = 2001, /* this is a test sample */
    FILE_ID_DFX_CRASH_C = 2002,
    FILE_ID_INTERRUPT_C = 2003,
    FILE_ID_SAMPLE_C = 2004,
    FILE_ID_DRV_INTERRUPT_C = 2005,
    FILE_ID_DRV_ASM_C = 3001,
    FILE_ID_DRV_XFE_C = 3002,
} file_id_enum;

#ifdef __cplusplus
#if __cplusplus
    }
#endif
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif /* FILE_ID_DEFS_H */
