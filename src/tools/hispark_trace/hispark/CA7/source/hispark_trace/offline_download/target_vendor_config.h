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
  * @file    target_vendor_config.h
  * @author  MCU Driver Team
  * @brief   target vendor config driver.
  */
#ifndef TARGET_VENDOR_CONFIG_H
#define TARGET_VENDOR_CONFIG_H

#define VENDOR_CONFIG_NAME_LEN                  20
#define VENDOR_CONFIG_VENDOR_NAME_NUM           7

#define VENDOR_CONFIG_HI_MODEL_NAME_NUM         3
#define VENDOR_CONFIG_ST_MODEL_NAME_NUM         3
#define VENDOR_CONFIG_TI_MODEL_NAME_NUM         1
#define VENDOR_CONFIG_RENESAS_MODEL_NAME_NUM    1
#define VENDOR_CONFIG_HM_MODEL_NAME_NUM         6
#define VENDOR_CONFIG_AUCU_MODEL_NAME_NUM       4

#define VENDOR_CONFIG_FLASH_TYPE_NAME_NUM       2
#define VENDOR_CONFIG_FLASH_REGION_INFO_NUM     4

extern char g_vendorName[VENDOR_CONFIG_VENDOR_NAME_NUM][VENDOR_CONFIG_NAME_LEN];

extern char g_hiModelName[VENDOR_CONFIG_HI_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN];

extern char g_stModelName[VENDOR_CONFIG_ST_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN];

extern char g_tiModelName[VENDOR_CONFIG_TI_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN];

extern char g_renesasModelName[VENDOR_CONFIG_RENESAS_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN];

extern char g_vendorHMModelName[VENDOR_CONFIG_HM_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN];

extern char g_aucuModelName[VENDOR_CONFIG_AUCU_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN];

extern char g_flashTypeName[VENDOR_CONFIG_FLASH_TYPE_NAME_NUM][VENDOR_CONFIG_NAME_LEN];
extern char g_regionInfo[VENDOR_CONFIG_FLASH_REGION_INFO_NUM][VENDOR_CONFIG_NAME_LEN];

/* vendor ID */
#define TARGET_VENDOR_HISILICON          0x00000001
#define TARGET_VENDOR_ST                 0x00000002
#define TARGET_VENDOR_TI                 0x00000003
#define TARGET_VENDOR_RENESAS            0x00000004
#define TARGET_VENDOR_HM                 0x00000005
#define TARGET_VENDOR_AUCU               0x00000006

/* model ID */

/* hisilicon */
#define TARGET_MODEL_HI3065             0x00000001
#define TARGET_MODEL_HM3061             0x00000002

/* ST */
#define TARGET_MODEL_STM32F103RB        0x00000001
#define TARGET_MODEL_STM32F407          0x00000002

/* TI */

/* Renesas */

/* vendorHM */
#define TARGET_MODEL_3065HRPIRZ      0x00000001
#define TARGET_MODEL_3065HRPICZ      0x00000002
#define TARGET_MODEL_3061HRPIKZ      0x00000003

/* AUCU */
#define TARGET_MODEL_AG32R2080LF    0x00000001
#define TARGET_MODEL_AG32R2160NF    0x00000002
#define TARGET_MODEL_AG32R2160PF    0x00000003


/* flash ID */
#define TARGET_FLASH_INCHIP                0x00000000
#define TARGET_FLASH_OUTCHIP               0x00000001

#endif /* #ifndef TARGET_VENDOR_CONFIG_H */