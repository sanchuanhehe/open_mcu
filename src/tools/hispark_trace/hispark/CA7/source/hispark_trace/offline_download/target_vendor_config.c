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
  * @file    target_vendor_config.c
  * @author  MCU Driver Team
  * @brief   target vendor config driver.
  */
#include "target_vendor_config.h"

char g_vendorName[VENDOR_CONFIG_VENDOR_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "Unknow_",
    "Hisilicon_",
    "Stm32_",
    "Ti_",
    "Renesas_",
    "VendorHM_",
    "AUCU_",
};

char g_hiModelName[VENDOR_CONFIG_HI_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "unknow",
    "3065",
    "3061",
};

char g_stModelName[VENDOR_CONFIG_ST_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "unknow",
    "f103rb",
    "f407",
};

char g_tiModelName[VENDOR_CONFIG_TI_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "unknow",
};

char g_renesasModelName[VENDOR_CONFIG_RENESAS_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "unknow",
};

char g_vendorHMModelName[VENDOR_CONFIG_HM_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "unknow",
    "3065HRPIRZ",
    "3065HRPICZ",
    "3061HRPIKZ",
    "3061MRPIKB",
    "3061MRPICB",
};

char g_aucuModelName[VENDOR_CONFIG_AUCU_MODEL_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "unknow",
    "AU301LDF51",
    "AU302NDF51",
    "AU302PDF51",
};

char g_flashTypeName[VENDOR_CONFIG_FLASH_TYPE_NAME_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "_inchip",
    "_outchip",
};

char g_regionInfo[VENDOR_CONFIG_FLASH_REGION_INFO_NUM][VENDOR_CONFIG_NAME_LEN] = {
    "_Main_rgn0",
    "_Main_rgn1",
    "_info_rgn1",
    "3",
};