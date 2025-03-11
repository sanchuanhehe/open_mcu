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
  * @file    config_storage_update.h
  * @author  MCU Driver Team
  * @brief   config storage update driver.
  */
#ifndef CONFIG_STORAGE_UPDATE_H
#define CONFIG_STORAGE_UPDATE_H

#include <stdint.h>
#include "target_lib_manager.h"

#define CONFIG_LOAD_TYPE_UNKNOW                               0
#define CONFIG_LOAD_TYPE_INTERFACE                            1
#define CONFIG_LOAD_TYPE_BOOT                                 2
#define CONFIG_LOAD_TYPE_IMAGE                                3
#define CONFIG_LOAD_TYPE_ALGO                                 4
#define CONFIG_LOAD_TYPE_TARGET                               5

#define CONFIG_FLASH_BASS_ADDR                                0x10048000

#define CONFIG_STRAT_FLAG_A                                   0xFFFFFFFF
#define CONFIG_STRAT_FLAG_B                                   0
#define CONFIG_STORAGE_MAX_BOOT_SIZE                          0x8000
#define CONFIG_STORAGE_MAX_INFACR_SIZE                        0x100000
#define CONFIG_STORAGE_ONE_IMAGE_MAX_SIZE                     TARGET_LIB_ONE_IMAGE_SIZE
#define CONFIG_STORAGE_ONE_ALGO_MAX_SIZE                      TARGET_LIB_ONE_ALGO_SIZE

#define CONFIG_STORAGE_MAX_SIZE                              0x2000
#define CONFIG_STORAGE_UPDATE_BASE_ADDR                     (0x10048000 + 0x8000)
#define CONFIG_STORAGE_UPDATE_BACKUP_BASE_ADDR              (CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_MAX_SIZE)

#define CONFIG_STORAGE_UPDATE_ALGO_LIST_OFFSET                0x0
#define CONFIG_STORAGE_UPDATE_ALGO_LIST_SIZE                  0x800

#define CONFIG_STORAGE_UPDATE_IMAGE_LIST_OFFSET               0x800
#define CONFIG_STORAGE_UPDATE_IMAGE_LIST_SIZE                 0x800

#define CONFIG_STORAGE_UPDATE_STARTUP_FLAG_OFFSET             0x1000
#define CONFIG_STORAGE_UPDATE_STARTUP_FLAG_SIZE               0x4

#define CONFIG_STORAGE_UPDATE_FREE_ALGO_FLAG_ARRY_OFFSET      0x1004
#define CONFIG_STORAGE_UPDATE_FREE_ALGO_FLAG_ARRY_SIZE        0x50

#define CONFIG_STORAGE_UPDATE_FREE_IMAGE_FLAG_ARRY_OFFSET     0x1054   /* 4 * 20 byte */
#define CONFIG_STORAGE_UPDATE_FREE_IMAGE_FLAG_ARRY_SIZE       0x28

#define CONFIG_STORAGE_UPDATE_FREE_FACTORY_IMAGE_FLAG_ARRY_OFFSET     0x107C   /* 4 * 10 byte */
#define CONFIG_STORAGE_UPDATE_FREE_FACTORY_IMAGE_FLAG_ARRY_SIZE       0x28

#define CONFIG_STORAGE_UPDATE_CURRENT_ALGO_INDEX_OFFSET       0x10A4
#define CONFIG_STORAGE_UPDATE_CURRENT_ALGO_INDEX_SIZE         0x4

#define CONFIG_STORAGE_UPDATE_FACTORY_ALGO_LIST_OFFSET        0x1100
#define CONFIG_STORAGE_UPDATE_FACTORY_ALGO_LIST_SIZE          0xD0

#define CONFIG_STORAGE_UPDATE_FACTORY_FLAG_OFFSET            0x12D0
#define CONFIG_STORAGE_UPDATE_FACTORY_FLAG_SIZE              0x4

#define CONFIG_STORAGE_UPDATE_POWER_STATUS_FLAG_OFFSET      0x12D4
#define CONFIG_STORAGE_UPDATE_POWER_STATUS_FLAG_SIZE        0x4


/* No add base config addr */
#define CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LIST_OFFSET       0x210000
#define CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LIST_SIZE         0x800

#define CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET   0x20C000
#define CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_SIZE     0x1000

#define CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET   0x20E000
#define CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE     0x1000
int32_t ConfigStartFlagSave(uint32_t flag);
uint32_t ConfigStartFlagRead(void);

#endif /* #ifndef CONFIG_STORAGE_UPDATE_H */
