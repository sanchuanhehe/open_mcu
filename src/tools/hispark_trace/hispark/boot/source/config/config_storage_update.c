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
  * @file    config_storage_update.c
  * @author  MCU Driver Team
  * @brief   config storage update driver.
  */
#include <stddef.h>
#include <stdbool.h>
#include "FlashPrg.h"
#include "config_storage_update.h"

#define ONE_PAGE_SIZE    256

/**
  * @brief Save Startup Flag.
  * @param flag Startup Flag
  * @retval Success or Failure Result
  */
int32_t ConfigStartFlagSave(uint32_t flag)
{
    bool algoFreeFlag[TARGET_LIB_ALGO_MAX_NUM];
    bool imageFreeFlag[TARGET_LIB_IMAGE_MAX_NUM];
    bool factoryImageFreeFlag[TARGET_LIB_FACTORY_IMAGE_MAX_NUM];
    uint32_t algoIndex;
    uint32_t addr;
    uint32_t factoryFlag;
    uint32_t powerStatus;

    UnInit(0);
    Init(0, 0, 0);
    /* Backing Up Data */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_CURRENT_ALGO_INDEX_OFFSET;
    FlashRead(addr, sizeof(algoIndex), (uint8_t *)&algoIndex);
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_ALGO_FLAG_ARRY_OFFSET;
    FlashRead(addr, sizeof(algoFreeFlag), (uint8_t *)algoFreeFlag);
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_IMAGE_FLAG_ARRY_OFFSET;
    FlashRead(addr, sizeof(imageFreeFlag), (uint8_t *)imageFreeFlag);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_FLAG_OFFSET;
    FlashRead(addr, sizeof(factoryFlag), (uint8_t *)&factoryFlag);
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_FACTORY_IMAGE_FLAG_ARRY_OFFSET;
    FlashRead(addr, sizeof(factoryImageFreeFlag), (uint8_t *)factoryImageFreeFlag);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_POWER_STATUS_FLAG_OFFSET;
    FlashRead(addr, sizeof(powerStatus), (uint8_t *)&powerStatus);
    /* Erasing the flash memory of the old data storage area */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_STARTUP_FLAG_OFFSET;
    EraseSector(addr);
    /* Restoring Data */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_CURRENT_ALGO_INDEX_OFFSET;
    ProgramPage(addr, sizeof(algoIndex), (uint32_t *)&algoIndex);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_ALGO_FLAG_ARRY_OFFSET;
    ProgramPage(addr, sizeof(algoFreeFlag), (uint32_t *)algoFreeFlag);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_FACTORY_IMAGE_FLAG_ARRY_OFFSET;
    ProgramPage(addr, sizeof(factoryImageFreeFlag), (uint32_t *)factoryImageFreeFlag);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_IMAGE_FLAG_ARRY_OFFSET;
    ProgramPage(addr, sizeof(imageFreeFlag), (uint32_t *)imageFreeFlag);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_FLAG_OFFSET;
    ProgramPage(addr, sizeof(factoryFlag), (uint32_t *)&factoryFlag);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_POWER_STATUS_FLAG_OFFSET;
    ProgramPage(addr, sizeof(powerStatus), (uint32_t *)&powerStatus);
    /* Update data */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_STARTUP_FLAG_OFFSET;
    ProgramPage(addr, sizeof(flag), (uint32_t *)&flag);
    UnInit(0);
    return 0;
}

/**
  * @brief Read Startup Flag.
  * @retval Startup Flag
  */
uint32_t ConfigStartFlagRead(void)
{
    uint32_t addr;
    uint32_t startFlag;

    UnInit(0);
    Init(0, 0, 0);
    /* Read start flag */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_STARTUP_FLAG_OFFSET;
    FlashRead(addr, sizeof(startFlag), (uint8_t *)&startFlag);
    UnInit(0);
    return startFlag;
}
