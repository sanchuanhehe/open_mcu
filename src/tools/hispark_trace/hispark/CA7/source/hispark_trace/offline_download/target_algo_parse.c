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
  * @file    target_algo_parse.c
  * @author  MCU Driver Team
  * @brief   offline sys config driver.
  */
#include <stdbool.h>
#include <string.h>
#include "securec.h"
#include "target_config.h"
#include "flash_blob.h"
#include "FlashPrg.h"
#include "target_lib_manager.h"
#include "factory_manager.h"
#include "debug.h"
#include "target_algo_parse.h"

static uint8_t g_targetAlgoParseBuff[TARGET_ALGO_PARSE_BUFF_SIZE];
static program_target_t newFlash;

/**
  * @brief Target algorithm parsing
  * @retval None
  */
static void TargetAlgoParse(void)
{
    /* Obtaining the Offset Address of the API Interface of the Target Flash Driver */
    newFlash.init = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_INIT_FUN_OFFSET);
    newFlash.uninit = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_UNINIT_FUN_OFFSET);
    newFlash.erase_chip = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_ERASE_CHIP_FUN_OFFSET);
    newFlash.erase_sector = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_ERASE_SECTOR_FUN_OFFSET);
    newFlash.program_page = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_PRG_PAGE_FUN_OFFSET);
    newFlash.verify = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_VERIFY_FUN_OFFSET);
    newFlash.read = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_READ_FUN_OFFSET);
    /* Breakpoint Pointer Offset Address */
    newFlash.sys_call_s.breakpoint = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_BK_ADDR_OFFSET);
    /* Stack Pointer Offset Address */
    newFlash.sys_call_s.static_base = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_STATIC_BASE_OFFSET);
    newFlash.sys_call_s.stack_pointer = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_STACK_POINTER_OFFSET);
    /* Start address of the flash algorithm running data buffer */
    newFlash.program_buffer = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_PRG_BUFF_OFFSET);
    /* Start address of the RAM for running the flash algorithm */
    newFlash.algo_start = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_RAM_START_OFFSET);
    /* Binary file size of the flash algorithm */
    newFlash.algo_size = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_FLASH_ALGO_LEN_OFFSET);
    /* Start address for storing binary files in the flash algorithm */
    newFlash.algo_blob = (uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_ALGO_BLOB_OFFSET);
    /* Cache size of the running data of the flash algorithm */
    newFlash.program_buffer_size = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_FLASH_SECTOR_SIZE_OFFSET);
    /* Flash information of the target chip */
    target_device.sectors_info->start = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_FLASH_START_OFFSET);
    target_device.sectors_info->size = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_FLASH_SIZE_OFFSET);
    target_device.sector_info_length = 1;
    target_device.flash_regions[0].start = *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_FLASH_START_OFFSET);
    target_device.flash_regions[0].flags = kRegionIsDefault;
    target_device.flash_regions[0].flash_algo = (program_target_t *) &newFlash;
    /* Start address of the RAM for running the flash algorithm of the target chip */
    target_device.ram_regions[0].start =  *(uint32_t*)(g_targetAlgoParseBuff + TARGET_ALGO_PARSE_RAM_START_OFFSET);
}

/**
  * @brief Configure the target algorithm based on the algorithm index.
  * @param algoIndex Index of the algorithm file in the list
  * @retval Configuration Result
  */
int32_t TargetAlgoConfig(int32_t algoIndex)
{
    TargetAlgoLibInfo newAlgoInfo;
    int32_t ret;
    errno_t rc = EOK;
    /* Configure the current burning algorithm based on the index. */
    ret = TargetLibAlgoSelect(algoIndex);
    if (ret != TARGET_LIB_OK) {
        return TARGET_ALGO_PARSE_ERROR;
    }
    /* Obtains the current programming algorithm information. */
    ret = TargetLibCurrentAlgoGet(&newAlgoInfo);
    if (ret != TARGET_LIB_OK || newAlgoInfo.fileStatus == TARGET_LIB_FILE_STATUS_BAD) {
        return TARGET_ALGO_PARSE_ERROR;
    }

    rc = memset_s(g_targetAlgoParseBuff, sizeof(g_targetAlgoParseBuff), 0xFF, sizeof(g_targetAlgoParseBuff));
    if (rc != EOK) {
        return TARGET_ALGO_PARSE_ERROR;
    }
    Init(0, 0, 0);
    /* Read the algorithm file from the debugger and parse it. */
    if (FlashRead(newAlgoInfo.startAddr, newAlgoInfo.info.fileSize, g_targetAlgoParseBuff) != 0) {
            return TARGET_ALGO_PARSE_ERROR;
    }

    /* Verifying */
    UserCrcInit(0, newAlgoInfo.info.fileCrc32);
    UserCrc32(g_targetAlgoParseBuff, newAlgoInfo.info.fileSize);
    if (IsCheckSuccess() == 0) {
        return TARGET_ALGO_PARSE_ERROR;
    }

    TargetAlgoParse();
    return TARGET_ALGO_PARSE_OK;
}