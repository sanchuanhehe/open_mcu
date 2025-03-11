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
#include <string.h>
#include "securec.h"
#include "FlashPrg.h"
#include "config_storage_update.h"

#define ONE_PAGE_SIZE    256

#define ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE    (TARGET_LIB_FACTORY_IMAGE_MAX_NUM * 4)
#define ONE_IMAGE_LOAD_CNT_LIST_SIZE            (TARGET_LIB_IMAGE_MAX_NUM * 4)
#define ALL_CFG_MEM_LEN              (CONFIG_STORAGE_UPDATE_CFG_END_OFFSET - CONFIG_STORAGE_UPDATE_CFG_START_OFFSET)

static int g_configLoadType = CONFIG_LOAD_TYPE_UNKNOW;
static int g_configLoadEndAddr = 0;
static uint32_t g_configLoadFileSize = 0;
static uint32_t g_imageLoadCntListStartAddr = CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET;
static uint32_t g_factoryImageLoadCntListStartAddr = CONFIG_FLASH_BASS_ADDR + \
                                                     CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET;

/**
  * @brief Flash segment storage processing
  * @param addr Storage start address
  * @param buf Pointer to stored data
  * @param len Storage data length
  * @retval None
  */
static void ProgramPageHandle(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t pageCnt;
    uint32_t tempAddr = addr;
    uint32_t sz = len;
    uint8_t *tempBuf = buf;

    pageCnt = sz / ONE_PAGE_SIZE;
    for (uint32_t i = 0; i < pageCnt; i++) {
        /* Full page writing */
        ProgramPage(tempAddr, ONE_PAGE_SIZE, (uint32_t *)tempBuf);
        tempAddr += ONE_PAGE_SIZE;
        sz -= ONE_PAGE_SIZE;
        tempBuf += ONE_PAGE_SIZE;
    }
    if (sz > 0) {
        /* Finally, write less than one page of data. */
        ProgramPage(tempAddr, sz, (uint32_t *)tempBuf);
    }
}

/**
  * @brief Update the configuration.
  * @param data Data to be updated
  * @param dataSize Data length
  * @param offset Offset position
  * @retval None
  */
static void ConfigUpdate(const uint8_t *data, uint32_t dataSize, uint32_t offset)
{
    uint8_t cfgBuff[ALL_CFG_MEM_LEN] = {0};
    uint32_t addr;
    errno_t rc = EOK;

    UnInit(0);
    Init(0, 0, 0);
    /* Backup image list. */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    FlashRead(addr, ALL_CFG_MEM_LEN, (uint8_t *)cfgBuff);
    rc = memcpy_s((cfgBuff + offset), dataSize, data, dataSize);
    if (rc != EOK) {
        UnInit(0);
        return;
    }
    EraseSector(addr);
    ProgramPageHandle(addr, cfgBuff, ALL_CFG_MEM_LEN);
    UnInit(0);
}

/**
  * @brief Save the algorithm list.
  * @param list Algorithm list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigAlgoListSave(TargetAlgoHandle *list, uint32_t listSize)
{
    TargetImageHandle tempImage;
    uint32_t addr;
    uint32_t sz;
    uint8_t *buf = NULL;

    UnInit(0);
    Init(0, 0, 0);
    /* Backup image list. */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LIST_OFFSET;
    FlashRead(addr, sizeof(tempImage), (uint8_t *)&tempImage);

    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_ALGO_LIST_OFFSET;
    EraseSector(addr);
    /* save algo */
    sz = listSize;
    buf = (uint8_t*)list;
    ProgramPageHandle(addr, buf, sz);

    /* save image */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LIST_OFFSET;
    sz = sizeof(tempImage);
    buf = (uint8_t*)&tempImage;
    ProgramPageHandle(addr, buf, sz);

    UnInit(0);
    return 0;
}

/**
  * @brief Read the algorithm list.
  * @param list Algorithm list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigAlgoListRead(TargetAlgoHandle *list, uint32_t listSize)
{
    uint32_t addr;

    if (list == NULL) {
        return -1;
    }

    UnInit(0);
    Init(0, 0, 0);
    /* Read algorithm list */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_ALGO_LIST_OFFSET;
    FlashRead(addr, listSize, (uint8_t *)list);
    UnInit(0);
    return 0;
}

/**
  * @brief Save the image list.
  * @param list image list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigImageListSave(TargetImageHandle *list, uint32_t listSize)
{
    if (list == NULL) {
        return -1;
    }
    TargetAlgoHandle tempAlgo;
    uint32_t addr;
    uint32_t sz;
    uint8_t *buf = NULL;
    /* Initialize the debugger flash operation. */
    UnInit(0);
    Init(0, 0, 0);
    /* Backup algorithm list. */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_ALGO_LIST_OFFSET;
    FlashRead(addr, sizeof(tempAlgo), (uint8_t *)&tempAlgo);
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_ALGO_LIST_OFFSET;
    EraseSector(addr);
    /* save image */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LIST_OFFSET;
    sz = listSize;
    buf = (uint8_t*)list;
    ProgramPageHandle(addr, buf, sz);

    /* save algo */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_ALGO_LIST_OFFSET;
    sz = sizeof(tempAlgo);
    buf = (uint8_t*)&tempAlgo;
    ProgramPageHandle(addr, buf, sz);

    UnInit(0);
    return 0;
}

/**
  * @brief Read the image list.
  * @param list image list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigImageListRead(TargetImageHandle *list, uint32_t listSize)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    /* Read image list */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LIST_OFFSET;
    FlashRead(addr, listSize, (uint8_t *)list);
    UnInit(0);
    return 0;
}

/**
  * @brief Holds an array of flags indicating whether the algorithm storage area is free.
  * @param flagArry Idle flag array
  * @param arrySize Array Length
  * @retval Success or Failure Result
  */
int32_t ConfigFreeAlgoFlagSave(const bool *flagArry, uint32_t arrySize)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_FREE_ALGO_FLAG_ARRY_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;

    if (flagArry == NULL) {
        return -1;
    }

    ConfigUpdate((const uint8_t *)flagArry, arrySize, offset);
    return 0;
}

/**
  * @brief Indicates the flag array for reading whether the storage area of the flag algorithm is idle.
  * @param flagArry Idle flag array
  * @param arrySize Array Length
  * @retval Success or Failure Result
  */
int32_t ConfigFreeAlgoFlagRead(bool *flagArry, uint32_t arrySize)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    /* Read free algorithm storage space flag. */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_ALGO_FLAG_ARRY_OFFSET;
    FlashRead(addr, arrySize, (uint8_t *)flagArry);
    UnInit(0);
    return 0;
}

/**
  * @brief Stores the array indicating whether the image storage area is idle.
  * @param flagArry Idle flag array
  * @param arrySize Array Length
  * @retval Success or Failure Result
  */
int32_t ConfigFreeImageFlagSave(const bool *flagArry, uint32_t arrySize)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_FREE_IMAGE_FLAG_ARRY_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;

    if (flagArry == NULL) {
        return -1;
    }
    ConfigUpdate((const uint8_t *)flagArry, arrySize, offset);
    return 0;
}

/**
  * @brief Reads the array indicating whether the image storage area is idle.
  * @param flagArry Idle flag array
  * @param arrySize Array Length
  * @retval Success or Failure Result
  */
int32_t ConfigFreeImageFlagRead(bool *flagArry, uint32_t arrySize)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    /* Read image free list */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_IMAGE_FLAG_ARRY_OFFSET;
    FlashRead(addr, arrySize, (uint8_t *)flagArry);
    UnInit(0);
    return 0;
}

/**
  * @brief Stores the array indicating whether the image file storage area is idle in factory mode.
  * @param flagArry Idle flag array
  * @param arrySize Array Length
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryFreeImageFlagSave(const bool *flagArry, uint32_t arrySize)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_FREE_FACTORY_IMAGE_FLAG_ARRY_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    
    if (flagArry == NULL) {
        return -1;
    }

    ConfigUpdate((const uint8_t *)flagArry, arrySize, offset);
    return 0;
}

/**
  * @brief Read the array indicating whether the image file storage area is idle in factory mode.
  * @param flagArry Idle flag array
  * @param arrySize Array Length
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryFreeImageFlagRead(bool *flagArry, uint32_t arrySize)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    /* Read free image flag */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FREE_FACTORY_IMAGE_FLAG_ARRY_OFFSET;
    FlashRead(addr, arrySize, (uint8_t *)flagArry);
    UnInit(0);
    return 0;
}

/**
  * @brief Save Startup Flag.
  * @param flag Startup Flag
  * @retval Success or Failure Result
  */
int32_t ConfigStartFlagSave(uint32_t flag)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_STARTUP_FLAG_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    ConfigUpdate((const uint8_t *)&flag, sizeof(flag), offset);
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

/**
  * @brief Saves the current algorithm index.
  * @param index Indicates the current algorithm index
  * @retval Success or Failure Result
  */
int32_t ConfigCurrentAlgoIndexSave(int32_t index)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_CURRENT_ALGO_INDEX_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    ConfigUpdate((const uint8_t *)&index, sizeof(index), offset);
    return 0;
}

/**
  * @brief Read the current algorithm index.
  * @retval current algorithm index
  */
int32_t ConfigCurrentAlgoIndexRead(void)
{
    uint32_t addr;
    int32_t algoIndex;

    UnInit(0);
    Init(0, 0, 0);
    /* Read algorithm index */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_CURRENT_ALGO_INDEX_OFFSET;
    FlashRead(addr, sizeof(algoIndex), (uint8_t *)&algoIndex);
    UnInit(0);
    return algoIndex;
}

/**
  * @brief Stores the list of image files in factory mode.
  * @param list image files list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryImageListSave(FactoryImageHandle *list, uint32_t listSize)
{
    uint32_t addr;
    uint32_t sz;
    uint8_t *buf = NULL;

    UnInit(0);
    Init(0, 0, 0);
    /* Erase the flash partition of the image list file in factory mode. */
    addr = CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LIST_OFFSET;
    EraseSector(addr);
    sz = listSize;
    buf = (uint8_t*)list;
    /* Update data */
    ProgramPageHandle(addr, buf, sz);

    UnInit(0);
    return 0;
}

/**
  * @brief Read the list of image files in factory mode.
  * @param list image files list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryImageListRead(FactoryImageHandle *list, uint32_t listSize)
{
    uint32_t addr;

    if (list == NULL) {
        return -1;
    }
    UnInit(0);
    Init(0, 0, 0);
    /* Read image list */
    addr = CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LIST_OFFSET;
    FlashRead(addr, listSize, (uint8_t *)list);
    UnInit(0);
    return 0;
}

/**
  * @brief Save Factory Mode Flag
  * @param flag Factory Mode Flag
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryFlagSave(uint32_t flag)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_FACTORY_FLAG_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    ConfigUpdate((const uint8_t *)&flag, sizeof(flag), offset);
    return 0;
}

/**
  * @brief Read Factory Mode Flag
  * @retval Factory Mode Flag
  */
uint32_t ConfigFactoryFlagRead(void)
{
    uint32_t addr;
    uint32_t factoryFlag;

    UnInit(0);
    Init(0, 0, 0);
    /* Read factory flag */
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_FLAG_OFFSET;
    FlashRead(addr, sizeof(factoryFlag), (uint8_t *)&factoryFlag);
    UnInit(0);
    return factoryFlag;
}

/**
  * @brief Save the external power supply status flag.
  * @param flag External power supply status flag
  * @retval Success or Failure Result
  */
int32_t ConfigPowerStatusFlagSave(uint32_t flag)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_POWER_STATUS_FLAG_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    ConfigUpdate((const uint8_t *)&flag, sizeof(flag), offset);
    return 0;
}

/**
  * @brief Read the external power supply status flag.
  * @retval External power supply status flag
  */
uint32_t ConfigPowerStatusFlagRead(void)
{
    uint32_t powerStatus;
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_POWER_STATUS_FLAG_OFFSET;
    FlashRead(addr, sizeof(powerStatus), (uint8_t *)&powerStatus);
    UnInit(0);
    return powerStatus;
}

/**
  * @brief Save Language Configuration Flag.
  * @param flag Language configuration flag
  * @retval Success or Failure Result
  */
int32_t ConfigLanguageFlagSave(uint32_t flag)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_LANGUAGE_FLAG_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    ConfigUpdate((const uint8_t *)&flag, sizeof(flag), offset);
    return 0;
}

/**
  * @brief Reads the language configuration flag.
  * @retval Language configuration flag
  */
uint32_t ConfigLanguageFlagRead(void)
{
    uint32_t languageFlag;
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_LANGUAGE_FLAG_OFFSET;
    FlashRead(addr, sizeof(languageFlag), (uint8_t *)&languageFlag);
    UnInit(0);
    return languageFlag;
}

/**
  * @brief Save Debug Port Configuration.
  * @param flag Debug Port Configuration flag
  * @retval Success or Failure Result
  */
int32_t ConfigDebugPortFlagSave(uint32_t flag)
{
    uint32_t offset = CONFIG_STORAGE_UPDATE_DEBUG_PORT_FLAG_OFFSET - \
                      CONFIG_STORAGE_UPDATE_CFG_START_OFFSET;
    ConfigUpdate((const uint8_t *)&flag, sizeof(flag), offset);
    return 0;
}

/**
  * @brief Read Debug Port Configuration.
  * @retval Debug Port Configuration
  */
uint32_t ConfigDebugPorFlagRead(void)
{
    uint32_t debugPortFlag;
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    addr = CONFIG_STORAGE_UPDATE_BASE_ADDR + CONFIG_STORAGE_UPDATE_DEBUG_PORT_FLAG_OFFSET;
    FlashRead(addr, sizeof(debugPortFlag), (uint8_t *)&debugPortFlag);
    UnInit(0);
    return debugPortFlag;
}

/**
  * @brief Save the list of download times.
  * @param list image load cnt list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigImageLoadCntSave(const TargetImageLoadCntList *list, uint32_t listSize)
{
    uint32_t addr;
    uint32_t left;
    TargetImageLoadCntList tempList;
    uint32_t flag = 0;
    uint8_t *temp = (uint8_t*)&tempList;

    UnInit(0);
    Init(0, 0, 0);
    /* Obtains the remaining size of the storage area. */
    left = CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET + \
           CONFIG_FLASH_BASS_ADDR + \
           CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_SIZE - g_imageLoadCntListStartAddr;
    if (left >= ONE_IMAGE_LOAD_CNT_LIST_SIZE) {
        /* The storage address is moved backward by a group of units. */
        addr = g_imageLoadCntListStartAddr + ONE_IMAGE_LOAD_CNT_LIST_SIZE;
    } else {
        /* The storage area is used up. Erase the entire flash memory again.
           The storage start address is looped back to the start position. */
        addr = CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET;
        EraseSector(addr);
    }
    /* Avoid false positives in the first storage area. */
    if (g_imageLoadCntListStartAddr == (CONFIG_FLASH_BASS_ADDR + \
                                               CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET)) {
        addr = g_imageLoadCntListStartAddr;
        FlashRead(addr, sizeof(TargetImageLoadCntList), (uint8_t *)&tempList);
        flag = 0;
        /* Determine whether data has been stored in the first block area */
        for (uint32_t i = 0; i < sizeof(TargetImageLoadCntList); i++) {
            if (temp[i] != 0xFF) {
                flag = 1;
                break;
            }
        }
        if (flag != 0) {
            addr = g_imageLoadCntListStartAddr + ONE_IMAGE_LOAD_CNT_LIST_SIZE;
        }
    }

    ProgramPage(addr, listSize, (uint32_t *)list);
    g_imageLoadCntListStartAddr = addr;
    return 0;
}

/**
  * @brief Read the list of download times.
  * @param list image load cnt list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigImageLoadCntRead(TargetImageLoadCntList *list, uint32_t listSize)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    addr = g_imageLoadCntListStartAddr;
    FlashRead(addr, listSize, (uint8_t *)list);
    UnInit(0);
    return 0;
}

/**
  * @brief Address for storing the number of times the calibration image is downloaded.
  * @retval Success or Failure Result
  */
int32_t ConfigImageLoadCntAddrCalibration(void)
{
    TargetImageLoadCntList tempList;
    uint32_t i;
    uint32_t addr = g_imageLoadCntListStartAddr;
    uint32_t left = CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_SIZE;
    uint32_t flag = 0;
    uint8_t *temp = (uint8_t*)&tempList;
    
    UnInit(0);
    Init(0, 0, 0);
    /* Find the first unstored area from left to right. */
    while (left > ONE_IMAGE_LOAD_CNT_LIST_SIZE) {
        FlashRead(addr, sizeof(TargetImageLoadCntList), (uint8_t *)&tempList);
        flag = 0;
        for (i = 0; i < sizeof(TargetImageLoadCntList); i++) {
            /* If a value other than 0xFF is found in a storage area,
               data has been written to the area. */
            if (temp[i] != 0xFF) {
                flag = 1;
                break;
            }
        }
        /* Confirm the location of the next set of updated data stores */
        if (flag == 0) {
            if (addr == (CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET)) {
                g_imageLoadCntListStartAddr = addr;
            } else {
                g_imageLoadCntListStartAddr = addr - ONE_IMAGE_LOAD_CNT_LIST_SIZE;
            }
            break;
        }
        addr += ONE_IMAGE_LOAD_CNT_LIST_SIZE;
        left = CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET + \
               CONFIG_FLASH_BASS_ADDR + \
               CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_SIZE - addr;
    }
    /* When all regions have been written, the address loops back to the start position. */
    if (flag != 0) {
        g_imageLoadCntListStartAddr = addr - ONE_IMAGE_LOAD_CNT_LIST_SIZE;
    }
    UnInit(0);
    return 0;
}

/**
  * @brief Clear the list of image download times.
  * @retval Success or Failure Result
  */
int32_t ConfigImageLoadCntClean(void)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);

    addr = CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_IMAGE_LOAD_CNT_LIST_OFFSET;
    EraseSector(addr);
    g_imageLoadCntListStartAddr = addr;
    UnInit(0);
    return 0;
}

/**
  * @brief Saves the list of download times in factory mode.
  * @param list image load cnt list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryImageLoadCntSave(const FactoryImageLoadCntList *list, uint32_t listSize)
{
    uint32_t addr;
    uint32_t left;
    FactoryImageLoadCntList tempList;
    uint8_t *temp = (uint8_t*)&tempList;
    uint32_t flag = 0;

    UnInit(0);
    Init(0, 0, 0);
    /* Obtains the remaining size of the storage area. */
    left = CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET + \
           CONFIG_FLASH_BASS_ADDR + \
           CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE - g_factoryImageLoadCntListStartAddr;
    if (left >= ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE) {
        /* The storage address is moved backward by a group of units. */
        addr = g_factoryImageLoadCntListStartAddr + ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE;
    } else {
        /* The storage area is used up. Erase the entire flash memory again.
           The storage start address is looped back to the start position. */
        addr = CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET;
        EraseSector(addr);
    }
    /* Avoid false positives in the first storage area. */
    if (g_factoryImageLoadCntListStartAddr == (CONFIG_FLASH_BASS_ADDR + \
                                               CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET)) {
        addr = g_factoryImageLoadCntListStartAddr;
        FlashRead(addr, sizeof(FactoryImageLoadCntList), (uint8_t *)&tempList);
        flag = 0;
        /* Determine whether data has been stored in the first block area */
        for (uint32_t i = 0; i < sizeof(FactoryImageLoadCntList); i++) {
            if (temp[i] != 0xFF) {
                flag = 1;
                break;
            }
        }
        if (flag != 0) {
            addr = g_factoryImageLoadCntListStartAddr + ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE;
        }
    }

    ProgramPage(addr, listSize, (uint32_t *)list);
    g_factoryImageLoadCntListStartAddr = addr;
    return 0;
}

/**
  * @brief Read the list of download times in factory mode.
  * @param list image load cnt list
  * @param listSize list size
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryImageLoadCntRead(FactoryImageLoadCntList *list, uint32_t listSize)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    addr = g_factoryImageLoadCntListStartAddr;
    FlashRead(addr, listSize, (uint8_t *)list);
    UnInit(0);
    return 0;
}

/**
  * @brief Address for downloading calibration images in factory mode.
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryImageLoadCntAddrCalibration(void)
{
    FactoryImageLoadCntList tempList;
    uint32_t i;
    uint32_t addr = g_factoryImageLoadCntListStartAddr;
    uint32_t left = CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE;
    uint32_t flag = 0;
    uint8_t *temp = (uint8_t*)&tempList;
    
    UnInit(0);
    Init(0, 0, 0);
    /* Find the first unstored area from left to right. */
    while (left > ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE) {
        FlashRead(addr, sizeof(FactoryImageLoadCntList), (uint8_t *)&tempList);
        flag = 0;
        for (i = 0; i < sizeof(FactoryImageLoadCntList); i++) {
            /* If a value other than 0xFF is found in a storage area,
               data has been written to the area. */
            if (temp[i] != 0xFF) {
                flag = 1;
                break;
            }
        }
        /* Confirm the location of the next set of updated data stores */
        if (flag == 0) {
            if (addr == (CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET)) {
                g_factoryImageLoadCntListStartAddr = addr;
            } else {
                g_factoryImageLoadCntListStartAddr = addr - ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE;
            }
            break;
        }
        addr += ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE;
        left = CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET + \
               CONFIG_FLASH_BASS_ADDR + \
               CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE - addr;
    }
    /* When all regions have been written, the address loops back to the start position. */
    if (flag != 0) {
        g_factoryImageLoadCntListStartAddr = addr - ONE_FACTORY_IMAGE_LOAD_CNT_LIST_SIZE;
    }
    UnInit(0);
    return 0;
}

/**
  * @brief Clear the list of image download times in factory mode.
  * @retval Success or Failure Result
  */
int32_t ConfigFactoryImageLoadCntClean(void)
{
    uint32_t addr;

    UnInit(0);
    Init(0, 0, 0);
    addr = CONFIG_FLASH_BASS_ADDR + CONFIG_STORAGE_UPDATE_FACTORY_IMAGE_LOAD_CNT_LIST_OFFSET;
    EraseSector(addr);
    g_factoryImageLoadCntListStartAddr = addr;
    return 0;
}

/**
  * @brief Download Type set
  * @param type Download Type
  * @retval Success or Failure Result
  */
int32_t ConfigLoadTypeSet(int32_t type)
{
    g_configLoadType = type;
    return 0;
}

/**
  * @brief Download Type get
  * @retval Download Type
  */
int32_t ConfigLoadTypeGet(void)
{
    return g_configLoadType;
}

/**
  * @brief Download End Address set
  * @param addr Download End Address
  * @retval Success or Failure Result
  */
int32_t ConfigLoadEndAddrSet(int32_t addr)
{
    g_configLoadEndAddr = addr;
    return 0;
}

/**
  * @brief Download End Address get
  * @retval Download End Address
  */
int32_t ConfigLoadEndAddrGet(void)
{
    return g_configLoadEndAddr;
}

/**
  * @brief Current file length set
  * @param len Current file length
  * @retval Success or Failure Result
  */
int32_t ConfigLoadCurrentFileSizeSet(uint32_t len)
{
    g_configLoadFileSize = len;
    return 0;
}

/**
  * @brief Current file length get
  * @retval Current file length
  */
uint32_t ConfigLoadCurrentFileSizeGet(void)
{
    return g_configLoadFileSize;
}