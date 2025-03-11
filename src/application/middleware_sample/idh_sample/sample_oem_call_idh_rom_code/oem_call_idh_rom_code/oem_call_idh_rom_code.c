/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2024. All rights reserved.
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
  * @file    oem_call_idh_rom_code.c
  * @author  MCU Driver Team
  * @brief   This file provides sample code for OEM calls IDH ROM-ized code.
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "oem_call_idh_rom_code.h"

static FLASH_Handle g_flashHandle;
static TIMER_Handle g_timerHandle;

#define TEST_CLC_NUM1 3
#define TEST_CLC_NUM2 8

#define STORE_COUNT 5

#define FLASH_STORE_WORD  0xAB
#define FLASH_OPERATE_ADDR FLASH_PAGE_58
#define FLASH_OPERATE_LENGTH 1024

#define TIMER_DELAY_MS 1000000

#define ODD_JUDGE_MASK 0x01

static unsigned int g_storeCountFlag = 0;

/**
  * @brief Timer callback function.
  * @param handle TIMER handle.
  * @retval None.
  */
static void UserTimer0Callback(void *handle)
{
    BASE_FUNC_UNUSED(handle);
    unsigned char dataBuff[FLASH_OPERATE_LENGTH + 1] = {0};

    g_storeCountFlag++;
    if (g_storeCountFlag > STORE_COUNT) {
        FIXED_OPERATOR_StoreStop(&g_timerHandle);
        DBG_PRINTF("Sample finish\r\n");
        return;
    }
    /* Odd numbers call RAM functions, even numbers call ROM functions. */
    if (g_storeCountFlag & ODD_JUDGE_MASK) {
        FIXED_OPERATOR_StoreSRAM(&g_flashHandle, (FLASH_STORE_WORD + g_storeCountFlag), FLASH_OPERATE_ADDR,
                                 FLASH_OPERATE_LENGTH); /* Store data to flash used function running sram. */
        DBG_PRINTF("FIXED_OPERATOR_StoreSRAM finish, data: %x\r\n", (FLASH_STORE_WORD + g_storeCountFlag));
    } else {
        FIXED_OPERATOR_Store(&g_flashHandle, (FLASH_STORE_WORD + g_storeCountFlag), FLASH_OPERATE_ADDR,
                             FLASH_OPERATE_LENGTH); /* Store data to flash used function running flash. */
        DBG_PRINTF("FIXED_OPERATOR_Store finish, data: %x\r\n", (FLASH_STORE_WORD + g_storeCountFlag));
    }

    /* Perform the read operation. */
    HAL_FLASH_Read(&g_flashHandle, FLASH_OPERATE_ADDR, FLASH_OPERATE_LENGTH,
                   dataBuff, FLASH_OPERATE_LENGTH);
    /* Check Data Correctness */
    for (unsigned int i = 0; i < FLASH_OPERATE_LENGTH; i++) {
        if (dataBuff[i] != (FLASH_STORE_WORD + g_storeCountFlag)) {
            DBG_PRINTF("Store flash fail, index:%d, data: %x\r\n ", dataBuff[i]);
            return;
        }
    }

    DBG_PRINTF("Store flash success, data: %x\r\n ", dataBuff[0]);
}

/**
  * @brief Registers the callback function of the HAL interrupt service.
  * @param None.
  * @retval None.
  */
static void RegisterIrqApi(void)
{
    FIXED_OPERATOR_IrqCallback irqApi;
    irqApi.irqEnableCb = IRQ_EnableN;
    irqApi.irqRegisterCb = IRQ_Register;
    irqApi.irqSetpriorityCb = IRQ_SetPriority;

    REGISTER_IRQ_CALLBACK(&irqApi); /* Invoke idh function. */
}

/**
  * @brief Registers the callback function of the HAL flash functions.
  * @param None.
  * @retval None.
  */
static void RegisterFlashApi(void)
{
    FIXED_OPERATOR_FlashCallback flashApi;
    flashApi.flashInitCb = HAL_FLASH_Init;
    flashApi.flashDeinitCb = HAL_FLASH_DeInit;
    flashApi.flashReadCb = HAL_FLASH_Read;
    flashApi.flashWriteBlockingCb = HAL_FLASH_WriteBlocking;
    flashApi.flashEraseBlockingCb = HAL_FLASH_EraseBlocking;

    REGISTER_FLASH_CALLBACK(&flashApi); /* Invoke idh function. */
}

/**
  * @brief Registers the callback function of the HAL timer functions.
  * @param None.
  * @retval None.
  */
static void RegisterTimerApi(void)
{
    FIXED_OPERATOR_TimerCallback timerApi;
    timerApi.timerInitCb = HAL_TIMER_Init;
    timerApi.timerDeinitCb = HAL_TIMER_DeInit;
    timerApi.timerStartCb = HAL_TIMER_Start;
    timerApi.timerStopCb = HAL_TIMER_Stop;
    timerApi.timerCallbackCb = HAL_TIMER_RegisterCallback;
    timerApi.timerIrqHandlerCb = HAL_TIMER_IrqHandler;
    REGISTER_TIMER_CALLBACK(&timerApi); /* Invoke idh function. */
}

/**
  * @brief Init flash module.
  * @param None.
  * @retval None.
  */
static void InitFlash(void)
{
    HAL_CRG_IpEnableSet(EFC_BASE, IP_CLK_ENABLE);  /* TIMER0 clock enable. */
    g_flashHandle.baseAddress = EFC;
    g_flashHandle.peMode = FLASH_PE_OP_BLOCK;
    FIXED_OPERATOR_FlashInit(&g_flashHandle);
    DBG_PRINTF("FIXED_OPERATOR_FlashInit\r\n");
}

/**
  * @brief Init timer module.
  * @param None.
  * @retval None.
  */
static void InitTimerIt(void)
{
    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE);  /* TIMER0 clock enable. */
    unsigned int loadData =
        (HAL_CRG_GetIpFreq((void *)TIMER0) / (1u << (TIMERPRESCALER_NO_DIV * 0x4)) / TIMER_DELAY_MS) * TIMER_DELAY_MS;
    g_timerHandle.baseAddress = TIMER0;
    g_timerHandle.load        = loadData - 1; /* Set timer value immediately */
    g_timerHandle.bgLoad      = loadData - 1; /* Set timer value */
    g_timerHandle.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timerHandle.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timerHandle.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timerHandle.interruptEn = BASE_CFG_ENABLE;
    g_timerHandle.adcSocReqEnable = BASE_CFG_DISABLE;
    g_timerHandle.dmaReqEnable = BASE_CFG_DISABLE;
    FIXED_OPERATOR_TimerInit(&g_timerHandle, IRQ_TIMER0, 0x01, TIMER_PERIOD_FIN, UserTimer0Callback);
    DBG_PRINTF("FIXED_OPERATOR_TimerInit\r\n");
}

/**
  * @brief OEM Sample Code Execution.
  * @param None.
  * @retval None.
  */
void RomServiceProcess(void)
{
    SystemInit();
    CopyIdhDataToSram(); /* Help IDH move data from flash to sram */

    /* Register HAL function to idh. */
    RegisterIrqApi();
    RegisterFlashApi();
    RegisterTimerApi();

    InitFlash(); /* Init idh used module. */
    InitTimerIt();

    /* Invoke the four arithmetic operations in flash. */
    int ret = FIXED_OPERATOR_Calculate(TEST_CLC_NUM1, TEST_CLC_NUM2, CALACULATE_MUL);
    DBG_PRINTF("fixed_operator_calculate %d * %d = %d\r\n", TEST_CLC_NUM1, TEST_CLC_NUM2, ret);
    /* Invoke the four arithmetic operations in sram. */
    ret = FIXED_OPERATOR_CalculateSRAM(TEST_CLC_NUM1, TEST_CLC_NUM2, CALACULATE_MUL);
    DBG_PRINTF("fixed_operator_calculate SRAM %d * %d = %d\r\n", TEST_CLC_NUM1, TEST_CLC_NUM2, ret);
    /* Start Scheduled Storage Task. */
    FIXED_OPERATOR_StoreStart(&g_timerHandle);
}