/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2024. All rights reserved.
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
 * @file    sample_capm_hall.h
 * @author  MCU Driver Team
 * @brief   CAPM HAL level module driver head file.
 *          This file shows a sample to get hall position information
 */
#include "sample_capm_hall.h"
#include "interrupt.h"
#include "debug.h"
#include "main.h"

#define FIRST_BIT_SHIFT   1
#define SECOND_BIT_SHIFT  2

/**
  * @brief Calculate current level
  * @param handle: CAPM handle.
  * @retval none
  */
static unsigned char CAPM_CalculateLevel(CAPM_Handle *handle)
{
    unsigned char hallNextECR;

    CAPM_ASSERT_PARAM(handle != NULL);
    hallNextECR = HAL_CAPM_GetNextLoadECRNum(handle); /* get next ECR number */
    if (hallNextECR == CAPM_NEXT_LOAD_ECR1 || hallNextECR == CAPM_NEXT_LOAD_ECR3) {
        return CAPM_LOW; /* current level is low */
    } else {
        return CAPM_HIGH; /* current level is high */
    }
}

/**
  * @brief Get current Hall position value.
  * @param None.
  * @retval current position:CAPM_PART_A~F.
  */
unsigned char CAPM_GetHallValue(void)
{
    unsigned char hallPosition;
#if defined (CHIP_3065PNPIMH) || defined (CHIP_3066MNPIRH) || \
    defined (CHIP_3065PNPIRH) || defined (CHIP_3065PNPIRE) || defined (CHIP_3065PNPIRA)
    /* 3066m */
    unsigned char hall0Level = CAPM_CalculateLevel(&g_capm0); /* get capm0 level */
    unsigned char hall1Level = CAPM_CalculateLevel(&g_capm1); /* get capm1 level */
    unsigned char hall2Level = CAPM_CalculateLevel(&g_capm2); /* get capm2 level */

    hallPosition = hall0Level << SECOND_BIT_SHIFT;    /* move to the 2nd bit */
    hallPosition |= hall1Level << FIRST_BIT_SHIFT;    /* move to the 1st bit */
    hallPosition |= hall2Level;
#else
    /* 3061m/3065h */
	unsigned char hallALevel = CAPM_CalculateLevel(&g_capmAConfig); /* get A phase's level */
    unsigned char hallBLevel = CAPM_CalculateLevel(&g_capmBConfig); /* get B phase's level */
    unsigned char hallCLevel = CAPM_CalculateLevel(&g_capmCConfig); /* get C phase's level */

    hallPosition = hallALevel << SECOND_BIT_SHIFT;    /* move to the 2nd bit */
    hallPosition |= hallBLevel << FIRST_BIT_SHIFT;    /* move to the 1st bit */
    hallPosition |= hallCLevel;
#endif

    return hallPosition;
}

/**
  * @brief Sample of reading hall sensor value.
  * @param None.
  * @retval None.
  */
void CAPM_HallSample(void)
{
    SystemInit();
    while (1) {
        unsigned char data = CAPM_GetHallValue(); /* get hall sensor value. */
        DBG_PRINTF("hall = 0x%x\r\n", data);
    }
}