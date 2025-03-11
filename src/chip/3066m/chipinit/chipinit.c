/**
  * @copyright Copyright (c) 2024, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      chipint.c
  * @author    MCU Driver Team
  * @brief     Chip Init modlue.
  * @details   Declare a function that needs to be executed as soon as the C
  *            runtime environment is ready
  */
#include "chipinc.h"
#include "crginit.h"
#include "systickinit.h"
#include "flashinit.h"
#include "crg.h"
#include "interrupt.h"
#ifdef NOS_TASK_SUPPORT
#include "nosinit.h"
#endif
#include "chipinit.h"

#include "anavrefinit.h"
#ifndef FPGA /* Delete the compilation macro after the ASIC to release the header file. */
#include "anatrim.h"
#endif

/**
 * @brief Chip Init Fail Process, deadloop if Chip Init fail
 * @param None
 * @retval None
 */
static inline void Chip_InitFail(void)
{
    while (1) {
        ;
    }
}

/**
 * @brief Chip Init
 * @param None
 * @retval None
 */
void Chip_Init(void)
{
    CRG_CoreClkSelect coreClkSelect;
    /* Config CRG */
    if (CRG_Config(&coreClkSelect) != BASE_STATUS_OK) {
        Chip_InitFail();
    }

    /* Config FLASH Clock */
    FLASH_ClockConfig(coreClkSelect);
    IRQ_Init();
    SYSTICK_Init();
    /* Set CoreClock Select after FLASH Config Done */
    CRG_SetCoreClockSelect(coreClkSelect);
#ifdef NOS_TASK_SUPPORT
    SYSTICK_IRQ_Enable();
#endif
    ANAVREF_Init();
#ifndef FPGA /* Delete the compilation macro after the ASIC to release the header file. */
    ANATRIM_Entry();
#endif

#ifdef NOS_TASK_SUPPORT
    NOS_Init();
#endif
    /* User Add Code Here */
}