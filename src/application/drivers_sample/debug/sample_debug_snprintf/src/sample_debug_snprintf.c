/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      sample_debug_snprintf.c
  * @author    MCU Driver Team
  * @brief     sample of snprintf test.
  */

#include "typedefs.h"
#include "feature.h"
#include "main.h"
#include "sample_debug_snprintf.h"

#define NOT_USE       0x1U
#define SECUREC_LIB   0x2U
#define DEBUG_LIB     0x3U

#define BUFFER_LENTH     256

#define SNPRITF_SELECT     DEBUG_LIB   /* The user selects the interface to be tested. */

#include "debug.h"
#if (SNPRITF_SELECT == SECUREC_LIB)
#include "securec.h"
#endif

void SnprintfTestEntry(void)
{
    SystemInit();
#if (SNPRITF_SELECT != NOT_USE)  /* The snprintf interface is not invoked for the test. */
    char buffer[BUFFER_LENTH] = {0};
    int len = 0;
    float x = 3.1415926; /* 3.1415926 : test data */
    int y = 10;  /* 10 : test data */
    char z = 'z';
    char* pz = &z;
    char* str = "hello world";
#endif
#if (SNPRITF_SELECT == SECUREC_LIB) /* Security function library snprintf interface test */
    len = sprintf_s(buffer, BUFFER_LENTH, \
                    "securec_test2: x = %f, y = %d, str = %s, z = %c, px = %p", x, y, str, z, pz);
#elif (SNPRITF_SELECT == DEBUG_LIB) /* Testing the snprintf interface of the debug module */
    len = DBG_SNPRINTF(buffer, BUFFER_LENTH, \
                       "debug_test3: x = %f, y = %d, str = %s, z = %c, px = %p", x, y, str, z, pz);
#endif
#if (SNPRITF_SELECT != NOT_USE)  /* The snprintf interface is not invoked for the test. */
    DBG_PRINTF("%s\r\n", buffer);
    DBG_PRINTF("Written characters: %d\r\n", len);
#endif
    while (1) {
    }
}