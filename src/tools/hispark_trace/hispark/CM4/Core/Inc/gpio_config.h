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
  * @file    gpio_config.h
  * @author  MCU Driver Team
  * @brief   gpio config.
  */
#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#define  NEW_BOARD

#ifdef NEW_BOARD

#define SWCLK_TCK_PIN_PORT           GPIOA
#define SWCLK_TCK_PIN                GPIO_PIN_9
#define SWCLK_TCK_PIN_BIT            9

#define SWDIO_OUT_TMS_PIN_PORT       GPIOA
#define SWDIO_OUT_TMS_PIN            GPIO_PIN_8
#define SWDIO_OUT_TMS_PIN_BIT        8

#define SWDIO_IN_PIN_PORT            GPIOA
#define SWDIO_IN_PIN                 GPIO_PIN_0
#define SWDIO_IN_PIN_BIT             0

#define SWDIO_INOUT_OE_PORT          GPIOA
#define SWDIO_INOUT_OE_PIN           GPIO_PIN_3
#define SWDIO_INOUT_OE_PIN_BIT       3

#define JTAG_TDI_PIN_PORT            GPIOA
#define JTAG_TDI_PIN                 GPIO_PIN_1
#define JTAG_TDI_PIN_BIT             1

#define JTAG_TDO_PIN_PORT            GPIOA
#define JTAG_TDO_PIN                 GPIO_PIN_2
#define JTAG_TDO_PIN_BIT             2

#define JTAG_TRST_PIN_PORT          GPIOA
#define JTAG_TRST_PIN               GPIO_PIN_14
#define JTAG_TPST_PIN_BIT           14

#define JTAG_TARGET_RST_PIN_PORT    GPIOA
#define JTAG_TARGET_RST_PIN         GPIO_PIN_13
#define JTAG_TARGET_RST_PIN_BIT     13

#endif

#ifdef OLD_BOARD

/* SWD */
#define SWCLK_TCK_PIN_PORT           GPIOA
#define SWCLK_TCK_PIN                GPIO_PIN_8
#define SWCLK_TCK_PIN_BIT            8

#define SWDIO_OUT_TMS_PIN_PORT       GPIOA
#define SWDIO_OUT_TMS_PIN            GPIO_PIN_9
#define SWDIO_OUT_TMS_PIN_BIT        9

#define SWDIO_IN_PIN_PORT            GPIOA
#define SWDIO_IN_PIN                 GPIO_PIN_7
#define SWDIO_IN_PIN_BIT             7

#define SWDIO_INOUT_OE_PORT          GPIOA
#define SWDIO_INOUT_OE_PIN           GPIO_PIN_6
#define SWDIO_INOUT_OE_PIN_BIT       6

#define JTAG_TDI_PIN_PORT            GPIOA
#define JTAG_TDI_PIN                 GPIO_PIN_10
#define JTAG_TDI_PIN_BIT             10

#define JTAG_TDO_PIN_PORT            GPIOA
#define JTAG_TDO_PIN                 GPIO_PIN_11
#define JTAG_TDO_PIN_BIT             11

#define JTAG_TRST_PIN_PORT          GPIOA
#define JTAG_TRST_PIN               GPIO_PIN_12
#define JTAG_TPST_PIN_BIT           12

#define JTAG_TARGET_RST_PIN_PORT    GPIOA
#define JTAG_TARGET_RST_PIN         GPIO_PIN_13
#define JTAG_TARGET_RST_PIN_BIT     13

#endif
void GpioConfigInit(void);
#endif
