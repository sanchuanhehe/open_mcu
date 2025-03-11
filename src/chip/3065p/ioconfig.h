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
  * @file    ioconfig.h
  * @author  MCU Driver Team
  * @brief   ioconfig module driver
  * @details This file provides IOConfig register mapping structure.
  */

/* Macro definitions */
#ifndef McuMagicTag_IOCONFIG_H
#define McuMagicTag_IOCONFIG_H

typedef union {
    unsigned int reg;
    struct {
        unsigned int func : 4;              /**< IO function selection. */
        unsigned int ds : 2;                /**< Pin drive capability selection. */
        unsigned int reserved0 : 1;
        unsigned int pd : 1;                /**< Pin pull down control. */
        unsigned int pu : 1;                /**< Pin pull up control. */
        unsigned int sr : 1;                /**< Electrical level shift speed control. */
        unsigned int se : 1;                /**< Schmidt input control. */
        unsigned int reserved1 : 1;         /**< Reserved: Open drain control reserved. */
        unsigned int reserved2 : 20;
    } BIT;
} volatile IOCMG_REG;

typedef struct {
    IOCMG_REG IOCFG_GPIO0_0;    /**< Pin GPIO0_0 IO Config Register, offset address:0x000000U */
    IOCMG_REG IOCFG_GPIO0_1;    /**< Pin GPIO0_1 IO Config Register, offset address:0x000004U */
    IOCMG_REG IOCFG_GPIO0_2;    /**< Pin GPIO0_2 IO Config Register, offset address:0x000008U */
    IOCMG_REG IOCFG_GPIO0_3;    /**< Pin GPIO0_3 IO Config Register, offset address:0x00000CU */
    IOCMG_REG IOCFG_GPIO0_4;    /**< Pin GPIO0_4 IO Config Register, offset address:0x000010U */
    IOCMG_REG IOCFG_GPIO0_5;    /**< Pin GPIO0_5 IO Config Register, offset address:0x000014U */
    IOCMG_REG IOCFG_GPIO0_6;    /**< Pin GPIO0_6 IO Config Register, offset address:0x000018U */
    IOCMG_REG IOCFG_GPIO0_7;    /**< Pin GPIO0_7 IO Config Register, offset address:0x00001CU */
    IOCMG_REG IOCFG_GPIO1_0;    /**< Pin GPIO1_0 IO Config Register, offset address:0x000020U */
    IOCMG_REG IOCFG_GPIO1_1;    /**< Pin GPIO1_1 IO Config Register, offset address:0x000024U */
    IOCMG_REG IOCFG_GPIO1_2;    /**< Pin GPIO1_2 IO Config Register, offset address:0x000028U */
    IOCMG_REG IOCFG_GPIO1_3;    /**< Pin GPIO1_3 IO Config Register, offset address:0x00002CU */
    IOCMG_REG IOCFG_GPIO1_4;    /**< Pin GPIO1_4 IO Config Register, offset address:0x000030U */
    IOCMG_REG IOCFG_GPIO1_5;    /**< Pin GPIO1_5 IO Config Register, offset address:0x000034U */
    IOCMG_REG IOCFG_GPIO1_6;    /**< Pin GPIO1_6 IO Config Register, offset address:0x000038U */
    IOCMG_REG IOCFG_GPIO1_7;    /**< Pin GPIO1_7 IO Config Register, offset address:0x00003CU */
    IOCMG_REG IOCFG_GPIO2_0;    /**< Pin GPIO2_0 IO Config Register, offset address:0x000040U */
    IOCMG_REG IOCFG_GPIO2_1;    /**< Pin GPIO2_1 IO Config Register, offset address:0x000044U */
    IOCMG_REG IOCFG_GPIO2_2;    /**< Pin GPIO2_2 IO Config Register, offset address:0x000048U */
    IOCMG_REG IOCFG_GPIO2_3;    /**< Pin GPIO2_3 IO Config Register, offset address:0x00004CU */
    IOCMG_REG IOCFG_GPIO2_4;    /**< Pin GPIO2_4 IO Config Register, offset address:0x000050U */
    IOCMG_REG IOCFG_GPIO2_5;    /**< Pin GPIO2_5 IO Config Register, offset address:0x000054U */
    IOCMG_REG IOCFG_GPIO2_6;    /**< Pin GPIO2_6 IO Config Register, offset address:0x000058U */
    IOCMG_REG IOCFG_GPIO2_7;    /**< Pin GPIO2_7 IO Config Register, offset address:0x00005CU */
    IOCMG_REG IOCFG_GPIO3_0;    /**< Pin GPIO3_0 IO Config Register, offset address:0x000060U */
    IOCMG_REG IOCFG_GPIO3_1;    /**< Pin GPIO3_1 IO Config Register, offset address:0x000064U */
    IOCMG_REG IOCFG_GPIO3_2;    /**< Pin GPIO3_2 IO Config Register, offset address:0x000068U */
    IOCMG_REG IOCFG_GPIO3_3;    /**< Pin GPIO3_3 IO Config Register, offset address:0x00006CU */
    IOCMG_REG IOCFG_GPIO3_4;    /**< Pin GPIO3_4 IO Config Register, offset address:0x000070U */
    IOCMG_REG IOCFG_GPIO3_5;    /**< Pin GPIO3_5 IO Config Register, offset address:0x000074U */
    IOCMG_REG IOCFG_GPIO3_6;    /**< Pin GPIO3_6 IO Config Register, offset address:0x000078U */
    IOCMG_REG IOCFG_GPIO3_7;    /**< Pin GPIO3_7 IO Config Register, offset address:0x00007CU */
    IOCMG_REG IOCFG_GPIO4_0;    /**< Pin GPIO4_0 IO Config Register, offset address:0x000080U */
    IOCMG_REG IOCFG_GPIO4_1;    /**< Pin GPIO4_1 IO Config Register, offset address:0x000084U */
    IOCMG_REG IOCFG_GPIO4_2;    /**< Pin GPIO4_2 IO Config Register, offset address:0x000088U */
    IOCMG_REG IOCFG_GPIO4_3;    /**< Pin GPIO4_3 IO Config Register, offset address:0x00008CU */
    IOCMG_REG IOCFG_GPIO4_4;    /**< Pin GPIO4_4 IO Config Register, offset address:0x000090U */
    IOCMG_REG IOCFG_GPIO4_5;    /**< Pin GPIO4_5 IO Config Register, offset address:0x000094U */
    IOCMG_REG IOCFG_GPIO4_6;    /**< Pin GPIO4_6 IO Config Register, offset address:0x000098U */
    IOCMG_REG IOCFG_GPIO4_7;    /**< Pin GPIO4_7 IO Config Register, offset address:0x00009CU */
    IOCMG_REG IOCFG_GPIO5_0;    /**< Pin GPIO5_0 IO Config Register, offset address:0x0000A0U */
    IOCMG_REG IOCFG_GPIO5_1;    /**< Pin GPIO5_1 IO Config Register, offset address:0x0000A4U */
    IOCMG_REG IOCFG_GPIO5_2;    /**< Pin GPIO5_2 IO Config Register, offset address:0x0000A8U */
    IOCMG_REG IOCFG_GPIO5_3;    /**< Pin GPIO5_3 IO Config Register, offset address:0x0000ACU */
    IOCMG_REG IOCFG_GPIO5_4;    /**< Pin GPIO5_4 IO Config Register, offset address:0x0000B0U */
    IOCMG_REG IOCFG_GPIO5_5;    /**< Pin GPIO5_5 IO Config Register, offset address:0x0000B4U */
    IOCMG_REG IOCFG_GPIO5_6;    /**< Pin GPIO5_6 IO Config Register, offset address:0x0000B8U */
    IOCMG_REG IOCFG_GPIO5_7;    /**< Pin GPIO5_7 IO Config Register, offset address:0x0000BCU */
    IOCMG_REG IOCFG_GPIO6_0;    /**< Pin GPIO6_0 IO Config Register, offset address:0x0000C0U */
    IOCMG_REG IOCFG_GPIO6_1;    /**< Pin GPIO6_1 IO Config Register, offset address:0x0000C4U */
    IOCMG_REG IOCFG_GPIO6_2;    /**< Pin GPIO6_2 IO Config Register, offset address:0x0000C8U */
    IOCMG_REG IOCFG_GPIO6_3;    /**< Pin GPIO6_3 IO Config Register, offset address:0x0000CCU */
    IOCMG_REG IOCFG_GPIO6_4;    /**< Pin GPIO6_4 IO Config Register, offset address:0x0000D0U */
    IOCMG_REG IOCFG_GPIO6_5;    /**< Pin GPIO6_5 IO Config Register, offset address:0x0000D4U */
    IOCMG_REG IOCFG_GPIO6_6;    /**< Pin GPIO6_6 IO Config Register, offset address:0x0000D8U */
    IOCMG_REG IOCFG_GPIO6_7;    /**< Pin GPIO6_7 IO Config Register, offset address:0x0000DCU */
    IOCMG_REG IOCFG_GPIO7_0;    /**< Pin GPIO7_0 IO Config Register, offset address:0x0000E0U */
    IOCMG_REG IOCFG_GPIO7_1;    /**< Pin GPIO7_1 IO Config Register, offset address:0x0000E4U */
    IOCMG_REG IOCFG_GPIO7_2;    /**< Pin GPIO7_2 IO Config Register, offset address:0x0000E8U */
    IOCMG_REG IOCFG_GPIO7_3;    /**< Pin GPIO7_3 IO Config Register, offset address:0x0000ECU */
    IOCMG_REG IOCFG_GPIO7_4;    /**< Pin GPIO7_4 IO Config Register, offset address:0x0000F0U */
    IOCMG_REG IOCFG_GPIO7_5;    /**< Pin GPIO7_5 IO Config Register, offset address:0x0000F4U */
    IOCMG_REG IOCFG_GPIO7_6;    /**< Pin GPIO7_6 IO Config Register, offset address:0x0000F8U */
    IOCMG_REG IOCFG_GPIO7_7;    /**< Pin GPIO7_7 IO Config Register, offset address:0x0000FCU */
    IOCMG_REG IOCFG_GPIO8_0;    /**< Pin GPIO8_0 IO Config Register, offset address:0x000100U */
    IOCMG_REG IOCFG_GPIO8_1;    /**< Pin GPIO8_1 IO Config Register, offset address:0x000104U */
    IOCMG_REG IOCFG_GPIO8_2;    /**< Pin GPIO8_2 IO Config Register, offset address:0x000108U */
    IOCMG_REG IOCFG_GPIO8_3;    /**< Pin GPIO8_3 IO Config Register, offset address:0x00010CU */
    IOCMG_REG IOCFG_GPIO8_4;    /**< Pin GPIO8_4 IO Config Register, offset address:0x000110U */
    IOCMG_REG IOCFG_GPIO8_5;    /**< Pin GPIO8_5 IO Config Register, offset address:0x000114U */
    IOCMG_REG IOCFG_GPIO8_6;    /**< Pin GPIO8_6 IO Config Register, offset address:0x000118U */
    IOCMG_REG IOCFG_GPIO8_7;    /**< Pin GPIO8_7 IO Config Register, offset address:0x00011CU */
    IOCMG_REG IOCFG_GPIO9_0;    /**< Pin GPIO9_0 IO Config Register, offset address:0x000120U */
} volatile IOConfig_RegStruct;

#endif /* McuMagicTag_IOCONFIG_H */