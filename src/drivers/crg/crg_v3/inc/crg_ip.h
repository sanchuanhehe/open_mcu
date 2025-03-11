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
 * @file      crg_ip.h
 * @author    MCU Driver Team
 * @brief     TIMER module driver.
 * @details   This file provides firmware functions to manage the following
 *            functionalities of the TIMER.
 *                + CRG register mapping structure
 *                + Direct Configuration Layer functions of CRG
 */
#ifndef McuMagicTag_CRG_IP_H
#define McuMagicTag_CRG_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/**
  * @addtogroup CRG
  * @{
  */

/**
  * @defgroup CRG_IP CRG_IP
  * @brief CRG_IP: crg_v1
  * @{
  */

/**
 * @defgroup CRG_Param_Def CRG Parameters Definition
 * @brief Definition of CRG configuration parameters.
 * @{
 */
#ifdef  CRG_PARAM_CHECK
#define CRG_ASSERT_PARAM          BASE_FUNC_ASSERT_PARAM
#define CRG_PARAM_CHECK_NO_RET    BASE_FUNC_PARAMCHECK_NO_RET
#define CRG_PARAM_CHECK_WITH_RET  BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define CRG_ASSERT_PARAM(para)               ((void)0U)
#define CRG_PARAM_CHECK_NO_RET(para)         ((void)0U)
#define CRG_PARAM_CHECK_WITH_RET(para, ret)  ((void)0U)
#endif

#define IP_CLK_DISABLE               0x00000000U   /**< IP Clock disable bitmask */
#define IP_CLK_ENABLE                0x00000001U   /**< IP Clock disable bitmask */
#define HPM_CLK_ENABLE               0x00000001U   /**< HPM Clock enable bitmask */
#define HPM_1M_CLK_ENABLE            0x00000002U   /**< HPM 1M Clock enable bitmask */
#define IP_SYSCLK_ENABLE             0x00000002U   /**< IP SysClock disable bitmask, Only valid for ADC */

#define DAC_DIV_BITLEN               4U            /**< DIV bit length */
#define DAC_DIV_MASK ((1 << DAC_DIV_BITLEN) - 1)   /**< DAC div mask, base on the bit length */

#define ADC_DIV_FACTOR          (1 << 1)           /**< ADC div min factor */
#define CRG_1MHZ_CLK_MAX_DIV    63
#define CRG_FREQ_1MHz           (1000 * 1000)
#define CRG_CLK_PFD_MIN_FREQ    (4 * CRG_FREQ_1MHz)
#define CRG_CLK_PFD_MAX_FREQ    (10 * CRG_FREQ_1MHz)
#define CRG_CLK_VCO_MIN_FREQ    (200 * CRG_FREQ_1MHz)
#define CRG_CLK_VCO_MAX_FREQ    (400 * CRG_FREQ_1MHz)
#define CRG_CLK_TARGET_MAX_FREQ (200 * CRG_FREQ_1MHz)
#define CRG_CLK_PST2_MAX_FREQ   (100 * CRG_FREQ_1MHz)
/**
 * @brief PLL refer clock Select
 */
typedef enum {
    CRG_PLL_REF_CLK_SELECT_HOSC = 0,
    CRG_PLL_REF_CLK_SELECT_XTAL = 1,
} CRG_PllRefClkSelect;

/**
 * @brief PLL previous divsion value in register
 */
typedef enum {
    CRG_PLL_PREDIV_1 = 0,
    CRG_PLL_PREDIV_2 = 1,
    CRG_PLL_PREDIV_3 = 2,
    CRG_PLL_PREDIV_4 = 3,
    CRG_PLL_PREDIV_5 = 4,
    CRG_PLL_PREDIV_6 = 5,
    CRG_PLL_PREDIV_7 = 6,
    CRG_PLL_PREDIV_8 = 7,
} CRG_PllPreDiv;

/**
 * @brief PLL previous divison value in Calc frequency
 */
typedef enum {
    PLL_PREDIV_OUT_1 = 1,
    PLL_PREDIV_OUT_2 = 2,
    PLL_PREDIV_OUT_3 = 3,
    PLL_PREDIV_OUT_4 = 4,
    PLL_PREDIV_OUT_5 = 5,
    PLL_PREDIV_OUT_6 = 6,
    PLL_PREDIV_OUT_7 = 7,
    PLL_PREDIV_OUT_8 = 8,
} PLL_PreDivOut;

/**
 * @brief PLL post division 1 value in register
 */
typedef enum {
    CRG_PLL_POSTDIV_1 = 0,
    CRG_PLL_POSTDIV_2 = 1,
    CRG_PLL_POSTDIV_3 = 2,
    CRG_PLL_POSTDIV_4 = 3,
    CRG_PLL_POSTDIV_5 = 4,
    CRG_PLL_POSTDIV_6 = 5,
    CRG_PLL_POSTDIV_7 = 6,
    CRG_PLL_POSTDIV_8 = 7,
} CRG_PllPostDiv;

/**
 * @brief PLL post division 2 value in register
 */
typedef enum {
    CRG_PLL_POSTDIV2_1 = 0,
    CRG_PLL_POSTDIV2_2 = 1,
    CRG_PLL_POSTDIV2_3 = 2,
    CRG_PLL_POSTDIV2_4 = 3,
    CRG_PLL_POSTDIV2_5 = 4,
    CRG_PLL_POSTDIV2_6 = 5,
    CRG_PLL_POSTDIV2_7 = 6,
    CRG_PLL_POSTDIV2_8 = 7,
} CRG_PllPostDiv2;


/**
 * @brief Core clock selection
 * @note  default select HOSC
 */
typedef enum {
    CRG_CORE_CLK_SELECT_HOSC = 0,
    CRG_CORE_CLK_SELECT_TCXO = 1,
    CRG_CORE_CLK_SELECT_PLL  = 2,
} CRG_CoreClkSelect;

/**
 * @brief Core clock selection 2
 * @note  default select HOSC
 */
typedef enum {
    CRG_CORE_CLK2_SELECT_HOSC = 0,
    CRG_CORE_CLK2_SELECT_TCXO = 1,
    CRG_CORE_CLK2_SELECT_PLL  = 2,
} CRG_CoreClkSelect2;

/**
 * @brief 1M clock selection
 * @note  default select HOSC
 */
typedef enum {
    CRG_1M_CLK_SELECT_HOSC = 0,
    CRG_1M_CLK_SELECT_TCXO = 1,
} CRG_1MClkSelect;

/**
 * @brief PLL frequency multiplication range
 */
typedef enum {
    CRG_PLL_FBDIV_MIN  = 6,
    CRG_PLL_FBDIV_MAX  = 127,
} CRG_PllFbDivRange;

/**
 * @brief PLL diagnose post div selection
 */
typedef enum {
    CRG_PLL_DIG_POST_DIV_SELECT_FREF = 0,
    CRG_PLL_DIG_POST_DIV_SELECT_PLL  = 1,
} CRG_PllDigPostDivInSelect;

/**
 * @brief PLL diagnose loct detect lpsel
 */
typedef enum {
    CRG_PLL_DIG_LOCKDET_LP_SELECT_2048 = 0,
    CRG_PLL_DIG_LOCKDET_LP_SELECT_1024 = 1,
    CRG_PLL_DIG_LOCKDET_LP_SELECT_512  = 2,
    CRG_PLL_DIG_LOCKDET_LP_SELECT_256  = 3,
} CRG_PllDigLockDetLpSelect;

/**
 * @brief PLL Test selection
 */
typedef enum {
    CRG_PLL_TEST_SELECT_FPFD   = 0,
    CRG_PLL_TEST_SELECT_CKFB   = 1,
    CRG_PLL_TEST_SELECT_LOCKDET_OUTPUT   = 2,
    CRG_PLL_TEST_SELECT_FOUTPOSTDIV_128  = 3,
    CRG_PLL_TEST_SELECT_OUTPUT_0 = 4,
} CRG_PllDigTestSelect;

/**
 * @brief CRG Test Clock Select
 */
typedef enum {
    CRG_TEST_CLK_HOSC            = 0x00000001U,
    CRG_TEST_CLK_LOSC            = 0x00000002U,
    CRG_TEST_CLK_TCXO            = 0x00000003U,
    CRG_TEST_CLK_BG_CHOPPER      = 0x00000004U,
    CRG_TEST_CLK_ADC_DIV4        = 0x00000005U,
    CRG_TEST_CLK_HCLK_DIV6       = 0x00000006U,
    CRG_TEST_CLK_HOSC_DIV        = 0x00000007U,
} CRG_TestClkSel;

/**
 * @brief ADC source clock select
 */
typedef enum {
    CRG_ADC_CLK_ASYN_HOSC = 0,
    CRG_ADC_CLK_ASYN_TCXO = 1,
    CRG_ADC_CLK_ASYN_PLL_DIV = 2,
    CRG_ADC_CLK_SYN_CORE = 3,
} CRG_AdcClkSelect;

/**
 * @brief ADC synchronous and asynchronous clock source selection
 */
typedef enum {
    CRG_ADC_CLK_ASYNCHRONOUS = 0,
    CRG_ADC_CLK_SYNCHRONOUS = 1,
} CRG_AdcClkModeSelect;

/**
 * @brief ADC Div set Value
 */
typedef enum {
    CRG_ADC_DIV_1 = 0,
    CRG_ADC_DIV_2 = 1,
    CRG_ADC_DIV_3 = 2,
    CRG_ADC_DIV_4 = 3,
} CRG_AdcDiv;

/**
  * @brief CRG Extra Handle, include CRG's other config
  */
typedef struct {
    CRG_PllPostDiv2     pllPostDiv2;      /**< PLL post 2  ratio */
    CRG_1MClkSelect     clk1MSelect;      /**< 1M clock selection */
    unsigned int        clk1MDiv;         /**< 1M clock ratio */
} CRG_ExtendHandle;

/**
 * @brief PLL Divison Config
 */
typedef struct {
    unsigned int PreDiv;
    unsigned int fbDiv;
    unsigned int postDiv;
} CRG_PllDivCfg;

/**
 * @brief APB_HS_SUBSYS IP config
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int  clkEnMask    : 16;
        unsigned int  softResetReq : 16;
    } BIT;
} volatile CRG_IpWoClkSelectCfg;

/**
 * @brief ADC config
 * @see   PERI_CRG41_Reg and PERI_CRG42_Reg and PERI_CRG43_Reg
 */
typedef union {
    unsigned int value[2];
    struct {
        unsigned int    eflash_cken           : 1;
        unsigned int    reserved0             : 31;
        unsigned int    eflash_clk_tick_cksel : 1;
        unsigned int    reserved1             : 31;
    } BIT;
} volatile CRG_EfcIpCfg;

typedef union {
    unsigned int  value[2];
    struct {
        unsigned int    clk_adc_div0       : 2;
        unsigned int    reserved0          : 6;
        unsigned int    clk_adc_div1       : 2;
        unsigned int    reserved1          : 22;
        unsigned int    clk_adc_cken       : 1;
        unsigned int    reserved2          : 15;
        unsigned int    adc_srst_req       : 1;
        unsigned int    reserved3          : 7;
        unsigned int    cfg_adc_ckmode_sel : 1;
        unsigned int    reserved4          : 7;
    } BIT;
} volatile CRG_AdcIpCfg;

/**
 * @brief DAC config
 * @see   PERI_CRG45_Reg
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int    clkEnMask    : 3;
        unsigned int    reserved0    : 1;
        unsigned int    div          : 12;
        unsigned int    softResetReq : 3;
        unsigned int    reserved1    : 13;
    } BIT;
} volatile CRG_DacIpCfg;

/**
 * @brief ANA config
 * @see   PERI_CRG664_Reg - PERI_CRG677_Reg
 */
typedef union {
    unsigned int  value;
    struct {
        unsigned int    reserved0    : 16;
        unsigned int    ip_srst_req  : 1;
        unsigned int    reserved1    : 15;
    } BIT;
} volatile CRG_AnaIpCfg;

/**
 * @brief IP match info for ip process
 */
typedef struct {
    void         *baseAddr;    /**< Base address of ip */
    unsigned int  offset;      /**< The offset in CRG_RegStruct */
    unsigned int  idx;         /**< index in Reg, for example: 0 -capm0_cken 1 - capm1_cken in PERI_CRG30_Reg */
} CRG_IpMatchInfo;

/**
  * @}
  */

/**
  * @brief CRG REG0 registers union structure definition.
  */
/* Define the union U_CRG_CKREF_CKEL */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_ref_cksel         : 1   ; /* [0] */
        unsigned int    reserved0             : 31  ; /* [31..1] */
    } BIT;
} volatile CRG_CKREF_CKEL_REG;

/* Define the union volatile CRG_PLL_PREDIV */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_prediv            : 4   ; /* [3..0] */
        unsigned int    reserved0             : 28  ; /* [31..4] */
    } BIT;
} volatile CRG_PLL_PREDIV_REG;

/* Define the union volatile CRG_PLL_FBDIV */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_fbdiv             : 8   ; /* [7..0] */
        unsigned int    reserved0             : 24  ; /* [31..8] */
    } BIT;
} volatile CRG_PLL_FBDIV_REG;

/* Define the union volatile CRG_PLL_PSTDIV */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_postdiv1          : 4   ; /* [3..0] */
        unsigned int    pll_postdiv2          : 4   ; /* [7..4] */
        unsigned int    reserved0             : 24  ; /* [31..8] */
    } BIT;
} volatile CRG_PLL_PSTDIV_REG;

/* Define the union volatile CRG_PLL_PD */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_pd                : 1   ; /* [0] */
        unsigned int    reserved0             : 31  ; /* [31..1] */
    } BIT;
} volatile CRG_PLL_PD_REG;

/* Define the union volatile CRG_PLL_CFG0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 13  ; /* [12..0] */
        unsigned int    pll_dig_eb_lockdet    : 1   ; /* [13] */
        unsigned int    pll_dig_e_foutvco     : 1   ; /* [14] */
        unsigned int    pll_dig_e_test        : 1   ; /* [15] */
        unsigned int    pll_dig_lockdet_lpsel : 3   ; /* [18..16] */
        unsigned int    reserved1             : 3   ; /* [21..19] */
        unsigned int    pll_dig_postdiv_in_sel : 1   ; /* [22] */
        unsigned int    reserved2             : 9   ; /* [31..23] */
    } BIT;
} volatile CRG_PLL_CFG0_REG;

/* Define the union volatile CRG_PLL_CFG1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_disc              : 1   ; /* [0] */
        unsigned int    pll_pull_h            : 1   ; /* [1] */
        unsigned int    pll_pull_l            : 1   ; /* [2] */
        unsigned int    pll_icp_sel           : 4   ; /* [6..3] */
        unsigned int    reserved0             : 1   ; /* [7] */
        unsigned int    pll_test_dc           : 5   ; /* [12..8] */
        unsigned int    reserved1             : 6   ; /* [18..13] */
        unsigned int    pll_dig_test_sel      : 3   ; /* [21..19] */
        unsigned int    reserved2             : 10  ; /* [31..22] */
    } BIT;
} volatile CRG_PLL_CFG1_REG;

/* Define the union volatile CRG_PLL_LOCK */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_lock              : 1   ; /* [0] */
        unsigned int    reserved0             : 31  ; /* [31..1] */
    } BIT;
} volatile CRG_PLL_LOCK_REG;

/* Define the union volatile CRG_PLL_LOCK_DGL */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pll_lock_deglitch     : 1   ; /* [0] */
        unsigned int    reserved0             : 31  ; /* [31..1] */
    } BIT;
} volatile CRG_PLL_LOCK_DGL_REG;

/* Define the union volatile CRG_SYS_CKSEL */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_pst1_sw_sel       : 2   ; /* [1..0] */
        unsigned int    reserved0             : 2   ; /* [3..2] */
        unsigned int    clk_pst2_sw_sel       : 2   ; /* [5..4] */
        unsigned int    reserved1             : 2   ; /* [7..6] */
        unsigned int    reserved2             : 24  ; /* [31..8] */
    } BIT;
} volatile CRG_SYS_CKSEL_REG;

/* Define the union volatile CRG_PVD_RST_EN */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    pvd_rst_enable        : 1   ; /* [0] */
        unsigned int    reserved0             : 3   ; /* [3..1] */
        unsigned int    reserved1             : 28  ; /* [31..4] */
    } BIT;
} volatile CRG_PVD_RST_EN_REG;

/* Define the union volatile CRG_1M_INI_CKSEL */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_1m_ini_cksel      : 1   ; /* [0] */
        unsigned int    reserved0             : 3   ; /* [3..1] */
        unsigned int    reserved1             : 28  ; /* [31..4] */
    } BIT;
} volatile CRG_1M_INI_CKSEL_REG;

/* Define the union volatile CRG_1M_DIV */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_1m_div            : 6   ; /* [5..0] */
        unsigned int    reserved0             : 2   ; /* [7..6] */
        unsigned int    reserved1             : 24  ; /* [31..8] */
    } BIT;
} volatile CRG_1M_DIV_REG;

/* Define the union volatile CRG_UART0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart0_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    uart0_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_UART0_REG;

/* Define the union volatile CRG_UART1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart1_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    uart1_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_UART1_REG;

/* Define the union volatile CRG_UART2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart2_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    uart2_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_UART2_REG;

/* Define the union volatile CRG_UART3 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart3_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    uart3_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_UART3_REG;

/* Define the union volatile CRG_UART4 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    uart4_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    uart4_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_UART4_REG;

/* Define the union volatile CRG_SPI0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    spi0_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    spi0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_SPI0_REG;

/* Define the union volatile CRG_SPI1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    spi1_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    spi1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_SPI1_REG;

/* Define the union volatile CRG_I2C */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    i2c_cken              : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    i2c_srst_req          : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_I2C_REG;

/* Define the union volatile CRG_TIMER0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer0_cken           : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    timer0_srst_req       : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_TIMER0_REG;

/* Define the union volatile CRG_TIMER1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer1_cken           : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    timer1_srst_req       : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_TIMER1_REG;

/* Define the union volatile CRG_TIMER2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer2_cken           : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    timer2_srst_req       : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_TIMER2_REG;

/* Define the union volatile CRG_TIMER3 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    timer3_cken           : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    timer3_srst_req       : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_TIMER3_REG;

/* Define the union volatile CRG_CAPM0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    capm0_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    capm0_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CAPM0_REG;

/* Define the union volatile CRG_CAPM1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    capm1_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    capm1_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CAPM1_REG;

/* Define the union volatile CRG_CAPM2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    capm2_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    capm2_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CAPM2_REG;

/* Define the union volatile CRG_CAN */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    can0_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    can0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CAN_REG;

/* Define the union volatile CRG_CAN1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    can1_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    can1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CAN1_REG;

/* Define the union volatile CRG_DMA */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    dma_cken              : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    dma_srst_req          : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_DMA_REG;

/* Define the union volatile CRG_CMM */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    cmm_cken              : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    cmm_srst_req          : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CMM_REG;

/* Define the union volatile CRG_CFD */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    cfd_cken              : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    cfd_srst_req          : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CFD_REG;

/* Define the union volatile CRG_CRC */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    crc_cken              : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    crc_srst_req          : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_CRC_REG;

/* Define the union volatile CRG_APT0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt0_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT0_REG;

/* Define the union volatile CRG_APT1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt1_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT1_REG;

/* Define the union volatile CRG_APT2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt2_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt2_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT2_REG;

/* Define the union volatile CRG_APT3 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt3_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt3_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT3_REG;

/* Define the union volatile CRG_APT4 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt4_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt4_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT4_REG;

/* Define the union volatile CRG_APT5 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt5_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt5_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT5_REG;

/* Define the union volatile CRG_APT6 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt6_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt6_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT6_REG;

/* Define the union volatile CRG_APT7 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt7_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt7_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT7_REG;

/* Define the union volatile CRG_APT8 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    apt8_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    apt8_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_APT8_REG;

/* Define the union volatile CRG_GPT0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpt0_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpt0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPT0_REG;

/* Define the union volatile CRG_GPT1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpt1_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpt1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPT1_REG;

/* Define the union volatile CRG_GPIO0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio0_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio0_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO0_REG;

/* Define the union volatile CRG_GPIO1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio1_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio1_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO1_REG;

/* Define the union volatile CRG_GPIO2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio2_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio2_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO2_REG;

/* Define the union volatile CRG_GPIO3 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio3_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio3_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO3_REG;

/* Define the union volatile CRG_GPIO4 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio4_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio4_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO4_REG;

/* Define the union volatile CRG_GPIO5 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio5_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio5_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO5_REG;

/* Define the union volatile CRG_GPIO6 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio6_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio6_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO6_REG;

/* Define the union volatile CRG_GPIO7 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio7_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio7_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO7_REG;

/* Define the union volatile CRG_GPIO8 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio8_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio8_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO8_REG;

/* Define the union volatile CRG_GPIO9 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    gpio9_cken            : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    gpio9_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_GPIO9_REG;

/* Define the union volatile CRG_QDM0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    qdm0_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    qdm0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_QDM0_REG;

/* Define the union volatile CRG_QDM1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    qdm1_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    qdm1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_QDM1_REG;

/* Define the union volatile CRG_QDM2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    qdm2_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    qdm2_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_QDM2_REG;

/* Define the union volatile CRG_QDM3 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    qdm3_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    qdm3_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_QDM3_REG;

/* Define the union volatile CRG_EFLASH */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    eflash_cken           : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    reserved1             : 16  ; /* [31..16] */
    } BIT;
} volatile CRG_EFLASH_REG;

/* Define the union volatile CRG_TEST_CKSEL */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    test_clk_en           : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    test_clk_sel          : 3   ; /* [18..16] */
        unsigned int    reserved1             : 13  ; /* [31..19] */
    } BIT;
} volatile CRG_TEST_CKSEL_REG;

/* Define the union volatile CRG_ADC0_DIV */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_adc0_div0         : 2   ; /* [1..0] */
        unsigned int    reserved0             : 6   ; /* [7..2] */
        unsigned int    clk_adc0_div1         : 2   ; /* [9..8] */
        unsigned int    reserved1             : 6   ; /* [15..10] */
        unsigned int    reserved2             : 16  ; /* [31..16] */
    } BIT;
} volatile CRG_ADC0_DIV_REG;

/* Define the union volatile CRG_ADC0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    adc0_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    adc0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    adc0_clk_mode         : 1   ; /* [24] */
        unsigned int    reserved2             : 7   ; /* [31..25] */
    } BIT;
} volatile CRG_ADC0_REG;

/* Define the union volatile CRG_ADC1_DIV */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_adc1_div0         : 2   ; /* [1..0] */
        unsigned int    reserved0             : 6   ; /* [7..2] */
        unsigned int    clk_adc1_div1         : 2   ; /* [9..8] */
        unsigned int    reserved1             : 6   ; /* [15..10] */
        unsigned int    reserved2             : 16  ; /* [31..16] */
    } BIT;
} volatile CRG_ADC1_DIV_REG;

/* Define the union volatile CRG_ADC1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    adc1_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    adc1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    adc1_clk_mode         : 1   ; /* [24] */
        unsigned int    reserved2             : 7   ; /* [31..25] */
    } BIT;
} volatile CRG_ADC1_REG;

/* Define the union volatile CRG_ADC2_DIV */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_adc2_div0         : 2   ; /* [1..0] */
        unsigned int    reserved0             : 6   ; /* [7..2] */
        unsigned int    clk_adc2_div1         : 2   ; /* [9..8] */
        unsigned int    reserved1             : 6   ; /* [15..10] */
        unsigned int    reserved2             : 16  ; /* [31..16] */
    } BIT;
} volatile CRG_ADC2_DIV_REG;

/* Define the union volatile CRG_ADC2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    adc2_cken             : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    adc2_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    adc2_clk_mode         : 1   ; /* [24] */
        unsigned int    reserved2             : 7   ; /* [31..25] */
    } BIT;
} volatile CRG_ADC2_REG;

/* Define the union volatile CRG_BG_CHOPPER */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_bg_chopper_div    : 6   ; /* [5..0] */
        unsigned int    reserved0             : 2   ; /* [7..6] */
        unsigned int    clk_bg_chopper_div_bypass : 1   ; /* [8] */
        unsigned int    reserved1             : 23  ; /* [31..9] */
    } BIT;
} volatile CRG_BG_CHOPPER_REG;

/* Define the union volatile CRG_ANA */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_ana_cken          : 1   ; /* [0] */
        unsigned int    reserved0             : 15  ; /* [15..1] */
        unsigned int    ana_srst_req          : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_ANA_REG;

/* Define the union volatile CRG_VREF */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    vref_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 7   ; /* [23..17] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_VREF_REG;

/* Define the union volatile CRG_ACMP0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    acmp0_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_ACMP0_REG;

/* Define the union volatile CRG_ACMP1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    acmp1_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_ACMP1_REG;

/* Define the union volatile CRG_ACMP2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    acmp2_srst_req        : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_ACMP2_REG;

/* Define the union volatile CRG_DAC0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    dac0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_DAC0_REG;

/* Define the union volatile CRG_DAC1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    dac1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_DAC1_REG;

/* Define the union volatile CRG_DAC2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    dac2_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_DAC2_REG;

/* Define the union volatile CRG_PGA0 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    pga0_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_PGA0_REG;

/* Define the union volatile CRG_PGA1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    pga1_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_PGA1_REG;

/* Define the union volatile CRG_PGA2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    reserved0             : 16  ; /* [15..0] */
        unsigned int    pga2_srst_req         : 1   ; /* [16] */
        unsigned int    reserved1             : 15  ; /* [31..17] */
    } BIT;
} volatile CRG_PGA2_REG;

/* Define the union volatile CRG_HPM */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hpm_cken              : 1   ; /* [0] */
        unsigned int    reserved0             : 3   ; /* [3..1] */
        unsigned int    hpm_1m_cken           : 1   ; /* [4] */
        unsigned int    reserved1             : 11  ; /* [15..5] */
        unsigned int    hpm_srst_req          : 1   ; /* [16] */
        unsigned int    reserved2             : 3   ; /* [19..17] */
        unsigned int    hpm_1m_srst_req       : 1   ; /* [20] */
        unsigned int    reserved3             : 3   ; /* [23..21] */
        unsigned int    reserved4             : 8   ; /* [31..24] */
    } BIT;
} volatile CRG_HPM_REG;

/* Define the union volatile CRG_DEBUG */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    clk_pst1_sw_mux_sel   : 2   ; /* [1..0] */
        unsigned int    reserved0             : 2   ; /* [3..2] */
        unsigned int    clk_pst2_sw_mux_sel   : 2   ; /* [5..4] */
        unsigned int    reserved1             : 2   ; /* [7..6] */
        unsigned int    reserved2             : 24  ; /* [31..8] */
    } BIT;
} volatile CRG_DEBUG_REG;

/* Define the union volatile HOSC_CTRL1 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hosc_ctrim_value      : 10  ; /* [9..0] */
        unsigned int    reserved0             : 22  ; /* [31..10] */
    } BIT;
} volatile HOSC_CTRL1_REG;

/* Define the union volatile HOSC_CTRL2 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hosc_rtrim_value      : 8   ; /* [7..0] */
        unsigned int    hosc_itrim_value      : 4   ; /* [11..8] */
        unsigned int    hosc_sel_vref         : 3   ; /* [14..12] */
        unsigned int    reserved0             : 1   ; /* [15] */
        unsigned int    hosc_kvco_sel         : 2   ; /* [17..16] */
        unsigned int    reserved1             : 2   ; /* [19..18] */
        unsigned int    hosc_lpfr_sel         : 4   ; /* [23..20] */
        unsigned int    reserved2             : 8   ; /* [31..24] */
    } BIT;
} volatile HOSC_CTRL2_REG;

/* Define the union volatile HOSC_CTRL3 */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hosc_ate_en           : 1   ; /* [0] */
        unsigned int    hosc_dft_en           : 1   ; /* [1] */
        unsigned int    reserved0             : 2   ; /* [3..2] */
        unsigned int    hosc_test             : 10  ; /* [13..4] */
        unsigned int    reserved1             : 2   ; /* [15..14] */
        unsigned int    hosc_div_sel          : 2   ; /* [17..16] */
        unsigned int    reserved2             : 6   ; /* [23..18] */
        unsigned int    hosc_start_up         : 1   ; /* [24] */
        unsigned int    reserved3             : 3   ; /* [27..25] */
        unsigned int    hosc_vsel_ldo_ref     : 2   ; /* [29..28] */
        unsigned int    reserved4             : 2   ; /* [31..30] */
    } BIT;
} volatile HOSC_CTRL3_REG;

/* Define the union volatile HOSC_PD */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hosc_pd               : 1   ; /* [0] */
        unsigned int    reserved0             : 31  ; /* [31..1] */
    } BIT;
} volatile HOSC_PD_REG;

/* Define the union volatile HOSC_LOCK */
typedef union {
    unsigned int reg;
    struct {
        unsigned int    hosc_lock             : 1   ; /* [0] */
        unsigned int    reserved0             : 31  ; /* [31..1] */
    } BIT;
} volatile HOSC_LOCK_REG;

typedef struct {
    CRG_CKREF_CKEL_REG         CRG_CKREF_CKEL;     /* 0x0 */
    CRG_PLL_PREDIV_REG         CRG_PLL_PREDIV;     /* 0x4 */
    CRG_PLL_FBDIV_REG          CRG_PLL_FBDIV;       /* 0x8 */
    CRG_PLL_PSTDIV_REG         CRG_PLL_PSTDIV;     /* 0xc */
    CRG_PLL_PD_REG             CRG_PLL_PD;             /* 0x10 */
    CRG_PLL_CFG0_REG           CRG_PLL_CFG0;         /* 0x14 */
    CRG_PLL_CFG1_REG           CRG_PLL_CFG1;         /* 0x18 */
    CRG_PLL_LOCK_REG           CRG_PLL_LOCK;         /* 0x1c */
    CRG_PLL_LOCK_DGL_REG       CRG_PLL_LOCK_DGL; /* 0x20 */
    unsigned int               reserved0[55];       /* 0x24~0xfc */
    CRG_SYS_CKSEL_REG          CRG_SYS_CKSEL;       /* 0x100 */
    CRG_PVD_RST_EN_REG         CRG_PVD_RST_EN;     /* 0x104 */
    CRG_1M_INI_CKSEL_REG       CRG_1M_INI_CKSEL; /* 0x108 */
    CRG_1M_DIV_REG             CRG_1M_DIV;             /* 0x10c */
    unsigned int               reserved1[12];       /* 0x110~0x13c */
    CRG_UART0_REG              CRG_UART0;               /* 0x140 */
    CRG_UART1_REG              CRG_UART1;               /* 0x144 */
    CRG_UART2_REG              CRG_UART2;               /* 0x148 */
    CRG_UART3_REG              CRG_UART3;               /* 0x14c */
    CRG_UART4_REG              CRG_UART4;               /* 0x150 */
    unsigned int               reserved2[11];       /* 0x154~0x17c */
    CRG_SPI0_REG               CRG_SPI0;                 /* 0x180 */
    CRG_SPI1_REG               CRG_SPI1;                 /* 0x184 */
    unsigned int               reserved3[14];       /* 0x188~0x1bc */
    CRG_I2C_REG                CRG_I2C;                   /* 0x1c0 */
    unsigned int               reserved4[31];       /* 0x1c4~0x23c */
    CRG_TIMER0_REG             CRG_TIMER0;             /* 0x240 */
    CRG_TIMER1_REG             CRG_TIMER1;             /* 0x244 */
    CRG_TIMER2_REG             CRG_TIMER2;             /* 0x248 */
    CRG_TIMER3_REG             CRG_TIMER3;             /* 0x24c */
    unsigned int               reserved5[12];       /* 0x250~0x27c */
    CRG_CAPM0_REG              CRG_CAPM0;               /* 0x280 */
    CRG_CAPM1_REG              CRG_CAPM1;               /* 0x284 */
    CRG_CAPM2_REG              CRG_CAPM2;               /* 0x288 */
    unsigned int               reserved6[13];       /* 0x28c~0x2bc */
    CRG_CAN_REG               CRG_CAN;                 /* 0x2c0 */
    CRG_CAN1_REG               CRG_CAN1;                 /* 0x2c4 */
    unsigned int               reserved7[14];       /* 0x2c8~0x2fc */
    CRG_DMA_REG                CRG_DMA;                   /* 0x300 */
    unsigned int               reserved8[15];       /* 0x304~0x33c */
    CRG_CMM_REG                CRG_CMM;                   /* 0x340 */
    CRG_CFD_REG                CRG_CFD;                   /* 0x344 */
    unsigned int               reserved9[14];       /* 0x348~0x37c */
    CRG_CRC_REG                CRG_CRC;                   /* 0x380 */
    unsigned int               reserved10[31];      /* 0x384~0x3fc */
    CRG_APT0_REG               CRG_APT0;                 /* 0x400 */
    CRG_APT1_REG               CRG_APT1;                 /* 0x404 */
    CRG_APT2_REG               CRG_APT2;                 /* 0x408 */
    CRG_APT3_REG               CRG_APT3;                 /* 0x40c */
    CRG_APT4_REG               CRG_APT4;                 /* 0x410 */
    CRG_APT5_REG               CRG_APT5;                 /* 0x414 */
    CRG_APT6_REG               CRG_APT6;                 /* 0x418 */
    CRG_APT7_REG               CRG_APT7;                 /* 0x41c */
    CRG_APT8_REG               CRG_APT8;                 /* 0x420 */
    unsigned int               reserved11[7];       /* 0x424~0x43c */
    CRG_GPT0_REG               CRG_GPT0;                 /* 0x440 */
    CRG_GPT1_REG               CRG_GPT1;                 /* 0x444 */
    unsigned int               reserved12[14];      /* 0x448~0x47c */
    CRG_GPIO0_REG              CRG_GPIO0;               /* 0x480 */
    CRG_GPIO1_REG              CRG_GPIO1;               /* 0x484 */
    CRG_GPIO2_REG              CRG_GPIO2;               /* 0x488 */
    CRG_GPIO3_REG              CRG_GPIO3;               /* 0x48c */
    CRG_GPIO4_REG              CRG_GPIO4;               /* 0x490 */
    CRG_GPIO5_REG              CRG_GPIO5;               /* 0x494 */
    CRG_GPIO6_REG              CRG_GPIO6;               /* 0x498 */
    CRG_GPIO7_REG              CRG_GPIO7;               /* 0x49c */
    CRG_GPIO8_REG              CRG_GPIO8;               /* 0x4a0 */
    CRG_GPIO9_REG              CRG_GPIO9;               /* 0x4a4 */
    unsigned int               reserved13[6];       /* 0x4a8~0x4bc */
    CRG_QDM0_REG               CRG_QDM0;                 /* 0x4c0 */
    CRG_QDM1_REG               CRG_QDM1;                 /* 0x4c4 */
    CRG_QDM2_REG               CRG_QDM2;                 /* 0x4c8 */
    CRG_QDM3_REG               CRG_QDM3;                 /* 0x4cc */
    unsigned int               reserved14[12];      /* 0x4d0~0x4fc */
    CRG_EFLASH_REG             CRG_EFLASH;             /* 0x500 */
    unsigned int               reserved15[318];     /* 0x504~0x9f8 */
    CRG_TEST_CKSEL_REG         CRG_TEST_CKSEL;     /* 0x9fc */
    CRG_ADC0_DIV_REG           CRG_ADC0_DIV;         /* 0xa00 */
    CRG_ADC0_REG               CRG_ADC0;                 /* 0xa04 */
    CRG_ADC1_DIV_REG           CRG_ADC1_DIV;         /* 0xa08 */
    CRG_ADC1_REG               CRG_ADC1;                 /* 0xa0c */
    CRG_ADC2_DIV_REG           CRG_ADC2_DIV;         /* 0xa10 */
    CRG_ADC2_REG               CRG_ADC2;                 /* 0xa14 */
    unsigned int               reserved16[10];      /* 0xa18~0xa3c */
    CRG_BG_CHOPPER_REG         CRG_BG_CHOPPER;     /* 0xa40 */
    unsigned int               reserved17[3];       /* 0xa44~0xa4c */
    CRG_ANA_REG                CRG_ANA;                   /* 0xa50 */
    unsigned int               reserved18[3];       /* 0xa54~0xa5c */
    CRG_VREF_REG               CRG_VREF;                 /* 0xa60 */
    unsigned int               reserved19[3];       /* 0xa64~0xa6c */
    CRG_ACMP0_REG              CRG_ACMP0;               /* 0xa70 */
    CRG_ACMP1_REG              CRG_ACMP1;               /* 0xa74 */
    CRG_ACMP2_REG              CRG_ACMP2;               /* 0xa78 */
    unsigned int               reserved20;          /* 0xa7c */
    CRG_DAC0_REG               CRG_DAC0;                 /* 0xa80 */
    CRG_DAC1_REG               CRG_DAC1;                 /* 0xa84 */
    CRG_DAC2_REG               CRG_DAC2;                 /* 0xa88 */
    unsigned int               reserved21;          /* 0xa8c */
    CRG_PGA0_REG               CRG_PGA0;                 /* 0xa90 */
    CRG_PGA1_REG               CRG_PGA1;                 /* 0xa94 */
    CRG_PGA2_REG               CRG_PGA2;                 /* 0xa98 */
    unsigned int               reserved22[25];      /* 0xa9c~0xafc */
    CRG_HPM_REG                CRG_HPM;                   /* 0xb00 */
    unsigned int               reserved23[254];     /* 0xb04~0xef8 */
    CRG_DEBUG_REG              CRG_DEBUG;               /* 0xefc */
    HOSC_CTRL1_REG             HOSC_CTRL1;             /* 0xf00 */
    HOSC_CTRL2_REG             HOSC_CTRL2;             /* 0xf04 */
    HOSC_CTRL3_REG             HOSC_CTRL3;             /* 0xf08 */
    HOSC_PD_REG                HOSC_PD;                   /* 0xf0c */
    HOSC_LOCK_REG              HOSC_LOCK;               /* 0xf10 */
} volatile CRG_RegStruct;

/**
  * @}
  */

/* Parameter Check -----------------------------------------------------------*/
/**
  * @brief Verify pll_ref_cksel configuration
  * @param clkSelect pll_ref_cksel
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllRefClkSelect(CRG_PllRefClkSelect clkSelect)
{
    return ((clkSelect == CRG_PLL_REF_CLK_SELECT_HOSC) ||
            (clkSelect == CRG_PLL_REF_CLK_SELECT_XTAL));
}

/**
  * @brief Verify Crg pll_prediv configuration
  * @param preDiv pll prediv value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllPreDiv(CRG_PllPreDiv preDiv)
{
    return ((preDiv >= CRG_PLL_PREDIV_1) &&
            (preDiv <= CRG_PLL_PREDIV_8));
}

/**
  * @brief Verify Crg pll_postdiv configuration
  * @param postDiv  pll_postdiv value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllPostDiv(CRG_PllPostDiv postDiv)
{
    return ((postDiv >= CRG_PLL_POSTDIV_1) &&
            (postDiv <= CRG_PLL_POSTDIV_8));
}

/**
  * @brief Verify Crg pll_postdiv2 configuration
  * @param postDiv  pll_postdiv2 value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllPostDiv2(CRG_PllPostDiv2 postDiv)
{
    return ((postDiv >= CRG_PLL_POSTDIV2_1) &&
            (postDiv <= CRG_PLL_POSTDIV2_8));
}

/**
  * @brief Verify Crg pll_fbdiv configuration
  * @param fbDiv  pll fbdiv value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllFbDiv(unsigned int fbDiv)
{
    return (fbDiv <= CRG_PLL_FBDIV_MAX);
}

/**
  * @brief Verify Crg pll_digpostdiv_in_sel configuration
  * @param select  pll_digpostdiv_in_sel value
  * @retval true
  * @retval false
  */
static inline bool IsCrgPllDigPostDivInSel(CRG_PllDigPostDivInSelect select)
{
    return ((select == CRG_PLL_DIG_POST_DIV_SELECT_FREF) ||
            (select == CRG_PLL_DIG_POST_DIV_SELECT_PLL));
}

/**
  * @brief Verify Crg core_cksel configuration
  * @param select  core_cksel value
  * @retval true
  * @retval false
  */
static inline bool IsCrgCoreCkSel(CRG_CoreClkSelect select)
{
    return ((select == CRG_CORE_CLK_SELECT_HOSC) ||
            (select == CRG_CORE_CLK_SELECT_TCXO) ||
            (select == CRG_CORE_CLK_SELECT_PLL));
}

/**
  * @brief Verify Crg configuration
  * @param select  1M clock selection
  * @retval true
  * @retval false
  */
static inline bool IsCrg1MCkSel(CRG_1MClkSelect select)
{
    return ((select == CRG_1M_CLK_SELECT_HOSC) ||
            (select == CRG_1M_CLK_SELECT_TCXO));
}

/**
  * @brief Verify Crg configuration
  * @param div  1M clock ratio
  * @retval true
  * @retval false
  */
static inline bool IsCrg1MCkDiv(unsigned int div)
{
    return (div <= CRG_1MHZ_CLK_MAX_DIV);
}

/**
  * @brief Verify Crg Ip (exclude adc) clock enable configuration
  * @param enable  ip clock enable value
  * @retval true
  * @retval false
  */
static inline bool IsCrgIpClkEnable(unsigned int enable)
{
    return ((enable == IP_CLK_DISABLE) ||
            (enable == IP_CLK_ENABLE));
}

/**
  * @brief Check the PLL PreDiv is valid or not
  * @param clkPllRef PLL Refer clock
  * @param preDiv PLL Previous Divsion
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidPreDiv(unsigned int pllRefFreq, unsigned int preDiv)
{
    unsigned int freq = pllRefFreq;
    if (preDiv != 0) {
        freq /= preDiv;
    }
    return (freq >= CRG_CLK_PFD_MIN_FREQ) && (freq <= CRG_CLK_PFD_MAX_FREQ);
}

/**
  * @brief Check the PLL FbDiv is valid or not
  * @param clkPfdFreq PLL PFD clock
  * @param fdDiv PLL FD Divsion
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidFdDiv(unsigned int clkPfdFreq, unsigned int fdDiv)
{
    if (clkPfdFreq > 30000000U) {  /* The maximum speed of the external clock source is 30000000U. */
        return false;
    } else if (fdDiv > CRG_PLL_FBDIV_MAX) {
        return false;
    }

    unsigned int freq = (fdDiv > 0x6) ? (clkPfdFreq * fdDiv) : (clkPfdFreq * 0x6); /* 0x0-0x6: divided by 0x6 */
    return (freq >= CRG_CLK_VCO_MIN_FREQ) && (freq <= CRG_CLK_VCO_MAX_FREQ);
}

/**
  * @brief Check the PLL PostDiv is valid or not
  * @param clkPllRef PLL Vco clock
  * @param postDiv PLL Post Divsion
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidPostDiv(unsigned int clkVcoFreq, unsigned int postDiv)
{
    unsigned int freq = clkVcoFreq;
    if (postDiv <= CRG_PLL_POSTDIV_8) {
        freq = (freq >> postDiv);
    }
    return (freq <= CRG_CLK_TARGET_MAX_FREQ);
}

/**
  * @brief Check the PLL PostDiv is valid or not
  * @param clkPllRef PLL Vco clock
  * @param postDiv2 PLL Post Divsion2
  * @retval true
  * @retval false
  */
static inline bool IsCrgValidPostDiv2(unsigned int clkVcoFreq, unsigned int postDiv2)
{
    unsigned int freq = clkVcoFreq;
    if (postDiv2 != 0) {
        freq /= (postDiv2 + 1);
    }
    return (freq <= CRG_CLK_PST2_MAX_FREQ);
}

/**
  * @brief Check the adc clock select value
  * @param adcClkSelect the value of adc clock select
  * @retval true
  * @retval false
  */
static inline bool IsCrgAdcClkModeSelect(CRG_AdcClkSelect adcClkSelect)
{
    return (adcClkSelect == CRG_ADC_CLK_ASYN_HOSC || \
            adcClkSelect == CRG_ADC_CLK_ASYN_TCXO || \
            adcClkSelect == CRG_ADC_CLK_ASYN_PLL_DIV || \
            adcClkSelect == CRG_ADC_CLK_SYN_CORE);
}

/**
  * @brief Check the adc clock div value
  * @param div the value of adc clock div
  * @retval true
  * @retval false
  */
static inline bool IsCrgAdcClkDiv(CRG_AdcDiv div)
{
    return (div == CRG_ADC_DIV_1 || \
            div == CRG_ADC_DIV_2 || \
            div == CRG_ADC_DIV_3 || \
            div == CRG_ADC_DIV_4);
}

/**
  * @brief Set Pll Ref clock select
  * @param clk     Clock register base address
  * @param clkSel  clock source select
  * @retval None
  */
static inline void DCL_CRG_SetPllRefClkSel(CRG_RegStruct *clk, CRG_PllRefClkSelect clkSel)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllRefClkSelect(clkSel));
    clk->CRG_CKREF_CKEL.BIT.pll_ref_cksel = (unsigned int)clkSel;
}

/**
  * @brief Get Pll Ref clock selection
  * @param clk             Clock register base address
  * @retval pll_ref_cksel  Ref clock selection
  */
static inline CRG_PllRefClkSelect DCL_CRG_GetPllRefClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllRefClkSelect)clk->CRG_CKREF_CKEL.BIT.pll_ref_cksel;
}

/**
  * @brief Set prevous division ratio
  * @param clk     Clock register base address
  * @param preDiv  prevous division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPreDiv(CRG_RegStruct *clk, CRG_PllPreDiv preDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPreDiv(preDiv));
    clk->CRG_PLL_PREDIV.BIT.pll_prediv = (unsigned int)preDiv;
}

/**
  * @brief Get prevous division ratio
  * @param clk      Clock register base address
  * @retval prediv  prevous division ratio
  */
static inline CRG_PllPreDiv DCL_CRG_GetPllPreDiv(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPreDiv)clk->CRG_PLL_PREDIV.BIT.pll_prediv;
}

/**
  * @brief Set PLL frequency multiplication factor
  * @param clk    Clock register base address
  * @param fbDiv  Multiplication factor
  * @retval None
  */
static inline void DCL_CRG_SetPllFbDiv(CRG_RegStruct *clk, unsigned int fbDiv)
{
    unsigned int div = fbDiv;
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllFbDiv(fbDiv));
    clk->CRG_PLL_FBDIV.BIT.pll_fbdiv = div;
}

/**
  * @brief Get PLL frequency multiplication factor
  * @param clk         Clock register base address
  * @retval pll_fbdiv  Multiplication factor
  */
static inline unsigned int DCL_CRG_GetPllFbDiv(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->CRG_PLL_FBDIV.BIT.pll_fbdiv;
}

/**
  * @brief Set PLL post division ratio
  * @param clk     Clock register base address
  * @param postDiv Post division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPostDiv1(CRG_RegStruct *clk, CRG_PllPostDiv postDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPostDiv(postDiv));
    clk->CRG_PLL_PSTDIV.BIT.pll_postdiv1 = (unsigned int)postDiv;
}

/**
  * @brief Get PLL post division ratio
  * @param clk           Clock register base address
  * @retval pll_postdiv  Post division ratio
  */
static inline CRG_PllPostDiv DCL_CRG_GetPllPostDiv1(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPostDiv)clk->CRG_PLL_PSTDIV.BIT.pll_postdiv1;
}

/**
  * @brief Set PLL post division ratio
  * @param clk     Clock register base address
  * @param postDiv Post division ratio
  * @retval None
  */
static inline void DCL_CRG_SetPllPostDiv2(CRG_RegStruct *clk, CRG_PllPostDiv postDiv)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgPllPostDiv(postDiv));
    clk->CRG_PLL_PSTDIV.BIT.pll_postdiv2 = (unsigned int)postDiv;
}

/**
  * @brief Get PLL post division ratio
  * @param clk           Clock register base address
  * @retval pll_postdiv  Post division ratio
  */
static inline CRG_PllPostDiv DCL_CRG_GetPllPostDiv2(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return (CRG_PllPostDiv)clk->CRG_PLL_PSTDIV.BIT.pll_postdiv2;
}

/**
  * @brief Set PLL Power
  * @param clk  Clock register base address
  * @param pd   pll power down or not
  * @retval None
  */
static inline void DCL_CRG_SetPllPd(CRG_RegStruct *clk, bool pd)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->CRG_PLL_PD.BIT.pll_pd = (unsigned int)pd;
}

/**
  * @brief Get PLL power status
  * @param clk  Clock register base address
  * @retval 0: power up, 1: power down
  */
static inline bool DCL_CRG_GetPllPd(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->CRG_PLL_PD.BIT.pll_pd;
}

/**
  * @brief Set core clock selection
  * @param clk  Clock register base address
  * @param select  Core clock selection
  * @retval None
  */
static inline void DCL_CRG_SetCoreClkSel(CRG_RegStruct *clk, CRG_CoreClkSelect select)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgCoreCkSel(select));
    clk->CRG_SYS_CKSEL.BIT.clk_pst1_sw_sel = select;
}

/**
  * @brief Get core clock selection
  * @param clk  Clock register base address
  * @retval Core clock selection
  */
static inline unsigned int DCL_CRG_GetCoreClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->CRG_SYS_CKSEL.BIT.clk_pst1_sw_sel;
}

/**
  * @brief Set core clock selection
  * @param clk  Clock register base address
  * @param select  Core clock selection
  * @retval None
  */
static inline void DCL_CRG_SetAdcAsynClkSel(CRG_RegStruct *clk, CRG_CoreClkSelect select)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrgCoreCkSel(select));
    clk->CRG_SYS_CKSEL.BIT.clk_pst2_sw_sel = select;
}

/**
  * @brief Get adc core clock selection
  * @param clk  Clock register base address
  * @retval Core clock selection
  */
static inline unsigned int DCL_CRG_GetAdcAsynClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->CRG_SYS_CKSEL.BIT.clk_pst2_sw_sel;
}

/**
  * @brief Set 1M clock selection
  * @param clk  Clock register base address
  * @param select  Core clock selection
  * @retval None
  */
static inline void DCL_CRG_Set1MClkSel(CRG_RegStruct *clk, CRG_1MClkSelect select)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(IsCrg1MCkSel(select));
    clk->CRG_1M_INI_CKSEL.BIT.clk_1m_ini_cksel = select;
}

/**
  * @brief Get 1M clock selection
  * @param clk  Clock register base address
  * @retval Core clock selection
  */
static inline unsigned int DCL_CRG_Get1MClkSel(const CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    return clk->CRG_1M_INI_CKSEL.BIT.clk_1m_ini_cksel;
}

/**
  * @brief Set 1M clock division ratio
  * @param clk  Clock register base address
  * @param div  Division ratio
  * @retval None
  */
static inline void DCL_CRG_Set1MClkDiv(CRG_RegStruct *clk, unsigned int div)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(div <= CRG_1MHZ_CLK_MAX_DIV);
    clk->CRG_1M_DIV.BIT.clk_1m_div = div;
}

/**
  * @brief  Enable test clock function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_TestClkEnable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->CRG_TEST_CKSEL.BIT.test_clk_en = BASE_CFG_ENABLE;   /* Enable the test clock. */
}

/**
  * @brief  Disable test clock function
  * @param clk  Clock register base address
  * @retval None
  */
static inline void DCL_CRG_TestClkDisable(CRG_RegStruct *clk)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    clk->CRG_TEST_CKSEL.BIT.test_clk_en = BASE_CFG_DISABLE;  /* Disable the test clock. */
}

/**
  * @brief CRG test clock select.
  * @param clk  Clock register base address
  * @param clkSel  Clock select.
  * @retval None
  */
static inline void DCL_CRG_TestClkSel(CRG_RegStruct *clk, CRG_TestClkSel clkSel)
{
    CRG_ASSERT_PARAM(IsCRGInstance(clk));
    CRG_PARAM_CHECK_NO_RET(clkSel >= CRG_TEST_CLK_HOSC);
    CRG_PARAM_CHECK_NO_RET(clkSel <= CRG_TEST_CLK_HOSC_DIV);
    clk->CRG_TEST_CKSEL.BIT.test_clk_sel = clkSel; /* Set the test clock select. */
}
/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_CRG_IP_H */
