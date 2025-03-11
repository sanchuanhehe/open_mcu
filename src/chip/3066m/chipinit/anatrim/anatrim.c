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
  * @file      anatrim.c
  * @author    MCU Driver Team
  * @brief     Chip Init modlue.
  * @details   Calibration of analog module parameters.
  */
#include "anatrim.h"

float g_tsensorGain = 0.00418f;

#ifdef FPGA
#else

/**
 * @brief Obtains the chip ID.
 * @param None
 * @retval None
 */
static bool CHIP_GetInfo(void)
{
    FOTP_INFO_RGN0_NUMBER_4 emptyData;
    FOTP_INFO_RGN0_NUMBER_2 idData;
    FOTP_InfoGet(FOTP_INFO_RNG0, 4U, (void *)&emptyData.comData);   /* 4 is the number of fotp_empty_flag in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 2U, (void *)&idData.comData);      /* 2 is the number of idData in otp */
    if (emptyData.REG.fotp_empty_flag != 0x5AA59669 || idData.REG.chip_id == 0xFFFFFFFF) {
        return false;
    }
    return true;
}

/**
 * @brief VREF trim config.
 * @param None
 * @retval None
 */
static void VREF_Trim(void)
{
    FOTP_INFO_RGN0_NUMBER_20 trimData20;
    FOTP_InfoGet(FOTP_INFO_RNG0, 20U, (void *)&trimData20.comData); /* 20 is the number of trim data in otp */
    /* VREF */
    VREF->VREF_TRIM0.BIT.da_iref_trim = trimData20.REG.data0.da_iref_trim;
    VREF->VREF_TRIM0.BIT.da_ref_vref_trim = trimData20.REG.data0.da_ref_vref_trim;
    VREF->VREF_TRIM0.BIT.da_ref_vbg_trim = trimData20.REG.data0.da_ref_vbg_trim;
    unsigned int value = trimData20.REG.data1.da_ref_temp_trim3;
    value |= (trimData20.REG.data1.da_ref_temp_trim2 << 8U);    /* Shift left by 8 bits */
    value |= (trimData20.REG.data1.da_ref_temp_trim1 << 16U);   /* Shift left by 16 bits */
    value |= (trimData20.REG.data0.da_ref_temp_trim0 << 24U);   /* Shift left by 24 bits */
    VREF->VREF_TRIM1.reg = value;
}

/**
 * @brief TSENSOR trim config.
 * @param None
 * @retval None
 */
static void TSENSOR_Trim(void)
{
    HAL_CRG_IpEnableSet((void *)ADC1, IP_CLK_ENABLE); /* Enable the clock for calibration */
    FOTP_INFO_RGN0_NUMBER_20 trimData20;
    FOTP_INFO_RGN0_NUMBER_21 trimData21;
    FOTP_InfoGet(FOTP_INFO_RNG0, 20U, (void *)&trimData20.comData); /* 20 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 21U, (void *)&trimData21.comData); /* 21 is the number of trim data in otp */
    /* TSENSOR */
    TSENSOR->TSENSOR_TRIM.reg = trimData20.REG.data1.da_ref_vptat_trim;
    ADC1->ADC_TSENSOR_TRIM.BIT.cfg_tsensor_ofst_trim = trimData21.REG.data1.ts_offset;
    HAL_CRG_IpEnableSet((void *)ADC1, IP_CLK_DISABLE); /* The clock is disabled after calibration */
}

/**
 * @brief PGA trim config.
 * @param None
 * @retval None
 */
static void PGA_Trim(void)
{
    HAL_CRG_IpEnableSet((void *)PGA0, IP_CLK_ENABLE);
    HAL_CRG_IpEnableSet((void *)PGA1, IP_CLK_ENABLE);
    HAL_CRG_IpEnableSet((void *)PGA2, IP_CLK_ENABLE);
    FOTP_INFO_RGN0_NUMBER_21 trimData21;
    FOTP_InfoGet(FOTP_INFO_RNG0, 21U, (void *)&trimData21.comData); /* 21 is the number of trim data in otp */
    /* PGA */
    PGA0->PGA_TRIM.BIT.da_pga_vos_trim = trimData21.REG.data0.da_pga0_vos_trim;
    PGA1->PGA_TRIM.BIT.da_pga_vos_trim = trimData21.REG.data0.da_pga1_vos_trim;
    PGA2->PGA_TRIM.BIT.da_pga_vos_trim = trimData21.REG.data1.da_pga2_vos_trim;
    HAL_CRG_IpEnableSet((void *)PGA0, IP_CLK_DISABLE);
    HAL_CRG_IpEnableSet((void *)PGA1, IP_CLK_DISABLE);
    HAL_CRG_IpEnableSet((void *)PGA2, IP_CLK_DISABLE);
}

/**
 * @brief ADC0 trim config.
 * @param None
 * @retval None
 */
static void ADC0_Trim(void)
{
    HAL_CRG_IpEnableSet((void *)ADC0, IP_CLK_ENABLE); /* Enable the clock for calibration */

    FOTP_INFO_RGN0_NUMBER_21 trimData21;
    FOTP_INFO_RGN0_NUMBER_22 trimData22;
    FOTP_INFO_RGN0_NUMBER_23 trimData23;
    FOTP_INFO_RGN0_NUMBER_24 trimData24;
    FOTP_InfoGet(FOTP_INFO_RNG0, 21U, (void *)&trimData21.comData); /* 21 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 22U, (void *)&trimData22.comData); /* 22 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 23U, (void *)&trimData23.comData); /* 23 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 24U, (void *)&trimData24.comData); /* 24 is the number of trim data in otp */

    ADC0->ADC_OEGE_TRIM.BIT.cfg_gain_cali_trim = trimData21.REG.data2.saradc0_gain;
    ADC0->ADC_OEGE_TRIM.BIT.cfg_ofst_cali_trim = trimData21.REG.data2.saradc0_offset;

    ADC0->ADC_AIN0_OEGE_TRIM0.BIT.cfg_ain0_gain_trim2 = trimData22.REG.data2.saradc0_ain0_gain2;
    ADC0->ADC_AIN0_OEGE_TRIM0.BIT.cfg_ain0_ofst_trim2 = trimData22.REG.data2.saradc0_ain0_offset2;
    ADC0->ADC_AIN0_OEGE_TRIM1.BIT.cfg_ain0_gain_trim4 = trimData22.REG.data3.saradc0_ain0_gain4;
    ADC0->ADC_AIN0_OEGE_TRIM1.BIT.cfg_ain0_ofst_trim4 = trimData22.REG.data3.saradc0_ain0_offset4;
    ADC0->ADC_AIN0_OEGE_TRIM2.BIT.cfg_ain0_gain_trim8 = trimData23.REG.data0.saradc0_ain0_gain8;
    ADC0->ADC_AIN0_OEGE_TRIM2.BIT.cfg_ain0_ofst_trim8 = trimData23.REG.data0.saradc0_ain0_offset8;
    ADC0->ADC_AIN0_OEGE_TRIM3.BIT.cfg_ain0_gain_trim16 = trimData23.REG.data1.saradc0_ain0_gain16;
    ADC0->ADC_AIN0_OEGE_TRIM3.BIT.cfg_ain0_ofst_trim16 = trimData23.REG.data1.saradc0_ain0_offset16;

    ADC0->ADC_AIN1_OEGE_TRIM0.BIT.cfg_ain1_gain_trim2 = trimData23.REG.data2.saradc0_ain1_gain2;
    ADC0->ADC_AIN1_OEGE_TRIM0.BIT.cfg_ain1_ofst_trim2 = trimData23.REG.data2.saradc0_ain1_offset2;
    ADC0->ADC_AIN1_OEGE_TRIM1.BIT.cfg_ain1_gain_trim4 = trimData23.REG.data3.saradc0_ain1_gain4;
    ADC0->ADC_AIN1_OEGE_TRIM1.BIT.cfg_ain1_ofst_trim4 = trimData23.REG.data3.saradc0_ain1_offset4;
    ADC0->ADC_AIN1_OEGE_TRIM2.BIT.cfg_ain1_gain_trim8 = trimData24.REG.data0.saradc0_ain1_gain8;
    ADC0->ADC_AIN1_OEGE_TRIM2.BIT.cfg_ain1_ofst_trim8 = trimData24.REG.data0.saradc0_ain1_offset8;
    ADC0->ADC_AIN1_OEGE_TRIM3.BIT.cfg_ain1_gain_trim16 = trimData24.REG.data1.saradc0_ain1_gain16;
    ADC0->ADC_AIN1_OEGE_TRIM3.BIT.cfg_ain1_ofst_trim16 = trimData24.REG.data1.saradc0_ain1_offset16;

    HAL_CRG_IpEnableSet((void *)ADC0, IP_CLK_DISABLE); /* The clock is disabled after calibration */
}

/**
 * @brief ADC1 trim config.
 * @param None
 * @retval None
 */
static void ADC1_Trim(void)
{
    HAL_CRG_IpEnableSet((void *)ADC1, IP_CLK_ENABLE); /* Enable the clock for calibration */

    FOTP_INFO_RGN0_NUMBER_21 trimData21;
    FOTP_INFO_RGN0_NUMBER_24 trimData24;
    FOTP_INFO_RGN0_NUMBER_25 trimData25;
    FOTP_INFO_RGN0_NUMBER_26 trimData26;
    FOTP_InfoGet(FOTP_INFO_RNG0, 21U, (void *)&trimData21.comData); /* 21 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 24U, (void *)&trimData24.comData); /* 24 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 25U, (void *)&trimData25.comData); /* 25 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 26U, (void *)&trimData26.comData); /* 26 is the number of trim data in otp */

    ADC1->ADC_OEGE_TRIM.BIT.cfg_gain_cali_trim = trimData21.REG.data3.saradc1_gain;
    ADC1->ADC_OEGE_TRIM.BIT.cfg_ofst_cali_trim = trimData21.REG.data3.saradc1_offset;

    ADC1->ADC_AIN0_OEGE_TRIM0.BIT.cfg_ain0_gain_trim2 = trimData24.REG.data2.saradc1_ain0_gain2;
    ADC1->ADC_AIN0_OEGE_TRIM0.BIT.cfg_ain0_ofst_trim2 = trimData24.REG.data2.saradc1_ain0_offset2;
    ADC1->ADC_AIN0_OEGE_TRIM1.BIT.cfg_ain0_gain_trim4 = trimData24.REG.data3.saradc1_ain0_gain4;
    ADC1->ADC_AIN0_OEGE_TRIM1.BIT.cfg_ain0_ofst_trim4 = trimData24.REG.data3.saradc1_ain0_offset4;
    ADC1->ADC_AIN0_OEGE_TRIM2.BIT.cfg_ain0_gain_trim8 = trimData25.REG.data0.saradc1_ain0_gain8;
    ADC1->ADC_AIN0_OEGE_TRIM2.BIT.cfg_ain0_ofst_trim8 = trimData25.REG.data0.saradc1_ain0_offset8;
    ADC1->ADC_AIN0_OEGE_TRIM3.BIT.cfg_ain0_gain_trim16 = trimData25.REG.data1.saradc1_ain0_gain16;
    ADC1->ADC_AIN0_OEGE_TRIM3.BIT.cfg_ain0_ofst_trim16 = trimData25.REG.data1.saradc1_ain0_offset16;

    ADC1->ADC_AIN1_OEGE_TRIM0.BIT.cfg_ain1_gain_trim2 = trimData25.REG.data2.saradc1_ain1_gain2;
    ADC1->ADC_AIN1_OEGE_TRIM0.BIT.cfg_ain1_ofst_trim2 = trimData25.REG.data2.saradc1_ain1_offset2;
    ADC1->ADC_AIN1_OEGE_TRIM1.BIT.cfg_ain1_gain_trim4 = trimData25.REG.data3.saradc1_ain1_gain4;
    ADC1->ADC_AIN1_OEGE_TRIM1.BIT.cfg_ain1_ofst_trim4 = trimData25.REG.data3.saradc1_ain1_offset4;
    ADC1->ADC_AIN1_OEGE_TRIM2.BIT.cfg_ain1_gain_trim8 = trimData26.REG.data0.saradc1_ain1_gain8;
    ADC1->ADC_AIN1_OEGE_TRIM2.BIT.cfg_ain1_ofst_trim8 = trimData26.REG.data0.saradc1_ain1_offset8;
    ADC1->ADC_AIN1_OEGE_TRIM3.BIT.cfg_ain1_gain_trim16 = trimData26.REG.data1.saradc1_ain1_gain16;
    ADC1->ADC_AIN1_OEGE_TRIM3.BIT.cfg_ain1_ofst_trim16 = trimData26.REG.data1.saradc1_ain1_offset16;

    HAL_CRG_IpEnableSet((void *)ADC1, IP_CLK_DISABLE); /* The clock is disabled after calibration */
}

/**
 * @brief ADC2 trim config.
 * @param None
 * @retval None
 */
static void ADC2_Trim(void)
{
    HAL_CRG_IpEnableSet((void *)ADC2, IP_CLK_ENABLE); /* Enable the clock for calibration */

    FOTP_INFO_RGN0_NUMBER_22 trimData22;
    FOTP_INFO_RGN0_NUMBER_26 trimData26;
    FOTP_INFO_RGN0_NUMBER_27 trimData27;
    FOTP_INFO_RGN0_NUMBER_28 trimData28;
    FOTP_InfoGet(FOTP_INFO_RNG0, 22U, (void *)&trimData22.comData); /* 22 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 26U, (void *)&trimData26.comData); /* 26 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 27U, (void *)&trimData27.comData); /* 27 is the number of trim data in otp */
    FOTP_InfoGet(FOTP_INFO_RNG0, 28U, (void *)&trimData28.comData); /* 28 is the number of trim data in otp */

    ADC2->ADC_OEGE_TRIM.BIT.cfg_gain_cali_trim = trimData22.REG.data0.saradc2_gain;
    ADC2->ADC_OEGE_TRIM.BIT.cfg_ofst_cali_trim = trimData22.REG.data0.saradc2_offset;

    ADC2->ADC_AIN0_OEGE_TRIM0.BIT.cfg_ain0_gain_trim2 = trimData26.REG.data2.saradc2_ain0_gain2;
    ADC2->ADC_AIN0_OEGE_TRIM0.BIT.cfg_ain0_ofst_trim2 = trimData26.REG.data2.saradc2_ain0_offset2;
    ADC2->ADC_AIN0_OEGE_TRIM1.BIT.cfg_ain0_gain_trim4 = trimData26.REG.data3.saradc2_ain0_gain4;
    ADC2->ADC_AIN0_OEGE_TRIM1.BIT.cfg_ain0_ofst_trim4 = trimData26.REG.data3.saradc2_ain0_offset4;
    ADC2->ADC_AIN0_OEGE_TRIM2.BIT.cfg_ain0_gain_trim8 = trimData27.REG.data0.saradc2_ain0_gain8;
    ADC2->ADC_AIN0_OEGE_TRIM2.BIT.cfg_ain0_ofst_trim8 = trimData27.REG.data0.saradc2_ain0_offset8;
    ADC2->ADC_AIN0_OEGE_TRIM3.BIT.cfg_ain0_gain_trim16 = trimData27.REG.data1.saradc2_ain0_gain16;
    ADC2->ADC_AIN0_OEGE_TRIM3.BIT.cfg_ain0_ofst_trim16 = trimData27.REG.data1.saradc2_ain0_offset16;

    ADC2->ADC_AIN1_OEGE_TRIM0.BIT.cfg_ain1_gain_trim2 = trimData27.REG.data2.saradc2_ain1_gain2;
    ADC2->ADC_AIN1_OEGE_TRIM0.BIT.cfg_ain1_ofst_trim2 = trimData27.REG.data2.saradc2_ain1_offset2;
    ADC2->ADC_AIN1_OEGE_TRIM1.BIT.cfg_ain1_gain_trim4 = trimData27.REG.data3.saradc2_ain1_gain4;
    ADC2->ADC_AIN1_OEGE_TRIM1.BIT.cfg_ain1_ofst_trim4 = trimData27.REG.data3.saradc2_ain1_offset4;
    ADC2->ADC_AIN1_OEGE_TRIM2.BIT.cfg_ain1_gain_trim8 = trimData28.REG.data0.saradc2_ain1_gain8;
    ADC2->ADC_AIN1_OEGE_TRIM2.BIT.cfg_ain1_ofst_trim8 = trimData28.REG.data0.saradc2_ain1_offset8;
    ADC2->ADC_AIN1_OEGE_TRIM3.BIT.cfg_ain1_gain_trim16 = trimData28.REG.data1.saradc2_ain1_gain16;
    ADC2->ADC_AIN1_OEGE_TRIM3.BIT.cfg_ain1_ofst_trim16 = trimData28.REG.data1.saradc2_ain1_offset16;

    HAL_CRG_IpEnableSet((void *)ADC2, IP_CLK_DISABLE); /* The clock is disabled after calibration */
}

/**
 * @brief Analog module trim.
 * @param None
 * @retval None
 */
static void CHIP_AnalogTrim(void)
{
    VREF_Trim();   /* VREF trim config */
    PGA_Trim();    /* PGA trim config */
    TSENSOR_Trim(); /* TSENSOR trim config */
    ADC0_Trim(); /* ADC0 trim config */
    ADC1_Trim(); /* ADC1 trim config */
    ADC2_Trim(); /* ADC2 trim config */
}

/**
 * @brief Parameter calibration entry of the analog module.
 * @param None
 * @retval None
 */
void ANATRIM_Entry(void)
{
    if (CHIP_GetInfo() == false) { /* If the chip information is incorrect, calibration is not performed */
        return;
    }
    CHIP_AnalogTrim();   /* Analog trim config */
}
#endif