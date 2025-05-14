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
  * @file mcs_incremental_enc.c
  * @author MCU Algorithm Team
  * @brief This file provides function of QDM module and encoder speed angle calculation.
  */
#include "mcs_inc_enc.h"
#include "crg.h"
#include "debug.h"
#include "mcs_math_const.h"

#define FILTER_TIME_A 100 /* Unit: clock cycles */
#define FILTER_TIME_B 100 /* Unit: clock cycles */
#define FILTER_TIME_Z 100 /* Unit: clock cycles */
#define ENC_SPEED_BUF_SIZE 10
#define ENC_HALF_PPR 5000u
#define ENC_MAX_POS  65536u
/**
  * @brief QMD Initialization.
  * @param qdmInit MCS_QdmInitStru.
  * @retval None.
  */
void MCS_QdmInit(MCS_QdmInitStru *qdmInit)
{
    QDM_RegStruct *qdm = qdmInit->qdmAddr;

    HAL_CRG_IpEnableSet(qdmInit->qdmAddr, CRG_ENABLE_VALUE);

    /* Internal counters are not affected by emulator pause CPU */
    qdm->QEMUMODE.BIT.emu_mode = QDM_EMULATION_MODE_RUN_FREE;

    /* Quadrature input decoding mode. Inputs are quadrature encoder A, B, Z phase signals. */
    qdm->QCTRL.BIT.qdu_mode = QDM_QUADRATURE_COUNT;

/**
  * This parameter is valid only when the quadrature count mode is
  * used. In other modes, the frequency is multiplied by 2x.
  */
    qdm->QCTRL.BIT.qdu_xclk = QDM_4X_RESOLUTION;

    /* xCLK-xDIRCount mode. */
    qdm->QPPUCTRL.BIT.pcnt_mode = QDM_PPU_COUNT_MODE_CLK_DIR;

    /* QDM position counter reset select: overflow reset (upflow, underflow) */
    qdm->QPPUCTRL.BIT.pcnt_rst_mode = QDM_POSITION_RESET_MAX_POS;

    /* QDM initialization position counter mode: disabled */
    qdm->QPPUCTRL.BIT.pcnt_idx_init_mode = QDM_POSITION_INIT_DO_NOTHING;

    /* QDM Index latch mode select: latch at the rising edge of index */
    qdm->QPPUCTRL.BIT.pcnt_idx_lock_mode = QDM_LOCK_RISING_INDEX;

    qdm->QPOSMAX = 0xFFFF; /* QDM Position Count Max */

    /* ABZ filtering time */
    qdm->QDMAFT.BIT.qdma_ft_level = FILTER_TIME_A;
    qdm->QDMBFT.BIT.qdmb_ft_level = FILTER_TIME_B;
    qdm->QDMIFT.BIT.qdmi_ft_level = FILTER_TIME_Z;
    
/**
  * QDM Position count value. When ppu_en is disabled or counting stops after the emulator is
  * connected, software can write QPOSCNT.
  */
    qdm->QPOSCNT = 0;

    /* QDM Position processing unit PPU enable 1: PPU position counter starts counting */
    qdm->QCTRL.BIT.ppu_en = BASE_CFG_ENABLE;

    /* Enable z-pulse interrupt */
    if (qdmInit->zPlusesIrqFunc != NULL && qdmInit->zPlusesNvic->baseAddr != NULL) {
        IRQ_SetPriority(qdmInit->zPlusesNvic->irqNum, qdmInit->zPlusesIrqPrio);
        IRQ_Register(qdmInit->zPlusesNvic->irqNum, qdmInit->zPlusesIrqFunc, qdmInit->zPlusesNvic);
        DCL_QDM_EnableInterrupt(qdm, QDM_INT_INDEX_EVNT_LATCH);
        IRQ_EnableN(qdmInit->zPlusesNvic->irqNum);
    }
}

/**
  * @brief Get the Encoder Cnt object.
  * @param enc encoder handle.
  * @param qdm QDM_RegStruct.
  * @retval None.
  */
void MCS_GetEncoderCnt(EncoderHandle *enc, QDM_RegStruct *qdm)
{
    signed short tmpS16;

    enc->cntNow = qdm->QPOSCNT;
    /* Get pulse conut in unit period */
    tmpS16 = (signed short)(enc->cntNow - enc->cntPre);

    enc->pulsePos += tmpS16; /* Total number of pulses since power-on. */
    enc->pulseAngle = (unsigned short)qdm->QPOSCNT;

    enc->cntPre = enc->cntNow;
}

/**
  * @brief Get the Elec Angle By Enc object.
  * @param enc Encoder handle.
  * @retval signed short elecAngle Angle in electronic, S16degree.
  */
void MCS_GetElecAngleByEnc(EncoderHandle *enc)
{
    signed int tmpS32;

    tmpS32 = enc->pulseAngle - enc->pulZCnt;
    /* Align electric angle & enc angle */
    tmpS32 += enc->zShift;
    /* Limit max value */
    if (tmpS32 > INT16_MAX) {
        tmpS32 -= ENC_MAX_POS;
    }
    /* Limit min value */
    if (tmpS32 < -INT16_MAX) {
        tmpS32 += ENC_MAX_POS;
    }
    while (tmpS32 >= enc->pulsePerElecRound) {
        tmpS32 -= enc->pulsePerElecRound;
    }
    while (tmpS32 < 0) {
        tmpS32 += enc->pulsePerElecRound;
    }
    float tempElecAngle = (signed short)(tmpS32 * enc->pulseToElecAngle);
    /* Convert short to float angle type */
    enc->elecAngle = tempElecAngle * DIGITAL_TO_RAD;
}

/**
  * @brief Get the Elec Speed By Enc object.
  * @param enc Encoder handle.
  * @retval None.
  */
void MCS_GetElecSpeedByEnc(EncoderHandle *enc)
{
    signed int     tmpS32;
    signed short   halfPPR = ENC_HALF_PPR;
    unsigned short index   = enc->speedBufIndex;

    enc->timeCnt++;
    if (enc->timeCnt >= enc->timeNum) {
        enc->timeCnt = 0;
    } else {
        return;
    }

    /* Get pulse conut in unit period */
    tmpS32 = enc->pulsePos - enc->speedBuf[index];
    /* Limit max value */
    if (tmpS32 > halfPPR) {
        tmpS32 -= ENC_MAX_POS;
    }
    /* Limit min value */
    if (tmpS32 < -halfPPR) {
        tmpS32 += ENC_MAX_POS;
    }
    /* Convert unit pulse count to Hz */
    enc->elecSpeed       = tmpS32 * enc->pulseToHzPu;
    enc->speedBuf[index] = enc->pulsePos;
    index++;
    /* Store current speed for calcule the average */
    if (index >= enc->speedBufSize) {
        enc->speedBufIndex = 0;
    } else {
        enc->speedBufIndex = index;
    }
}

/**
  * @brief Clear the historical cache and time count.
  * @param enc Encoder handle.
  * @retval None.
  */
void MCS_EncoderClear(EncoderHandle *enc)
{
    enc->speedBufIndex = 0;
    for (unsigned short i = 0; i < enc->speedBufSize; i++) {
        enc->speedBuf[i] = enc->pulsePos;
    }

    enc->timeCnt = 0;
}

/**
  * @brief Initialzer of encoder struct handle.
  * @param enc Encoder handle.
  * @param encParam Encoder and Motor parameter.
  * @retval None.
  */
void MCS_EncoderInit(EncoderHandle *enc, MCS_EncInitStru *encParam)
{
    if (encParam->mtrPPMR == 0 || encParam->mtrNp == 0) {
        return;
    }

    enc->pulsePerMechRound = encParam->mtrPPMR;
    enc->pulsePerElecRound = encParam->mtrPPMR / encParam->mtrNp;
    enc->zShift = encParam->zShift;
    /* Clear count value */
    enc->cntNow = 0;
    enc->cntPre = 0;

    enc->pulsePos   = 0;
    enc->pulseAngle = 0;
    enc->elecAngle  = 0;
    /* Initial speed buffer */
    enc->speedBufSize  = ENC_SPEED_BUF_SIZE;  /* Max = 10 */
    enc->speedBufIndex = 0;
    for (unsigned short i = 0; i < enc->speedBufSize; i++) {
        enc->speedBuf[i] = enc->pulsePos;
    }
    /* Clear time count value */
    enc->timeCnt = 0;
    enc->timeNum = encParam->timeNum;
    enc->elecSpeed = 0.0f;  /* Hz */
    /* Convert unit pulse count to Hz */
    enc->pulseToHzPu = (float)(1.0f /
                       (enc->pulsePerElecRound * enc->timeNum * enc->speedBufSize * encParam->ctrlPeriod));
    /* Convert unit pulse count to electric angle */
    enc->pulseToElecAngle = (float)(65536.0f / enc->pulsePerElecRound);
}