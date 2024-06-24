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
  * @file mcs_incremental_enc.h
  * @author MCU Algorithm Team
  * @brief This file provides function of QDM module and encoder speed angle calculation.
  */
 
/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef MCU_MAGICTAG_MCS_INC_ENC_H
#define MCU_MAGICTAG_MCS_INC_ENC_H

/* Includes ------------------------------------------------------------------------------------ */
#include "qdm_ip.h"
#include "mcs_ex_common.h"

/* Macro definitions --------------------------------------------------------------------------- */
#define SPDBUF_MAXSIZE 20

/**
  * @brief QDM Peripheral management.
  */
typedef struct {
    void *qdmAddr;  /* QDM peripheral address */
    NvicHandle zPulsesNvic;
} MCS_QdmHandle;

/**
  * @brief QDM initialization structure.
  */
typedef struct {
    void *qdmAddr;                /**< QDM peripheral address */
    NvicHandle *zPlusesNvic;      /**< z Pulse Interrupt Configuration */
    unsigned int zPlusesIrqPrio;  /**< Interrupt priority */
    IRQ_PROC_FUNC zPlusesIrqFunc; /**< Interrupt function */
} MCS_QdmInitStru;

/**
  * @brief Encoder motor parameter structure.
  */
typedef struct {
    signed int mtrPPMR;   /**< pulse per mechanical round */
    unsigned int zShift;  /**< pulse Z shift */
    unsigned int mtrNp;   /**< numbers of pole pairs */
    float ctrlPeriod;     /**< The encoder calculates the control period. */
    /**< the times of compute the angle, if reach the specified num, it will be reset to 0 again */
    unsigned int timeNum;
} MCS_EncInitStru;

/**
  * @brief encoder control data structure.
  */
typedef struct {
    signed int pulsePerMechRound;           /**< pulses of each mechanical round */
    signed int pulsePerElecRound;           /**< pulses of each eletricity period */
    signed short zShift;                    /**< Z-pulse cheap compensation */
    float elecAngle;                        /**< electricity angle */
    float mechAngle;						/**< motor mechine angle */
    unsigned short cntNow;                  /**< counter for now */
    unsigned short cntPre;                  /**< counter of last record */
    signed short pulsePos;                  /**< Number of pulses corresponding to mechanical
                                                  position [-32768, 32767] */
    float elecSpeed;                        /**< elec speed, in HZ */
    signed short speedBuf[SPDBUF_MAXSIZE];  /**< speed buffer */
    signed short speedBufSize;              /**< speed buffer size */
    signed short speedBufIndex;             /**< speed bufer index */
    float pulseToHzPu;                      /**< pulse to HZ transition value */

    unsigned short timeCnt;                 /**< times of compute the angle, the carrier ISR call times */
    /**< the times of compute the angle, if reach the specified num, it will be reset to 0 again */
    unsigned short timeNum;

    unsigned short pulZCnt;                 /**< counter of Z pulse */
    unsigned short pulseAngle;              /**< Number of pulses corresponding to electrical angle.
                                                 [0, pulsePerElecRound-1] */
    float pulseToElecAngle;                 /**< pulse to electricity angle transition */
    unsigned short testAngle;
} EncoderHandle;
/**
  * @}
  */

/**
  * @defgroup MCS_INC_ENC_Declaration incremental encoder.
  * @{
  */
void MCS_QdmInit(MCS_QdmInitStru *qdmInit);
void MCS_GetEncoderCnt(EncoderHandle *handle, QDM_RegStruct *qdm);
void MCS_GetElecAngleByEnc(EncoderHandle *handle);
void MCS_GetElecSpeedByEnc(EncoderHandle *handle);
void MCS_EncoderClear(EncoderHandle *handle);
void MCS_EncoderInit(EncoderHandle *handle, MCS_EncInitStru *encParam);
/**
  * @brief Get the QDM position counter.
  * @retval unsigned short QDM SCNT.
  */
static inline unsigned short MCS_GetQdmPosCnt(QDM_RegStruct *qdm)
{
    return (unsigned short)(qdm->QPOSCNT);
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* McuMagicTag_MCS_INC_ENC_H */