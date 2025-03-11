/**
  * @ Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2023. All rights reserved.
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
  * @file      fault_det.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of fault detection.
  *            Include open phase, statll, over current, under voltage, interturn.
  */


#include "fault_det.h"
#include "mcs_math.h"


/**
  * @brief Clear fault status.
  * @param faultStatus Pointer of fault status handle.
  * @retval None.
  */
void Fault_Clear(FAULT_Status *faultStatus)
{
    faultStatus->all = 0;
}


/**
  * @brief Initialize overcurrent detection.
  * @param ocd Pointer of OCD handle.
  * @param overCurrThr Software overcurrent protection threshold (A).
  * @param detWindow Overcurrent detection duration (s).
  * @param ts Control period.
  * @retval None.
  */
void FP_OCD_Init(FP_OCD_Handle *ocd, float overCurrThr, float detWindow, float ts)
{
    MCS_ASSERT_PARAM(ocd != NULL);
    ocd->ts = ts;
    /* Software overcurrent protection threshold. */
    ocd->overCurrThr = overCurrThr;
    ocd->detWindow = detWindow;
}

/**
  * @brief Perform software overcurrent detection.
  * @param ocd Pointer of OCD handle.
  * @param faultStatus Pointer of FaultStatus struct.
  * @param iuvw Three-phase current.
  * @retval None.
  */
void FP_OCD_Exec(FP_OCD_Handle *ocd, FAULT_Status *faultStatus, UvwAxis iuvw)
{
    MCS_ASSERT_PARAM(ocd != NULL);
    /* The absolute value of the three-phase current is used. */
    float iuAbs = Abs(iuvw.u);
    float ivAbs = Abs(iuvw.v);
    float iwAbs = Abs(iuvw.w);
    float overCurrThr = ocd->overCurrThr;
    /* Threshold judgement. */
    if (iuAbs > overCurrThr || ivAbs > overCurrThr || iwAbs > overCurrThr) {
        ocd->timer += ocd->ts;
        if (ocd->timer > ocd->detWindow) {
            faultStatus->Bit.softOverCurFault = 1;
            ocd->timer = 0.0f;
        }
    } else {
        /* Continuous overcurrent is not met, and the timer is cleared. */
        ocd->timer = 0.0f;
    }
}

/**
  * @brief Clear ocd history value.
  * @param ocd Pointer of OCD handle.
  * @retval None.
  */
void FP_OCD_Clear(FP_OCD_Handle *ocd)
{
    ocd->timer = 0.0f;
}


/**
  * @brief Initialize under/over voltage detection.
  * @param uovdHandle Pointer of HSF_UOVD_Handle.
  * @param uovdInit Pointer of HSF_UOVD_Param.
  * @param ts Control period.
  * @retval None.
  */
void FP_UOVD_Init(FP_UOVD_Handle *uovdHandle, FP_UOVD_Param *uovdInit, float ts)
{
    uovdHandle->ts = ts;
    /* Overvoltage protection Value */
    uovdHandle->overProThr = uovdInit->overProThr;
    /* Overvoltage protection Recovery Value */
    uovdHandle->overRecThr = uovdInit->overRecThr;
    /* Undervoltage protection value */
    uovdHandle->underProThr = uovdInit->underProThr;
    /* Undervoltage protection Recovery Value */
    uovdHandle->underRecThr = uovdInit->underRecThr;
    /* Protection Detect Filter Count */
    uovdHandle->proTicks = (unsigned int)(uovdInit->detWindow / ts);
    /* Protection Recovery Detect Filter Count */
    uovdHandle->recTicks = (unsigned int)(uovdInit->recWindow / ts);
}

/**
  * @brief Perform under/over voltage detection.
  * @param uovdHandle Pointer of under/over voltage detection struct handle.
  * @param faultStatus Pointer of FaultStatus.
  * @param busVolt DC bus voltage.
  * @retval None.
  */
void FP_UOVD_Exec(FP_UOVD_Handle *uovdHandle, FAULT_Status *faultStatus, float busVolt)
{
    float overProThr = uovdHandle->overProThr;
    float underProThr = uovdHandle->underProThr;
    unsigned int proTicks = uovdHandle->proTicks;

        if (busVolt > overProThr) {
            /* Overvoltage protection filter count */
            uovdHandle->overVolCnt++;
            uovdHandle->underVolCnt = 0;
        } else if (busVolt < underProThr) {
            /* Undervoltage protection filter count */
            uovdHandle->underVolCnt++;
            uovdHandle->overVolCnt = 0;
        } else {
            uovdHandle->overVolCnt = 0;
            uovdHandle->underVolCnt = 0;
        }

        if (uovdHandle->overVolCnt > proTicks) {
            /* Overvoltage protection conditions are met */
            faultStatus->Bit.overVoltFault = 1;
            uovdHandle->overVolCnt = 0;
        } else if (uovdHandle->underVolCnt > proTicks) {
            /* Undervoltage protection conditions are met */
            faultStatus->Bit.underVoltFault = 1;
            uovdHandle->underVolCnt = 0;
        }
}

/**
  * @brief Perform under/over voltage recovery.
  * @param uovdHandle Pointer of under/over voltage detection struct handle.
  * @param faultStatus Pointer of FaultStatus.
  * @param busVolt DC bus voltage.
  * @retval None.
  */
void FP_UOVR_Exec(FP_UOVD_Handle *uovdHandle, FAULT_Status *faultStatus, float busVolt)
{
    float overRecThr = uovdHandle->overRecThr;
    float underRecThr = uovdHandle->underRecThr;
    unsigned int recTicks = uovdHandle->recTicks;

    if (faultStatus->Bit.overVoltFault) {
        if (busVolt < overRecThr) {
            /* Overvoltage Protection Recovery Filter Count */
            uovdHandle->overVolCnt++;
        } else {
            uovdHandle->overVolCnt = 0;
        }
        
        if (uovdHandle->overVolCnt > recTicks) {
            faultStatus->Bit.overVoltFault = 0;
            /* Overvoltage protection Recovery conditions are met */
            uovdHandle->recoveryFlag = 1;
        }
    } else if (faultStatus->Bit.underVoltFault) {
        if (busVolt > underRecThr) {
            /* Undervoltage Protection Recovery Filter Count */
            uovdHandle->underVolCnt++;
        } else {
            uovdHandle->underVolCnt = 0;
        }

        if (uovdHandle->underVolCnt > recTicks) {
            faultStatus->Bit.underVoltFault = 0;
            /* Undervoltage protection Recovery conditions are met */
            uovdHandle->recoveryFlag = 1;
        }
    }
}

/**
  * @brief Clear historical values of under/over voltage detection.
  * @param uovdHandle Under/over voltage detection struct handle.
  * @retval None.
  */
void FP_UOVD_Clear(FP_UOVD_Handle *uovdHandle)
{
    /* Clear the count value and flag bit */
    uovdHandle->overVolCnt = 0;
    uovdHandle->underVolCnt = 0;
    uovdHandle->recoveryFlag = 0;
}

/**
  * @brief Initialize over temperature detection of board and motor.
  * @param otd Pointer of over temperature detection struct handle.
  * @param mtrTempThr Motor overtemperature threshold.
  * @param brdTempThr Board overtemperature threshold.
  * @param mtrDetWindow Detection duration (s).
  * @param brdDetWindow Detection duration (s).
  * @param ts: Control period.
  * @retval None.
  */
void FP_OTD_Init(FP_OTD_Handle *otd, float mtrTempThr, float brdTempThr,
                 float mtrDetWindow, float brdDetWindow, float ts)
{
    otd->ts = ts;
    otd->mtrTempThr = mtrTempThr;
    otd->brdTempThr = brdTempThr;
    /* Detet window. */
    otd->mtrDetWindow = mtrDetWindow;
    otd->brdDetWindow = brdDetWindow;
}

/**
  * @brief Perform over temperature detection.
  * @param otd Pointer of over temperature detection struct handle.
  * @param faultStatus Pointer of FaultStatus.
  * @param mtrTemp Motor temperature.
  * @param brdTemp Board temperature.
  * @retval None.
  */
void FP_OTD_Exec(FP_OTD_Handle *otd, FAULT_Status *faultStatus, float mtrTemp, float brdTemp)
{
    /* Protect motor when the motor over temperature exceed a certain time. */
    if (mtrTemp > otd->mtrTempThr) {
        otd->mtrTimer += otd->ts;
        if (otd->mtrTimer > otd->mtrDetWindow) {
            faultStatus->Bit.mtrOverTempFault = 1;
            otd->mtrTimer = 0.0f;
        }
    } else {
        /* Clear timer. */
        otd->mtrTimer = 0.0f;
    }
    /* Protect board when the board temperature exceed a certain time. */
    if (brdTemp > otd->brdTempThr) {
        otd->brdTimer += otd->ts;
        if (otd->brdTimer > otd->brdDetWindow) {
            faultStatus->Bit.brdOverTempFault = 1;
            otd->brdTimer = 0.0f;
        }
    } else {
        /* Clear timer. */
        otd->brdTimer = 0.0f;
    }
}

/**
  * @brief Clear historical values of over temperature detection.
  * @param otd Over temperature detection struct handle.
  * @retval None.
  */
void FP_OTD_Clear(FP_OTD_Handle *otd)
{
    otd->mtrTimer = 0.0f;
    otd->brdTimer = 0.0f;
}

/**
  * @brief Initialize motor stall detection.
  * @param std Pointer of stall detection struct handle.
  * @param spdLowerLim Minimum speed threshold.
  * @param spdUpperLim Maximum speed threshold.
  * @param detWindow Detection duration (s).
  * @param ts: Control period.
  * @retval None.
  */
void FP_STD_Init(FP_STD_Handle *std, float spdLowerLim, float spdUpperLim, float detWindow, float ts)
{
    std->ts = ts;
    std->detWindow = detWindow;
    std->spdLowerLim = spdLowerLim;
    std->spdUpperLim = spdUpperLim;
}

/**
  * @brief Perform motor stall detection.
  * @param std Pointer of over temperature detection struct handle.
  * @param faultStatus Pointer of FaultStatus.
  * @param hallSecTimer Rotor time in sectors.
  * @param spdFbk Speed feedback.
  * @retval None.
  */
void FP_STD_Exec(FP_STD_Handle *std, FAULT_Status *faultStatus, float hallSecTimer, float spdFbk)
{
    /* Time out detection. */
    if (hallSecTimer > std->detWindow) {
        std->timeOutFlag = 1;
    }
    /* Lose speed detection. */
    if (spdFbk < std->spdLowerLim || spdFbk > std->spdUpperLim) {
        std->timer += std->ts;
        if (std->timer > std->detWindow) {
            std->loseSpdFlag = 1;
            std->timer = 0.0f;
        }
    } else {
        std->timer = 0.0f;
    }
    /* Motor stall detection. */
    if (std->timeOutFlag || std->loseSpdFlag) {
        faultStatus->Bit.stallFault = 1;
    }
}

/**
  * @brief Clear historical values of stall detection.
  * @param otd Motor stall detection struct handle.
  * @retval None.
  */
void FP_STD_Clear(FP_STD_Handle *std)
{
    std->loseSpdFlag = 0;
    std->timeOutFlag = 0;
    std->timer = 0.0f;
}

/**
  * @brief Initialize open phase detection.
  * @param opd Pointer of open phase detection struct handle.
  * @param opdCurrThr Minimum open phase current (A).
  * @param detWindow Detection duration (s).
  * @param ts: Control period.
  * @retval None.
  */
void FP_OPD_Init(FP_OPD_Handle *opd, float opdCurrThr, float detWindow, float ts)
{
    opd->opdCurrThr = opdCurrThr;
    opd->detWindow = detWindow;
    opd->ts = ts;
}

/**
  * @brief Perform motor open phase detection.
  * @param opd Pointer of open phase detection struct handle.
  * @param faultStatus Pointer of FaultStatus.
  * @param iuvw Current of uvw phase.
  * @retval None.
  */
void FP_OPD_Exec(FP_OPD_Handle *opd, FAULT_Status *faultStatus, UvwAxis iuvw)
{
    /* Calc integral threshold for open-phase current. */
    float integralThr = opd->opdCurrThr * opd->detWindow;

    if (opd->timer < opd->detWindow) {
        /* Calculate the integral value of the absolute value of the three-phase current. */
        opd->iuAbsIntegral += (Abs(iuvw.u) * opd->ts);
        opd->ivAbsIntegral += (Abs(iuvw.v) * opd->ts);
        opd->iwAbsIntegral += (Abs(iuvw.w) * opd->ts);
    } else {
        /* Open phase detection of phase u. */
        if (opd->iuAbsIntegral < integralThr) {
            faultStatus->Bit.openPhaseFault_U = 1;
            faultStatus->Bit.openPhaseFault = 1;
        }
        /* Open phase detection of phase V. */
        if (opd->ivAbsIntegral < integralThr) {
            faultStatus->Bit.openPhaseFault_V = 1;
            faultStatus->Bit.openPhaseFault = 1;
        }
        /* Open phase detection of phase W. */
        if (opd->iwAbsIntegral < integralThr) {
            faultStatus->Bit.openPhaseFault_W = 1;
            faultStatus->Bit.openPhaseFault = 1;
        }
        opd->iuAbsIntegral = 0.0f;
        opd->ivAbsIntegral = 0.0f;
        opd->iwAbsIntegral = 0.0f;
        opd->timer = 0.0f;
    }
    /* Time Accumulation. */
    opd->timer += opd->ts;
}

/**
  * @brief Clear historical values of open phase detection.
  * @param opd Pointer of open phase detection struct handle.
  * @retval None.
  */
void FP_OPD_Clear(FP_OPD_Handle *opd)
{
    opd->iuAbsIntegral = 0.0f;
    opd->ivAbsIntegral = 0.0f;
    opd->iwAbsIntegral = 0.0f;
    opd->timer = 0.0f;
}

