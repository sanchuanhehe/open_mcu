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
  * @file      mcs_six_step.c
  * @author    MCU Algorithm Team
  * @brief     The header file contains the following declaration:
  *             + StepEnum enum definition.
  *             + PhaseEnum enum definition.
  *             + UVW_AptReg structure definition.
  *             + SixStepHandle handle structure definition.
  *             + AptOutCombination structure definition.
  *             + Six Step Pwm Function And Six Step Pwm Overlap Function.
  */

#ifndef MCS_SIX_STEP_H
#define MCS_SIX_STEP_H

#include "apt.h"

typedef enum {
    STEP1 = 0,
    STEP2,
    STEP3,
    STEP4,
    STEP5,
    STEP6,
    STEP_MAX_NUM
} StepEnum;

typedef enum {
    U = 0,
    V,
    W,
    PHASE_MAX_NUMS
} PhaseEnum;

typedef enum {
    APT_CHA_PWM_CHB_LOW,
    APT_CHA_LOW_CHB_HIGH,
    APT_CHA_LOW_CHB_LOW
} APT_Act;

/**
  * @brief Three-phase static coordinate frame variable.
  */
typedef struct {
    APT_Handle *u; /**< Apt handle for controlling phase U. */
    APT_Handle *v; /**< Apt handle for controlling phase V. */
    APT_Handle *w; /**< Apt handle for controlling phase W. */
} UVW_AptReg;

typedef struct {
    unsigned char  phaseStep;   /* Current step. */
    UVW_AptReg controlApt;      /* Apt handles for controlling H bridge. */
} SixStepHandle;

typedef struct {
    unsigned int upperout;   /* High tube level */
    unsigned int upperstate; /* High tube state */
    unsigned int lowerout;   /* Low tube level */
    unsigned int lowerstate; /* Low tube state */
} AptOutCombination;

void SixStepPwm(const SixStepHandle *handle);
#endif