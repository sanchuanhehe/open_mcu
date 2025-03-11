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
  * @file      mcs_motor_process.c
  * @author    MCU Algorithm Team
  * @brief     This file provides motor application for ECMCU105H board.
  * @details   Single FOC application based on the ECMCU105H board
  *            1) Motor model is Gimbal GBM2804H-100T.
  *            2) Select the pmsm sensorless 1shunt foc example in the sample column
                  of chipConfig and click Generate Code.
  *            3) It's power supply must be changed to 24V.
  */
#include "main.h"
#include "mcs_user_config.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_carrier.h"
#include "mcs_motor_process.h"
#include "mcs_pll.h"
#include "mcs_sensor_hall.h"
#include "debug.h"
#include "mcs_fsm.h"
#include "mcs_chip_config.h"

/*------------------------------- Macro Definition -----------------------------------------------*/
#define TEMP_3                  3.0f
#define TEMP_15                 15.0f
#define TEMP_30                 30.0f
#define TEMP_45                 45.0f
#define TEMP_60                 60.0f
#define TEMP_RES_15             78.327f
#define TEMP_RES_30             36.776f
#define TEMP_RES_45             18.301f
#define TEMP_RES_60             9.607f

#define HALL_VALUE_1            1
#define HALL_VALUE_2            2
#define HALL_VALUE_3            3
#define HALL_VALUE_4            4
#define HALL_VALUE_5            5
#define HALL_VALUE_6            6

#define US_PER_MS               1000
#define APT_FULL_DUTY           1.0f
/*------------------------------- Param Definition -----------------------------------------------*/
/* Motor parameters. */
static MOTOR_Param g_motorParam = MOTORPARAM_DEFAULTS;

static APT_RegStruct* g_aptAddr[PHASE_MAX_NUM] = {APT_U_CP, APT_V_CP, APT_W_CP};

/* Motor control handle */
static MTRCTRL_Handle g_mc;

static HALL_Handle g_hall;

/*------------------------------- Function Definition -----------------------------------------------*/
/**
  * @brief Receive hall signal.
  * @retval Hall value.
  */
static unsigned int GetHallValue(void)
{
#if defined (CHIP_3061MNPICA) || defined (CHIP_3061MNPIKA) || defined (CHIP_3061MNNICA) || \
    defined (CHIP_3061MNNIKA) || defined (CHIP_3061MNPIC8) || defined(CHIP_3061MNNIC8) || \
    defined (CHIP_3061MNPIK8) || defined (CHIP_3061MNNIK8)
    /* Get three hall values. */
    unsigned int h1 = HAL_CAPM_GetCrtEdge(&g_capm2) & 0x01;
    unsigned int h2 = HAL_CAPM_GetCrtEdge(&g_capm1) & 0x01;
    unsigned int h3 = HAL_CAPM_GetCrtEdge(&g_capm0) & 0x01;
#endif

#if defined (CHIP_3065HRPIRZ) || defined (CHIP_3065ARPIRZ)
    /* Get three hall values. */
    unsigned int h1 = HAL_CAPM_GetCrtEdge(&g_capm1) & 0x01;
    unsigned int h2 = HAL_CAPM_GetCrtEdge(&g_capm0) & 0x01;
    unsigned int h3 = HAL_CAPM_GetCrtEdge(&g_capm2) & 0x01;
#endif

#if defined CHIP_3066MNPIRH || defined CHIP_3065PNPIRH || defined CHIP_3065PNPIRE || defined CHIP_3065PNPIRA
    /* Get three hall values. */
    unsigned int h1 = HAL_CAPM_GetCrtEdge(&g_capm2) & 0x01;
    unsigned int h2 = HAL_CAPM_GetCrtEdge(&g_capm0) & 0x01;
    unsigned int h3 = HAL_CAPM_GetCrtEdge(&g_capm1) & 0x01;
#endif

    /* Terminal connection sequence: WVU-+  -->  H3H2H1-+ */
    unsigned int retValue = (h1 << 2) + (h2 << 1) + (h3 << 0);
    return retValue;
}


/**
  * @brief CW: Matching sector corresponding to the hall value.
  * @param hallValue Sum of hallC << 2, hallB << 1 and hallA << 1.
  * @retval Motor sector.
  */
static HALL_SECTOR MatchCw(unsigned int hallValue)
{
    HALL_SECTOR sector = SECTOR1;
    switch (hallValue) {
        case HALL_VALUE_3:
            /* B+, A- */
            sector = SECTOR3;
            break;
        case HALL_VALUE_2:
            /* B+, C- */
            sector = SECTOR2;
            break;
        case HALL_VALUE_6:
            /* A+, C- */
            sector = SECTOR1;
            break;
        case HALL_VALUE_4:
            /* A+, B- */
            sector = SECTOR6;
            break;
        case HALL_VALUE_5:
            /* C+, B- */
            sector = SECTOR5;
            break;
        case HALL_VALUE_1:
            /* C+, A- */
            sector = SECTOR4;
            break;
        default:
            break;
    }
    return sector;
}

/**
  * @brief CCW: Matching sector corresponding to the hall value.
  * @param hallValue Sum of hallC << 2, hallB << 1 and hallA << 1.
  * @retval Motor sector.
  */
static HALL_SECTOR MatchCcw(unsigned int hallValue)
{
    HALL_SECTOR sector = SECTOR1;
    switch (hallValue) {
        case HALL_VALUE_3:
            /* A+, B- */
            sector = SECTOR6;
            break;
        case HALL_VALUE_2:
            /* C+, B- */
            sector = SECTOR5;
            break;
        case HALL_VALUE_6:
            /* C+, A- */
            sector = SECTOR4;
            break;
        case HALL_VALUE_4:
            /* B+, A- */
            sector = SECTOR3;
            break;
        case HALL_VALUE_5:
            /* B+, C- */
            sector = SECTOR2;
            break;
        case HALL_VALUE_1:
            /* A+, C- */
            sector = SECTOR1;
            break;
        default:
            break;
    }
    return sector;
}

/**
  * @brief Matching sector corresponding to the hall value.
  * @param hallValue Sum of hallC << 2, hallB << 1 and hallA << 1.
  * @retval Motor sector.
  */
static HALL_SECTOR SectorMatch(unsigned int hallValue)
{
    int dir = g_hall.dir;
    if (dir == 1) { /* Clock Wise rotation: sector increase direction. */
        return MatchCw(hallValue);
    } else if (dir == -1) { /* Counter Clock Wise rotation: sector decrease direction. */
        return MatchCcw(hallValue);
    } else {
        return SECTOR1;
    }
}

/**
  * @brief Set pwm duty.
  * @param aptUvw APT base address of U V W.
  * @param maxDutyCnt APT count maximun value.
  * @param duty Pwm duty: 0 ~ 1.
  * @retval None.
  */
static void SetPwmDuty(APT_RegStruct **aptUvw, unsigned short maxDutyCnt, float duty)
{
    MCS_ASSERT_PARAM(aptUvw != NULL);
    MCS_ASSERT_PARAM(duty >= 0.0f);
    MCS_ASSERT_PARAM(duty <= 1.0f);
    unsigned short dutyCnt = (unsigned short)Clamp((float)maxDutyCnt * (1.0f - duty), (float)maxDutyCnt, 0.0f);
    /* Set three phase duty. */
    for (int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = aptUvw[i];
        DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_C, dutyCnt);
        DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_D, dutyCnt);
    }
}

/**
  * @brief Initialzer of system tick.
  * @param mtrCtrl Motor control struct handle.
  * @retval None.
  */
static void TimerTickInit(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->sysTickCnt = 0;
    mtrCtrl->msTickNum = US_PER_MS / SYSTICK_PERIOD_US;
    mtrCtrl->capChargeTickNum = (INV_CAP_CHARGE_MS * US_PER_MS / SYSTICK_PERIOD_US);
}


/**
  * @brief Initialzer of speed PI control struct handle.
  * @param mtrCtrl Speed control struct handle.
  * @param piParam Speed control pi parameter.
  * @param ts Speed control period (s).
  * @retval None.
  */
static void SpdCtrlInit(MTRCTRL_Handle *mtrCtrl, float ts)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    PID_Reset(&mtrCtrl->spdPi); /* reset speed loop PI */
    mtrCtrl->spdPi.kp = SPD_KP;
    mtrCtrl->spdPi.ki = SPD_KI;
    mtrCtrl->spdPi.upperLimit = SPD_UPPERLIM;
    mtrCtrl->spdPi.lowerLimit = SPD_LOWERLIM;
    mtrCtrl->spdPi.ts = ts;
}

/**
  * @brief Initialzer of under and over voltage fault detection.
  * @param uovd Under and over voltage struct handle.
  * @param ts Speed control period (s).
  * @retval None.
  */
static void FP_UOVD_InitWrapper(FP_UOVD_Handle *uovd, float ts)
{
    /* Parameter init. */
    /* Include under and over voltage detection parameter. */
    FP_UOVD_Param param = {
        .overProThr = OVD_MAX_VOLT_V,
        .overRecThr = OVD_REC_VOLT_V,
        .underProThr = LVD_MIN_VOLT_V,
        .underRecThr = LVD_REC_VOLT_V,
        .detWindow = VOLT_DET_WINDOW_S,
        .recWindow = VOLT_REC_WINDOW_S,
    };
    FP_UOVD_Init(uovd, &param, ts);
}

/**
  * @brief Init motor control task.
  * @retval None.
  */
static void TSK_InitCp(void)
{
    g_mc.stateMachine = FSM_IDLE;
    g_mc.aptMaxCntCmp = g_apt0.waveform.timerPeriod;
    g_mc.hall = &g_hall;

    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE); /* Init speed slope */

    MtrParamInit(&g_mc.mtrParam, g_motorParam);

    TimerTickInit(&g_mc);

    SpdCtrlInit(&g_mc, CTRL_SYSTICK_PERIOD); /* Init speed controller */

    HALL_Init(&g_hall, HALL_DIR, HALL_ANGLE_PLL_BDW, HALL_SPD_FILTER_FC, CTRL_CURR_PERIOD);
    /* Protection. */
    FP_UOVD_InitWrapper(&g_mc.uovd, CTRL_SYSTICK_PERIOD);

    FP_OTD_Init(&g_mc.otd, OTD_MAX_TEMP_MTR, OTD_MAX_TEMP_BRD, OTD_MTR_WINDOW_S, OTD_BRD_WINDOW_S, CTRL_SYSTICK_PERIOD);

    FP_OCD_Init(&g_mc.ocd, OCD_MAX_CURR_A, OCD_WINDOW_S, CTRL_CURR_PERIOD);

    FP_STD_Init(&g_mc.std, STD_LOSESPD_LOWER_HZ, STD_LOSESPD_UPPER_HZ, STD_DET_WINDOW_S, CTRL_CURR_PERIOD);

    FP_OPD_Init(&g_mc.opd, OPD_MIN_CURR_A, OPD_WINDOW_S, CTRL_CURR_PERIOD);
}

/**
  * @brief Clear historical values of all controller before start-up.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ClearBeforeStartup(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->sysTickCnt = 0;
    mtrCtrl->spdRef = 0.0f;
    mtrCtrl->pwmDuty = 0.0f;
    /* Clear the history value of speed slope control */
    RMG_Clear(&mtrCtrl->spdRmg);

    PID_Clear(&mtrCtrl->spdPi);

    HALL_Clear(&g_hall);
    /* Clear protection history value. */

    Fault_Clear(&mtrCtrl->faultStatus);

    FP_UOVD_Clear(&mtrCtrl->uovd);

    FP_OTD_Clear(&mtrCtrl->otd);

    FP_OCD_Clear(&mtrCtrl->ocd);

    FP_STD_Clear(&mtrCtrl->std);

    FP_OPD_Clear(&mtrCtrl->opd);
}


/**
  * @brief Open the three-phase lower pipe.
  * @param aptAddr Three-phase APT address pointer.
  * @param maxDutyCnt Max duty count.
  * @retval None.
  */
static void AptTurnOnLowSidePwm(APT_RegStruct **aptAddr, unsigned short maxDutyCnt)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    MCS_ASSERT_PARAM(maxDutyCnt != 0);
    unsigned short dutyCnt;
    dutyCnt = (unsigned short)(maxDutyCnt * APT_FULL_DUTY);
    /* Open the three-phase lower pipe */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_C, dutyCnt);
        DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_D, dutyCnt);
    }
}

/**
  * @brief Enable three-phase pwm output.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void MotorPwmOutputEnable(APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* Enable three-phase pwm output */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = aptAddr[i];
        DCL_APT_DisableSwContPWMAction(aptx, APT_PWM_CHANNEL_A);
        DCL_APT_DisableSwContPWMAction(aptx, APT_PWM_CHANNEL_B);
    }
}

/**
  * @brief Disable three-phase pwm output.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void MotorPwmOutputDisable(volatile APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* Disable three-phase pwm output. */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = aptAddr[i];
        DCL_APT_ForcePWMOutputLow(aptx);
    }
}

static void FualtDetCarr(void)
{
    if (g_mc.stateMachine == FSM_RUN) {
        /* Over current det. */
        FP_OCD_Exec(&g_mc.ocd, &g_mc.faultStatus, g_mc.iuvw);
        /* Open phase det. */
        FP_OPD_Exec(&g_mc.opd, &g_mc.faultStatus, g_mc.iuvw);
        /* Stall det. */
        FP_STD_Exec(&g_mc.std, &g_mc.faultStatus, g_mc.hall->timer, g_mc.spdFbk);
    }
    /* Fault judgement. */
    if (g_mc.stateMachine == FSM_RUN || g_mc.stateMachine == FSM_STARTUP) {
        if (g_mc.faultStatus.all) {
            g_mc.stateMachine = FSM_FAULT;
        }
    }
}


/**
  * @brief Pre-processing of motor status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MotorStatePreProc(SysStatusReg *statusReg, volatile FSM_State *stateMachine)
{
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* Get system status */
    if (SysIsError(statusReg)) {
        *stateMachine = FSM_FAULT;
    }
    if (SysGetCmdStop(statusReg)) {
        SysCmdStopClr(statusReg);
        *stateMachine = FSM_STOP;
    }
}

/**
  * @brief Check over current status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckOverCurrentState(SysStatusReg *statusReg, volatile FSM_State *stateMachine)
{
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* check systerm error status */
    if (SysIsError(statusReg) == false) {
        *stateMachine = FSM_IDLE;
    }
}

/**
  * @brief Check bootstrap capacitor charge time.
  * @param mtrCtrl The motor control handle.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckBootstrpCapChargeTime(MTRCTRL_Handle *mtrCtrl, volatile FSM_State *stateMachine)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    mtrCtrl->sysTickCnt++;
    /* check bootstrap capacitor charge time */
    if (mtrCtrl->sysTickCnt == mtrCtrl->capChargeTickNum) {
        *stateMachine = FSM_OFFSET_CALIB;
        mtrCtrl->sysTickCnt = 0;
    }
}


/**
  * @brief Check systerm cmd start signal.
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckSysCmdStart(MTRCTRL_Handle *mtrCtrl, volatile APT_RegStruct **aptAddr,
    SysStatusReg *statusReg, volatile FSM_State *stateMachine)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* check start cmd */
    if (SysGetCmdStart(statusReg)) {
        SysRunningSet(statusReg);
        SysCmdStartClr(statusReg);
        mtrCtrl->sysTickCnt = 0;
        *stateMachine = FSM_CAP_CHARGE;
        /* Preparation for charging the bootstrap capacitor. */
        AptTurnOnLowSidePwm(aptAddr, mtrCtrl->aptMaxCntCmp);
        /* Out put pwm */
        MotorPwmOutputEnable(aptAddr);
    }
}

/**
  * @brief Check adc calibration status.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void CheckAdcClabrStatus(MTRCTRL_Handle *mtrCtrl)
{
    if (mtrCtrl->adcCalibr.state == ADC_CALIBR_FINISH) {
        mtrCtrl->stateMachine = FSM_CLEAR;
    }
}

/**
  * @brief System timer tick task.
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void TSK_SystickIsr(MTRCTRL_Handle *mtrCtrl, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    SysStatusReg *statusReg = &mtrCtrl->statusReg;
    volatile FSM_State *stateMachine = &mtrCtrl->stateMachine;
    mtrCtrl->msTickCnt++;
    /* Pre-processing of motor status. */
    MotorStatePreProc(statusReg, stateMachine);
    /* statemachine */
    switch (*stateMachine) {
        case FSM_IDLE:
            CheckSysCmdStart(mtrCtrl, aptAddr, statusReg, stateMachine);
            break;

        case FSM_CAP_CHARGE:
            /* Bootstrap Capacitor Charging Timing */
            CheckBootstrpCapChargeTime(mtrCtrl, stateMachine);
            break;

        case FSM_OFFSET_CALIB:
            /* Adc offset calibration. */
            CheckAdcClabrStatus(mtrCtrl);
            break;

        case FSM_CLEAR:
            /* Clear history value. */
            ClearBeforeStartup(mtrCtrl);
            *stateMachine = FSM_STARTUP;
            break;

        case FSM_STARTUP:
            *stateMachine = FSM_RUN;
            break;

        case FSM_RUN:
            /* Speed ramp control */
            mtrCtrl->spdRef = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmd);
            /* Speed loop control */
            mtrCtrl->spdPi.error = mtrCtrl->spdRef - mtrCtrl->spdFbk;
            mtrCtrl->pwmDuty = PI_Exec(&mtrCtrl->spdPi);
            break;

        case FSM_STOP:
            /* shunt dowm pwm output. */
            MotorPwmOutputDisable(aptAddr);
            /* Clear run state. */
            SysRunningClr(statusReg);
            mtrCtrl->spdFbk = 0.0f;
            *stateMachine = FSM_IDLE;
            break;

        case FSM_FAULT: /* Overcurrent state */
            CheckOverCurrentState(statusReg, stateMachine);
            MotorPwmOutputDisable(aptAddr);
            break;

        default:
            break;
    }
}

/**
  * @brief Three-phase current bias calibration.
  * @param IBIAS_Handle Adc calibration struct.
  * @retval None.
  */
static void readCurrBiasCb(IBIAS_Handle *iuvwAdcBias)
{
    MCS_ASSERT_PARAM(iuvwAdcBias != NULL);
    iuvwAdcBias->iBusAdcBias = ADCCALIBR_Exec(&g_mc.adcCalibr, &ADC_U_HANDLE, ADC_U_SOC_NUM);
}


/**
  * @brief Temprature table.
  * @param tempResisValue Temperature sensor resistance.
  * @retval None.
  */
static float TempTable(float tempResisValue)
{
    float boardTemp = 0.0f;
    /* Temperatures between 15 and 30. */
    if (tempResisValue > TEMP_RES_30 &&  tempResisValue <= TEMP_RES_15) {
        boardTemp = TEMP_15 + (TEMP_30 - TEMP_15) * (TEMP_RES_15 - tempResisValue) / (TEMP_RES_15 - TEMP_RES_30);
    } else if (tempResisValue > TEMP_RES_45 &&  tempResisValue <= TEMP_RES_30) { /* Temperature between 30 and 45. */
        boardTemp = TEMP_30 + (TEMP_45 - TEMP_30) * (TEMP_RES_30 - tempResisValue) / (TEMP_RES_30 - TEMP_RES_45);
    } else if (tempResisValue > TEMP_RES_60 &&  tempResisValue <= TEMP_RES_45) { /* Temperature between 45 and 50. */
        boardTemp = TEMP_45 + (TEMP_60 - TEMP_45) * (TEMP_RES_45 - tempResisValue) / (TEMP_RES_45 - TEMP_RES_60);
    } else if (tempResisValue <= TEMP_RES_60) {
        boardTemp = TEMP_60;    /* If temperature is over 60, set temperature as 60. */
    } else if (tempResisValue >= TEMP_RES_15) {
        boardTemp = TEMP_15;    /* If temperature is lower 15, set temperature as 15. */
    }
    return boardTemp;
}

/**
  * @brief Read power board temperature and udc.
  * @retval None.
  */
static void ReadBoardTemp(void)
{
    HAL_ADC_SoftTrigSample(&ADC_TEMP_HANDLE, ADC_TEMP_SOC_NUM);
    /* Force convert to float type. */
    float resisAdcValue = (float)HAL_ADC_GetConvResult(&ADC_TEMP_HANDLE, ADC_TEMP_SOC_NUM);
    float resisValue = (4096 * 10.0f - 10.0f * resisAdcValue) / resisAdcValue;
    g_mc.powerBoardTemp = TempTable(resisValue);
}

/**
  * @brief System timer ISR for Motor Statemachine CallBack function.
  * @param param The systick timer handle.
  * @retval None.
  */
void MotorStatemachineCallBack(void *param)
{
    MCS_ASSERT_PARAM(param != NULL);
    TIMER_Handle *timer = (TIMER_Handle *)param;
    TIMER_ASSERT_PARAM(timer != NULL);
    TIMER_ASSERT_PARAM(timer->baseAddress != NULL);
    /* Clear timer interrupt. */
    DCL_TIMER_IrqClear(timer->baseAddress);
    /* Get board temperature. */
    ReadBoardTemp();
    TSK_SystickIsr(&g_mc, g_aptAddr);
    if (g_mc.stateMachine == FSM_RUN) {
        /* Over and under voltage det. */
        FP_UOVD_Exec(&g_mc.uovd, &g_mc.faultStatus, g_mc.udc);
        /* Over temperature det. */
        FP_OTD_Exec(&g_mc.otd, &g_mc.faultStatus, 0.0f, g_mc.powerBoardTemp);
    }
}

/**
  * @brief Check Potentiometer Value callback function.
  * @param param The TIMER_Handle.
  * @retval None.
  */
void CheckPotentiometerValueCallback(void *param)
{
    static unsigned int potentiomitorAdcValue = 0;
    static float spdCmd = 0;
    static float spdCmdHzLast = USER_MAX_SPD_HZ;
    MCS_ASSERT_PARAM(param != NULL);
    TIMER_Handle *timer = (TIMER_Handle *)param;
    TIMER_ASSERT_PARAM(timer != NULL);
    TIMER_ASSERT_PARAM(timer->baseAddress != NULL);
    /* Clear timer interrupt. */
    DCL_TIMER_IrqClear(timer->baseAddress);
    /* Speed triger. */
    HAL_ADC_SoftTrigSample(&ADC_SPEED_HANDLE, ADC_SOC_NUM2);
    potentiomitorAdcValue = HAL_ADC_GetConvResult(&ADC_SPEED_HANDLE, ADC_SOC_NUM2);
    /* 4045.0 is adc sample max value of potentiomitor, make sure max spd 180 */
    spdCmd = (float)((float)potentiomitorAdcValue / 4045.0f * USER_MAX_SPD_HZ);
    /* Do not change speedCmd if speed command increment is less than 1.0. */
    if (Abs(spdCmdHzLast - spdCmd) < 1.0) {
        return;
    }
    spdCmdHzLast = spdCmd;
    /* Restrict min speed. */
    if (spdCmd < USER_MIN_SPD_HZ) {
        spdCmd = USER_MIN_SPD_HZ;
    }
    /* Restrict max speed. */
    if (spdCmd > USER_MAX_SPD_HZ) {
        spdCmd = USER_MAX_SPD_HZ;
    }
    DBG_PRINTF("speed cmd Hz is %f, potentiomitor value is %d \r\n", spdCmd, potentiomitorAdcValue);
    g_mc.spdCmd = spdCmd;
}

/**
  * @brief Motor blockage protection function.
  * @retval None.
  */
static void MotorBlockageProtect(void)
{   /* If no Hall signal jump is detected for more than 1s, the generator blocking protection is triggered. */
    if (g_hall.timer > 2.0f) { /* Block protect when hall time is larger than 2.0s. */
        /* disables the three-phase PWM output. */
        MotorPwmOutputDisable(g_aptAddr);
        /* Status setting error */
        SysErrorSet(&g_mc.statusReg);
        g_hall.spdEst = 0.0f;
    }
}


/**
  * @brief Event interrupt callback function of APT module.
  * @param para APT module handle.
  * @retval None.
  */
void MotorSysErrCallback(void *para)
{
    MCS_ASSERT_PARAM(para != NULL);
    APT_Handle *handle = (APT_Handle *)para;
    /* The fan IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_aptAddr);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptAddr[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptAddr[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptAddr[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_mc.statusReg);
    g_mc.stateMachine = FSM_FAULT;
    DBG_PRINTF("APT error! \r\n");
    g_mc.faultStatus.Bit.hardOverCurFault = 1;
    HAL_GPIO_SetValue(&LED2_HANDLE, LED2_PIN, GPIO_LOW_LEVEL);
    BASE_FUNC_UNUSED(handle);
}


/**
  * @brief Init motor controller's data structure.
  * @retval None.
  */
static void InitSoftware(void)
{
    /* Initializing motor Control Tasks */
    TSK_InitCp();
}

/**
  * @brief Read the ADC current sampling value.
  * @param iuvw Three-phase current.
  * @retval None.
  */
static void ReadCurrUvw(UvwAxis *iuvw, HALL_SECTOR sector)
{
    MCS_ASSERT_PARAM(iuvw != NULL);
    /* Sample point is set to pwm falling edge left offset by 100. */
    float point = g_mc.aptMaxCntCmp * (1.0f - g_mc.pwmDuty) + 100.0f; /* Point shift is 100. */
    unsigned short samplePoint = (unsigned short)Clamp(point, g_mc.aptMaxCntCmp - 1.0f, 1.0f);
    DCL_APT_SetCounterCompare(g_apt0.baseAddress, APT_COMPARE_REFERENCE_A, samplePoint);
    float iBusAbs = Abs(((float)HAL_ADC_GetConvResult(&ADC_U_HANDLE, ADC_U_SOC_NUM)
                    - g_mc.iuvwAdcBias.iBusAdcBias) * ADC_CURR_COFFI);
    float iu, iv, iw;
    /* Calc uvw phase current according to sector. */
    switch (sector) {
        case SECTOR1: /* A+C- */
            iu = iBusAbs;
            iv = 0.0f;
            iw = -iBusAbs;
            break;

        case SECTOR2: /* B+C- */
            iu = 0.0f;
            iv = iBusAbs;
            iw = -iBusAbs;
            break;

        case SECTOR3: /* B+A- */
            iu = -iBusAbs;
            iv = iBusAbs;
            iw = 0.0f;
            break;

        case SECTOR4: /* C+A- */
            iu = -iBusAbs;
            iv = 0.0f;
            iw = iBusAbs;
            break;

        case SECTOR5: /* C+B- */
            iu = 0.0f;
            iv = -iBusAbs;
            iw = iBusAbs;
            break;

        case SECTOR6: /* A+B- */
            iu = iBusAbs;
            iv = -iBusAbs;
            iw = 0.0f;
            break;

        default:
            iu = 0.0f;
            iv = 0.0f;
            iw = 0.0f;
            break;
        }
    iuvw->u = iu;
    iuvw->v = iv;
    iuvw->w = iw;
}

/**
  * @brief The carrier ISR wrapper function.
  * @param aptHandle The APT handle.
  * @retval None.
  */
void MotorCarrierProcessCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* Calculate hall sector. */
    HALL_SECTOR sector = SectorMatch(GetHallValue());
    HALL_Exec(&g_hall, sector);
    g_mc.spdFbk = g_hall.spdEst;

    if (g_mc.stateMachine == FSM_OFFSET_CALIB) {
        readCurrBiasCb(&g_mc.iuvwAdcBias);
    }
    if (g_mc.stateMachine == FSM_RUN) {
        SIXSTEP_AptConfig(g_aptAddr, sector);
        /* Stall fault detection. */
        SetPwmDuty(g_aptAddr, g_mc.aptMaxCntCmp, g_mc.pwmDuty);
        MotorBlockageProtect();
    }

    /* Fault detection. */
    if (g_mc.stateMachine == FSM_STARTUP || g_mc.stateMachine == FSM_RUN) {
        FualtDetCarr();
    }
    g_mc.udc = (float)HAL_ADC_GetConvResult(&ADC_UDC_HANDLE, ADC_UDC_SOC_NUM) * ADC_VOLT_COFFI;
    ReadCurrUvw(&g_mc.iuvw, sector);
}


/**
  * @brief Config the KEY func.
  * @param handle The GPIO handle.
  * @retval None.
  */
static KEY_State Key_StateRead(GPIO_Handle *handle)
{
    if (HAL_GPIO_GetPinValue(handle, handle->pins) == 0) {
        BASE_FUNC_DELAY_MS(30);  /* delay 30ms for deshake */
        if (HAL_GPIO_GetPinValue(handle, handle->pins) == 0) {
            while (HAL_GPIO_GetPinValue(handle, handle->pins) == 0) {
            }
            return KEY_DOWN;
        }
    }
    return KEY_UP;
}

/**
  * @brief Control motor start and stop state by key func.
  * @param param The GPIO handle.
  * @retval None.
  */
void MotorStartStopKeyCallback(void *param)
{
    static unsigned char motorStateFlag = 0;
    GPIO_Handle *handle = (GPIO_Handle *)param;
    if (Key_StateRead(handle) == KEY_DOWN) {
        if (motorStateFlag == 0) { /* stop apt output, motor is off status */
            motorStateFlag = 1;
            SysCmdStartSet(&g_mc.statusReg);
            HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);
        } else { /* start apt output, motor is on status */
            motorStateFlag = 0;
            SysCmdStopSet(&g_mc.statusReg);
            HAL_APT_StopModule(RUN_APT0 | RUN_APT1 | RUN_APT2);
        }
    }
}


/**
  * @brief User application main entry for ECMCU105H board.
  * @retval BSP_OK.
  */
int MotorMain(void)
{
    unsigned int tickNum1Ms = 2; /* 1ms tick */
    static unsigned int tickCnt1Ms = 0;
    unsigned int tickNum500Ms = 1000; /* 500ms tick */
    static unsigned int tickCnt500Ms = 0;
    for (int i = 0; i < PHASE_MAX_NUM; i++) {
        /* 针对强制输出而言, PG_BUF_EN寄存器rg_frc_buf_en位的作用是强制输出是0立即更新还是1缓存加载（下周期更新）. */
        /*                 PG_ACT_LD寄存器rg_pg_frcld_zroen位的作用为强制输出是否在0点生效. */
        /* 下面两行语句所表示的含义是强制输出使能为缓存加载方式，并且在下周期0点生效, 默认两者为0。 */
        g_aptAddr[i]->PG_BUF_EN.BIT.rg_frc_buf_en = 1;
        g_aptAddr[i]->PG_ACT_LD.BIT.rg_pg_frcld_zroen = 1;
    }

    SystemInit();

    /* System Timer clock. */
    HAL_TIMER_Start(&g_timer0);
    HAL_TIMER_Start(&g_timer1);

    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_aptAddr);

    /* Software initialization. */
    InitSoftware();

    /* Start the PWM clock. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);

    while (1) {
        if (g_mc.msTickCnt - tickCnt1Ms >= tickNum1Ms) {
            tickCnt1Ms = g_mc.msTickCnt;
            /* User Code 1ms Event */
            /* User Code 1ms Event */
        }

        if (g_mc.msTickCnt - tickCnt500Ms >= tickNum500Ms) {
            if (SysIsError(&g_mc.statusReg) != true) {
               /* The LED blinks when no status is not error. */
                HAL_GPIO_TogglePin(&LED1_HANDLE, LED1_PIN);
            }
            tickCnt500Ms = g_mc.msTickCnt;
        }
    }
    return 0;
}