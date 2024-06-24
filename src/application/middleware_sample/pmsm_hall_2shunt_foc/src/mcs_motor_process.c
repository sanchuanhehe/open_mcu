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
  * @file      mcs_motor_process.c
  * @author    MCU Algorithm Team
  * @brief     This file provides motor application.
  * @details   Single FOC application based on the ECBMCU201MPC board
  *            1) Motor model is 42JSF630AS-1000.
  *            2) Select the pmsm hall 2shunt foc example in the sample column
                  of chipConfig and click Generate Code.
  *            3) It's power supply must be changed to 24V.
  */
#include "main.h"
#include "mcs_user_config.h"
#include "mcs_math.h"
#include "mcs_ctlmode_config.h"
#include "mcs_prot_user.h"
#include "mcs_prot_user_config.h"
#include "mcs_math_const.h"
#include "mcs_motor_process.h"
#include "mcs_chip_config.h"
#include "mcs_sensor_hall.h"
#include "mcs_carrier.h"

/*------------------------------- Macro Definition -----------------------------------------------*/
#define US_PER_MS               1000
#define APT_FULL_DUTY           1.0f
#define TEMP_3                  3.0f
#define TEMP_15                 15.0f
#define TEMP_30                 30.0f
#define TEMP_45                 45.0f
#define TEMP_60                 60.0f
#define TEMP_RES_15             78.327f
#define TEMP_RES_30             36.776f
#define TEMP_RES_45             18.301f
#define TEMP_RES_60             9.607f
#define MOTOR_START_DELAY       2
#define ADC_READINIT_DELAY      1
#define ADC_READINIT_TIMES      20
#define ADC_TRIMVALUE_MIN       1800.0f
#define ADC_TRIMVALUE_MAX       2200.0f
/*------------------------------- Param Definition -----------------------------------------------*/
/* Motor parameters. */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
static MOTOR_Param g_motorParam = MOTORPARAM_DEFAULTS;
static APT_RegStruct* g_apt[PHASE_MAX_NUM] = {APT_U, APT_V, APT_W};
/* Motor control handle */
static MTRCTRL_Handle g_mc = {0};
static HALL_Handle g_hall = {0};

/* Motor speed loop PI param. */
static void SPDCTRL_InitWrapper(SPDCTRL_Handle *spdHandle, float ts)
{
    /* Speed loop param assignment. */
    PI_Param spdPi = {
        .kp = SPD_KP,
        .ki = SPD_KI,
        .lowerLim = SPD_LOWERLIM,
        .upperLim = SPD_UPPERLIM,
    };
    /* Speed loop param init. */
    SPDCTRL_Init(spdHandle, &g_motorParam, spdPi, ts);
}

/* Motor current Loop PI param. */
static void CURRCTRL_InitWrapper(CURRCTRL_Handle *currHandle, DqAxis *idqRef, DqAxis *idqFbk, float ts)
{
    /* Axis-D current loop param assignment. */
    PI_Param dCurrPi = {
        .kp = CURRDAXIS_KP,
        .ki = CURRDAXIS_KI,
        .lowerLim = CURR_LOWERLIM,
        .upperLim = CURR_UPPERLIM,
    };
    /* Axis-Q current loop param assignment. */
    PI_Param qCurrPi = {
        .kp = CURRQAXIS_KP,
        .ki = CURRQAXIS_KI,
        .lowerLim = CURR_LOWERLIM,
        .upperLim = CURR_UPPERLIM,
    };
    /* Current loop param init. */
    CURRCTRL_Init(currHandle, &g_motorParam, idqRef, idqFbk, dCurrPi, qCurrPi, ts);
}

/*------------------------------- Function Definition -----------------------------------------------*/
/**
 * @brief Get the encoder speed and angle.
 * @param speed electricity speed.
 * @param angle electricity angle.
 * @retval None.
 */
static void GetHallAngSpd(float* speed, float* angle)
{
    static TrigVal localTrigVal;
    /* Hall speed and angle calculation */
    HALL_AngSpdCalcExec(&g_hall);

    /* Hall angle PLL filter */
    TrigCalc(&localTrigVal, g_hall.angle);
    PLL_Exec(&g_mc.hallAnglePll, localTrigVal.sin, localTrigVal.cos);
    /* Hall speed first order low-pass filter */
    *speed = FOLPF_Exec(&g_mc.hallSpdFilter, g_hall.spd);
    *angle = g_mc.hallAnglePll.angle;
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
  * @brief Init motor control task.
  * @retval None.
  */
static void TSK_Init(void)
{
    g_mc.motorStateFlag = 0;
    g_mc.uartHeartDetCnt = 0;
    g_mc.uartTimeStamp = 0;
    g_mc.stateMachine = FSM_IDLE;
    g_mc.currCtrlPeriod = CTRL_CURR_PERIOD; /* Init current controller */
    g_mc.aptMaxcntCmp = g_apt0.waveform.timerPeriod;
    g_mc.sampleMode = DUAL_RESISTORS;
    g_mc.obserType = FOC_OBSERVERTYPE_ENC;      /* Init foc observe mode */
    g_mc.controlMode = SIXSTEPWAVE_CONTROLMODE;     /* Init motor control mode */
    g_mc.adcCurrCofe = ADC_CURR_COFFI;
    g_mc.spdAdjustMode = CUST_SPEED_ADJUST;
    g_mc.uartConnectFlag = DISCONNECT;
    g_mc.spdCmdHz = USER_INIT_SPD_HZ;  /* Motor initialization speed */
    g_mc.hallHandle = &g_hall;

    g_mc.adc0Compensate = ADC0COMPENSATE;  /* Phase-u current init adc shift trim value */
    g_mc.adc1Compensate = ADC1COMPENSATE;  /* Phase-w current init adc shift trim value */

    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE); /* Init speed slope */
    MtrParamInit(&g_mc.mtrParam, g_motorParam);

    TimerTickInit(&g_mc);
    SVPWM_Init(&g_mc.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);
    R1SVPWM_Init(&g_mc.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);

    SPDCTRL_InitWrapper(&g_mc.spdCtrl, CTRL_SYSTICK_PERIOD);
    CURRCTRL_InitWrapper(&g_mc.currCtrl, &g_mc.idqRef, &g_mc.idqFbk, CTRL_CURR_PERIOD);

    /* Init hall module */
    HALL_Init(&g_hall, HALL_PHASESHIFT, CTRL_CURR_PERIOD);
    PLL_Init(&g_mc.hallAnglePll, CTRL_CURR_PERIOD, HALL_ANGLE_PLL_BDW);
    FOLPF_Init(&g_mc.hallSpdFilter, CTRL_CURR_PERIOD, HALL_SPD_FILTER_FC);
}

/**
  * @brief Clear historical values of all controller before start-up.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ClearBeforeStartup(MTRCTRL_Handle *mtrCtrl)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    /* The initial angle is 0. */
    mtrCtrl->axisAngle = 0;
    mtrCtrl->hallSpeed = 0;
    
    mtrCtrl->spdRefHz = 0.0f;
    /* The initial dq-axis reference current is 0. */
    mtrCtrl->idqRef.d = 0.0f;
    mtrCtrl->idqRef.q = 0.0f;

    mtrCtrl->vdqRef.d = 0.0f;
    mtrCtrl->vdqRef.q = 0.0f;
    /* Clear Duty Cycle Value. The initial duty cycle is 0.5. */
    mtrCtrl->dutyUvwLeft.u = 0.5f;
    mtrCtrl->dutyUvwLeft.v = 0.5f;
    mtrCtrl->dutyUvwLeft.w = 0.5f;
    mtrCtrl->dutyUvwRight.u = 0.5f;
    mtrCtrl->dutyUvwRight.v = 0.5f;
    mtrCtrl->dutyUvwRight.w = 0.5f;

    mtrCtrl->prot.motorErrStatus.all = 0x00;

    RMG_Clear(&mtrCtrl->spdRmg); /* Clear the history value of speed slope control */
    CURRCTRL_Clear(&mtrCtrl->currCtrl);
    IF_Clear(&mtrCtrl->ifCtrl);
    SPDCTRL_Clear(&mtrCtrl->spdCtrl);
    STARTUP_Clear(&mtrCtrl->startup);
    R1SVPWM_Clear(&mtrCtrl->r1Sv);
    HALL_ParamClear(&g_hall);
}

/**
  * @brief To set the comparison value of the IGBT single-resistance ADC sampling trigger position.
  * @param aptx The APT register struct handle.
  * @param cntCmpA A Count compare reference of time-base counter.
  * @param cntCmpB B Count compare reference of time-base counter.
  * @param maxCntCmp Maximum Comparison Value
  * @retval None.
  */
static void MCS_SetAdcCompareR1(APT_RegStruct *aptx, unsigned short cntCmpA,
                                unsigned short cntCmpB, unsigned short maxCntCmp)
{
    unsigned short tmp;
    /* Sets the A Count compare reference of time-base counter. */
    tmp = (unsigned short)Clamp((float)(cntCmpA), (float)(maxCntCmp - 1), 1.0f);
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_A, tmp);
    /* Sets the B Count compare reference of time-base counter. */
    tmp = (unsigned short)Clamp((float)(cntCmpB), (float)(maxCntCmp - 1), 1.0f);
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_B, tmp);
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
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_UNSET;
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_UNSET;
    }
}

/**
  * @brief Disable three-phase pwm output.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void MotorPwmOutputDisable(APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* Disable three-phase pwm output. */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_SET;
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_SET;
        DCL_APT_ForcePWMOutputLow(aptx);
    }
}

/**
  * @brief Set six step angel at start up stage.
  * @param HALL_Handle Hall struct handle.
  * @retval None.
  */
static float CalcSixStepRadian(HALL_Handle *hallHandle)
{
    /* Angle assignment in different sector */
    float sixStepAngle = 0.0f;
    switch (hallHandle->currentHallValue) {
        /* Sector one the angle is -150 */
        case SECTOR_ONE:
            sixStepAngle = -EANGLE150;
            break;
        /* Sector two the angle is -90 degree */
        case SECTOR_TWO:
            sixStepAngle = -EANGLE90;
            break;
        /* Sector three the angle is -30 degree */
        case SECTOR_THREE:
            sixStepAngle = -EANGLE30;
            break;
        /* Sector four the angle is 30 degree */
        case SECTOR_FOUR:
            sixStepAngle = EANGLE30;
            break;
        /* Sector five the angle is 90 degree */
        case SECTOR_FIVE:
            sixStepAngle = EANGLE90;
            break;
        /* Sector six the angle is 150 degree */
        case SECTOR_SIX:
            sixStepAngle = EANGLE150;
            break;

        default:
            break;
    }
    return sixStepAngle;
}

/**
  * @brief Construct a new mcs startupswitch object.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_StartupSwitch(MTRCTRL_Handle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    /* State machine */
    STARTUP_Handle *startup = &mtrCtrl->startup;
    float spdRefHz = mtrCtrl->spdRefHz;
    switch (startup->stage) {
        case STARTUP_STAGE_CURR:
            /* Calculate motor init angle */
            g_mc.hallSixStepAngle = CalcSixStepRadian(&g_hall);
            g_mc.idqRef.d = 0.0f;
            /* Cut to speed loop control */
            mtrCtrl->stateMachine = FSM_RUN;
            break;

        default:
            break;
    }
    mtrCtrl->spdRefHz = spdRefHz;
}

/**
  * @brief Pre-processing of motor status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MotorStatePerProc(SysStatusReg *statusReg, volatile FsmState *stateMachine)
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
static void CheckOverCurrentState(SysStatusReg *statusReg, FsmState *stateMachine)
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
static void CheckBootstrpCapChargeTime(MTRCTRL_Handle *mtrCtrl, FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    mtrCtrl->sysTickCnt++;
    /* check bootstrap capacitor charge time */
    if (mtrCtrl->sysTickCnt == mtrCtrl->capChargeTickNum) {
        *stateMachine = FSM_CLEAR;
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
static void CheckSysCmdStart(MTRCTRL_Handle *mtrCtrl,
                             APT_RegStruct **aptAddr,
                             SysStatusReg *statusReg,
                             FsmState *stateMachine)
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
        AptTurnOnLowSidePwm(aptAddr, mtrCtrl->aptMaxcntCmp);
        /* Out put pwm */
        MotorPwmOutputEnable(aptAddr);
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
    FsmState *stateMachine = &mtrCtrl->stateMachine;
    mtrCtrl->msTickCnt++;
    /* Pre-processing of motor status. */
    MotorStatePerProc(statusReg, stateMachine);
    /* statemachine */
    switch (*stateMachine) {
        case FSM_IDLE:
            /* Set smo estimate speed before motor start-up */
            CheckSysCmdStart(mtrCtrl, aptAddr, statusReg, stateMachine);
            break;
        case FSM_CAP_CHARGE:
            /* Bootstrap Capacitor Charging Timing */
            CheckBootstrpCapChargeTime(mtrCtrl, stateMachine);
            break;
            /* Clear parameter before start */
        case FSM_CLEAR:
            ClearBeforeStartup(mtrCtrl);
            *stateMachine = FSM_STARTUP;
            break;
        case FSM_STARTUP:
            MCS_StartupSwitch(mtrCtrl);
            break;
        case FSM_RUN:
            /* Speed ramp control */
            mtrCtrl->spdRefHz = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmdHz);
            /* Speed loop control */
            mtrCtrl->idqRef.q = SPDCTRL_Exec(&mtrCtrl->spdCtrl, mtrCtrl->spdRefHz, mtrCtrl->hallSpeed);
           
            break;
        case FSM_STOP:
            mtrCtrl->spdRefHz = 0.0f;
            mtrCtrl->controlMode = SIXSTEPWAVE_CONTROLMODE;
            MotorPwmOutputDisable(aptAddr);
            SysRunningClr(statusReg);
            *stateMachine = FSM_IDLE;
            break;
        case FSM_FAULT: /* Overcurrent state */
            CheckOverCurrentState(statusReg, stateMachine);
            break;
        default:
            break;
    }
}

/**
  * @brief Read the ADC initialize bias trim value.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void TrimInitAdcShiftValue(MTRCTRL_Handle *mtrCtrl)
{
    float adc0SampleTemp = 0.0f;  /* Current bias value for temp store */
    float adc1SampleTemp = 0.0f;
    float adc0TempSum = 0.0f;
    float adc1TempSum = 0.0f;  /* Current bias sum value for 20 times */
    float adcSampleTimes = 0.0f;  /* ADC sample times */
    for (int i = 0; i < ADC_READINIT_TIMES; i++) {
        adc0SampleTemp = (float)HAL_ADC_GetConvResult(&ADCU_HANDLE, ADCUSOCNUM);
        adc1SampleTemp = (float)HAL_ADC_GetConvResult(&ADCW_HANDLE, ADCWSOCNUM);
        BASE_FUNC_DELAY_US(200);    /* 200 is delay count, delay 200us triger adc sampling */
        if (adc0SampleTemp > 1000.0f && adc1SampleTemp > 1000.0f) {
            adcSampleTimes++;
            adc0TempSum += adc0SampleTemp;
            adc1TempSum += adc1SampleTemp;
        }
    }
    adc0SampleTemp = adc0TempSum / adcSampleTimes;
    adc1SampleTemp = adc1TempSum / adcSampleTimes;
    /* Force convert to float */
    mtrCtrl->adc0Compensate = (float) adc0SampleTemp;
    mtrCtrl->adc1Compensate = (float) adc1SampleTemp;
    /* The normal value scope: 1800 < adc0Compensate < 2200 */
    if(g_mc.adc0Compensate < ADC_TRIMVALUE_MIN || g_mc.adc0Compensate > ADC_TRIMVALUE_MAX \
       || g_mc.adc1Compensate < ADC_TRIMVALUE_MIN || g_mc.adc1Compensate > ADC_TRIMVALUE_MAX) {
        DBG_PRINTF("ADC trim value error,please reset!");
        HAL_GPIO_SetValue(&LED2_HANDLE, LED2_PIN, GPIO_LOW_LEVEL);
    }
    adcSampleTimes = 0;
    adc0TempSum = 0;
    adc1TempSum = 0;
}

/**
  * @brief Read the ADC current sampling value.
  * @param CurrUvw Three-phase current.
  * @retval None.
  */
static void ReadCurrUvw(UvwAxis *CurrUvw)
{
    MCS_ASSERT_PARAM(CurrUvw != NULL);
    float adc0 = (float)HAL_ADC_GetConvResult(&ADCU_HANDLE, ADCUSOCNUM);
    float adc1 = (float)HAL_ADC_GetConvResult(&ADCW_HANDLE, ADCWSOCNUM);
    /* Convert adc sample value to current value */
    CurrUvw->u = -(adc0 - g_mc.adc0Compensate) * g_mc.adcCurrCofe;
    CurrUvw->w = -(adc1 - g_mc.adc1Compensate) * g_mc.adcCurrCofe;
    CurrUvw->v = -CurrUvw->u - CurrUvw->w;
}

/**
  * @brief Setting the APT Output Duty Cycle.
  * @param aptx APT register base address.
  * @param leftDuty Left duty cycle.
  * @param rightDuty Right duty cycle.
  * @retval None.
  */
static void SetPwmDuty(APT_Handle *aptx, float leftDuty, float rightDuty)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    MCS_ASSERT_PARAM(leftDuty > 0);
    MCS_ASSERT_PARAM(rightDuty > 0);
    unsigned short maxPeriodCnt = aptx->waveform.timerPeriod;
    unsigned short cntCmpLeftEdge = (unsigned short)(leftDuty * maxPeriodCnt);
    unsigned short cntCmpRightEdge = (unsigned short)(rightDuty * maxPeriodCnt);
    /* avoid overflowing */
    cntCmpLeftEdge = (cntCmpLeftEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpLeftEdge;
    cntCmpRightEdge = (cntCmpRightEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpRightEdge;
    HAL_APT_SetPWMDuty(aptx, cntCmpLeftEdge, cntCmpRightEdge);
}

/**
  * @brief Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyCp(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    MCS_ASSERT_PARAM(dutyUvwLeft != NULL);
    MCS_ASSERT_PARAM(dutyUvwRight != NULL);
    /* Setting the Three-Phase Duty Cycle */
    SetPwmDuty(&g_apt0, dutyUvwLeft->u, dutyUvwRight->u);
    SetPwmDuty(&g_apt1, dutyUvwLeft->v, dutyUvwRight->v);
    SetPwmDuty(&g_apt2, dutyUvwLeft->w, dutyUvwRight->w);
}

/**
  * @brief To set the ADC sampling trigger comparison value.
  * @param cntCmpSOCA Soca Compare Count Value.
  * @param cntCmpSOCB Socb Compare Count Value.
  * @retval None.
  */
static void SetADCTriggerTime(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB)
{
    MCS_SetAdcCompareR1(g_apt[PHASE_U], cntCmpSOCA, cntCmpSOCB, g_mc.aptMaxcntCmp);
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
static void ReadBoardTempAndUdc(void)
{
    HAL_ADC_SoftTrigSample(&ADCRESIS_HANDLE, ADCRESISSOCNUM);
    HAL_ADC_SoftTrigSample(&ADCUDC_HANDLE, ADCUDCSOCNUM);
    BASE_FUNC_DELAY_US(10); /* Delay 10 us. */
    /* Force convert to float type. */
    float resisAdcValue = (float)HAL_ADC_GetConvResult(&ADCRESIS_HANDLE, ADCRESISSOCNUM);
    /* 10 / (x + 10) * 3.3 = resisAdcValue / 4096 * 3.3, x is resisValue, 10kohm is resistor divider value. */
    float resisValue = (4096.0f * 10.0f - 10.0f * resisAdcValue) / resisAdcValue;
    g_mc.powerBoardTemp = TempTable(resisValue);
    g_mc.udc = ((float)HAL_ADC_GetConvResult(&ADCUDC_HANDLE, ADCUDCSOCNUM)) * ADC_UDC_COFFI;
}

/**
  * @brief Check Potentiometer Value callback function.
  * @param param The TIMER_Handle.
  * @retval None.
  */
void CheckPotentiometerValueCallback(void *param)
{
    MCS_ASSERT_PARAM(param != NULL);
    BASE_FUNC_UNUSED(param);
    static float potentiomitorAdcValue = 0.0f;
    static float spdCmdHz = 0;
    static float spdCmdHzLast = USER_INIT_SPD_HZ;  /* 35.0 is spdCmdHzLast init value */
    HAL_ADC_SoftTrigSample(&ADCPTT_HANDLE, ADCPTTSOCNUM);
    BASE_FUNC_DELAY_US(10); /* Delay 10 us. */
    potentiomitorAdcValue = (float)HAL_ADC_GetConvResult(&ADCPTT_HANDLE, ADCPTTSOCNUM);
    /* 4045.0 is adc sample max value of potentiomitor, convert max spd to 200.0Hz */
    spdCmdHz = potentiomitorAdcValue / 4045.0f * USER_MAX_SPD_HZ;
    if (Abs(spdCmdHzLast - spdCmdHz) < 1.0f) { /* Ignore changes less than 1. */
        return;
    }
    spdCmdHzLast = spdCmdHz;
    if (spdCmdHz < USER_MIN_SPD_HZ) {  /* USER_MIN_SPD_HZ is spdCmdHz lower limit */
        spdCmdHz = USER_MIN_SPD_HZ;
    }
    if (spdCmdHz > g_mc.mtrParam.maxElecSpd) { /* spdCmdHz upper limit */
        spdCmdHz = g_mc.mtrParam.maxElecSpd;
    }
    if (g_mc.spdAdjustMode == CUST_SPEED_ADJUST) {
        g_mc.spdCmdHz = spdCmdHz;
    }
}

/**
  * @brief System timer ISR for Motor Statemachine CallBack function.
  * @param param The systick timer handle.
  * @retval None.
  */
void MotorStatemachineCallBack(void *param)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(param != NULL);
    BASE_FUNC_UNUSED(param);
    /* Read power board temprature and voltage. */
    ReadBoardTempAndUdc();
    /* Motor error state check. */
    TSK_SystickIsr(&g_mc, g_apt);
}

/**
  * @brief The carrier ISR wrapper function.
  * @param aptHandle The APT handle.
  * @retval None.
  */
void MotorCarrierProcessCallback(void *aptHandle)
{
    MCS_ASSERT_PARAM(aptHandle != NULL);
    BASE_FUNC_UNUSED(aptHandle);
    /* the carrierprocess of motor */
    MCS_CarrierProcess(&g_mc);
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
    /* The IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_apt);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_apt[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_apt[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_apt[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_mc.statusReg);
    DBG_PRINTF("APT error! \r\n");
    HAL_GPIO_SetValue(&LED2_HANDLE, LED2_PIN, GPIO_LOW_LEVEL);
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief Hall edge jump callback function of the hall sensor.
  * @param param The capm handle.
  * @param intFlag Interrupt event flag.
  * @retval None.
  */

void CAPM0EventFinishCallback(void *param, CAPM_IntEvent intFlag)
{
    CAPM_Handle *handle = (CAPM_Handle *)param;
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(intFlag);
    /* USER CODE BEGIN CAPM ITCallBackFunc */
    HALL_InformationUpdate(&g_hall);
    g_mc.hallSixStepAngle = CalcSixStepRadian(&g_hall);
    /* USER CODE END CAPM ITCallBackFunc */
}

void CAPM1EventFinishCallback(void *param, CAPM_IntEvent intFlag)
{
    CAPM_Handle *handle = (CAPM_Handle *)param;
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(intFlag);
    /* USER CODE BEGIN CAPM ITCallBackFunc */
    HALL_InformationUpdate(&g_hall);
    g_mc.hallSixStepAngle = CalcSixStepRadian(&g_hall);
    /* USER CODE END CAPM ITCallBackFunc */
}

void CAPM2EventFinishCallback(void *param, CAPM_IntEvent intFlag)
{
    CAPM_Handle *handle = (CAPM_Handle *)param;
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(intFlag);
    /* USER CODE BEGIN CAPM ITCallBackFunc */
    HALL_InformationUpdate(&g_hall);
    g_mc.hallSixStepAngle = CalcSixStepRadian(&g_hall);
    /* USER CODE END CAPM ITCallBackFunc */
}

static unsigned int GetHallValue(void)
{
#if defined CHIP_3061MNPICA
    /* Get three hall values. */
    unsigned int hallA = HAL_CAPM_GetCrtEdge(&g_capm0) & 0x01;
    unsigned int hallB = HAL_CAPM_GetCrtEdge(&g_capm1) & 0x01;
    unsigned int hallC = HAL_CAPM_GetCrtEdge(&g_capm2) & 0x01;
#endif

#if defined (CHIP_3065HRPIRZ) || defined (CHIP_3065ARPIRZ)
    /* Get three hall values. */
    unsigned int hallA = HAL_CAPM_GetCrtEdge(&g_capm2) & 0x01;
    unsigned int hallB = HAL_CAPM_GetCrtEdge(&g_capm0) & 0x01;
    unsigned int hallC = HAL_CAPM_GetCrtEdge(&g_capm1) & 0x01;
#endif
    /* WVU-+  -->  H3H2H1-+ */
    unsigned int retValue = (hallC << 2) + (hallB << 1) + (hallA << 0);
    return retValue;
}
/**
  * @brief Init motor controller's data structure.
  * @retval None.
  */
static void InitSoftware(void)
{
    /* Read phase-uvw current */
    g_mc.readCurrUvwCb = ReadCurrUvw;
    g_mc.setPwmDutyCb = SetPwmDutyCp;
    g_mc.setADCTriggerTimeCb = SetADCTriggerTime;
    /* Callback function for obtaining the hall speed angle. */
    g_mc.getHallAngSpd = GetHallAngSpd;
    g_hall.getHallValue = GetHallValue;
    /* Initializing motor control param */
    TSK_Init();
}

/**
  * @brief Config the master APT.
  * @param aptx The master APT handle.
  * @retval None.
  */
static void AptMasterSet(APT_Handle *aptx)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    /* Config the master APT. */
    HAL_APT_MasterSyncInit(aptx, APT_SYNC_OUT_ON_CNTR_ZERO);
}

/**
  * @brief Config the slave APT.
  * @param aptx The slave APT handle.
  * @retval None.
  */
static void AptSalveSet(APT_Handle *aptx)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    APT_SlaveSyncIn slave;
    /* Config the slave APT. */
    slave.divPhase = 0;
    slave.cntPhase = 0;
    slave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    slave.syncInSrc = APT_SYNC_IN_SRC;
    slave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(aptx, &slave);
}
/**
  * @brief Configuring Master and Slave APTs.
  * @retval None.
  */
static void AptMasterSalveSet(void)
{
    /* motor fan APT master/slave synchronization */
    AptMasterSet(&g_apt0);
    AptSalveSet(&g_apt1);
    AptSalveSet(&g_apt2);
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
    GPIO_Handle *handle = (GPIO_Handle *)param;
    if (Key_StateRead(handle) == KEY_DOWN) {
        if (g_mc.motorStateFlag == 0) { /* start motor */
            g_mc.motorStateFlag = 1;
            SysCmdStartSet(&g_mc.statusReg);
        } else if (g_mc.motorStateFlag == 1) { /* stop motor */
            g_mc.motorStateFlag = 0;
            SysCmdStopSet(&g_mc.statusReg);
        }
    }
}

/**
  * @brief User application main entry function.
  * @retval BSP_OK.
  */
int MotorMainProcess(void)
{
    unsigned int tickNum1Ms = 2; /* 1ms tick */
    static unsigned int tickCnt1Ms = 0;
    unsigned int tickNum500Ms = 1000; /* 500ms tick */
    static unsigned int tickCnt500Ms = 0;
    SystemInit();
    HAL_TIMER_Start(&g_timer0);
    HAL_TIMER_Start(&g_timer1);

    AptMasterSalveSet();
    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_apt);
    /* Software initialization. */
    InitSoftware();

    /* Start the PWM clock. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);
    /* System Timer clock. */
    BASE_FUNC_DELAY_MS(ADC_READINIT_DELAY);
    TrimInitAdcShiftValue(&g_mc);
    BASE_FUNC_DELAY_MS(MOTOR_START_DELAY);
    while (1) {
        if (g_mc.msTickCnt - tickCnt1Ms >= tickNum1Ms) {
            tickCnt1Ms = g_mc.msTickCnt;
            /* User Code 1ms Event */
            /* User Code 1ms Event */
        }

        if (g_mc.msTickCnt - tickCnt500Ms >= tickNum500Ms) {
            if (SysIsError(&g_mc.statusReg) != true) {
               /* LED toggle in normal status. */
                HAL_GPIO_TogglePin(&LED1_HANDLE, LED1_PIN);
            }
            tickCnt500Ms = g_mc.msTickCnt;
        }
    }
    return 0;
}
