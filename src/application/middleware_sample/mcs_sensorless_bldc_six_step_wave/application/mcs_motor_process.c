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
  * @brief     This file provides motor application for ECBMCU201MPC board.
  * @details   BLDC six-step application based on the ECBMCU201MPC board
  *            1) Motor model is Gimbal GBM2804H-100T.
  *            2) Select the bldc six-step sensorless example in the sample column
                  of chipConfig and click Generate Code.
  *            3) It's power supply must be changed to 12V.
  */

#include "debug.h"
#include "mcs_assert.h"
#include "mcs_user_config.h"
#include "mcs_status.h"
#include "mcs_carrier.h"
#include "mcs_motor_process.h"

/*------------------------------- Macro Definition -----------------------------------------------*/
#define US_PER_MS               1000
#define APT_FULL_DUTY           1.0f

/* Motor control handle for bldc */
static MtrCtrlHandle g_mc;

static APT_RegStruct* g_aptCp[PHASE_MAX_NUM] = {BRIDGE_CTR_APT_U, BRIDGE_CTR_APT_V, BRIDGE_CTR_APT_W};

/**
  * @brief Initialzer of system tick.
  * @param mtrCtrl Motor control struct handle.
  * @retval None.
  */
static void TimerTickInit(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->sysTickCnt = 0;
    mtrCtrl->msTickNum = US_PER_MS / SYSTICK_PERIOD_US;
    mtrCtrl->capChargeTickNum = (INV_CAP_CHARGE_MS * US_PER_MS / SYSTICK_PERIOD_US);
}

/**
  * @brief APT Synchronize initialize.
  * @retval None.
  */
static void APT_SyncMasterInit(APT_Handle *aptHandle)
{
    HAL_APT_MasterSyncInit(aptHandle, APT_SYNC_OUT_ON_CNTR_ZERO);
}

/**
  * @brief Config the slave APT.
  * @param aptx The slave APT handle.
  * @retval None.
  */
static void APT_SyncSlaveInit(APT_Handle *aptHandle)
{
    APT_SlaveSyncIn aptSlave;
    aptSlave.divPhase = 0; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNC_IN_SRC; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(aptHandle, &aptSlave);
}

/**
  * @brief Configuring Master and Slave APTs.
  * @retval None.
  */
static void AptMasterSalveSet(void)
{
    /* Compressor fan APT master/slave synchronization */
    APT_SyncMasterInit(&g_apt0);
    APT_SyncSlaveInit(&g_apt1);
    APT_SyncSlaveInit(&g_apt2);
}

/**
  * @brief Read the ADC current sampling value of the compressor.
  * @param CurrUvw Three-phase current.
  * @retval None.
  */
static void ReadBemfUVWMotor(UVWBemf *bemfUVW)
{
    MCS_ASSERT_PARAM(bemfUVW != NULL);
    bemfUVW->u = (int)(HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM2)&0xFFF);
    bemfUVW->v = (int)(HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM5)&0xFFF);
    bemfUVW->w = (int)(HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM6)&0xFFF);
}

/**
  * @brief Init motor control task.
  * @retval None.
  */
static void TSK_InitMotor(void)
{
    /* Initialize target speed. */
    g_mc.spdCmdHz = SDP_TARGET_VALUE;
    /* zeroPoint = IN_VOLTAGE_BUS / 2.0; 4095/3.3 :ADC value corresponding to 1 V */
    g_mc.zeroPoint = ((((float)IN_VOLTAGE_BUS / 2.0) * VOL_DIVIDER_COEFFICIENT) * 4095 / 3.3);
    g_mc.pwmDuty = FORCE_DRAG_MINDUTY;

    /* Sets the number of sample filtering times for zero-crossing sampling. */
    g_mc.sysVar.bemfFilterCnt = FILTER_COUNT;
    g_mc.sysVar.dragChangePhaseTime = DRAG_START_INTERVAL;

    g_mc.stateMachine = FSM_IDLE;
    g_mc.aptMaxcntCmp = g_apt0.waveform.timerPeriod;

    /* Speed management initialization. */
    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE); /* Init speed slope */
    TimerTickInit(&g_mc);

    /* Pid control parameter initialization. */
    g_mc.spdPi.kp = SPD_PID_KP;
    g_mc.spdPi.ki = SPD_PID_KI;
    g_mc.spdPi.ts = SPD_PID_TS;
    g_mc.spdPi.upperLimit = APT_DUTYLIMIT_MAX;
    g_mc.spdPi.lowerLimit = APT_DUTYLIMIT_MIN;

    /* BLDC six-step control initialization. */
    g_mc.stepCtrl.phaseStep = STEP1;
    g_mc.stepCtrl.controlApt.u = &g_apt0;
    g_mc.stepCtrl.controlApt.v = &g_apt1;
    g_mc.stepCtrl.controlApt.w = &g_apt2;
}


/**
  * @brief Software Initialization.
  * @retval None.
  */
static void InitSoftware(void)
{
    TSK_InitMotor();

    g_mc.readBemfUVW = ReadBemfUVWMotor;
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
        APT_RegStruct *aptx = aptAddr[i];
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_SET;
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_SET;
        DCL_APT_ForcePWMOutputLow(aptx);
    }
}

/**
  * @brief Force rotor alignment to home position.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ForceAlign(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->pwmDuty = FORCE_DRAG_MINDUTY;
    mtrCtrl->sysVar.dragChangePhaseTime = DRAG_START_INTERVAL;
    MCS_SetCtrAptDuty(mtrCtrl, mtrCtrl->pwmDuty);
    MotorPwmOutputEnable(g_aptCp);
    mtrCtrl->stepCtrl.phaseStep = STEP6;
    SixStepPwm(&mtrCtrl->stepCtrl);
    BASE_FUNC_DELAY_MS(100); /* Delay 100 ms waiting for rotor alignment. */
    mtrCtrl->stepCtrl.phaseStep = STEP1;
    SixStepPwm(&mtrCtrl->stepCtrl);
    BASE_FUNC_DELAY_MS(100); /* Delay 100 ms waiting for rotor alignment. */
    mtrCtrl->sysVar.lastZeroPoint = DCL_SYSTICK_GetTick();
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
  * @brief Open the three-phase lower pipe.
  * @param aptAddr Three-phase APT address pointer.
  * @param maxDutyCnt Max duty count.
  * @retval None.
  */
static void AptTurnOnLowSidePwm(APT_RegStruct **aptAddr, unsigned int maxDutyCnt)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    MCS_ASSERT_PARAM(maxDutyCnt != 0);
    unsigned int dutyCnt;
    dutyCnt = maxDutyCnt * APT_FULL_DUTY;
    /* Open the three-phase lower pipe */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = aptAddr[i];
        aptx->TC_REFC.BIT.rg_cnt_refc = dutyCnt;
        aptx->TC_REFD.BIT.rg_cnt_refd = dutyCnt;
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
static void CheckSysCmdStart(MtrCtrlHandle *mtrCtrl, APT_RegStruct **aptAddr, SysStatusReg *statusReg,
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
  * @brief Clear historical values of all controller before start-up.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ClearBeforeStartup(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->aptMaxcntCmp = g_apt0.waveform.timerPeriod;

    /* Clear bemf parameters. */
    mtrCtrl->bemf.u = 0;
    mtrCtrl->bemf.v = 0;
    mtrCtrl->bemf.w = 0;

    /* Clear tickcnt. */
    mtrCtrl->msTickCnt = 0;

    mtrCtrl->pwmDuty = FORCE_DRAG_MINDUTY;

    /* Clear spd parameters. */
    mtrCtrl->spdEstHz = 0;
    mtrCtrl->spdRefHz = 0;

    mtrCtrl->stepCtrl.phaseStep = STEP1;

    mtrCtrl->sysTickCnt = 0;

    mtrCtrl->sysVar.dragChangePhaseTime = DRAG_START_INTERVAL;
    mtrCtrl->sysVar.changePhaseFlag = 0;
    mtrCtrl->sysVar.firstEventFilterFlag = 0;
    mtrCtrl->sysVar.stepTimeNum = 0;
    mtrCtrl->sysVar.stepTimeFilterEnable = 0;
    for (int i = 0; i < STEP_MAX_NUM; i++) {
        mtrCtrl->sysVar.stepTime[i] = 0;
    }
    /* RMG CLEAR */
    RMG_Clear(&mtrCtrl->spdRmg); /* Clear the history value of speed slope control */
    /* SPDCTRL CLEAR */
    PID_Clear(&mtrCtrl->spdPi);
}

/**
  * @brief Check bootstrap capacitor charge time.
  * @param mtrCtrl The motor control handle.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckBootstrpCapChargeTime(MtrCtrlHandle *mtrCtrl, FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    mtrCtrl->sysTickCnt++;
    /* check bootstrap capacitor charge time */
    if (mtrCtrl->sysTickCnt == mtrCtrl->capChargeTickNum) {
        /* Update Status. */
        *stateMachine = FSM_CLEAR;
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
  * @brief System timer tick task.
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void TSK_SystickIsr(MtrCtrlHandle *mtrCtrl, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    SysStatusReg *statusReg = &mtrCtrl->statusReg;
    volatile FsmState *stateMachine = &mtrCtrl->stateMachine;
    mtrCtrl->msTickCnt++;
    /* Pre-processing of motor status. */
    MotorStatePerProc(statusReg, stateMachine);
    /* statemachine */
    switch (*stateMachine) {
        case FSM_IDLE:
            CheckSysCmdStart(mtrCtrl, aptAddr, statusReg, (FsmState *)stateMachine);
            break;
        case FSM_CAP_CHARGE:
            /* Bootstrap Capacitor Charging Timing */
            CheckBootstrpCapChargeTime(mtrCtrl, (FsmState *)stateMachine);
            break;
        case FSM_CLEAR:
            /* Clearing control parameters. */
            ClearBeforeStartup(mtrCtrl);
            /* Rotor alignment. */
            ForceAlign(mtrCtrl);
            *stateMachine = FSM_STARTUP;
            break;
        case FSM_STARTUP:
            /* Forced drag. */
            break;
        case FSM_RUN:
            /* Speed ramp control */
            mtrCtrl->spdRefHz = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmdHz);
            mtrCtrl->spdPi.error = mtrCtrl->spdRefHz - mtrCtrl->spdEstHz;
            /* Speed loop control */
            mtrCtrl->pwmDuty = PI_Exec(&mtrCtrl->spdPi);
            break;
        case FSM_STOP:
            MotorPwmOutputDisable(aptAddr);
            SysRunningClr(statusReg);
            *stateMachine = FSM_IDLE;
            break;
        case FSM_FAULT:
            /* Overcurrent state */
            CheckOverCurrentState(statusReg, (FsmState *)stateMachine);
            break;
        default:
            break;
    }
}

/**
  * @brief Overcurrent protection.
  * @param aptHandle The apt Handle.
  * @retval None.
  */
void MotorSysErrCallback(void *aptHandle)
{
    /* Overcurrent protection callback function. */
    BASE_FUNC_UNUSED(aptHandle);
    MotorPwmOutputDisable(g_aptCp);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_mc.statusReg);
    DBG_PRINTF("APT error! \r\n");
    HAL_GPIO_SetValue(&g_gpio2, GPIO_PIN_3, GPIO_LOW_LEVEL);
}

/**
  * @brief Motor blockage protection function.
  * @retval None.
  */
static void MotorBlockageProtect(void)
{
    unsigned int currentTick = DCL_SYSTICK_GetTick(); /* Get the current tick value. */
    unsigned int intervalTick = (currentTick >= g_mc.sysVar.lastZeroPoint)
                              ? currentTick - g_mc.sysVar.lastZeroPoint
                              : SYSTICK_MAX_VALUE - g_mc.sysVar.lastZeroPoint + currentTick + 1;
    if (intervalTick > SYSTICK_GetCRGHZ()) {
        MotorPwmOutputDisable(g_aptCp);
        SysErrorSet(&g_mc.statusReg);
        g_mc.spdEstHz = 0;
        return;
    }
}

/**
  * @brief Carrier Interruption.
  * @param aptHandle The apt Handle.
  * @retval None.
  */
void MotorCarrierProcessCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_TIMER_INTERRUPT */
    MCS_CarrierProcess(&g_mc);
    if (g_mc.stateMachine == FSM_RUN) {
        MotorBlockageProtect();
    }
    /* USER CODE END APT0_TIMER_INTERRUPT */
}

/**
  * @brief Change phase delay callback function.
  * @param handle The TIMER1 Handle.
  * @retval None.
  */
void MotorStatemachineCallBack(void *handle)
{
    /* TIMER1CallbackFunction */
    BASE_FUNC_UNUSED(handle);
    TSK_SystickIsr(&g_mc, g_aptCp);
}

/**
  * @brief Check Potentiometer Value and adjust speed.
  * @param None.
  * @retval None.
  */
static void AdjustSpeedFunction(void)
{
    static unsigned int potentiomitorAdcValue = 0;
    static float spdCmdHz = 0;
    static float spdCmdHzLast = SDP_MAX_VALUE;
    HAL_ADC_SoftTrigSample(&g_adc0, ADC_SOC_NUM9); /* Get the speed adjustment resistance. */
    potentiomitorAdcValue = HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM9);
    /* 4045.0 is adc sample max value of potentiomitor */
    spdCmdHz = (float)potentiomitorAdcValue / 4045.0 * SDP_MAX_VALUE;
    if (spdCmdHz < SDP_MIN_VALUE) { /* Speed protection. */
        spdCmdHz = SDP_MIN_VALUE;
    }
    if (spdCmdHz > SDP_MAX_VALUE) {
        spdCmdHz = SDP_MAX_VALUE;
    }
    float delta = spdCmdHzLast > spdCmdHz ? (spdCmdHzLast - spdCmdHz) : (spdCmdHz - spdCmdHzLast);
    /* 1.0 : If the speed fluctuation is less than 1.0Hz, no change is made. */
    if (delta < 1.0) {
        return;
    }
    spdCmdHzLast = spdCmdHz;
    g_mc.spdCmdHz = spdCmdHz;
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
        if (SysIsRunning(&g_mc.statusReg)) { /* stop apt output, motor is off status */
            SysCmdStopSet(&g_mc.statusReg);
        } else { /* start apt output, motor is on status */
            HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);
            SysCmdStartSet(&g_mc.statusReg);
        }
    }
}

/**
  * @brief User application entry.
  * @retval BSP_OK.
  */
int MotorMainProcess(void)
{
    SystemInit();
    /* System Initialization. */
    unsigned int tickNum100Ms = 150; /* 100ms tick */
    static unsigned int tickCnt100Ms = 0;
    unsigned int tickNum500Ms = 750; /* 500ms tick */
    static unsigned int tickCnt500Ms = 0;

    HAL_TIMER_Start(&g_timer1);
    AptMasterSalveSet();
    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_aptCp);
    InitSoftware();
    /* Start the PWM clock. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);

    while (true) {
        if (g_mc.msTickCnt - tickCnt100Ms >= tickNum100Ms) {
            tickCnt100Ms = g_mc.msTickCnt;
            /* User Code 100ms Event */
            AdjustSpeedFunction();
        }
        if (g_mc.msTickCnt - tickCnt500Ms >= tickNum500Ms) {
            if (SysIsError(&g_mc.statusReg) == true) {
                break;
            }
            /* The LED blinks when no status is not error. */
            HAL_GPIO_TogglePin(&g_gpio2, GPIO_PIN_3);
            tickCnt500Ms = g_mc.msTickCnt;
        }
    }
    return 0;
}