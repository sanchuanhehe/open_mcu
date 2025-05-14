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
  * @details   Single FOC application based on the ECMCU105H/ECBMCU201MPC board
  *            1) Motor model is 42JSF630AS-1000.
  *            2) Select the pmsm encode qdm 2shunt foc example in the sample column
                  of chipConfig and click Generate Code.
  *            3) It's power supply must be changed to 24V.
  */
#include "main.h"
#include "mcs_user_config.h"
#include "mcs_math.h"
#include "hmi_module.h"
#include "mcs_ctlmode_config.h"
#include "mcs_prot_user.h"
#include "mcs_prot_user_config.h"
#include "mcs_math_const.h"
#include "mcs_motor_process.h"
#include "mcs_chip_config.h"
#include "mcs_inc_enc.h"
#include <math.h>
#include "debug.h"


/*------------------------------- Macro Definition -----------------------------------------------*/
#define US_PER_MS               1000
#define ANGLE_RANGE_ABS         65536
#define ANGLE_360_F             65536.0f /* 0 - 65536 indicates 0 to 360. */
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
#define CNT_10                  10
#define CNT_5000                5000
#define LEVEL_4                 4
#define MOTOR_START_DELAY       2
#define ADC_READINIT_DELAY      1
#define ADC_READINIT_TIMES      20
#define ADC_TRIMVALUE_MIN       1800.0f
#define ADC_TRIMVALUE_MAX       2200.0f
#define IRQ_QDM0_PRIORITY 7  /* the QDM encoder IRQ priority, highest */
#define ENC_TIMES_NUM           5
/*------------------------------- Param Definition -----------------------------------------------*/
/* Motor parameters. */
/* Np, Rs, Ld, Lq, Psif, J, Nmax, Currmax, PPMR, zShift */
static MOTOR_Param g_motorParam = MOTORPARAM_DEFAULTS;
static APT_RegStruct* g_apt[PHASE_MAX_NUM] = {APT_U, APT_V, APT_W};

extern __UINT16_TYPE__ last_raw_Angle;
/* QDM */
typedef struct {
    MCS_QdmHandle qdm;
} PeriphHandle;
static PeriphHandle g_periph = {.qdm = {.qdmAddr = QDMBASEADDR,
                                        .zPulsesNvic = {.irqNum = QDMIRQNUM,
                                                        .baseAddr = QDMBASEADDR}}};
/* Motor control handle */
static MTRCTRL_Handle g_mc = {0};
/* QDM control handle */
static EncoderHandle g_enc = {0};




/**
  * @brief 获取编码器的角度和速度。
  * @param speed 速度指针。
  * @param angle 角度指针。
  * @retval 无。
  */

static void GetEncAngSpd(float* speed, float* angle)
{
    MCS_GetEncoderCnt(&g_enc, QDMNUM);// 获取编码器计数值
      // /* 计算电机电角度 -π ~ π */
    MCS_GetElecAngleByEnc(&g_enc);  // 根据编码器数据计算电机电角度
   // /* 计算电机速度 */
    MCS_GetElecSpeedByEnc(&g_enc);  // 根据编码器数据计算电机速度
    *speed = g_enc.elecSpeed;  // 将计算得到的电机速度赋值给传入的速度指针
    *angle = g_enc.elecAngle;  // 将计算得到的电机电角度赋值给传入的角度指针

 


}


static void ISR_QdmzPulses(void* args)
{
    NvicHandle* qdmHandle = (NvicHandle*)args;  // 将传入的参数转换为NvicHandle指针
    /* 清除QDM中断标志 */
    DCL_QDM_ClearInterrupt((QDM_RegStruct*)qdmHandle->baseAddr, QDM_INT_INDEX_EVNT_LATCH); // 清除QDM事件捕获中断标志
    IRQ_ClearN(qdmHandle->irqNum);  // 清除中断请求

    g_enc.pulZCnt = MCS_GetQdmPosCnt(qdmHandle->baseAddr);
    g_mc.motorSpinPos++;  // 增加电机旋转位置计数
    if (g_mc.encReady == 0) {
        /* Z脉冲标志用于确定在初始启动期间IF预设位是否成功使用。 */
        g_mc.encReady = 1; // 如果编码器尚未准备好，设置编码器准备好标志
    }

}



/* Motor POS loop PID param. */
static void POSCTRL_InitWrapper(POSCTRL_Handle *posHandle, float ts)
{
    /* Position loop param assignment. */
    PID_Param posPi = {
        .kp = POS_KP,
        .ki = POS_KI,
        .kd = POS_KD,
        .ns = POS_NS,
        .lowerLim = POS_LOWERLIM,
        .upperLim = POS_UPPERLIM,
    };
    /* Position loop param init. */
    POSCTRL_Init(posHandle, &posPi, ts);
}

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
  * @brief 系统标记的初始化器。
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
    g_mc.currCtrlPeriod = CTRL_CURR_PERIOD; /* 初始化电流控制器 */
    g_mc.aptMaxcntCmp = g_apt0.waveform.timerPeriod;
    g_mc.sampleMode = DUAL_RESISTORS;
    g_mc.obserType = FOC_OBSERVERTYPE_ENC;      /* 启动foc观察模式*/
    g_mc.controlMode = FOC_CONTROLMODE_POS;     /* 启动电机控制模式 */
    g_mc.adcCurrCofe = ADC_CURR_COFFI;
    g_mc.spdAdjustMode = CUST_SPEED_ADJUST;
    g_mc.uartConnectFlag = DISCONNECT;
    g_mc.spdCmdHz = 35.0f;  /* Motor initialization speed */

    g_mc.adc0Compensate = ADC0COMPENSATE;  /* u相电流初始化adc移位微调值 */
    g_mc.adc1Compensate = ADC1COMPENSATE;  /* w相电流初始化adc移位微调值*/

    IF_Init(&g_mc.ifCtrl, CTRL_IF_CURR_AMP_A, USER_CURR_SLOPE, CTRL_SYSTICK_PERIOD, CTRL_CURR_PERIOD);//IF初始化电流
    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE); /* 初始速度斜率*/
    MtrParamInit(&g_mc.mtrParam, g_motorParam); //电机参数初始化

    TimerTickInit(&g_mc);
    SVPWM_Init(&g_mc.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);
    R1SVPWM_Init(&g_mc.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);
    //电机速度回路PI参数。
    SPDCTRL_InitWrapper(&g_mc.spdCtrl, CTRL_SYSTICK_PERIOD);
    //电机位置回路PI参数。
    POSCTRL_InitWrapper(&g_mc.posCtrl, CTRL_SYSTICK_PERIOD * 5.0f); /* Position loop control period */
    //电机电流回路PI参数。
    CURRCTRL_InitWrapper(&g_mc.currCtrl, &g_mc.idqRef, &g_mc.idqFbk, CTRL_CURR_PERIOD);

    MotorProt_Init(&g_mc.prot); /* 初始化保护状态通用 */
    OCP_Init(&g_mc.prot.ocp, CTRL_CURR_PERIOD); // 启动过电流保护功能。
    OVP_Init(&g_mc.prot.ovp, CTRL_SYSTICK_PERIOD); //启动直流链路电压保护功能。
    LVP_Init(&g_mc.prot.lvp, CTRL_SYSTICK_PERIOD); //启动直流链路电压保护功能。
    OTP_Init(&g_mc.prot.otp, CTRL_SYSTICK_PERIOD); //初始化超温保护功能。
    STP_Init(&g_mc.prot.stall, CTRL_SYSTICK_PERIOD, PROT_STALLING_CURR_AMP_LIMIT,
        PROT_STALLING_SPD_LIMIT, PROT_STALLING_TIME_LIMIT);//启动电机失速保护功能。
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
    mtrCtrl->axisAngle = 0.0f;
    mtrCtrl->spdRefHz = 0.0f;
    mtrCtrl->motorSpinPos = 0;
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
    POSCTRL_Clear(&mtrCtrl->posCtrl);
    MCS_EncoderClear(&g_enc);

    OTP_Clear(&mtrCtrl->prot.otp);
    OCP_Clear(&mtrCtrl->prot.ocp);
    OVP_Clear(&mtrCtrl->prot.ovp);
    LVP_Clear(&mtrCtrl->prot.lvp);
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
  * @brief 构造一个新的mcs启动开关对象。
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_StartupSwitch(MTRCTRL_Handle *mtrCtrl)
{
    // 确保传递的电机控制句柄不是NULL
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
     // 获取启动阶段处理结构体的指针
    STARTUP_Handle *startup = &mtrCtrl->startup;
     // 获取dq轴参考值结构体的指针
    DqAxis *idqRef = &mtrCtrl->idqRef;
     // 获取目标电流幅值
    float iftargetAmp = mtrCtrl->ifCtrl.targetAmp;
    // 获取速度参考值（Hz）
    float spdRefHz = mtrCtrl->spdRefHz;
   // 根据当前启动阶段执行相应操作
    switch (startup->stage) {
        case STARTUP_STAGE_CURR:
            if (mtrCtrl->ifCtrl.curAmp >= iftargetAmp) {  //IF采样电流大于IF目标电流时。
                /* 阶段转换 */
                // 设置q轴参考电流为目标电流值
                idqRef->q = iftargetAmp;
                // 更改启动阶段至速度阶段
                startup->stage = STARTUP_STAGE_SPD;
                mtrCtrl->encReady = 0;  /* 启动阶段清除Z信号干扰错误 */

            } else {
                 /* 增加电流幅值 */
                // 计算并设置新的q轴参考电流
                idqRef->q = IF_CurrAmpCalc(&mtrCtrl->ifCtrl);
                 // 在此阶段保持速度参考为0
                spdRefHz = 0.0f;
            }
            break;

        case STARTUP_STAGE_SPD:  // 当前处于速度启动阶段
            /* 提高电流频率 */
            // 判断电机是否已经旋转了足够圈数
            if (mtrCtrl->motorSpinPos > 3) {  /* 3是If模式下的电机转数 */
                 /* 阶段转换 */
                // 转换状态机至运行状态
                mtrCtrl->stateMachine = FSM_RUN;
            } else {
                 /* 速度斜坡生成 */
                // 执行速度斜坡生成函数，并设定一个固定的拖动速度
                spdRefHz = RMG_Exec(&mtrCtrl->spdRmg, 5.0f); /*5.0f为If模式力拖动速度*/
            }
            break;

        default:
         // 默认情况，不执行任何操作
            break;
    }
    // 更新电机控制结构中的速度参考值
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
        /* 为自举电容器充电做准备。 */
        AptTurnOnLowSidePwm(aptAddr, mtrCtrl->aptMaxcntCmp);
        /* Out put pwm */
        MotorPwmOutputEnable(aptAddr);
    }
}

/*****************************************
  * @brief 系统计时器勾选任务。
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  * 速度环
  * 
  * 
  ****************************************/
static void TSK_SystickIsr(MTRCTRL_Handle *mtrCtrl, APT_RegStruct **aptAddr)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    SysStatusReg *statusReg = &mtrCtrl->statusReg;
    FsmState *stateMachine = &mtrCtrl->stateMachine;
    mtrCtrl->msTickCnt++;
    /* 电机状态的预处理。 */
    MotorStatePerProc(statusReg, stateMachine);
    /* statemachine */
    switch (*stateMachine) {
        case FSM_IDLE:
            /*在电机启动前设置smo估计速度*/
            CheckSysCmdStart(mtrCtrl, aptAddr, statusReg, stateMachine);
            break;
        case FSM_CAP_CHARGE:
            /* Bootstrap电容充电定时 */
            CheckBootstrpCapChargeTime(mtrCtrl, stateMachine);
            break;
            /* 启动前清除参数 */
        case FSM_CLEAR:
            ClearBeforeStartup(mtrCtrl);
            *stateMachine = FSM_STARTUP;
            break;
        case FSM_STARTUP:
        //    构造一个新的mcs启动开关对象。
            MCS_StartupSwitch(mtrCtrl);
            break;
        case FSM_RUN:
            if (mtrCtrl->controlMode == FOC_CONTROLMODE_SPEED) { /* Speed control mode */
                 /* 速度斜坡控制 */
                mtrCtrl->spdRefHz = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmdHz);
            } else if (mtrCtrl->controlMode == FOC_CONTROLMODE_POS) { /* Position control mode */
                mtrCtrl->sysTickCnt++;
                POSCTRL_SetSlope(&mtrCtrl->posCtrl, mtrCtrl->spdCmdHz);//设置位置斜坡
                /* 200.0是目标位置，用户可以重新定义 */
                //  POSCTRL_SetTarget(&mtrCtrl->posCtrl, 0.5 *DOUBLE_PI * g_motorParam.mtrNp);//定位环目标位置设置。

                float posFbk = POSCTRL_AngleExpand(&mtrCtrl->posCtrl, mtrCtrl->axisAngle);

                if (mtrCtrl->sysTickCnt % 5 == 0) { /* 5是位置环划分系数。*/
                    mtrCtrl->spdRefHz = POSCTRL_Exec(&mtrCtrl->posCtrl, mtrCtrl->posCtrl.posTarget, posFbk);
                }
            }
            /* 速度环路控制 */
            mtrCtrl->idqRef.q = SPDCTRL_Exec(&mtrCtrl->spdCtrl, mtrCtrl->spdRefHz, mtrCtrl->encSpeed);//用速度偏差值，求出所需Q轴电流
           
            break;
        case FSM_STOP:
            mtrCtrl->spdRefHz = 0.0f;
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
    BASE_FUNC_DELAY_US(CNT_10); /* Delay 10 us. */
    /* Force convert to float type. */
    float resisAdcValue = (float)HAL_ADC_GetConvResult(&ADCRESIS_HANDLE, ADCRESISSOCNUM);
    /* 10 / (x + 10) * 3.3 = resisAdcValue / 4096 * 3.3, x is resisValue, 10kohm is resistor divider value. */
    float resisValue = (4096.0f * 10.0f - 10.0f * resisAdcValue) / resisAdcValue;
    g_mc.powerBoardTemp = TempTable(resisValue);
    g_mc.udc = ((float)HAL_ADC_GetConvResult(&ADCUDC_HANDLE, ADCUDCSOCNUM)) * ADC_UDC_COFFI;
}

/**
  * @brief 执行异常反馈速度保护动作。
  * @retval None.
  */
static void SpdFbkErrorProt_Exec(void)
{
    if (g_mc.prot.motorErrStatus.Bit.motorStalling == 0 &&
        g_mc.prot.motorErrStatus.Bit.overVoltErr == 0 &&
        g_mc.prot.motorErrStatus.Bit.lowerVoltErr == 0 &&
        g_mc.prot.motorErrStatus.Bit.overIpmTempErr == 0 &&
        g_mc.prot.motorErrStatus.Bit.overCurrErr == 0) {
        g_mc.prot.motorErrStatus.Bit.revRotErr = 1;
        /* If revRotErr, execute protect motion. */
        ProtSpo_Exec(g_apt);
    }
}
/**
  * @brief Check abnormal feedback speed.
  * @retval None.
  */
static void CheckSpdFbkStatus(void)
{
    static short errorSpdStatus = 0;
    static short errorDeltaSpdStatus = 0;
    /* Detect the nan value. */
    if (isnan(g_mc.encSpeed) || isnan(g_mc.idqRef.q)) {
        errorSpdStatus++;
    } else {
        errorSpdStatus = 0;
    }
    if (g_mc.stateMachine == FSM_RUN) {
        /* 检测反馈速度异常。定义10为速度误差值，0.5为当前误差值*/
        if (Abs(g_mc.spdRefHz - g_mc.encSpeed) >= CNT_10 && g_mc.idqRef.q <= 0.5f) {
            errorDeltaSpdStatus++;
        }
    }
    /*如果计数超过5000次，则执行保护动作. */
    if (errorSpdStatus >= CNT_5000 || errorDeltaSpdStatus >= CNT_5000) {
        errorSpdStatus = 0;
        errorDeltaSpdStatus = 0;
        SpdFbkErrorProt_Exec();
    }
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
    static float spdCmdHzLast = 35.0f;  /* 35.0 is spdCmdHzLast init value */
    HAL_ADC_SoftTrigSample(&ADCPTT_HANDLE, ADCPTTSOCNUM);
    BASE_FUNC_DELAY_US(10); /* Delay 10 us. */
    potentiomitorAdcValue = (float)HAL_ADC_GetConvResult(&ADCPTT_HANDLE, ADCPTTSOCNUM);
    /* 4045.0 is adc sample max value of potentiomitor, convert max spd to 200.0Hz */
    spdCmdHz = potentiomitorAdcValue / 4045.0f * USER_MAX_SPD_HZ;//电位器控制速度
    // spdCmdHz=uart_spdcmd;//串口控制速度
  
    POSCTRL_SetTarget(&g_mc.posCtrl,uart_poscmd * g_motorParam.mtrNp);//串口控制位置转动
    
    // POSCTRL_SetTarget(&g_mc.posCtrl,potentiomitorAdcValue / 4045.0f * 6.28 * g_motorParam.mtrNp);//电位器控制电机位置转动
   

    if (Abs(spdCmdHzLast - spdCmdHz) < 1.0f) { /* Ignore changes less than 1. */
        return;
    }
    spdCmdHzLast = spdCmdHz;
    if (spdCmdHz < USER_MIN_SPD_HZ) {  /* USER_MIN_SPD_HZ is spdCmdHz lower limit */
        spdCmdHz = USER_MIN_SPD_HZ;    /* USER_MIN_SPD_HZ is spdCmdHz lower limit */
    }
    if (spdCmdHz > g_mc.mtrParam.maxElecSpd) { /* spdCmdHz upper limit */
        spdCmdHz = g_mc.mtrParam.maxElecSpd;   /* spdCmdHz upper limit */
    }
    if (g_mc.spdAdjustMode == CUST_SPEED_ADJUST) {
        g_mc.spdCmdHz = spdCmdHz;
    }
}

/**
  * @brief 用于电机状态机回叫功能的系统定时器ISR。
  * @param param The systick timer handle.
  * @retval None.
  * 
  */
void MotorStatemachineCallBack(void *param)
{
    /* Verify Parameters */
    MCS_ASSERT_PARAM(param != NULL);
    BASE_FUNC_UNUSED(param);
    /* 读取电源板温度和电压。*/
    ReadBoardTempAndUdc();
    /* 电机错误状态检查. */
    TSK_SystickIsr(&g_mc, g_apt);
    if (g_mc.prot.motorErrStatus.all == 0) {
        /* 电机错误状态检查 */
        CheckSpdFbkStatus();
        /* 电机失速检测。 */
        STP_Det_ByCurrSpd(&g_mc.prot.stall, &g_mc.prot.motorErrStatus, g_mc.encSpeed, g_mc.idqFbk);
        STP_Exec(&g_mc.prot.motorErrStatus, g_apt);
    }
    /* 电机过电压检测。 */
    OVP_Det(&g_mc.prot.ovp, &g_mc.prot.motorErrStatus, g_mc.udc);
    OVP_Exec(&g_mc.prot.ovp, &g_mc.spdRefHz, g_apt);
    OVP_Recy(&g_mc.prot.ovp, &g_mc.prot.motorErrStatus, g_mc.udc);
    /* 电机低电压检测. */
    LVP_Det(&g_mc.prot.lvp, &g_mc.prot.motorErrStatus, g_mc.udc);
    LVP_Exec(&g_mc.prot.lvp, &g_mc.spdRefHz, g_apt);
    LVP_Recy(&g_mc.prot.lvp, &g_mc.prot.motorErrStatus, g_mc.udc);
    /* 电源板超温检测. */
    OTP_Det(&g_mc.prot.otp,  &g_mc.prot.motorErrStatus, OTP_IPM_ERR_BIT, g_mc.powerBoardTemp);
    OTP_Exec(&g_mc.prot.otp, &g_mc.spdRefHz, g_apt);
    OTP_Recy(&g_mc.prot.otp, &g_mc.prot.motorErrStatus, OTP_IPM_ERR_BIT, g_mc.powerBoardTemp);

    /* 如果保护级别==4，则将电机状态设置为停止。 */
    if (g_mc.prot.ovp.protLevel == LEVEL_4 || g_mc.prot.lvp.protLevel == LEVEL_4 \
        || g_mc.prot.otp.protLevel == LEVEL_4) {
        SysCmdStopSet(&g_mc.statusReg);
    }
}

/**
  * @brief The carrier ISR wrapper function.
  * @param aptHandle The APT handle.
  * @retval None.
  * 
  * 载波中断函数，
  */
void MotorCarrierProcessCallback(void *aptHandle)
{
    MCS_ASSERT_PARAM(aptHandle != NULL);
    BASE_FUNC_UNUSED(aptHandle);
    /* the carrierprocess of motor */
    MCS_CarrierProcess(&g_mc);
    /* Over current protect */
    if (g_mc.stateMachine == FSM_RUN || g_mc.stateMachine == FSM_STARTUP) {
        OCP_Det(&g_mc.prot.ocp, &g_mc.prot.motorErrStatus, g_mc.idqFbk);
        OCP_Exec(&g_mc.prot.ocp, &g_mc.idqFbk, g_apt);                       /* Execute over current protect motion */
        if (g_mc.prot.ocp.protLevel < LEVEL_4) {
            OCP_Recy(&g_mc.prot.ocp, &g_mc.prot.motorErrStatus);
        }
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
  * @brief 初始化电机控制器的数据结构。
  * @retval None.
  */
static void InitSoftware(void)
{
    /* 正在初始化电机控制参数 */
    TSK_Init();
    /* Read phase-uvw current */
    g_mc.readCurrUvwCb = ReadCurrUvw;
    g_mc.setPwmDutyCb = SetPwmDutyCp;
    g_mc.setADCTriggerTimeCb = SetADCTriggerTime;
    g_mc.encHandle = &g_enc;

    MCS_EncInitStru encMotorParam;
    EncoderHandle* enc = &g_enc;
    encMotorParam.mtrNp = g_motorParam.mtrNp;
    encMotorParam.mtrPPMR = g_motorParam.mtrPPMR;
    encMotorParam.zShift = g_motorParam.zShift;
    encMotorParam.ctrlPeriod = CTRL_CURR_PERIOD;
    encMotorParam.timeNum = ENC_TIMES_NUM;
    MCS_EncoderInit(enc, &encMotorParam); /* encoder Initializing Parameter Configurations. */

    /* MCU peripheral configuration function used for initial motor control. */
    g_mc.getEncAngSpd = GetEncAngSpd;  /* Callback function for obtaining the encoder speed angle. */
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
  * @brief 用户应用程序的主入口函数。
  * @retval BSP_OK 返回状态。
  */
int MotorMainProcess(void)
{
    unsigned int tickNum1Ms = 2; /* 定义1毫秒的计时器滴答数 */
    static unsigned int tickCnt1Ms = 0; /* 静态变量，用于记录1毫秒滴答计数 */
    unsigned int tickNum500Ms = 1000; /* 定义500毫秒的计时器滴答数 */
    static unsigned int tickCnt500Ms = 0; /* 静态变量，用于记录500毫秒滴答计数 */
    
    SystemInit(); /* 系统初始化 */
    HMI_Init(); /* 初始化串口中断，用于人机交互接口 */
    HAL_TIMER_Start(&g_timer0); /* 启动定时器0 */
    HAL_TIMER_Start(&g_timer1); /* 启动定时器1 */
   

    AptMasterSalveSet(); /* 设置主从设备 */
    /* 在启动前禁用PWM输出。 */
    MotorPwmOutputDisable(g_apt); /* 禁用PWM输出 */
    /* 软件初始化。 */
    InitSoftware(); /* 初始化软件相关设置  电机参数初始化*/

     /* QDM使用算法驱动的速度和角度采集 */
    static MCS_QdmInitStru qdmInit; /* QDM初始化结构体 */
    qdmInit.qdmAddr = g_periph.qdm.qdmAddr; /* 设置QDM地址 */

    qdmInit.zPlusesIrqFunc = ISR_QdmzPulses; /* 设置QDM零脉冲中断服务函数 */

    qdmInit.zPlusesNvic = &g_periph.qdm.zPulsesNvic; /* 设置QDM零脉冲中断控制器 */
    qdmInit.zPlusesIrqPrio = IRQ_QDM0_PRIORITY; /* 设置QDM零脉冲中断优先级 */
    MCS_QdmInit(&qdmInit); /* 初始化QDM，必须在使能载波中断之前执行。 */

    /* 启动PWM时钟。 */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2); /* 启动APT模块 */
    /* 系统定时器时钟延迟。 */
    BASE_FUNC_DELAY_MS(ADC_READINIT_DELAY); /* 延迟，等待ADC初始化 */
    TrimInitAdcShiftValue(&g_mc); /* 初始化ADC移位值 */
    BASE_FUNC_DELAY_MS(MOTOR_START_DELAY); /* 延迟，等待电机启动 */
    while (1) {

       /* 循环发送数据到主机 */
        HMI_Process_Tx(&g_mc); /* 处理人机交互接口的发送数据 */
        if (g_mc.msTickCnt - tickCnt1Ms >= tickNum1Ms) { /* 检查是否达到1毫秒的计时 */
            tickCnt1Ms = g_mc.msTickCnt; /* 更新1毫秒滴答计数 */
            /* 用户代码1毫秒事件 */
            HMI_Process_Rx(&g_mc); /* 处理人机交互接口的接收数据 */
            /* 用户代码1毫秒事件 */
        }

        if (g_mc.msTickCnt - tickCnt500Ms >= tickNum500Ms) { /* 检查是否达到500毫秒的计时 */
            if (SysIsError(&g_mc.statusReg) != true) { /* 检查系统是否出现错误 */
               /* 如果系统状态正常，则切换LED灯的状态。 */
                HAL_GPIO_TogglePin(&LED1_HANDLE, LED1_PIN); /* 切换LED灯的状态 */
            }
            tickCnt500Ms = g_mc.msTickCnt; /* 更新500毫秒滴答计数 */
        }
    }
    return 0;
}
