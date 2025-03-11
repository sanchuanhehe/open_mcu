/**
  * @copyright Copyright (c) Hisilicon Technologies Co., Ltd 2024-2024. All rights reserved.
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
 * @file    sample_timer_task.c
 * @author  MCU Driver Team
 * @brief   timertask sample module.
 * @details This file provides users with sample code to help use TIMER TASK function
 */
#include "feature.h"
#include "main.h"
#ifdef NOS_TASK_SUPPORT
#include "debug.h"
#include "systick.h"
#include "nos_task.h"
#include "crg.h"
#include "nos_task_schedule.h"

#define TIMER_DFX_CNT 100
#define TIMER0_TIMEOUT 100
#define TIMER1_TIMEOUT 200
#define TIMER2_TIMEOUT 1000
#define TIMER0_PRIORITY 0
#define TIMER1_PRIORITY 1
#define TIMER2_PRIORITY 2
#define SAMPLE_RUNNING_TIME 100

#define TIMER_NUM 3
#define TIMER0_INDEX 0
#define TIMER1_INDEX 1
#define TIMER2_INDEX 2

static unsigned char __attribute__((aligned(16))) g_task0StackSpace[0x200] = {0};
static unsigned char __attribute__((aligned(16))) g_task1StackSpace[0x200] = {0};
static unsigned char __attribute__((aligned(16))) g_task2StackSpace[0x200] = {0};

typedef struct {
    unsigned int param;
    unsigned int time;
} DfxTimerTask;

static DfxTimerTask g_dfxTest[TIMER_DFX_CNT] = {0};
static unsigned int g_testCnt;

/**
  * @brief task0 callback function.
  * @param param task param.
  * @retval None.
  */
static void TIMER0_CallbackFunc(void *param)
{
    if (g_testCnt < TIMER_DFX_CNT) {
        g_dfxTest[g_testCnt].param = (unsigned int)(uintptr_t)param;
        g_dfxTest[g_testCnt].time = (unsigned int)SYSTICK_GetTimeStampUs(); /* get time stamp us */
        g_testCnt++;  /* task has been scheduled times */
    }
}

/**
  * @brief task1 callback function.
  * @param param task param.
  * @retval None.
  */
static void TIMER1_CallbackFunc(void *param)
{
    if (g_testCnt < TIMER_DFX_CNT) {
        g_dfxTest[g_testCnt].param = (unsigned int)(uintptr_t)param;
        g_dfxTest[g_testCnt].time = (unsigned int)SYSTICK_GetTimeStampUs(); /* get time stamp us */
        g_testCnt++;  /* task has been scheduled times */
    }
}

/**
  * @brief task2 callback function.
  * @param param task param.
  * @retval None.
  */
static void TIMER2_CallbackFunc(void *param)
{
    if (g_testCnt < TIMER_DFX_CNT) {
        g_dfxTest[g_testCnt].param = (unsigned int)(uintptr_t)param;
        g_dfxTest[g_testCnt].time = (unsigned int)SYSTICK_GetTimeStampUs(); /* get time stamp us */
        g_testCnt++;  /* task has been scheduled times */
    }
}

static unsigned int g_sampleTaskPid[TIMER_NUM];
static NOS_TimerTaskInitParam g_timerTaskParams[TIMER_NUM] = {
    {
        .name = "task0",
        .timeout = TIMER0_TIMEOUT,
        .callback = TIMER0_CallbackFunc, /* task0 callback function. */
        .callbackParam = (void *)TIMER0_TIMEOUT,
        .priority = TIMER0_PRIORITY,
        .stackSize = sizeof(g_task0StackSpace),
        .stackAddr = (unsigned int)g_task0StackSpace
    },
    {
        .name = "task1",
        .timeout = TIMER1_TIMEOUT,
        .callback = TIMER1_CallbackFunc, /* task1 callback function. */
        .callbackParam = (void *)TIMER1_TIMEOUT,
        .priority = TIMER1_PRIORITY,
        .stackSize = sizeof(g_task1StackSpace),
        .stackAddr = (unsigned int)g_task1StackSpace
    },
    {
        .name = "task2",
        .timeout = TIMER2_TIMEOUT,
        .callback = TIMER2_CallbackFunc, /* task2 callback function. */
        .callbackParam = (void *)TIMER2_TIMEOUT,
        .priority = TIMER2_PRIORITY,
        .stackSize = sizeof(g_task2StackSpace),
        .stackAddr = (unsigned int)g_task2StackSpace
    }
};

/**
  * @brief Creat timer task and return task id for task.
  * @param None.
  * @retval unsigned int 0 means function call success.
  */
static unsigned int TIMER_SampleCreateTimer(void)
{
    DBG_PRINTF("****************************************************** \r\n");
    DBG_PRINTF("Test for NosTimerTask: timer create & start\r\n");

    DBG_PRINTF("****************************************************** \r\n");

    for (int i = 0; i < TIMER_NUM; i++) {
         /* Creat timer task and return task id for task. */
        (void)NOS_CreateTimerTask(&g_sampleTaskPid[i], &g_timerTaskParams[i]);
    }

    for (int i = 0; i < TIMER_NUM; i++) {
         /* start timer task. */
        (void)NOS_StartTimerTask(g_sampleTaskPid[i]);
    }
    return 0;
}

/**
  * @brief Print task schedule result.
  * @param None.
  * @retval None.
  */
static void TASK_PrintResult(void)
{
    int cnt[TIMER_NUM] = {0};
    for (int i = 0; i < TIMER_DFX_CNT; i++) {
        if (g_dfxTest[i].param == TIMER0_TIMEOUT) {
            cnt[TIMER0_INDEX]++;  /* task0 schedule times */
        } else if (g_dfxTest[i].param == TIMER1_TIMEOUT) {
            cnt[TIMER1_INDEX]++;  /* task1 schedule times */
        } else if (g_dfxTest[i].param == TIMER2_TIMEOUT) {
            cnt[TIMER2_INDEX]++;  /* task2 schedule times */
        }
        DBG_PRINTF("enter callback timeout=%d  timestamp=%dus \r\n", g_dfxTest[i].param, g_dfxTest[i].time);
    }
    DBG_PRINTF("total cnt:%d cnt[100us]:%d,cnt[200us]:%d, cnt[1000us]:%d \r\n", TIMER_DFX_CNT, cnt[TIMER0_INDEX],
        cnt[TIMER1_INDEX], cnt[TIMER2_INDEX]);
}

static void TIMER_SampleStopTimer(void)
{
    for (int i = 0; i < TIMER_NUM; i++) {
        NOS_StopTimerTask(g_sampleTaskPid[i]); /* stop timer task. */
    }
}

void NOS_TaskSchedule(void)
{
    SystemInit();
    (void)TIMER_SampleCreateTimer();  /* creat and start task schedule */
    BASE_FUNC_DelayMs(SAMPLE_RUNNING_TIME);
    TIMER_SampleStopTimer();   /* stop task schedule */
    TASK_PrintResult();   /* print task schedule result */
}
#endif /* NOS_TASK_SUPPORT */