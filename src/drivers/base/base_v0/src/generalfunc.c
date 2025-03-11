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
  * @file      basic.c
  * @author    MCU Driver Team
  * @brief   BASE module driver
  * @details   This file provides firmware functions to manage the following
  * functionalities of the basic functions.
  * + Verifying the timeout function
  * + 8-bit, 16-bit checksum function
  * + Sliding averaging function
  * + General state machine
  * @verbatim
  * Sliding averaging interface usage:
  * 1) Call the BASE_FUNC_AverageInit() function to initialize and configure the buffer,
  *    average the window size, and set the index value for identification.
  * 2) Call the BASE_FUNC_GetSlipAverageVal() function based on the index value transferred
  *    in the initialization function to obtain the average value of the current window.
  * 3) Call the BASE_FUNC_AverageDeInit() function to close the current index channel.
  *
  * General state machine usage:
  * 1) Add your status to enum BASE_FSM_Status;
  * 2) Write your code for each state. Note that the function prototype is BASE_FSM_Status xxx(void);
  * 3) Use BASE_FSM_FunRegister() to register your functions and their status;
  * 4) Start the state machine using BASE_FSM_Run().
  * @endverbatim
  */

/* Includes ------------------------------------------------------------------ */
#include "systick.h"
#include "generalfunc.h"

/* Macro definitions ---------------------------------------------------------*/
#define MEA_MAX_NUM 2  /* the num of the measurement group */
#define AVE_MAX_NUM 10 /* the num of the saving current ticks */

/* Typedef definitions -------------------------------------------------------*/
typedef enum {
    STOP,
    START
} MeasureStatus;

typedef struct {
    unsigned int startTick; /* start tick */
    unsigned int stopTick;  /* stop tick */
    unsigned int curr;     /* current tick, value is (stopTick - startTick) */
    unsigned int peak;     /* maximum tick value */
    unsigned int valley;   /* minimum tick value */
    unsigned int average;  /* average value of the ave[AVE_MAX_NUM] */
    unsigned int ave[AVE_MAX_NUM]; /* save current tick for average */
    unsigned char size;   /* the size of the ave[AVE_MAX_NUM] */
    unsigned char index;  /* the index of the ave[AVE_MAX_NUM] */
    MeasureStatus status; /* the status of the measurement(start or stop) */
} Measurement;

/* Private variables --------------------------------------------------------- */
BASE_AverageHandle g_averageHandle[BASE_DEFINE_SLIPAVERAGE_NUM];
BASE_FSM_Handle g_fsmHandle;

#ifdef MEASURE_SUPPORT
static Measurement g_measure[MEA_MAX_NUM];
#endif

/**
  * @brief Obtains the current tick value.
  * @retval unsigned int. Current tick value.
  */
unsigned int BASE_FUNC_GetTick(void)
{
    return DCL_SYSTICK_GetTick();
}

/**
  * @brief Query an element in an array using dichotomous lookup. Note: Arrays are sorted in ascending order.
  *        Returns the left index when the array element does not exist.
  * @param nums Array to be searched.
  * @param leng Array Length.
  * @param value Value to be searched for.
  * @return unsigned int Index value corresponding to value.
  */
unsigned int BASE_FUNC_FindArrayValue(const unsigned short *nums, unsigned int leng, unsigned int value)
{
    BASE_FUNC_ASSERT_PARAM(nums != NULL);
    BASE_FUNC_PARAMCHECK_WITH_RET(leng > 0, 0);
    unsigned int left = 0;
    unsigned int right = leng - 1;
    while (left < right) {
        unsigned int mid = (left + right) / 2;
        if (value >= nums[mid] && value < nums[mid + 1]) {
            return mid;
        } else if (value < nums[mid]) {
            right = mid - 1;
        } else {
            left = mid + 1;
        }
    }
    return left;
}

/**
  * @brief 8-bit checksum.
  * @param pt Pointer to the data to be computed.
  * @param len Data length.
  * @return unsigned char Calculation result.
  */
unsigned char BASE_FUNC_CalcSumByte(const unsigned char *pt, unsigned int len)
{
    BASE_FUNC_ASSERT_PARAM(pt != NULL);
    BASE_FUNC_PARAMCHECK_WITH_RET((len > 0), 0);

    unsigned int sum = 0;
    /* calculate sum value */
    while (len--) {
        sum += *pt;
        pt++;
    }
    /* Use 8 digits */
    return (unsigned char)sum;
}

/**
  * @brief 16-bit checksum.
  * @param pt Pointer to the data to be computed.
  * @param len Data length.
  * @return unsigned char Calculation result.
  */
unsigned short BASE_FUNC_CalcSumShort(unsigned char const * pt, unsigned int len)
{
    BASE_FUNC_ASSERT_PARAM(pt != NULL);
    BASE_FUNC_PARAMCHECK_WITH_RET((len > 0), 0);
    unsigned int sum = 0;
    /* calculate sum value */
    while (len--) {
        sum += *pt;
        pt++;
    }
    /* Use 16 digits */
    return (unsigned short)sum;
}

/**
  * @brief Sliding average initialization function.
  * @param index User-entered index value used to identify the channel, in [0, BASE_DEFINE_SLIPAVERAGE_NUM).
  * @param buf Pointer to the ring buffer, it stores historical data.
  * @param size Ring buffer size.
  * @param calNum Indicates the average window size, that is, the number of pieces of data to be averaged.
  * @return BASE_StatusType @ref BASE_StatusType.
  */
BASE_StatusType BASE_FUNC_AverageInit(unsigned int index, float *buf, unsigned int size, unsigned int calNum)
{
    /* verify param */
    BASE_FUNC_ASSERT_PARAM(buf != NULL);
    BASE_FUNC_PARAMCHECK_WITH_RET((calNum > 0), BASE_STATUS_ERROR);
    BASE_FUNC_PARAMCHECK_WITH_RET((size >= calNum), BASE_STATUS_ERROR);
    BASE_FUNC_PARAMCHECK_WITH_RET((index < BASE_DEFINE_SLIPAVERAGE_NUM), BASE_STATUS_ERROR);
    /* init handle's member */
    g_averageHandle[index].buf = buf;
    g_averageHandle[index].size = size;
    g_averageHandle[index].at = 0;
    g_averageHandle[index].calNum = calNum;
    g_averageHandle[index].total = 0;
    g_averageHandle[index].cnt = 0;

    return BASE_STATUS_OK;
}

/**
  * @brief Transfer new data and return the average value after the new data is inserted.
  * @param index Index value of the handle array, which is set by the user in the initialization function.
  * @param val Data value.
  * @return float Calculated average.
  */
float BASE_FUNC_GetSlipAverageVal(unsigned int index, float val)
{
    /* verify param */
    BASE_FUNC_ASSERT_PARAM(index < BASE_DEFINE_SLIPAVERAGE_NUM);
    /* The processing data volume does not reach the constant average amount. */
    if (g_averageHandle[index].cnt < g_averageHandle[index].calNum) {
        (g_averageHandle[index].cnt)++;
        g_averageHandle[index].total += val;
        g_averageHandle[index].buf[g_averageHandle[index].at] = val;
        (g_averageHandle[index].at)++;
        return g_averageHandle[index].total / g_averageHandle[index].cnt; /* g_averageHandle[index].cnt > 0 */
    }
    /* The processing data volume reach the constant average amount. */
    g_averageHandle[index].total += val - g_averageHandle[index].buf[(g_averageHandle[index].at + \
                                    g_averageHandle[index].size - g_averageHandle[index].calNum) % \
                                    g_averageHandle[index].size];
    g_averageHandle[index].buf[g_averageHandle[index].at] = val;
    g_averageHandle[index].at = (g_averageHandle[index].at + 1) % g_averageHandle[index].size;
    return g_averageHandle[index].total / g_averageHandle[index].calNum; /* g_averageHandle[index].calNum > 0 */
}

/**
  * @brief Disables the channel specified by index.
  * @param index Index value of the handle array, which is set by the user in the initialization function.
  * @return None.
  */
void BASE_FUNC_AverageDeInit(unsigned int index)
{
    /* verify param */
    BASE_FUNC_ASSERT_PARAM(index < BASE_DEFINE_SLIPAVERAGE_NUM);
    g_averageHandle[index].buf = NULL;
}

/**
  * @brief Registering functions to the state machine. Note that the function prototype is BASE_FSM_Status xxx(void).
  * @param index Status of the function.
  * @param funAddress Function Pointer.
  * @return None.
  */
void BASE_FSM_FunRegister(BASE_FSM_Status index, FunType funAddress)
{
    BASE_FUNC_PARAMCHECK_NO_RET(index >= BASE_FSM_START && index <= BASE_DEFINE_FSM_END);
    g_fsmHandle.funList[index] = funAddress;
}

/**
  * @brief Start the state machine.
  * @param delayTime State switching delay time.
  * @param delayUnit Indicates the unit of the state switch delay.
  * @return None.
  */
void BASE_FSM_Run(unsigned int delayTime, BASE_DelayUnit delayUnit)
{
    g_fsmHandle.nextFun = BASE_FSM_START;

    FunType execFun;
    while (1) {
        execFun = g_fsmHandle.funList[g_fsmHandle.nextFun];
        g_fsmHandle.nextFun = execFun();
        if (g_fsmHandle.nextFun < BASE_FSM_START || g_fsmHandle.nextFun > BASE_DEFINE_FSM_END) {
            break;
        }
        BASE_FUNC_Delay(delayTime, delayUnit);
    }
}

#ifdef MEASURE_SUPPORT
/**
 * @brief   Start Measurement. Get the systick for Measurement.
 * @param   id Selecting a measurement group
 * @retval  None
 */
void BASE_FUNC_MeasurementStart(unsigned int id)
{
    if (id >= MEA_MAX_NUM) {
        return;
    }

    Measurement *measure = &g_measure[id];
    if (measure->status == START) {
        /* START and STOP must appear in pairs. Drop the superfluous START. */
        return;
    }

    measure->status = START;
    measure->startTick = DCL_SYSTICK_GetTick();
}

/**
 * @brief   Average measurement.
 * @param   measure a measurement group
 * @retval  None
 */
static void BASE_FUNC_MeasurementAve(Measurement *measure)
{
    if (measure == NULL) {
        return;
    }
    /* Add cur value into average queue */
    if (measure->index < AVE_MAX_NUM) {
        measure->ave[measure->index] = measure->curr;
        if (measure->size < AVE_MAX_NUM) {
            measure->size++;
        }
        measure->index = (measure->index + 1) % AVE_MAX_NUM;
    }

    /* Calculate the average in the queue */
    unsigned long long tmp = 0;
    for (int i = 0; i < measure->size; i++) {
        tmp += measure->ave[i];
    }
    if (measure->size != 0) {
        float f = (float)tmp / (float)measure->size;
        measure->average = (unsigned int)f;
    }
}

/**
 * @brief   Stop measurement. Get the systick and statistic the systick.
 * @param   id Selecting a measurement group
 * @retval  None
 */
void BASE_FUNC_MeasurementStop(unsigned int id)
{
    unsigned int tick = DCL_SYSTICK_GetTick();
    if (id >= MEA_MAX_NUM) {
        return;
    }

    Measurement *measure = &g_measure[id];
    if (measure->status == STOP) {
        /* START and STOP must appear in pairs. Drop the superfluous STOP. */
        return;
    }

    measure->status = STOP;
    measure->stopTick = tick;
    measure->curr =
        tick > measure->startTick ? tick - measure->startTick : SYSTICK_MAX_VALUE - measure->startTick + tick;
    /* max measurement. */
    if (measure->curr > measure->peak) {
        measure->peak = measure->curr;
    }

    /* min measurement. */
    if (measure->curr < measure->valley || measure->valley == 0) {
        measure->valley = measure->curr;
    }

    BASE_FUNC_MeasurementAve(measure);
}
#endif