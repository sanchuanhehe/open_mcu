/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file      mcs_prot_user_config.h
  * @author    MCU Algorithm Team
  * @brief     This file contans user macro definition of the protection function.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_PROT_USER_CONFIG_H
#define McuMagicTag_MCS_PROT_USER_CONFIG_H

/* Macro definitions --------------------------------------------------------------------------- */

/**< COMMON */
/**<连续运行时电机或IPM的峰值相电流（A）*/
#define PROT_MOTOR_RATED_CURR           (1.0f)
/**<只有几次连续的故障检测才能触发错误状态*/
#define PROT_CNT_LIMIT                  (100)
/**<只有几个连续的无故障检测才能触发错误状态的消除*/
#define RECY_CNT_LIMIT                  (10000)
/**<只有几个连续的无故障检测才能触发错误状态的消除*/
#define OVER_VOLT_RECY_CNT_LIMIT        (100)
/**<只有几个连续的无故障检测才能触发错误状态的消除*/
#define LOWER_VOLT_RECY_CNT_LIMIT       (100)

/**<过电流保护*/
/**<处于1级时的过电流触发值（A）*/
#define PROT_OVER_CURR_POW_DN1          (2.0f * PROT_MOTOR_RATED_CURR)
/**<处于2级时的过电流触发值（A）*/
#define PROT_OVER_CURR_POW_DN2          (2.0f * PROT_MOTOR_RATED_CURR)
/**<处于3级时的过电流触发值（A）*/
#define PROT_OVER_CURR_POW_DN3          (2.0f * PROT_MOTOR_RATED_CURR)
/**<处于4级时的过电流触发值（A）*/
#define PROT_OVER_CURR_POW_OFF          (2.0f * PROT_MOTOR_RATED_CURR)
#define PROT_OVER_CURR_RECY_DELTA       (0.0035f)    /*<从保护状态恢复时的电流间隙（A）*/
#define PROT_OVER_CURR_LIMIT1_TIME_SEC  (30.0f)    /**<20%过载的最长持续时间：30秒*/
#define PROT_OVER_CURR_LIMIT2_TIME_SEC  (10.0f)    /**<20%过载的最长持续时间：10秒*/
#define PROT_OVER_CURR_LIMIT3_TIME_SEC  (2.0f)    /**<20%过载的最长持续时间：2秒*/

/**< Over voltage protection */
#define PROT_OVER_VOLT_BRK_ON1          (26.0f)   /**<处于1级时，直流链路电压触发值（V）过高*/
#define PROT_OVER_VOLT_BRK_ON2          (27.0f)   /**< Over dc-link voltage trigger value (V) when in level 2. */
#define PROT_OVER_VOLT_BRK_ON3          (28.0f)   /**< Over dc-link voltage trigger value (V) when in level 3. */
#define PROT_OVER_VOLT_BRK_ALL          (30.0f)   /**< Over dc-link voltage trigger value (V) when in level 4. */
#define PROT_OVER_VOLT_RECY_DELTA       (0.5f)     /**<从保护状态恢复时的电压间隙（V）*/
#define PROT_OVER_VOLT_LIMIT1_TIME_SEC  (5.0f)     /**<overload1可以持续最长时间（秒）*/
#define PROT_OVER_VOLT_LIMIT2_TIME_SEC  (3.0f)     /**<overload2可以持续最长时间（秒）*/
#define PROT_OVER_VOLT_LIMIT3_TIME_SEC  (1.0f)     /**<overload3可以持续最长时间（秒）*/
#define PROT_POW_DN1_PCT                (1.0f)     /*断电级别为1级*/
#define PROT_POW_DN2_PCT                (1.0f)     /* Power down level in level 2. */
#define PROT_POW_DN3_PCT                (1.0f)     /* Power down level in level 3. */

/**<传导负载需要用制动回路的功率进行校准*/
#define PROT_OVER_VOLT_BRK_DUTY1        (0.25f)    /**<1级制动回路的传导占空比*/
#define PROT_OVER_VOLT_BRK_DUTY2        (0.50f)    /**< Conduction duty of the brake loop in level 2. */
#define PROT_OVER_VOLT_BRK_DUTY3        (0.75f)    /**< Conduction duty of the brake loop in level 3. */
#define PROT_OVER_VOLT_BRK_DUTY4        (1.00f)    /**< Conduction duty of the brake loop in level 4. */

/**<低电压保护*/
#define PROT_LOWER_VOLT_POW_DN1         (10.3f)  /**<处于1级时，直流链路电压触发值（V）较低*/
#define PROT_LOWER_VOLT_POW_DN2         (10.0f)  /**< Lower dc-link voltage trigger value (V) when in level 2. */
#define PROT_LOWER_VOLT_POW_DN3         (9.0f)   /**< Lower dc-link voltage trigger value (V) when in level 3. */
#define PROT_LOWER_VOLT_POW_OFF         (8.0f)   /**< Lower dc-link voltage trigger value (V) when in level 4. */
#define PROT_LOWER_VOLT_RECY_DELTA      (0.5f)   /**<从保护状态恢复时的电压间隙（A）*/
#define PROT_LOWER_VOLT_LIMIT1_TIME_SEC (3.0f)   /**< 20% overload can last maximum time: 3 sec. */
#define PROT_LOWER_VOLT_LIMIT2_TIME_SEC (3.0f)   /**< 20% overload can last maximum time: 3 sec. */
#define PROT_LOWER_VOLT_LIMIT3_TIME_SEC (3.0f)   /**< 20% overload can last maximum time: 3 sec. */

/**<IPM温度保护*/
#define PROT_OVER_IPM_TEMP_POW_DN1      (40.0f)  /**< Over IPM temperature trigger value (celsius) when in level 1. */
#define PROT_OVER_IPM_TEMP_POW_DN2      (42.0f)  /**< Over IPM temperature trigger value (celsius) when in level 2. */
#define PROT_OVER_IPM_TEMP_POW_DN3      (44.0f)  /**< Over IPM temperature trigger value (celsius) when in level 3. */
#define PROT_OVER_IPM_TEMP_POW_OFF      (45.0f)  /**< Over IPM temperature trigger value (celsius) when in level 4. */
#define PROT_OVER_IPM_TEMP_RECY_DELTA   (0.5f) /**<从保护状态恢复时的温度间隙（摄氏度）*/
#define PROT_OVER_TEMP_LIMIT1_TIME_SEC  (10.0f)  /**< 20% overload can last maximum time: 10 sec. */
#define PROT_OVER_TEMP_LIMIT2_TIME_SEC  (10.0f)  /**< 20% overload can last maximum time: 10 sec. */
#define PROT_OVER_TEMP_LIMIT3_TIME_SEC  (10.0f)  /**< 20% overload can last maximum time: 10 sec. */

/**< Motor stalling detection */
/**< 反馈电流高于此值会触发故障. (A). */
#define PROT_STALLING_CURR_AMP_LIMIT  (PROT_MOTOR_RATED_CURR * 1.5f)
/**< 低于此值的反馈速度会触发故障（Hz）。 */
#define PROT_STALLING_SPD_LIMIT       10
/**< 电流和速度在一定范围内反馈的阈值时间。 */
#define PROT_STALLING_TIME_LIMIT      (1.5f)

/**<电流失衡检测*/
#define UNBAL_STARTUP_DETECT_TIME_SEC (0.5f)   /**<启动检测延迟*/
#define UNBAL_PROT_CNT_LIMIT          (50000)
#define UNBAL_RECY_CNT_LIMIT          (50000)
#define UNBAL_CURRENT_DELTA           (1.5f)   /**<用于检测当前周期中的过零点*/
#define UNBAL_DEGREE_LIMIT            (0.035f) /**<不平衡度阈值*/
#define UNBAL_DEGREE_AVG_FLT_COFFI    (0.03f)  /**<不平衡度平均滤波系数*/

/**<位置传感器检测*/
#define POS_SNSR_FAULT_CNT          (100000)   /*连续故障次数*/

#define POS_SNSR_RECY_CNT           (10000)   /*连续通信丢失次数*/



/**<相绕组完整性检测*/
#define OPEN_PHS_CURR_THR_A (0.1f)  /*确定断相无电流的阈值（A）*/

/**<位置传感器零位检测*/
#define POS_SNSR_CALIBR_UD_REF      (15.0f)  /* V */
#define POS_SNSR_CALIBR_UD_SLOPE    (15.0f)  /* (V/S) */
#define POS_SNSR_CALIBR_DETECT_TIME (2.0f)   /* S */
#define POS_SNSR_RECORD_TIMES       (2000)   /* 2000 * TS */


/*多循环模式：>1；单循环模式：<0.5*/
#define POS_SNSR_IPD_INJ_PERIOD     (4)

#endif