/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
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
  * @file    key.c
  * @author  MCU Driver Team
  * @brief   key press process
  */
#include <stdint.h>
#include "stm32mp1xx.h"
#include "stm32mp1xx_hal.h"
#include "key.h"

/**
 * @brief       Key Init
 * @param       No
 * @retval      No
 */
void KeyInit(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    /* Key gpio clock enable */
    KeyUpGpioClkEnable();
    KeyDownGpioClkEnable();
    KeyConfirmGpioClkEnable();
   
    /* GPIO config */
    gpio_init_struct.Pin = KEY_UP_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(KEY_UP_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = KEY_DOWN_GPIO_PIN;
    HAL_GPIO_Init(KEY_DOWN_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = KEY_CONFIRM_GPIO_PIN;
    HAL_GPIO_Init(KEY_CONFIRM_GPIO_PORT, &gpio_init_struct);
}

/**
 * @brief       Is Key press enough time to press the keys
 * @param       mode:0： short key, 1: long key
 * @param       keyCnt: Statistics on consecutive key presses
 * @retval      true or false
 */
static unsigned int IsKey(uint8_t mode, unsigned int keyCnt)
{
#define KEY_VALID_TIMES 2
    if (mode == LONG_KEY) {
        return (keyCnt >= KEY_VALID_TIMES);
    }
    return (keyCnt == KEY_VALID_TIMES);
}

/**
 * @brief       Key scan press process
 * @param       mode:
 * @retval      KeyValue
 */
uint8_t KeyScan(uint8_t mode)
{
    static unsigned int key0Cnt = 0;
    static unsigned int key1Cnt = 0;
    static unsigned int key2Cnt = 0;
    uint8_t keyval = 0;

    if (KEY2 == 0) {  /* Confirm Key Pressed */
        key2Cnt++;
        key0Cnt = 0;
        key1Cnt = 0;
    } else if (KEY1 == 0) { /* MoveDown Key Pressed */
        key1Cnt++;
        key0Cnt = 0;
        key2Cnt = 0;
    } else if (KEY0 == 0) { /* MoveUp Key Pressed */
        key0Cnt++;
        key1Cnt = 0;
        key2Cnt = 0;
    } else {                /* No Key Pressed, Clear KeyPress Count */
        key0Cnt = 0;
        key1Cnt = 0;
        key2Cnt = 0;
    }
    /* If Key Pressed, set key value */
    if (IsKey(mode, key2Cnt)) {
        keyval = KEY_CONFIRM;
    } else if (IsKey(mode, key1Cnt)) {
        keyval = KEY_DOWN;
    } else if (IsKey(mode, key0Cnt)) {
        keyval = KEY_UP;
    } else {
        keyval = 0; /* Invalid value */
    }
    /* return key press value */
    return keyval;
}
