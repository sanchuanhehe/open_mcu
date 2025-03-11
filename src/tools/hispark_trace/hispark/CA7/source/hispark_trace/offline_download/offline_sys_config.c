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
  * @file    offline_sys_config.c
  * @author  MCU Driver Team
  * @brief   offline sys config driver.
  */
#include "DAP.h"
#include "swd_host.h"
#include "jtag_host.h"
#include "swd_jtag_config.h"
#include "debug_cm.h"
#include "target_lib_manager.h"
#include "config_storage_update.h"
#include "factory_manager.h"
#include "IO_Config.h"
#include "offline_download.h"
#include "offline_sys_config.h"

#define UNLOCK_SECURITY_TIMEOUT  5000
#define UNLOCK_SECURITY_SUCCESS  0x1
#define SOFT_RESET_TIME          100

static uint32_t g_sysStartFlag = CONFIG_STRAT_FLAG_A;
static uint32_t g_sysLanguage = SYS_LANGUAGE_CN;
static bool g_sysSoftResetFlag = true;

/**
  * @brief Obtains the tick difference..
  * @param curTicks Current tick
  * @param preTicks Previous tick
  * @retval Tick Delta value
  */
static uint32_t GetTickDelta(uint32_t curTicks, uint32_t preTicks)
{
    if (curTicks >= preTicks) {
        return curTicks - preTicks;
    }
    /* Prevent inversion errors */
    return 0xFFFFFFFF - preTicks + curTicks + 1;
}

/**
  * @brief Jtag reset.
  * @retval true or false
  */
static bool JtagReset(void)
{
    uint32_t val;
    int timeout = SOFT_RESET_TIME;
    uint32_t tmp = 0;
    int i = 0;

    /* Switching from the SWD interface to the JTAG interface. */
    if (!SWD2JTAG()) {
        return false;
    }
    /* clear jtag errors. */
    if (!jtag_clear_errors()) {
        return false;
    }
    /* Select AP */
    if (!jtag_write_dp(DP_SELECT, 0)) {
        return false;
    }

    /* Power up */
    if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
        return false;
    }
    /* Wait until the power-on is successful. */
    for (i = 0; i < timeout; i++) {
        if (!jtag_read_dp(DP_CTRL_STAT, &tmp)) {
            return false;
        }

        if ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK)) {
            /* Break from loop if powerup is complete */
            break;
        }
    }

    if (i == timeout) {
        /* Unable to powerup DP */
        return false;
    }

    if (!jtag_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE)) {
        return false;
    }
    return true;
}

/**
  * @brief Swd reset.
  * @retval true or false
  */
static bool SwdReset(void)
{
    uint32_t val;
    int timeout = SOFT_RESET_TIME;
    uint32_t tmp = 0;
    int i = 0;

    /* Switching from the JTAG interface to the SWD interface. */
    if (JTAG2SWD() == 0) {
        return false;
    }
    /* clear swd errors. */
    if (!swd_clear_errors()) {
        return false;
    }
    /* Select AP */
    if (!swd_write_dp(DP_SELECT, 0)) {
        return false;
    }

    /* Power up */
    if (!swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)) {
        return false;
    }
    /* Wait until the power-on is successful. */
    for (i = 0; i < timeout; i++) {
        if (!swd_read_dp(DP_CTRL_STAT, &tmp)) {
            return false;
        }
        if ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) == (CDBGPWRUPACK | CSYSPWRUPACK)) {
            /* Break from loop if powerup is complete */
            break;
        }
    }
    if ((i == timeout)) {
        /* Unable to powerup DP */
        return false;
    }

    if (!swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE)) {
        return false;
    }
    return true;
}

/**
  * @brief Initializing the Offline Programming System Configuration.
  * @retval 0 success
  */
int32_t OfflineSysConfigInit(void)
{
    uint32_t powerStatus;

    g_sysStartFlag = ConfigStartFlagRead();
    powerStatus = SysExternalPowerStatusGet();
    SysExternalPowerSet(powerStatus);
    ConfigFactoryImageLoadCntAddrCalibration();
    ConfigImageLoadCntAddrCalibration();
    return 0;
}

/**
  * @brief Restoring Factory Settings.
  * @retval 0 success
  */
int32_t ConfigRestoreFactory(void)
{
    uint32_t startFlag;

    TargetLibImageDeleteAll();
    TargetLibAlgoDeleteAll();
    FactoryImageFreeAll();
    FactoryImageFreeAll();
    ConfigFactoryImageLoadCntClean();
    ConfigImageLoadCntClean();

    ConfigFactoryFlagSave(0);

    startFlag = ConfigStartFlagRead();
    if (startFlag != CONFIG_STRAT_FLAG_A && startFlag != CONFIG_STRAT_FLAG_B) {
        startFlag = CONFIG_STRAT_FLAG_A;
        ConfigStartFlagSave(startFlag);
    }
    SysExternalPowerSet(SYS_EXTERNAL_POWER_OFF);
    OfflineDownLoadInit();
    TargetLibInit();
    FactoryInit();
    return 0;
}

/**
  * @brief Startup flag acquisition.
  * @retval Startup flag
  */
uint32_t SysStartFlagGet(void)
{
    return g_sysStartFlag;
}

/**
  * @brief Configuration of the External Power Supply Function.
  * @param status External power supply configuration status
  * @retval None
  */
void SysExternalPowerSet(uint32_t status)
{
    uint32_t currentStatus;
    if (status == SYS_EXTERNAL_POWER_ON) {
        POWER_SWITCH_PORT->BSRR = POWER_SWITCH_PIN;
    } else {
        POWER_SWITCH_PORT->BSRR = (POWER_SWITCH_PIN << 16); /* Shift left by 16 bits */
    }

    currentStatus = ConfigPowerStatusFlagRead();
    if (currentStatus != status) {
        ConfigPowerStatusFlagSave(status);
    }
}

/**
  * @brief Obtaining the Configuration Status of External Power Supply.
  * @retval External power supply configuration status
  */
uint32_t SysExternalPowerStatusGet(void)
{
    return ConfigPowerStatusFlagRead();
}

/**
  * @brief Setting the status of the offline burning indicator
  * @param status Indicator status
  * @retval None
  */
void OfflineLoadLedSet(uint32_t status)
{
    if ((status & 1) != 0) {
        OFFLINE_DOWNLOAD_STATUS_LED_PORT->BSRR = OFFLINE_DOWNLOAD_STATUS_LED_PIN;
    } else {
        OFFLINE_DOWNLOAD_STATUS_LED_PORT->BSRR = (OFFLINE_DOWNLOAD_STATUS_LED_PIN << 16); /* Shift left by 16 bits */
    }
}

/**
  * @brief Reading the board ID
  * @retval board ID
  */
uint32_t SysBoardIdGet(void)
{
    uint32_t boardId = 0;
    boardId |= (((BOARD_ID2_PORT->IDR & BOARD_ID2_PIN) ? 1 : 0) << 2); /* Obtains the 2 digit of the ID. */
    boardId |= (((BOARD_ID1_PORT->IDR & BOARD_ID1_PIN) ? 1 : 0) << 1); /* Obtains the 1 digit of the ID. */
    boardId |= ((BOARD_ID0_PORT->IDR & BOARD_ID0_PIN) ? 1 : 0); /* Obtains the 0 digit of the ID. */
    return boardId;
}

/**
  * @brief SWD Unlock the security configuration and roll back the flash protection of the target chip.
  * @retval false or true
  */
static bool SwdSysUnlockSecurity(void)
{
    uint32_t val;
    uint32_t preTicks;
    uint32_t curTicks;
    uint32_t delta;
    int32_t clockLevel;

    clockLevel = SwdJtagClockLevelGet();
    SwJtagClockLevelLevelSet(clockLevel);
    while (!SwdReset()) {
        if (clockLevel <= SWD_JTAG_CLOCK_LEVEL_9) {
            SwJtagClockLevelLevelSet(clockLevel);
            clockLevel++;
        } else {
            break;
        }
    }
    preTicks = HAL_GetTick();
    swd_write_dp(DP_SELECT, 0x1000000);
    swd_write_ap(AP_CSW, 0x0000002, 0x1000000);
    swd_write_ap(AP_TAR, 0x14724000, 0x1000000);
    swd_write_ap(AP_DRW, 0xABEFCD00, 0x1000000);
    while (1) {
        curTicks = HAL_GetTick();
        delta = GetTickDelta(curTicks, preTicks);
        swd_write_ap(AP_TAR, 0x14724004, 0x1000000);
        swd_read_ap(AP_DRW, &val, 0x1000000);
        if (val == UNLOCK_SECURITY_SUCCESS) {
            return true;
        }

        if (delta > UNLOCK_SECURITY_TIMEOUT && val != UNLOCK_SECURITY_SUCCESS) {
            return false;
        }
    }
}

/**
  * @brief JTAG Unlock the security configuration and roll back the flash protection of the target chip.
  * @retval false or true
  */
static bool JtagSysUnlockSecurity(void)
{
    uint32_t val;
    uint32_t preTicks;
    uint32_t curTicks;
    uint32_t delta;
    int32_t clockLevel;

    clockLevel = SwdJtagClockLevelGet();
    while (!JtagReset()) {
        if (clockLevel <= SWD_JTAG_CLOCK_LEVEL_9) {
            SwJtagClockLevelLevelSet(clockLevel);
            clockLevel++;
        } else {
            break;
        }
    }
    preTicks = HAL_GetTick();
    jtag_write_dp(DP_SELECT, 0x1000000);
    jtag_write_ap(AP_CSW, 0x0000002, 0x1000000);
    jtag_write_ap(AP_TAR, 0x14724000, 0x1000000);
    jtag_write_ap(AP_DRW, 0xABEFCD00, 0x1000000);
    while (1) {
        curTicks = HAL_GetTick();
        delta = GetTickDelta(curTicks, preTicks);
        jtag_write_ap(AP_TAR, 0x14724004, 0x1000000);
        jtag_read_ap(AP_DRW, &val, 0x1000000);
        jtag_read_dp(DP_RDBUFF, &val);
        if (val == UNLOCK_SECURITY_SUCCESS) {
            return true;
        }

        if (delta > UNLOCK_SECURITY_TIMEOUT && val != UNLOCK_SECURITY_SUCCESS) {
            return false;
        }
    }
}

/**
  * @brief Unlock the security configuration and roll back the flash protection of the target chip.
  * @retval false or true
  */
bool SysUnlockSecurity(void)
{
    uint8_t debugPort;

    debugPort = SwdJtagDebugPortGet();
    if (debugPort == DAP_PORT_JTAG) {
        return JtagSysUnlockSecurity();
    } else {
        return SwdSysUnlockSecurity();
    }
}

/**
  * @brief Setting the language.
  * @param language Language to be set.
  * @retval 0 success
  */
int32_t SysLanguageSet(uint32_t language)
{
    g_sysLanguage = language;
    return ConfigLanguageFlagSave(g_sysLanguage);
}

/**
  * @brief Setting the language.
  * @retval language
  */
uint32_t SysLanguageGet(void)
{
    uint32_t language;

    language = ConfigLanguageFlagRead();
    if (language == SYS_LANGUAGE_EN) {
        g_sysLanguage = SYS_LANGUAGE_EN;
    } else {
        g_sysLanguage = SYS_LANGUAGE_CN;
    }
    return g_sysLanguage;
}

/**
  * @brief Getting the soft reset flag
  * @retval true or false
  */
bool SysSoftResetFlagGet(void)
{
    return g_sysSoftResetFlag;
}

/**
  * @brief Setting the soft reset flag
  * @param flag true or false
  * @retval None
  */
void SysSoftResetFlagSet(bool flag)
{
    g_sysSoftResetFlag = flag;
}