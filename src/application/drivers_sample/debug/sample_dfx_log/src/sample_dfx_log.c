/**
  * @copyright Copyright (c) 2024, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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
  * @file    sample_dfx_log.h
  * @author  MCU Driver Team
  * @brief   dfx_log module sample.
  * @details This file provides sample code for users to help use
  *          the functionalities of the dfx log.
  */
#include "cmd.h"
#include "dfx_log.h"
#include "log.h"
#include "module.h"
#include "config.h"
#include "event.h"
#include "ext_log.h"
#include "dfx_debug.h"
#include "cmd_common.h"
#include "main.h"
#include "sample_dfx_log.h"

#define THIS_FILE_ID    FILE_ID_SAMPLE_C  /* dfx sample file ID */

/**
  * @brief Registering a Function Instance.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int TestCmdRegister(unsigned int argc, const char *argv[])
{
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);
    /* A message is displayed indicating that the registration is successful. */
    EXT_PRINT("cmd register sucessful\n");
    return 0;
}

/**
  * @brief ExtLog0 output test at different levels.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int VerifyLogLevel(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    /* Displays information about different debugging levels. */
    ExtLog0(FATAL, EXT_MODULE_APP_MAIN, "printf FATAL level log\n");
    ExtLog0(ERR, EXT_MODULE_APP_MAIN, "printf ERR level log\n");
    ExtLog0(WARN, EXT_MODULE_APP_MAIN, "printf WARN level log\n");
    ExtLog0(INFO, EXT_MODULE_APP_MAIN, "printf INFO level log\n");
    ExtLog0(DBG, EXT_MODULE_APP_MAIN, "printf DBG level log\n");
    return 0;
}

/**
  * @brief Test on the number of parameters at different levels in the app main module.
  * @retval None.
  */
static void VerifyLogOutAppMain(void)
{
    ExtLog0(FATAL, EXT_MODULE_APP_MAIN, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_APP_MAIN, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 4294967295 */
    ExtLog2(WARN, EXT_MODULE_APP_MAIN, "printf WARN level log%d %d\n", 1, 4294967295);
    /* printf value is 2 and 4294967295 */
    ExtLog3(INFO, EXT_MODULE_APP_MAIN, "printf INFO level log%d %d %d\n", 2, 2, 4294967295);
}

/**
  * @brief Test on the number of parameters at different levels in the app console module.
  * @retval None.
  */
static void VerifyLogOutAppConsole(void)
{
    ExtLog0(FATAL, EXT_MODULE_APP_CONSOLE, "printf FATAL level log\n");
    /* printf value is 4294967295 */
    ExtLog1(ERR, EXT_MODULE_APP_CONSOLE, "printf ERR level log %d\n", 4294967295);
    /* printf value is 100 and 199999 */
    ExtLog2(WARN, EXT_MODULE_APP_CONSOLE, "printf WARN level log%d %d\n", 100, 199999);
    /* printf value is 214752 and 23649 and 4294967295 */
    ExtLog3(INFO, EXT_MODULE_APP_CONSOLE, "printf INFO level log%d %d %d\n", 214752, 23649, 4294967295);
}

/**
  * @brief Test on the number of parameters at different levels in the app chip module.
  * @retval None.
  */
static void VerifyLogOutAppChip(void)
{
    ExtLog0(FATAL, EXT_MODULE_APP_CHIP, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_APP_CHIP, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 1 */
    ExtLog2(WARN, EXT_MODULE_APP_CHIP, "printf WARN level log%d %d\n", 1, 1);
    /* printf value is 2 and 2 and 2 */
    ExtLog3(INFO, EXT_MODULE_APP_CHIP, "printf INFO level log%d %d %d\n", 2, 2, 2);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers base module.
  * @retval None.
  */
static void VerifyLogOutDrvBase(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_BASE, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_BASE, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 1 */
    ExtLog2(WARN, EXT_MODULE_DRV_BASE, "printf WARN level log%d %d\n", 1, 1);
    /* printf value is 2 and 2 and 2 */
    ExtLog3(INFO, EXT_MODULE_DRV_BASE, "printf INFO level log%d %d %d\n", 2, 2, 2);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers chip module.
  * @retval None.
  */
static void VerifyLogOutDrvChip(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_CHIPS, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_CHIPS, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 1 */
    ExtLog2(WARN, EXT_MODULE_DRV_CHIPS, "printf WARN level log%d %d\n", 1, 1);
    /* printf value is 2 and 2 and 2 */
    ExtLog3(INFO, EXT_MODULE_DRV_CHIPS, "printf INFO level log%d %d %d\n", 2, 2, 2);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers crg module.
  * @retval None.
  */
static void VerifyLogOutDrvCRg(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_CRG, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_CRG, "printf ERR level log %d\n", 0);
    /* printf value is 1 and  4294967295 */
    ExtLog2(WARN, EXT_MODULE_DRV_CRG, "printf WARN level log%d %d\n", 1, 4294967295);
    /* printf value is 2 and 2 and  4294967295 */
    ExtLog3(INFO, EXT_MODULE_DRV_CRG, "printf INFO level log%d %d %d\n", 2, 2, 4294967295);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers gpio module.
  * @retval None.
  */
static void VerifyLogOutDrvGpio(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_GPIO, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_GPIO, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 8 */
    ExtLog2(WARN, EXT_MODULE_DRV_GPIO, "printf WARN level log%d %d\n", 1, 8);
    /* printf value is 2 and 2 and 9 */
    ExtLog3(INFO, EXT_MODULE_DRV_GPIO, "printf INFO level log%d %d %d\n", 2, 2, 9);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers i2c module.
  * @retval None.
  */
static void VerifyLogOutDrvI2c(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_I2C, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_I2C, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 5 */
    ExtLog2(WARN, EXT_MODULE_DRV_I2C, "printf WARN level log%d %d\n", 1, 5);
    /* printf value is 2 and 2 and 9 */
    ExtLog3(INFO, EXT_MODULE_DRV_I2C, "printf INFO level log%d %d %d\n", 2, 2, 9);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers irq module.
  * @retval None.
  */
static void VerifyLogOutDrvIrq(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_IRQ, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_IRQ, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 55 */
    ExtLog2(WARN, EXT_MODULE_DRV_IRQ, "printf WARN level log%d %d\n", 1, 55);
    /* printf value is 2 and 2 and 0302 */
    ExtLog3(INFO, EXT_MODULE_DRV_IRQ, "printf INFO level log%d %d %d\n", 2, 2, 0302);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers pinctrl module.
  * @retval None.
  */
static void VerifyLogOutDrvPinctrl(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_PINCTRL, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_PINCTRL, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 88 */
    ExtLog2(WARN, EXT_MODULE_DRV_PINCTRL, "printf WARN level log%d %d\n", 1, 88);
    /* printf value is 2 and 2 and 55 */
    ExtLog3(INFO, EXT_MODULE_DRV_PINCTRL, "printf INFO level log%d %d %d\n", 2, 2, 55);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers timer module.
  * @retval None.
  */
static void VerifyLogOutDrvTimer(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_TIMER, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_TIMER, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 9999 */
    ExtLog2(WARN, EXT_MODULE_DRV_TIMER, "printf WARN level log%d %d\n", 1, 9999);
    /* printf value is 2 and 2 and 99999 */
    ExtLog3(INFO, EXT_MODULE_DRV_TIMER, "printf INFO level log%d %d %d\n", 2, 2, 99999);
}

/**
  * @brief Test on the number of parameters at different levels in the drivers uart module.
  * @retval None.
  */
static void VerifyLogOutDrvUart(void)
{
    ExtLog0(FATAL, EXT_MODULE_DRV_UART, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DRV_UART, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 77744 */
    ExtLog2(WARN, EXT_MODULE_DRV_UART, "printf WARN level log%d %d\n", 1, 77744);
    /* printf value is 2 and 2 and 8985 */
    ExtLog3(INFO, EXT_MODULE_DRV_UART, "printf INFO level log%d %d %d\n", 2, 2, 8985);
}

/**
  * @brief Test on the number of parameters at different levels in the dfx module.
  * @retval None.
  */
static void VerifyLogOutDfx(void)
{
    ExtLog0(FATAL, EXT_MODULE_DFX, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_DFX, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 245 */
    ExtLog2(WARN, EXT_MODULE_DFX, "printf WARN level log%d %d\n", 1, 245);
    /* printf value is 2 and 2 and 654654 */
    ExtLog3(INFO, EXT_MODULE_DFX, "printf INFO level log%d %d %d\n", 2, 2, 654654);
}

/**
  * @brief Test on the number of parameters at different levels in the butt module.
  * @retval None.
  */
static void VerifyLogOutButt(void)
{
    ExtLog0(FATAL, EXT_MODULE_BUTT, "printf FATAL level log\n");
    ExtLog1(ERR, EXT_MODULE_BUTT, "printf ERR level log %d\n", 0);
    /* printf value is 1 and 8524 */
    ExtLog2(WARN, EXT_MODULE_BUTT, "printf WARN level log%d %d\n", 1, 8524);
    /* printf value is 2 and 2 and 99554 */
    ExtLog3(INFO, EXT_MODULE_BUTT, "printf INFO level log%d %d %d\n", 2, 2, 99554);
}

/**
  * @brief Log output test of different levels on different modules.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int VerifyLogOut(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);
    /* App module */
    VerifyLogOutAppMain();
    VerifyLogOutAppConsole();
    VerifyLogOutAppChip();
    /* Drive module */
    VerifyLogOutDrvBase();
    VerifyLogOutDrvChip();
    VerifyLogOutDrvCRg();
    VerifyLogOutDrvGpio();
    VerifyLogOutDrvI2c();
    VerifyLogOutDrvIrq();
    VerifyLogOutDrvPinctrl();
    VerifyLogOutDrvTimer();
    VerifyLogOutDrvUart();
    /* Other modules */
    VerifyLogOutDfx();
    VerifyLogOutButt();
    return 0;
}

/**
  * @brief Test the log output cache of a specified size.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int VerifyLogOutBuf(unsigned int argc, const char *argv[])
{
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    unsigned int buf[10]; /* test out buf len is 10 */
    buf[0] = 0x81; /* test buf[0] data is 0x81 */
    buf[1] = 0x82; /* test buf[1] data is 0x82 */
    buf[2] = 0x83; /* test buf[2] data is 0x83 */
    buf[3] = 0x84; /* test buf[3] data is 0x84 */
    buf[4] = 0x85; /* test buf[4] data is 0x85 */
    buf[5] = 0x86; /* test buf[5] data is 0x86 */
    buf[6] = 0x87; /* test buf[6] data is 0x87 */
    buf[7] = 0x88; /* test buf[7] data is 0x88 */
    buf[8] = 0x89; /* test buf[8] data is 0x89 */
    buf[9] = 0x90; /* test buf[9] data is 0x90 */
    ExtLogBuf(FATAL, EXT_MODULE_DRV_PINCTRL, "printf out buf log 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
              buf, 10); /* test out buf len is 10 */
    return 0;
}

/**
  * @brief Enable the debugging function by running commands.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int CmdGetDebugModeEnable(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    struct SysDebugSwitch *debugSwitch = GetDebugSwitch();
    /* Configure the debugging mode. */
    ExtSetDebugMode(DEBUG);
    EXT_PRINT("debug_enable : %d\n", debugSwitch->enable);
    return 0;
}

/**
  * @brief Disable the debugging function by running commands.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int CmdGetDebugModeDisable(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    struct SysDebugSwitch *debugSwitch = GetDebugSwitch();
    /* Configure the running mode. */
    ExtSetDebugMode(RUNNING);
    EXT_PRINT("debug_disable : %d\n", debugSwitch->enable);
    return 0;
}

/**
  * @brief Configuring the Debug Level by Using Commands.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int SetLogLevel(unsigned int argc, const char *argv[])
{
    BASE_FUNC_UNUSED(argc);
    int id = (int)*argv[1] - 48; /* argv[1] is a number in ASCII format, which needs to be subtracted by 48. */
    int level = (int)*argv[2] - 48; /* argv[2] is a number in ASCII format, which needs to be subtracted by 48. */
    if (!ExtDrvLogSetLogLevel(id, level)) {
        EXT_PRINT("set success\r\n");
    } else {
        EXT_PRINT("set failed!\r\n");
    }
    return 0;
}

#define TEST_LOAD_ADDR  0x2000000  /* Test loading address 0x2000000 */

/**
  * @brief Writes logs to the specified address.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int GetAddressWrite(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    char value[] = {'h', 'e', 'l', 'l', 'o', '!', '\0'};
    unsigned int len = sizeof(value) / sizeof(char);
    EXT_PRINT("value: %s\n", value);
    EXT_PRINT("len = %d\n", len);
    /* Writes logs to the specified address. */
    ExtLoadWrite(TEST_LOAD_ADDR, value, len);
    return 0;
}

/**
  * @brief Reads logs from a specified address.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int GetAddressRead(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    char value[7] = {0}; /* test value len is 7 */
    unsigned int len = 7; /* test value len is 7 */

    ExtLoadRead(TEST_LOAD_ADDR, value, len);
    EXT_PRINT("value1: %s\n", value);
    return 0;
}

#define TEST_WRITE_ARR_MAX_LEN    49

/**
  * @brief Write Configuration.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int GetConfigWrite(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    char configWriteArr[] = {'h', 'e', 'l', 'l', 'o', '!', 'a', 'b', 'c', 'd',
                             'e', 'f', 'g', 'e', 'h', 'i', 'j', 'k', 'l', '!',
                             '@', '#', '$', '&', 'h', 'e', 'l', 'l', 'o', '!',
                             'a', 'b', 'c', 'd', 'e', 'f', 'g', 'e', 'h', 'i',
                             'j', 'k', 'l', '!', '@', '#', '$', '&', '\0'};
    char configReadArr[TEST_WRITE_ARR_MAX_LEN] = {0};
    char *vbuff = configWriteArr;

    EXT_PRINT("configWriteArr: %s\n", configWriteArr);
    /* Writes the configuration data to the specified address. */
    ExtConfigWrite(DATA_ITEM_EVENT, vbuff, TEST_WRITE_ARR_MAX_LEN);
    /* Read the configuration data from the specified address. */
    ExtConfigRead(DATA_ITEM_EVENT, configReadArr, TEST_WRITE_ARR_MAX_LEN);
    EXT_PRINT("read: %s\n", configReadArr);
    return 0;
}

/**
  * @brief Test report.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int TestReport(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    UserEventObj eventObj;
    eventObj.report.event.eventType = 1;
    eventObj.report.reportType = 1;
    /* Initializing event reporting */
    EventInit();
    UserReportEvent(&eventObj);
    return 0;
}

/**
  * @brief Run commands to obtain version information.
  * @param argc argument count.
  * @param argv argument vector.
  * @retval 0 is success.
  */
static int TestCmdGetVersionInfo(unsigned int argc, const char *argv[])
{
    /* The input parameter is not used. */
    BASE_FUNC_UNUSED(argc);
    BASE_FUNC_UNUSED(argv);

    CmdGetVersionInfo();
    return 0;
}

/**
  * @brief The sample executes the main function.
  * @retval None.
  */
void DFX_SampleMain(void)
{
    /* Chip Initialization Configuration */
    SystemInit();
    ConsoleInit(g_uart0);

    /* Registering Basic Interfaces */
    ExtCmdRegister("cmdregister", &TestCmdRegister);
    ExtCmdRegister("getversion", &TestCmdGetVersionInfo);
    ExtCmdRegister("verifyloglevel", &VerifyLogLevel);
    ExtCmdRegister("verifylogout", &VerifyLogOut);
    ExtCmdRegister("verifylogoutbuf", &VerifyLogOutBuf);

    ExtCmdRegister("userreport", &TestReport);

    /* Registration of configuration interfaces */
    ExtCmdRegister("getdebugenable", &CmdGetDebugModeEnable);
    ExtCmdRegister("getdebugdisable", &CmdGetDebugModeDisable);
    ExtCmdRegister("setloglevel", &SetLogLevel);

    /* Address operation interface registration */
    ExtCmdRegister("addresswrite", &GetAddressWrite);
    ExtCmdRegister("addressread", &GetAddressRead);
    ExtCmdRegister("configwrite", &GetConfigWrite);

    while (1) {
        ExtAppCmdProcess();
    }
}