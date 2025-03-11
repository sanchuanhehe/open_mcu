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
  * @file    res_en.h
  * @author  MCU Driver Team
  * @brief   resource string of english header
  */
#ifndef RES_EN_H
#define RES_EN_H

#define DBG_STATUS_EN          "1.Debug Status"
#define FACTORY_MODE_EN        "2.Factory Flasher"
#define FLASHER_MODE_EN        "3.Flasher Mode"
#define FLASHER_ERASE_EN       "4.Flasher Erase"
#define FLASHER_UNLOCK_EN      "5.Flash Protection Off"
#define SETTING_EN             "6.Setting"

#define IMAGE_SELECT_EN        "1.Upgrade Target Image"
#define IMAGE_DELETE_EN        "2.Delete Target Image"
#define ALGO_SELECT_EN         "3.Select Flash Algorithm"
#define ALGO_DELETE_EN         "4.Delete Flash Algorithm"
#define ENABLE_MULTI_EN        "5.Enable Multiple Copies"

#define FACTORY_IMAGE_LOAD_EN  "1.Upgrade Target Image"
#define FACTORY_CRC_CHECK_EN   "2.Check Target Image"
#define FACTORY_VIEW_EN        "3.View"
#define FACTORY_DELETE_EN      "4.Delete"
#define FACTORY_EXIT_EN        "5.Exit"

#define LANGUAGE_SELECT_EN     "1.Language"
#define POWER_SUPPLY_EN        "2.Power Supply"
#define DBG_INTERFACE_EN       "3.Debug Interface"
#define FW_VER_EN              "4.Firmware: "
#define FW_UPGRADE_EN          "5.Firmware Upgrade"
#define HW_VER_EN              "6.Hardware: "
#define SN_EN                  "7.SN:"
#define HS_SAMPLE_SIMU_EN      "8.HS Sampling Simulation"
#define RESTORE_EN             "9.Restore Factory"

#define OLD_DBG_INTERFACE_EN   "2.Debug Interface"
#define OLD_FW_VER_EN          "3.Firmware: "
#define OLD_FW_UPGRADE_EN      "4.Firmware Upgrade"
#define OLD_HW_VER_EN          "5.Hardware: "
#define OLD_SN_EN              "6.SN:"
#define OLD_HS_SAMPLE_SIMU_EN  "7.HS Sampling Simulation"
#define OLD_RESTORE_EN         "8.Restore Factory"

#define BACK_EN                "Back"
#define TARGET_NOT_CONNECT_EN  "Target Not Connected"
#define CONNECTED_EN           "Connected"
#define TARGET_IS_RUNNING_EN   "Target is running"
#define TARGET_IS_HALT_EN      "Target is halt"
#define TARGET_IS_RESET_EN     "Target is reset"

#define IMAGE_STORED_EN        "Image Stored!"
#define IMAGE_FAIL_EN          "Image Fail!"
#define MULTI_COPIES_FAIL_EN   "Can't Store Multiple Copies"
#define IMAGE_CRC_FAIL_EN      "Image CRC Fail!"

#define ALGO_STORED_EN         "Alogrithm Stored!"
#define ALGO_DEFAULT_EN        "Set as default"
#define ALGO_STORED_FAIL_EN    "Algorithm Store Fail!"
#define ALGO_STORED_MULTI_COPIES_FAIL_EN    "Algorithm Store Fail, Can't Store Multiple Copies"
#define ALGO_CRC_FAIL_EN       "Algorithm CRC Fail!"

#define FW_UPGRADEING_EN       "Firmware Upgrade..."
#define FW_UPGRADE_SUCC_EN     "Upgrade Success!"
#define FW_CRC_FAIL_EN         "Firmware CRC Fail!"

#define DAP_NOT_SAMPLING_EN    "Not Working"
#define DAP_DEBGU_EN           "Debuging"
#define DAP_SAMPLING_EN        "Sampling"
#define DAP_TARGET_STOP_EN     "Target Stop Sampling"
#define DAP_PC_STOP_EN         "PC Stop Sampling"


#define DAP_SAMPLE_CNT_EN      "Sampling:"
#define DAP_SAMPLE_ERROR_EN    "Error:"
#define DAP_ERROR_RATE_EN      "ErrorRate:"
#define DAP_OVERFLOW_CNT_EN    "OverFlow:"

#define SELECTED_EN            "(Seleted)"

#define DELETE_ALL_EN          "1.Delete All"

#define ON_EN                  "1.On"
#define OFF_EN                 "2.Off"

#define LANGUAGE_SELECT_EN     "1.Language"
#define FW_UPGRADE_MSG_EN      "Please download firmware to this probe by PC tool"
#define GET_MORE_IMAGE_EN      "Please download Image to this probe by PC tool"
#define GET_MORE_ALGO_EN       "Please download Algorithm to this probe by PC tool"
#define GET_MORE_IMAGE_ALGO_EN "Please download Image and Algorithm to this probe by PC tool"

#define GET_FLASH_ALGO_EN      "Please download Algorithm to this probe by PC tool"
#define NO_FLASH_ALGO_EN       "Flash drive missing"

#define DEBUGGER_RST_EN        "Factory Reset..."
#define DEBUGGER_RST_SUCC_EN   "Factory Reset Done"

#define CHINESE_EN             "ÖÐÎÄ"
#define ENGLISH_EN             "English"

#define TARGET_UPGRADE_SUCC_EN   "Target Upgrade Success"
#define TARGET_VERIFY_FAIL_EN    "Target Verify Fail"
#define TARGET_WRITE_FAIL_EN     "Target Write Fail"
#define TARGET_GET_IMAGE_FAIL_EN "Get Image Fail"
#define TARGET_VERIFYING_EN      "Target Verifying..."
#define TARGET_PROGRAMMING_EN    "Target Programing..."
#define TARGET_GET_ALGO_FAIL_EN  "The Corresponding Algorithm was not found, please check whether it exists"
#define TARGET_CONNECTED_FAIL_EN "Target Connected Fail"
#define TARGET_NO_CONNECTED_EN   "Target Not Connected"
#define TARGET_IMAGE_ERROR_EN    "Image Error"
#define TARGET_ALGO_ERROR_EN     "Algorithm Error"
#define IMAGE_UPGRADEING_EN      "Image Upgrading..."
#define TARGET_RUN_ALGO_FAIL_EN  "Run algo fail"
#define WELCOME_EN               "Welcome to HiSpark-Trace"
#define TARGET_IMAGE_MAX_LOADS_ERROR_EN "The number of burning times exceeds the maximum"

#define DELETEING_EN            "Deleting..."
#define DELETE_SUCCESS_EN       "Delete Success"
#define DELETE_FAIL_EN          "Delete Fail"

#define ERASEING_EN             "Erasing..."
#define ERASE_SUCCESS_EN        "Erase OK"
#define ERASE_FAIL_EN           "Erase Fail"

#define FACTORY_MODE_TITLE_EN   "      Factory Mode      "

#define MENU_EN                 "Menu"
#define CONNECT_RATE_EN         " Rate:"
#define CRC_EXPECT_EN           "Expected CRC:"
#define CRC_ACTUAL_EN           "Actual CRC:    "
#define CRC_CHK_LENGTH_EN       "Chk Length:"
#define CRC_CHK_ADDR_EN         "Chk Addr:"
#define CRC_CHK_SUCC_EN         "     CRC Is Correct     "
#define CRC_CHK_FAIL_EN         "     Incorrect CRC      "

#define END_OF_LIST_EN          "Reached the end of the list"
#define GET_IMAGE_FAIL_EN      "Get Target Image Fail"

#define IMAGE_EN               "Image: "
#define ALGO_EN                "Algorithm: "
#define EMPTY_EN               "<Empty>"
#define NO_IMAGE_EN            "Image:<Empty>"
#define NO_ALGO_EN             "Algorithm:<Empty>"

#define MISS_IMAGE_EN           "Miss Image"
#define MISS_ALGO_EN            "Miss Algorithm"
#define MISS_IMAGE_ALGO_EN      "Miss Image and Algorithm"

#define IMAGE_ERROR_EN          "Image Error"
#define ALGO_ERROR_EN            "Algorithm Error"
#define IMAGE_ALGO_ERROR_EN      "Image and Algorithm Error"

#define IMAGE_ALGO_NO_MATCH_EN  "Image and Algorithm not match"

#define FACTORY_DEL_IMAGE_ALGO_EN "Delete Image and Algorithm"
#define FACTORY_DEL_IMAGE_EN      "Delete Image:"
#define FACTORY_DEL_ALGO_EN       "Delete Algo:"

#define TIMES_EN                  ""
#define FILE_ERR_EN               "CRC is incorrect"
#define FILE_ERROR_EN             "(Bad)"

#define DELETE_ALL_TITLE_EN       "       Delete All       "
#define FACTORY_RESET_TITLE_EN    "      Factory Reset     "
#define YES_EN                    "1.Yes"
#define NO_EN                     "2.No"

#define IMAGE_ALGO_NO_MATCH_TITLE_EN "Image and Algo Not Match"
#define IMAGE_RESTRICTED_TITLE_EN    " Prohibit Target Upgrade"

#define CONTINUE_PROGRAM_EN       "Program Continue"
#define CHIP_ERASE_TITLE_EN       "       Chip Erase      "
#define READ_CRC_TITLE_EN         "        Read CRC       "

#define FLASH_PROTECT_OFF_RUNING_EN " Flash Protection Off..."
#define FLASH_PROTECT_OFF_EN      "  Flash Protection Off  "
#define FLASH_PROTECT_OFF_SUCC_EN " Flash Protection Off OK"
#define FLASH_PROTECT_OFF_FAIL_EN " Flash Protection Off Fail "

#define DOWNLOAD_EN               "Cur"
#define PERIMIT_EN                "Max"

#define SWD_EN               "SWD"
#define JTAG_EN              "JTAG"
#endif
