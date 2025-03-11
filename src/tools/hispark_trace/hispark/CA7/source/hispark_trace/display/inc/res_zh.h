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
  * @file    res_zh.h
  * @author  MCU Driver Team
  * @brief   resource string of chinese header
  */

#ifndef RES_ZH_H
#define RES_ZH_H

#define DBG_STATUS_ZH          "1.调试状态"
#define FACTORY_MODE_ZH        "2.工厂烧录"
#define FLASHER_MODE_ZH        "3.文件烧录"
#define FLASHER_ERASE_ZH       "4.整片擦除"
#define FLASHER_UNLOCK_ZH      "5.回退Flash保护"
#define SETTING_ZH             "6.系统设置"

#define IMAGE_SELECT_ZH        "1.文件烧录"
#define IMAGE_DELETE_ZH        "2.删除烧录文件"
#define ALGO_SELECT_ZH         "3.选择默认拖拽烧录算法"
#define ALGO_DELETE_ZH         "4.删除烧录算法"
#define ENABLE_MULTI_ZH        "5.允许存储多个版本"

#define FACTORY_IMAGE_LOAD_ZH  "1.烧录"
#define FACTORY_CRC_CHECK_ZH   "2.读CRC"
#define FACTORY_VIEW_ZH        "3.查看"
#define FACTORY_DELETE_ZH      "4.删除"
#define FACTORY_EXIT_ZH        "5.退出"

#define LANGUAGE_SELECT_ZH     "1.语言选择"
#define POWER_SUPPLY_ZH        "2.调试器对外3.3V供电"
#define DBG_INTERFACE_ZH       "3.调试接口"
#define FW_VER_ZH              "4.软件版本: "
#define FW_UPGRADE_ZH          "5.软件升级"
#define HW_VER_ZH              "6.硬件版本: "
#define SN_ZH                  "7.序列号:"
#define HS_SAMPLE_SIMU_ZH      "8.高速采样模拟(IDE调试)"
#define RESTORE_ZH             "9.恢复出厂"

#define OLD_DBG_INTERFACE_ZH   "2.调试接口"
#define OLD_FW_VER_ZH          "3.软件版本: "
#define OLD_FW_UPGRADE_ZH      "4.软件升级"
#define OLD_HW_VER_ZH          "5.硬件版本: "
#define OLD_SN_ZH              "6.序列号:"
#define OLD_HS_SAMPLE_SIMU_ZH  "7.高速采样模拟(IDE调试)"
#define OLD_RESTORE_ZH         "8.恢复出厂"

#define BACK_ZH                "返回"
#define TARGET_NOT_CONNECT_ZH  "目标板接线异常"
#define CONNECTED_ZH           "目标板连接OK"
#define TARGET_IS_RUNNING_ZH   "目标板程序正在运行"
#define TARGET_IS_HALT_ZH      "目标板程序已暂停"
#define TARGET_IS_RESET_ZH     "目标板程序被复位"

#define IMAGE_STORED_ZH        "烧录文件保存成功!"
#define IMAGE_FAIL_ZH          "烧录文件保存失败!"
#define MULTI_COPIES_FAIL_ZH   "不允许存储多个版本"
#define IMAGE_CRC_FAIL_ZH      "文件CRC校验失败!"

#define ALGO_STORED_ZH         "算法保存成功!"
#define ALGO_DEFAULT_ZH        "已设为默认算法"
#define ALGO_STORED_FAIL_ZH    "算法保存失败!"
#define ALGO_STORED_MULTI_COPIES_FAIL_ZH    "算法保存失败,不允许存储多个版本"
#define ALGO_CRC_FAIL_ZH       "算法CRC校验失败!"

#define FW_UPGRADEING_ZH       "升级调试器软件..."
#define FW_UPGRADE_SUCC_ZH     "软件升级成功!"
#define FW_CRC_FAIL_ZH         "软件CRC校验失败!"

#define DAP_NOT_SAMPLING_ZH    "停止采样"
#define DAP_DEBGU_ZH           "调试中"
#define DAP_SAMPLING_ZH        "正在变量采样"
#define DAP_TARGET_STOP_ZH     "单板端异常停止采样"
#define DAP_PC_STOP_ZH         "电脑端异常停止采样"

#define DAP_SAMPLE_CNT_ZH      "采样:"
#define DAP_SAMPLE_ERROR_ZH    "错误:"
#define DAP_ERROR_RATE_ZH      "错误率:"
#define DAP_OVERFLOW_CNT_ZH    "缓存溢出:"

#define SELECTED_ZH            "(已选择)"

#define DELETE_ALL_ZH          "1.删除全部"

#define ON_ZH                  "1.开启"
#define OFF_ZH                 "2.关闭"

#define FW_UPGRADE_MSG_ZH      "请在电脑上用\"文件加载工具\"加载\"调试烧录器软件\"到此烧录器"
#define GET_MORE_IMAGE_ZH      "请在电脑上用\"文件加载工具\"加载脱机烧录文件"
#define GET_MORE_ALGO_ZH       "请在电脑上用\"文件加载工具\"加载烧录算法"
#define GET_MORE_IMAGE_ALGO_ZH "请在电脑上用\"文件加载工具\"加载烧录文件和算法"

#define GET_FLASH_ALGO_ZH      "请在电脑上用\"文件加载工具\"加载驱动算法"
#define NO_FLASH_ALGO_ZH       "缺少目标芯片FLASH驱动"


#define DEBUGGER_RST_ZH        "调试器正在恢复出厂..."
#define DEBUGGER_RST_SUCC_ZH   "调试器恢复出厂完成"

#define CHINESE_ZH             "中文"
#define ENGLISH_ZH             "English"

#define TARGET_UPGRADE_SUCC_ZH  "烧录OK"
#define TARGET_VERIFY_FAIL_ZH   "校验失败"
#define TARGET_WRITE_FAIL_ZH    "烧录失败"
#define TARGET_GET_IMAGE_FAIL_ZH "获取烧录文件失败"
#define TARGET_VERIFYING_ZH     "校验中..."
#define TARGET_PROGRAMMING_ZH   "烧写中..."
#define TARGET_GET_ALGO_FAIL_ZH "获取算法失败，请检查是否有对应算法"
#define TARGET_CONNECTED_FAIL_ZH "目标板未连接"
#define TARGET_IMAGE_ERROR_ZH    "烧录文件已损坏"
#define TARGET_ALGO_ERROR_ZH     "算法已损坏"
#define TARGET_RUN_ALGO_FAIL_ZH  "目标板运行算法失败"
#define IMAGE_UPGRADEING_ZH     "准备中..."
#define WELCOME_ZH              "欢迎使用HiSpark-Trace"
#define TARGET_IMAGE_MAX_LOADS_ERROR_ZH "烧录超出最大次数"

#define DELETEING_ZH            "删除中..."
#define DELETE_SUCCESS_ZH       "删除成功"
#define DELETE_FAIL_ZH          "删除失败"

#define ERASEING_ZH             "整片擦除中..."
#define ERASE_SUCCESS_ZH        "整片擦除OK"
#define ERASE_FAIL_ZH           "整片擦除失败"

#define FACTORY_MODE_TITLE_ZH   "        工厂烧录        "

#define MENU_ZH                 "菜单"
#define CONNECT_RATE_ZH         "速率:"
#define CRC_EXPECT_ZH           "预期CRC:"
#define CRC_ACTUAL_ZH           "实际CRC:"
#define CRC_CHK_LENGTH_ZH       "校验长度:"
#define CRC_CHK_ADDR_ZH         "校验地址:"
#define CRC_CHK_SUCC_ZH         "         CRC正确        "
#define CRC_CHK_FAIL_ZH         "         CRC错误        "

#define END_OF_LIST_ZH          "已到列表末尾"

#define GET_IMAGE_FAIL_ZH       "读目标板文件失败"
#define IMAGE_ZH                "文件:"
#define ALGO_ZH                 "算法:"
#define EMPTY_ZH                "<无>"
#define NO_IMAGE_ZH             "文件:<无>"
#define NO_ALGO_ZH              "算法:<无>"

#define MISS_IMAGE_ZH           "        文件<无>        "
#define MISS_ALGO_ZH            "缺少烧录算法"
#define MISS_IMAGE_ALGO_ZH      "缺少烧录文件和算法"
#define IMAGE_ERROR_ZH          "烧录文件已损坏"
#define ALGO_ERROR_ZH           "算法已损坏"
#define IMAGE_ALGO_ERROR_ZH      "烧录文件和算法已损坏"
#define IMAGE_ALGO_NO_MATCH_ZH  "烧录文件、算法不匹配"

#define FACTORY_DEL_IMAGE_ALGO_ZH "删除文件和算法"
#define FACTORY_DEL_IMAGE_ZH      "删除文件:"
#define FACTORY_DEL_ALGO_ZH       "删除算法:"

#define TIMES_ZH                  "次"
#define FILE_ERR_ZH               "CRC不正确"
#define FILE_ERROR_ZH             "(已损坏)"

#define DELETE_ALL_TITLE_ZH       "        删除全部        "
#define FACTORY_RESET_TITLE_ZH    "        恢复出厂        "

#define YES_ZH                    "1.确认"
#define NO_ZH                     "2.取消"

#define IMAGE_ALGO_NO_MATCH_TITLE_ZH "      缺少以下算法     "
#define IMAGE_RESTRICTED_TITLE_ZH    "以下文件烧录次数达到上限"

#define CONTINUE_PROGRAM_ZH       "继续烧录"
#define CHIP_ERASE_TITLE_ZH       "        整片擦除        "
#define READ_CRC_TITLE_ZH         "          读CRC        "

#define FLASH_PROTECT_OFF_RUNING_ZH "   回退Flash保护中...   "
#define FLASH_PROTECT_OFF_ZH      "     回退Flash保护      "
#define FLASH_PROTECT_OFF_SUCC_ZH "   回退Flash保护成功    "
#define FLASH_PROTECT_OFF_FAIL_ZH "   回退Flash保护失败    "

#define DOWNLOAD_ZH               "已烧"
#define PERIMIT_ZH                "允许"

#define SWD_ZH               "SWD"
#define JTAG_ZH              "JTAG"
#endif
