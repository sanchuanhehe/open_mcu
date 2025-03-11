# 获取PWM输出状态和POE电平
## 关键字: PWM输出、POE电平

**【功能描述】**
+ 该示例为APT强制动作状态下，读取PWM的输出状态和POE电平；
+ 以及手动触发POE保护后，读取PWM的输出状态和POE电平。

**【示例配置】**
+ 配置APT0输出模式为AHBL，POE0信号高保护。

+ APT保护行为是低电平。

+ 以上配置可通过APT配置界面进行更改，或在“system_init.c”中更改APT对应的配置。

**【示例效果】**
+ APT未发生保护，读取g_testRes1和g_testRes2的结果，若结果为1，说明PWM状态和POE状态正确。

+ 手动触发POE0高电平，APT进入保护模式，APT模块会停止PWM波的输出，读取g_testRes3，若结果为1，说明PWM状态和POE状态正确。
 
**【注意事项】**
+ 此示例init文件夹下的工程初始化配置是基于3066MNPIRH芯片工程生成的。
+ 不通过IDE创建生成场景：初始化配置请参考sample下的init文件内容。
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。