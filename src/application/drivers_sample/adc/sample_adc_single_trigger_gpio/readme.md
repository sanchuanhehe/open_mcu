# 配置ADC的采样功能，采用GPIO电平翻转的方式进行触发

**【功能描述】**
+ 通过配置GPIO翻转的方式触发ADC采样功能

**【示例配置】**
+ ADC触发源：GPIO。通过配置ADC触发方式为GPIO 进行ADC采样触发。

+ ADC采样源： 挂载到SOC0 外部采样源。SOC的在文件“system_init.c”中配置，SOC可以配置为“ADC_SOC_NUM0~ADC_SOC_NUM15”中任何一个。

+ ADC采样结果：ADC可以选择4个数据完成中断的任一个，配置请见"System_Init()”。在回调函数中调用“HAL_ADC_GetConvResult()”获取结果。
+ 用户需将配置的GPIO2_1（J2.14）管脚与ADC采样触发管脚[GPIO5_1（J1.39）3061MNPICA芯片ECBMCU201M系列生态板]/[GPIO6_3（J1.37）3066MNPIRH芯片ECBMCU301M系列生态板]通过跳线进行连接。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，示例代码中会对ADC进行触发采样。采样成功后，会通过串口打印读取到的ADC采样结果。

**【注意事项】**
+ 示例代码使用UART0进行结果打印输出，需要对UART0配置。
+ 此示例init文件夹下的工程初始化配置是基于3061MNPICA芯片工程生成的,选用3061M 系列通用生态板作为示例对象。
+ 不通过IDE创建生成场景：初始化配置请参考sample下的init文件内容；
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。