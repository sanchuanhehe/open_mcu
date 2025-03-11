# 通用脉宽调制PWM-周期中断
## 关键字: PWM, 周期中断

**【功能描述】**
+ 该GPT示例中产生连续GPT周期中断。 

**【示例配置】**
+ 每个GPT周期计数完成之后，都会产生周期中断，调用周期中断回调函数，示例样例回调函数为“GPTPWMPeriodOutputFinishCallBack”。

**【示例效果】**
+ 串口0打印GPT输出信息和中断回调函数输出的信息。
 
**【注意事项】**
+ 更改PWM的周期和占空比一般先使用“HAL_GPT_GetConfig”获取GPT的配置，再使用“HAL_GPT_Config”更改GPT的配置。

+ 串口0输出提示信息。

+ ```c输出的PWMPWM频率=工作时钟频率/((分频系数+1)*(计数周期值+1))```

+ 不通过IDE创建生成场景：初始化配置请参考sample下的init文件内容；
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。