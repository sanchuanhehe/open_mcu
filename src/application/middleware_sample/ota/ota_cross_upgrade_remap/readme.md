# MCU芯片 OTA Cross Upgrade Remap（交叉升级-remap）参考示例
## 关键字: OTA，cross upgrade，升级，Remap

**【功能描述】**
+ OTA是一种通过无线通信实现远程更新设备固件的方法。MCU OTA为通过外部具有无线通信的器件接收新固件数据，外部器件将新固件数据通过MCU串行通信接口发送给MCU，实现固件更新。MCU的OTA主要工作是将串行通信接口接收的新固件数据更新到用户指定的程序区，完成固件的升级。
+ OTA的交叉升级remap在功能上包括1个user boot flash分区、1个APP运行flash分区、1个参数存储flash区。user boot flash分区具有判断跳转、remap功能。APP运行flash分区具有升级功能、业务逻辑处理功能。参数存储flash区用于存储升级相关配置参数。User boot分区与APP运行分区在flash上未进行严格划分，boot跳转到APP运行的地址是动态获取的。
+ flash空间使用：
   1.boot 和 APP flash分区：起始地址0x3000000，大小255K;
   2.参数存储flash分区：起始地址0x307FC00，大小1K;

**【示例配置】**
+ 配置UART0波特率为115200作为升级速率。

+ 配置CRC使用16bit校验结果，用于校验通信数据正确性。

+ 配置flash阻塞方式进行擦除、编写，用于更新flash存储固件。


**【示例效果】**
+ 当用户复位芯片后，MCU首先会执行boot程序，boot进行判断是否执行remap功能，完成判断后跳转到APP运行地址执行程序。
+ 升级功能是APP完成的，当升级GPIO管脚获取信号后，会进入OTA升级环节。主机在1s内发起与OTA程序握手，握手成功进入固件升级阶段，完成固件升级后，主机下发命令复位MCU。MCU再次重新运行。

**【注意事项】**
+ 示例代码使用UART0作为升级通道，需要硬件预留UART0的升级管脚。
+ 升级模型需开启中断服务函数基地址不锁定宏，保证APP中断正常执行。
+ 此示例init文件夹下的工程初始化配置是基于3066MNPINH芯片工程生成的。
+ 不通过IDE创建生成场景：初始化配置请参考sample下的init文件内容。
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。