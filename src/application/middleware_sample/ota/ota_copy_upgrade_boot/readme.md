# MCU芯片 OTA Copy Upgrade Boot（拷贝升级Boot）参考示例
## 关键字: OTA，Copy upgrade，升级，Boot

**【功能描述】**
+ OTA是一种通过无线通信实现远程更新设备固件的方法。MCU OTA为通过外部具有无线通信的器件接收新固件数据，外部器件将新固件数据通过MCU串行通信接口发送给MCU，实现固件更新。MCU的OTA主要工作是将串行通信接口接收的新固件数据更新到用户指定的程序区，完成固件的升级。
+ OTA的拷贝升级在功能上包括1个user boot flash分区、1个APP运行flash分区、1个APP备份flash分区、1个参数存储flash区。user boot flash分区具有判断、拷贝、跳转功能。APP运行flash分区具有升级功能、业务逻辑处理功能。APP备份flash分区用于存储新的固件，不运行。参数存储flash区用于存储升级相关配置参数。User boot分区与APP运行分区在flash上进行严格划分，由boot根据参数存储区的参数配置决定是否将APP备份flash分区数据拷贝到APP运行flash分区，并跳转执行。
+ flash空间使用：
   1.boot flash分区：起始地址0x3000000，大小16K;
   2.参数存储flash分区：起始地址0x301FC00，大小1K;

**【示例配置】**

+ 配置flash阻塞方式进行读取参数存储区数据，用于判断boot是否执行APP数据拷贝和跳转地址。


**【示例效果】**
+ 当用户复位芯片后，MCU首先会执行boot程序，boot读取参数存储区数据，判断获取跳转APP运行地址和拷贝标志，进行拷贝（根据拷贝标志决定）、跳转。

**【注意事项】**
+ 升级模型需开启中断服务函数基地址不锁定宏，保证APP中断正常执行。
+ 此示例init文件夹下的工程初始化配置是基于3066MNPINH芯片工程生成的。
+ 不通过IDE创建生成场景：初始化配置请参考sample下的init文件内容。
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。