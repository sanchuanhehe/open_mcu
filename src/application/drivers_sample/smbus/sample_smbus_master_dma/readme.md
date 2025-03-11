# 配置SMBus作为主机以DMA方式进行数据的接收、发送，通过Debug串口打印数据收发的结果
## 关键字: SMBus，主机，数据读取，数据发送，DMA方式

**【功能描述】**
+ 示例代码基于HAL接口完成时钟、SMBus控制器初始化和功能配置。通过中断方式与EEPROM(AT24C64)进行数据的接收和数据的发送，通过Debug串口打印数据收发成功或失败的信息。

**【示例配置】**
+ SMBus控制器选择：选择SMBus实现与EEPROM的数据接收和发送。

+ SMBus初始化：调用接口"HAL_SMBUS_Init()”完成对示例代码中SMBus的基地址、主从模式、寻址模式（7bit寻址或10bit寻址）、通讯速率、DMA通道等参数进行配置。调用"HAL_SMBUS_RegisterCallback()"接口注册收发完成、传输错误时的回调函数。

+ 操作SMBus写入和读取EEPROM的内容：通过调用 "HAL_SMBUS_MasterWriteDMA()"接口以DMA方式向EEPROM目标内存地址写入数据；通过调用"HAL_SMBUS_MasterReadDMA()"接口以DMA方式实现从EEPROM的目标地址中接收数据。

**【示例效果】**
+ 当用户烧录编译后的示例代码后，初始化和配置完成后，示例代码中会通过SMBus以DMA方式向EEPROM写入和读取写入的内容，Debug串口打印SMBus与EEPROM进行通信交互的结果信息，当写入和读取的数据一致时Debug串口打印"SMBUS Data Success"；当写入和读取的数据不一致时Debug串口打印错误信息。

**【注意事项】**
+ 示例代码使用UART0进行结果打印输出，需要对UART0配置。
+ 示例代码中SMBus必须必须对接EEPROM(AT24C64)，且保证EEPROM的7bit地址为0x50，否则示例代码无法正常运行。

+ 此示例init文件夹下的工程初始化配置是基于3066MNPINH芯片工程生成的。
+ 不通过IDE创建生成场景：初始化配置请参考sample下的init文件内容。
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。