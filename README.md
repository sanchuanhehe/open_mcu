# open_mcu开发指南

## 介绍

  open_mcu代码仓为支持3061M和3065M解决方案SDK。

## 购买渠道

|    名称    | 硬件资料                                                     | 介绍              |
| :--------: | ------------------------------------------------------------ | ----------------- |
| 3061MNPIKA | [购买链接](https://www.ickey.cn/detail/1003001013207471/3061MNPIKA.html#00d92d22-8a7b-43f5-a338-c16634462049) | 3061M系列解决方案 |
| 3061MNPICA | [购买链接](https://www.ickey.cn/detail/1003001013207477/3061MNPICA.html#8b9f1c65-2326-40b5-91be-f7ac10bb890a) | 3061M系列解决方案 |
| 3061MNNICA | [购买链接](https://www.ickey.cn/detail/1003001013207473/3061MNNICA.html#909633a3-1823-41f3-aec5-2baf839e8582) | 3061M系列解决方案 |
| 3061MNNIKA | [购买链接](https://www.ickey.cn/detail/1003001013207474/3061MNNIKA.html#878faa99-3c91-4673-acdf-328557b0dbdb) | 3061M系列解决方案 |
| 3065HRPIRZ | [购买链接](https://www.ickey.cn/detail/1003001013207476/3065HRPIRZ.html#84b508e7-3eac-4f2b-943a-4aea2f0cc901) | 3065H系列解决方案 |

<img src="docs/pic/readme/1732937293142.png" alt="1732937293142" style="zoom:50%;" /><img src="docs/pic/readme/1732937308965.png" alt="1732937308965" style="zoom: 50%;" />

## 支持的开发板

|                 开发板名称                 | 硬件资料                                                     | 软件资料     | 购买链接                                                     | 开发板介绍                                                   |
| :----------------------------------------: | ------------------------------------------------------------ | :----------- | :----------------------------------------------------------- | :----------------------------------------------------------- |
| ECBMCU201MPC![](docs/pic/readme/3061M.png) | [开发板硬件原理图](https://gitee.com/HiSpark/open_mcu/tree/master/docs/hardware/3061M) | 参考示例教程 | [开发板购买链接](https://m.tb.cn/h.gMEbHlepTLs5DNB?tk=aKIe356U3bY) | Hi3061M是针对家电、工业等领域设计的高性价比MCU，使用海思自有RISC-V内核，支持150MHz主频，支持AI场景扩展；支持32KB SRAM和128KB 内置Flash，可支持客户产品功能持续迭代和算法升级；可应用于冰洗空、电动自行车、高速风机、电动工具、按摩椅等比较广泛的应用场景。 |
|  ECBMCU105H![](docs/pic/readme/3061M.png)  | [开发板硬件原理图](https://gitee.com/HiSpark/open_mcu/tree/master/docs/hardware/3065H) | 参考示例教程 | [开发板购买链接](https://m.tb.cn/h.gMEbHlepTLs5DNB?tk=aKIe356U3bY) | Hi3065H是基于海思自研RISC-V内核的高性能实时控制专用MCU， 具有高性能、高集成度、高可靠性、易开发的特点，同时配合海思强大的算法团队和嵌入式AI能力，使得Hi3065H上市后快速获得家电、能源、工业等行业内多个客户的认可和好评。 |
## 目录介绍

| 目录   | 介绍                   |
| ------ | ---------------------- |
| docs   | 硬件原理图             |
| src    | SDK源码目录            |
| tools  | 开发工具及环境搭建指南 |
| vendor | 存放对应开发板案例     |

## 硬件介绍

### 3061M介绍

3061M系列 生态板由 ECBMCU201MPC（核心板）和 ECBMOTORA（电机驱动板组成）。

ECBMCU201MPC是针对 3061M系列 MCU开发的生态核心板，用于 3061M初始评估和设计参考，内嵌一块 USB接口的调试板。

ECBMOTORA是电机驱动扩展板，支持一个 BLDC或 PMSM电机控制。该单板支持24V/12V DCIN输入。

核心板电机驱动扩展板的常用组装方式是电机驱动板通过两个40pin连接器扣接到核心板，如下图所示。

![image-20240715162059747](docs/pic/tools/image-20240715162059747.png)

### 3061M硬件说明

 3061M通用生态板通过 ECBMCU201MPC核心板实现控制、 ECBMOTORA 扩展板实现接口扩展以及电源接口，同时提供USB TypeC线进行调试 / 供电、12V电源适配器和一个电机。

![image-20240715162244103](docs/pic/tools/image-20240715162244103.png)

3061M通用生态板用户手册详细内容请查阅：<a href="https://gitee.com/HiSpark/open_mcu/tree/master/src/document/hardware/2.%203061/1.%203061%20%E7%94%9F%E6%80%81%E6%9D%BF%E5%8F%82%E8%80%83%E8%AE%BE%E8%AE%A1" title="超链接title">Hi3061M系列 通用生态板用户手册 00B01</a>

### 3065H介绍

3065H 通用生态板由 ECBMCU105H （核心板）和 ECBMOTORA （电机驱动板）组成。

ECBMCU105H是针对 3065H 芯片开发的生态核心板，用于 3065H 芯片初始评估和设计参考，内嵌一块 USB 接口的调试板。

ECBMOTORA是电机驱动扩展板，支持一个 BLDC 或 PMSM 电机控制。该单板支持24V/12V DCIN 输入。

核心板电机驱动扩展板的常用组装方式是电机驱动板通过两个40pin 连接器扣接到核心板，如下图所示。

<img src="docs/pic/tools/image-20240530173305431.png" alt="image-20240530173305431" style="zoom:80%;" />

### 3065H硬件说明

 3065H通用生态板通过ECBMCU105H 核心板实现控制、 ECBMOTORA 扩展板实现接口扩展以及电源接口，同时提供USB TypeC线进行调试 / 供电、12V电源适配器和一个电机。

![image-20240527103127826](docs/pic/tools/image-20240527103127826.png)  

3065H通用生态板用户手册详细内容请查阅：<a href="https://gitee.com/HiSpark/open_mcu/tree/master/src/document/hardware/1.%203065" title="超链接title">Hi3065H通用生态板用户手册 V03</a>

## 开发环境搭建

[参考tools目录README搭建环境](https://gitee.com/HiSpark/open_mcu/tree/master/tools)

## Demo

3061M/3065H提供了以下Demo供开发参考，sample存放路径：[application_sample](https://gitee.com/HiSpark/open_mcu/tree/master/src/application)

**主目录结构说明**

| 文件夹名          | 描述           |
| ----------------- | -------------- |
| board\_sample     | 开发板示例。   |
| drivers_sample    | 驱动程序示例。 |
| middleware_sample | 中间件示例。   |
| user              | 用户相关。     |

**表 1  board\_sample目录结构说明**

| **文件夹名** | **描述**                   |
| ------------ | -------------------------- |
| dimming      | 呼吸灯功能示例。           |
| key          | 按键检查功能示例。         |
| led          | 数码管功能示例。           |
| pulses       | gpio发送pwm波功能示例。    |
| softserial   | gpio实现串口通信功能示例。 |

**表 2  acmp目录结构说明**

| **文件夹名** | **描述**         |
| ------------ | ---------------- |
| sample_acmp  | 比较器使用示例。 |

**表 3  adc目录结构说明**

| **文件夹名**                          | **描述**            |
| ------------------------------------- | ------------------- |
| sample_adc_associative_trigger_of_apt | APT触发ADC。        |
| sample_adc_continue_trigger           | ADC连续采样。       |
| sample_adc_over_sample                | ADC过采样。         |
| sample_adc_single_trigger             | ADC单次采样。       |
| sample_adc_single_trigger_dma         | ADC单次采样带DMA。  |
| sample_adc_single_trigger_it          | ADC单次采样带中断。 |
| sample_adc_sync_sample                | ADC同步采样。       |
| sample_adc_sync_sample_dma            | ADC同步采样带DMA。  |
| sample_adc_sync_sample_it             | ADC同步采样带中断。 |

**表 4  apt目录结构说明**

| **文件夹名**               | **描述**                                          |
| -------------------------- | ------------------------------------------------- |
| sample_apt_single_resistor | APT单电阻采样示例，仅在U相触发ADC采样信号。       |
| sample_apt_three_resistor  | APT三电阻采样示例，在U、V和W相都触发ADC采样信号。 |

**表 5  can目录结构说明**

| **文件夹名**            | **描述**                |
| ----------------------- | ----------------------- |
| sample_can_send_receive | CAN发送和接收数据示例。 |

**表 6  capm目录结构说明**

| **文件夹名**     | **描述**                   |
| ---------------- | -------------------------- |
| capm_hall_sample | CAPM读取霍尔传感器值示例。 |

**表 7  cfd目录结构说明**

| **文件夹名**           | **描述**                              |
| ---------------------- | ------------------------------------- |
| sample_cfd_check_error | cfd注入错误前后监测目标时钟异常功能。 |

**表 8  cmm目录结构说明**

| **文件夹名**           | **描述**                              |
| ---------------------- | ------------------------------------- |
| sample_cmm_check_error | cmm注入错误前后监测目标时钟异常功能。 |

**表 9  crc目录结构说明**

| **文件夹名**     | **描述**                                                 |
| ---------------- | -------------------------------------------------------- |
| sample_crc_check | 测试CRC不同算法和输入有效位宽，生成并校验crc值。         |
| sample_crc_gen   | 计算并生成CRC数值。                                      |
| sample_crc_load  | 通过load初始值将xmodem算法改为ccit-false算法并校验结果。 |

**表 10  dac目录结构说明**

| **文件夹名** | **描述**                |
| ------------ | ----------------------- |
| sample_dac   | DAC电压输出到管脚示例。 |

**表 11  dma目录结构说明**

| **文件夹名**                      | **描述**                  |
| --------------------------------- | ------------------------- |
| sample_dma_list_transfer          | DMA链式传输。             |
| sample_dma_list_transfer_continue | DMA链式传输实现连续功能。 |
| sample_dma_mem_to_mem             | DMA内存到内存传输。       |
| sample_dma_mem_to_per             | DMA内存到外设传输。       |
| sample_dma_per_to_mem             | DMA外设到内存传输。       |
| sample_dma_per_to_per             | DMA外设到外设传输。       |

**表 12  flash目录结构说明**

| **文件夹名**           | **描述**            |
| ---------------------- | ------------------- |
| sample_flash_blocking  | 阻塞模式操作flash。 |
| sample_flash_interrupt | 中断方式操作flash。 |

**表 13  gpio目录结构说明**

| **文件夹名**          | **描述**                     |
| --------------------- | ---------------------------- |
| sample_gpio_circle    | GPIO环回测试电平和方向属性。 |
| sample_gpio_interrupt | 测试GPIO不同中断类型。       |
| sample_gpio_key       | GPIO用作按键功能。           |
| sample_gpio_led       | GPIO周期控制led亮灭功能。    |

**表 14  gpt目录结构说明**

| **文件夹名**         | **描述**         |
| -------------------- | ---------------- |
| sample_gpt_simplerun | gpt产生PWM波形。 |

**表 15  i2c目录结构说明**

| **文件夹名**                | **描述**                       |
| --------------------------- | ------------------------------ |
| sample_i2c_blocking_stlm75  | 使用阻塞的方式读写温度传感器。 |
| sample_i2c_interrupt_stlm75 | 使用中断的方式读写温度传感器。 |
| sample_i2c_dma_stlm75       | 使用dma方式读写温度传感器。    |

**表 16  iocmg目录结构说明**

| **文件夹名**  | **描述**                            |
| ------------- | ----------------------------------- |
| iolist_sample | iocmg初始化管脚列表的属性配置功能。 |

**表 17  pga目录结构说明**

| **文件夹名**              | **描述**              |
| ------------------------- | --------------------- |
| sample_pga                | PGA内部电阻放大示例。 |
| sample_pga_extra_resistor | PGA外部电阻放大示例。 |

**表 18  pmc目录结构说明**

| **文件夹名**      | **描述**            |
| ----------------- | ------------------- |
| sample_pmc_pvd    | PMC掉电检测示例。   |
| sample_pmc_wakeup | PMC定时器唤醒示例。 |

**表 19  qdm目录结构说明**

| **文件夹名**  | **描述**                        |
| ------------- | ------------------------------- |
| sample_qdm_m  | QDM使用M法读取电机转速的示例。  |
| sample_qdm_mt | QDM使用MT法读取电机转速的示例。 |

**表 20  spi目录机构说明**

| **文件夹名**                 | **描述**                          |
| ---------------------------- | --------------------------------- |
| sample_spi_blocking_kta7953  | 使用阻塞方式读写ADC。             |
| sample_spi_dma_kta7953       | 使用dma方式读写ADC。              |
| sample_spi_interrupt_kta7953 | 使用中断方式读写ADC。             |
| sample_spi_microwire_master  | 演示如何使用microwire master。    |
| sample_spi_microwire_slave   | 演示如何使用microwire slave。     |
| sample_spi_slave             | 演示如何使用motorola spi slaver。 |

**表 21  timer目录结构说明**

| **文件夹名**           | **描述**                              |
| ---------------------- | ------------------------------------- |
| sample_timer_interrupt | timer定时触发中断，执行用户串口打印。 |

**表 22  tsensor目录结构说明**

| **文件夹名**   | **描述**                |
| -------------- | ----------------------- |
| sample_tsensor | tsensor对器件结温采样。 |

**表 23  uart目录结构说明**

| **文件夹名**                             | **描述**                                   |
| ---------------------------------------- | ------------------------------------------ |
| sample_uart_blocking_rx                  | UART阻塞接收。                             |
| sample_uart_blocking_tx                  | UART阻塞发送。                             |
| sample_uart_dma_rx                       | UART带DMA接收。                            |
| sample_uart_dma_tx                       | UART带DMA发送。                            |
| sample_uart_interrupt_tx_after_rx        | UART中断接收数据之后，再中断发送此数据。   |
| sample_uart_interrupt_rx                 | UART中断接收。                             |
| sample_uart_interrupt_tx                 | UART中断发送。                             |
| sample_uart_dma_tx_dma_rx_simultaneously | UART全双工模式，DMA同时发送和接收。        |
| sample_uart_dma_tx_int_rx_simultaneously | UART全双工模式，DMA发送的同时，中断接收。  |
| sample_uart_int_tx_dma_rx_simultaneously | UART全双工模式，中断发送的同时，DMA接收。  |
| sample_uart_int_tx_int_rx_simultaneously | UART全双工模式，中断发送的同时，中断接收。 |
| sample_uart_dma_rx_cyclically_stored     | UART使用DMA循环搬运数据到指定内存。        |
| sample_uart_single_wire_communication    | UART单线通信示例。                         |

**表 24  wdg目录机构说明**

| **文件夹名**      | **描述**                 |
| ----------------- | ------------------------ |
| sample_wdg_reset  | 测试wdg不喂狗复位功能。  |
| sample_iwdg_reset | 测试iwdg不喂狗复位功能。 |

**表 25  middleware\_sample目录机构说明**

| **文件夹名**               | **描述**                                |
| -------------------------- | --------------------------------------- |
| mcs_65ldemo                | 电机控制算法在AD101HDMA_VER.B板的示例。 |
| mcs_65demo                 | 电机控制算法在AD105HDMA_VER.B板的示例。 |
| pmsm_sensorless_1shunt_foc | 永磁同步电机单电阻采样无感FOC应用。     |
| pmsm_sensorless_2shunt_foc | 永磁同步电机双电阻采样无感FOC应用。     |

## **问题与解答**

如果你对项目中的代码或者文档存在疑问, 欢迎在Issues中提出你的问题(别忘了先在FAQ中看一看是否已经有答案了😎). 如果你自己解决了一个了不起的问题, 非常欢迎你把问题和解决方法发到Issues里, 如果你看到别人的问题而你正好有答案, 也欢迎你帮助解答其他人的问题, 所谓"授人玫瑰手有余香"嘛。

## **参与贡献**

我们非常欢迎你能对这个项目提出代码上的改进或扩展, 方法是:

1. Fork 本仓库
2. 下载到本地, 修改, 提交
3. 推送代码
4. 在页面点击 Pull Request

这样我们就能接到你的推送申请。

## **最后的话**

Hispark Studio是一款年轻且处于快速发展的IDE。在使用过程中，你可能会碰到一些棘手的问题，但别担心，你可以尝试多种方法去解决，比如用搜索引擎寻找答案，或者向社区寻求帮助。记住，所有技术大神都是从解决这些问题中成长起来的。我们和其他开发者也会尽力提供帮助。

最后的最后, 欢迎来到Hispark Studio的世界探险!