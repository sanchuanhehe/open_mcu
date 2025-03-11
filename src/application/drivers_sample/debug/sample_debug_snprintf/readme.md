# 格式化缓存输出snprintf用法对比介绍
## 关键字: snprintf, C标准库， 安全函数库， DBG_SNPRINTF

**【功能描述】**
+ 示例代码基于安全函数封装的snprintf_s接口以及基于SDK封装的DBG_SNPRINTF宏函数对同一组数据进行测试比较，可通过log查看接口使用的正确性和通过最终生成的bin文件查看调用不同的snprintf接口时的内存占用，供用户不同场景性能与安全需求时进行选择。

**【示例配置】**
+ SNPRITF_SELECT：该宏定义由用户定义，可选择测试不同的snprintf接口函数。

+ NOT_USE： 即三种snprintf接口均不调用。

+ SECUREC_LIB： 调用安全函数库提供的snprintf_s功能，用于安全要求高的场景，对应的内存占用也更大。

+ DEBUG_LIB： 调用SDK debug库提供的DBG_SNPRINTF功能，内存占用最小。

+ 打印结果：不同snprintf接口对同一组格式化缓存输出的接口打印结果相同。

**【示例效果】**
+ 不同snprintf接口对同一组格式化缓存输出的接口打印结果相同。

**【注意事项】**
+ 示例代码使用UART0进行结果打印输出，需要对UART0进行初始化配置，且需使能DEBUG功能，DBG_SNPRINTF和DBG_PRINTF内部实现有共用部分接口。
+ 示例代码使用UART0进行结果打印输出，需要对UART0进行初始化配置。
+ 此示例init文件夹下的工程初始化配置是基于3061MNPICA芯片工程生成的。
+ 不通过IDE创建生成场景：初始化配置请参考sample下的init文件内容；
+ 通过IDE创建生成场景：初始化配置请参考user/generatecode下的文件内容。