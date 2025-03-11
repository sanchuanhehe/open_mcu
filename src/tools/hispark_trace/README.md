# 编译构建依赖

## 工具依赖

+ 编译构建需要依赖make工具和python工具，请确保机器上已安装。
+ 依赖“gcc-arm-none-eabi-10.3-2021.07”交叉编译工具链

## 第三方库依赖

+ DAPLink开源包“DAPLink-0257.tar.gz”DAPLink
+ STM32MP153驱动包“STM32CubeMP1-1.4.0.tar.gz”
+ 安全函数库“libboundscheck_v1.1.16”

# 编译工具链安装与配置

## Windows

+ 解压下载好的编译工具链压缩包"gcc-arm-none-eabi-10.3-2021.07-win32.zip"。
+ 新建环境变量“TOOLCHAIN_DIR”值配置为工具链的根目录。注意目录分隔符。例如：“D:/tools/gcc-arm-none-eabi-10.3-2021.07-win32”。
+ 把工具链bin文件夹路径加入Path环境变量，值为%TOOLCHAIN_DIR%\bin。

## Linux X86_64

+ 解压下载好的编译工具链压缩包“gcc-arm-none-eabi-10.3-2021.07-x86_64-linux.tar.bz2”。
+ 新增环境变量“TOOLCHAIN_DIR”工具链目录，值配置为工具链根目录，例如：export TOOLCHAIN_DIR=~/tools/gcc-arm-none-eabi-10.3-2021.07。
+ 把工具链bin文件夹路径加入PATH环境变量；export PATH=$TOOLCHAIN_DIR/bin:$PATH。

# 版本编译

+ 把DAPLink开源包和STM32MP153驱动包的压缩包拷贝到open_source目录下
+ HiSpark-Trace源码的根目录下执行编译构建脚本“build.sh”，例如：./build.sh all

**【注意事项】**
+ linux环境下可以直接执行build.sh脚本。windows环境下需要依赖shell执行工具命令窗口执行build.sh脚本。
+ HiSpark-Trace编译构建依赖SDK“SDK\middleware\hisilicon\libboundscheck_v1.1.16”目录下的安全函数库，请确保该目录下安全函数源码存在。如果需要修改安全函数库存放路径，请修改build.sh脚本中“SECURE_CODE_SRC”变量指向新路径。

**【编译脚本的传参和对应的动作】**
> build.sh            : same of build.sh ca7
> build.sh prepare    : create the directory of build and tar
> build.sh clean      : remove the output and intermediate files      
> build.sh boot       : build boot
> build.sh boot clean : remove output and intermediate file of boot
> build.sh ca7        : build daplink code in Cortex-A
> build.sh ca7 clean  : remove output and intermediate file of Cortex-A
> build.sh cm4        : build daplink code in Cortex-M
> build.sh boot clean : remove output and intermediate file of Cortex-M
> build.sh mkimages   : create images
> build.sh all        : build and create images
> build.sh rebuild    : clean all and rebuild and create images
> build.sh build      : build and create imagesbuild.sh -h|--help  : print help message

**【须知】**
+ 更详细的HiSpark-Trace编译构建教程，请参阅《HiSpark-Trace硬件本体软件编译构建指导》。