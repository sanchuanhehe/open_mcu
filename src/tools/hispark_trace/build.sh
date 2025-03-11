#!/bin/bash
# Compiling and Building HiSpark-Trace Codes.
# Copyright (c) 2024, HiSilicon Technologies Co., Ltd. All rights reserved.

function global_env_init()
{
    GCC_TOOLCHAIN_PATH=""                               #please input your toolchain path
    #set gcc toolchain path
    #export PATH=/$GCC_TOOLCHAIN_PATH/gcc-arm-10.3-2021.07-x86_64-arm-none-eabi/bin:$PATH
    #set gcc lib path
    #export TOOLCHAIN_DIR=/$GCC_TOOLCHAIN_PATH/gcc-arm-10.3-2021.07-x86_64-arm-none-eabi
}

function glogal_variable_init()
{
    CUR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"; pwd)"
    if [[ -z "$CUR_DIR" ]];then
      CUR_DIR='.'
    fi
    BUILD="build"   
    BUILD_BOOT="build/boot"
    BUILD_CA7="build/CA7"
    BUILD_CM4="build/CM4"
    BUILD_CA7_SOURCE="build/CA7/source"
    SECURE_CODE_SRC="../../middleware/hisilicon/libboundscheck_v1.1.16"
    DAPLINK_SRC="open_source/DAPLink-0257.tar.gz"
    SDK_SRC="open_source/STM32CubeMP1-1.4.0.tar.gz"
    HISPARK="hispark"
    BOOT="boot"
    TMP_DIR=".tmp"
    IMAGES_DIR="images"
    STM32MP1XX=$BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx
    STM32MP1XX_HAL=$STM32MP1XX/STM32MP1xx_HAL_Driver
}

function create_stm32mp1xx_dir()
{
    SRC=$1
    DST=$2  
    cp $SRC/*.c $DST
    cp $SRC/*.h $DST
    rm $DST/usbd_STM32F103.c
    mkdir -p $DST/gcc
    mkdir -p $DST/cmsis
    chmod -R 777 $DST
}

function recursive_copy()
{
    dirlist=$(ls $1)
    for name in ${dirlist[*]}
        do
            if [ -f "$1"/"$name" ]; then
                if [ ! -f "$2"/"$name" ]; then
                    cp $1/$name $2/$name
                fi
            elif [ -d "$1"/"$name" ]; then
                if [ ! -d "$2"/"$name" ]; then
                    mkdir -p $2/$name
                fi
                recursive_copy $1/$name $2/$name
            fi
        done
}

function do_patch()
{   
    echo "do_patch..."
    cd build
    PATCHS=patchs/$1
    patchlist=$(ls $PATCHS)
    for name in ${patchlist[*]}     
        do
            echo $name
            if [ -f "$PATCHS"/"$name" ]; then
                patch -p1 < $PATCHS/$name
            fi
        done
    cd -
    echo "do_patch done"
}

function build_common_prepare()
{
	#将DAPLINK解压到build目录
    mkdir -p $BUILD
    tar -xf $DAPLINK_SRC -C $BUILD
    mv $BUILD/$(basename $DAPLINK_SRC .tar.gz) $BUILD_CA7
	
    #基于STM32F103XB和STM32MP1XX SDK构造stm32mp1xx目录
    tar -xf $SDK_SRC
    mv $(basename $SDK_SRC .tar.gz) $TMP_DIR	
		
	recursive_copy $HISPARK $BUILD	
	
	#拷贝打补丁前的DAP到CM4
    CORE=$BUILD_CM4/Core
	chmod -R 777 $CORE	
    if [ ! -f "$CORE"/Src/JTAG_DP.c ]; then
        cp $BUILD_CA7_SOURCE/daplink/cmsis-dap/JTAG_DP.c $CORE/Src
    fi
    if [ ! -f "$CORE"/Inc/SW_DP.c ]; then
        cp $BUILD_CA7_SOURCE/daplink/cmsis-dap/SW_DP.c $CORE/Src
    fi    
    if [ ! -f "$CORE"/Inc/DAP.h ]; then
        cp $BUILD_CA7_SOURCE/daplink/cmsis-dap/DAP.h $CORE/Inc       
    fi    
    if [ ! -f "$CORE"/Inc/DAP_Config.h ]; then
        cp $BUILD_CA7_SOURCE/hic_hal/stm32/stm32f103xb/DAP_config.h $CORE/Inc
    fi	
}

function build_ca7_prepare()
{
    echo "prepare ca7 files ..."
    echo "copy STM32MP1xx_HAL_Driver to build ..."
    mkdir -p $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx
    #基于stm32f103文件目录创建vendorHM的family
    mkdir -p $BUILD_CA7_SOURCE/family/vendorHM/306x
    cp $BUILD_CA7_SOURCE/family/st/stm32f103rb/*.c $BUILD_CA7_SOURCE/family/vendorHM/306x

    cp -rf $TMP_DIR/Drivers/STM32MP1xx_HAL_Driver $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx
    
    #删除不要的文件，这里主要是LL文件和template文件
    SRC_PATH=$BUILD_CA7_SOURCE/hic_hal/stm32/stm32f103xb    
    rm $STM32MP1XX/STM32MP1xx_HAL_Driver/Inc/*_ll_*.h
    rm $STM32MP1XX/STM32MP1xx_HAL_Driver/Src/*_ll_*.c
    rm $STM32MP1XX/STM32MP1xx_HAL_Driver/Src/*template.c
    create_stm32mp1xx_dir $SRC_PATH $STM32MP1XX
                 
    cp $TMP_DIR/Drivers/CMSIS/Device/ST/STM32MP1xx/Include/stm32mp1xx.h $STM32MP1XX
    cp $TMP_DIR/Drivers/CMSIS/Device/ST/STM32MP1xx/Include/stm32mp153dxx_ca7.h $STM32MP1XX
    cp $TMP_DIR/Drivers/CMSIS/Device/ST/STM32MP1xx/Include/system_stm32mp1xx.h $STM32MP1XX/cmsis
    cp $TMP_DIR/Drivers/CMSIS/Device/ST/STM32MP1xx/Source/Templates/system_stm32mp1xx.c $STM32MP1XX/cmsis
    cp $TMP_DIR/Drivers/CMSIS/Core_A/Include/cmsis_cp15.h $BUILD_CA7_SOURCE/cmsis-core
    cp $TMP_DIR/Drivers/CMSIS/Core_A/Include/core_ca.h $BUILD_CA7_SOURCE/cmsis-core
    cp $TMP_DIR/Projects/STM32MP157C-EV1/Examples/QSPI/QSPI_ReadWrite_IT/Src/stm32mp1xx_hal_msp.c $STM32MP1XX/cmsis
    cp $TMP_DIR/Projects/STM32MP157C-EV1/Examples/QSPI/QSPI_ReadWrite_IT/Src/stm32mp1xx_hal_msp.c $STM32MP1XX/cmsis      
    cp $TMP_DIR/Projects/STM32MP157C-EV1/Templates/Inc/stm32mp1xx_hal_conf.h $STM32MP1XX/STM32MP1xx_HAL_Driver 
	
    mkdir -p $BUILD_CA7_SOURCE/securec
    cp -rf $SECURE_CODE_SRC/* $BUILD_CA7_SOURCE/securec
	
    do_patch "CA7"
    echo "prepare ca7 files done"
}

function build_cm4_prepare()
{
    echo "prepare cm4 files..."
    DRIVERS=$BUILD_CM4/Drivers
    if [ ! -e "$DRIVERS" ]; then
        mkdir $DRIVERS
        cp -rf $STM32MP1XX_HAL $DRIVERS
        cp -rf $TMP_DIR/Drivers/CMSIS $DRIVERS
        
        cp $DRIVERS/CMSIS/Device/ST/STM32MP1xx/Source/Templates/system_stm32mp1xx.c $BUILD_CM4/Core/Src
        cp $STM32MP1XX_HAL/stm32mp1xx_hal_conf.h $BUILD_CM4/Core/Inc
        cp $TMP_DIR/Projects/STM32MP157C-DK2/Applications/OpenAMP/OpenAMP_raw/STM32CubeIDE/CM4/Application/User/*.c  $BUILD_CM4/Core/Src
    fi
		
    #拷贝A核Msg_Queue代码到M核，两者使用相同的代码
    if [ ! -f "$CORE"/Src/msg_queue.c ]; then
        cp $BUILD_CA7_SOURCE/hispark_trace/msgQueue/*.c $CORE/Src
    fi
    if [ ! -f "$CORE"/Inc/msg_queue.h ]; then
        cp $BUILD_CA7_SOURCE/hispark_trace/msgQueue/*.h $CORE/Inc
    fi
	
    #拷贝A核debug代码到M核，两者使用相同的代码
    if [ ! -f "$CORE"/Src/debug.c ]; then
        cp $BUILD_CA7_SOURCE/hispark_trace/debug/*.c $CORE/Src
    fi
    if [ ! -f "$CORE"/Inc/debug.h ]; then
        cp $BUILD_CA7_SOURCE/hispark_trace/debug/*.h $CORE/Inc
    fi	
    
    mkdir -p $BUILD_CM4/securec
    cp -rf $SECURE_CODE_SRC/* $BUILD_CM4/securec
	
    do_patch "CM4"
    echo "prepare cm4 files done"
}

function build_boot_prepare()
{
    echo "prepare boot files..."
    BUILD_BOOT_SOURCE=$BUILD_BOOT/source  
    cp -rf $BUILD_CA7_SOURCE/cmsis-core $BUILD_BOOT_SOURCE
    cp -rf $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx    
    mkdir $BUILD_BOOT_SOURCE/crc
    cp $BUILD_CA7_SOURCE/hispark_trace/crc/crc32.* $BUILD_BOOT_SOURCE/crc	
    cp -rf $BUILD_CA7_SOURCE/hispark_trace/drivers/oled $BUILD_BOOT_SOURCE/mmi
    cp -rf $BUILD_CA7_SOURCE/hispark_trace/drivers/key $BUILD_BOOT_SOURCE/mmi
    cp $BUILD_CA7_SOURCE/hic_hal/sdk.h $BUILD_BOOT_SOURCE/hic_hal
    cp $BUILD_CA7_SOURCE/hic_hal/FlashPrg.h $BUILD_BOOT_SOURCE/hic_hal
    cp $BUILD_CA7_SOURCE/hic_hal/flash_hal.h $BUILD_BOOT_SOURCE/hic_hal
    
    cp -rf $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/cmsis $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx
    cp $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/stm32mp153dxx_ca7.h $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx
    cp $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/stm32mp1xx.h $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx
    cp $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/IO_Config.h $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx
    cp $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/daplink_addr.h $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx
    cp $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/sdk.c $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx
    cp $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/flash.c $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx
	
    mkdir -p $BUILD_BOOT/source/securec
    cp -rf $SECURE_CODE_SRC/* $BUILD_BOOT/source/securec
    echo "prepare boot files done"
}

function remove_unused_files()
{
    #删除不需要的文件，如加入编译有告警  		
    if [ -f "$BUILD_CA7_SOURCE"/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c ]; then
        rm $BUILD_CA7_SOURCE/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c
    fi
    if [ -f "$BUILD_CA7_SOURCE"/daplink/sdk_stub.c ]; then
        rm $BUILD_CA7_SOURCE/daplink/sdk_stub.c
    fi
    if [ -f "$BUILD_BOOT_SOURCE"/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c ]; then
        rm $BUILD_BOOT_SOURCE/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c
    fi
}

function build_prepare()
{
    if [ ! -d "$BUILD" ]; then
        build_common_prepare
        build_ca7_prepare
        build_cm4_prepare
        build_boot_prepare
        remove_unused_files
        chmod -R 777 $BUILD
    fi
    [ -n "${TMP_DIR}" ] && rm -rf ${TMP_DIR}
}

function build_ca7()
{
    echo "build ca7..."	
    cd $BUILD_CA7
    make
    cd -
    echo "build ca7 done"
}

function build_ca7_clean()
{
    cd $BUILD_CA7
    make clean
}

function build_cm4()
{
    echo "build cm4..."	
    cd $BUILD_CM4
    make
    cd -
    echo "build cm4 done"
}

function build_cm4_clean()
{
    cd $BUILD_CM4
    make clean
    cd -
}

function build_boot()
{
    echo "build boot..."
    cd $BUILD_BOOT
    make
    cd -
    echo "build boot done"
}

function build_boot_clean()
{
    cd $BUILD_BOOT
    make clean
    cd -
}

function clean_up()
{
    [ -n "${BUILD}" ] && rm -rf ${BUILD}
    [ -n "${TMP_DIR}" ] && rm -rf ${TMP_DIR}
    [ -n "${IMAGES_DIR}" ] && rm -rf $IMAGES_DIR
}

function create_images()
{
    python ./tools/daplink_create_loader.py
    chmod 777 images/daplink_boot.stm32
    chmod 777 images/HiSpark-Trace_Boot_CRC32_forDAPLINK.bin
	python ./tools/daplink_create_allinone_interface_1M_CRC32.py
	chmod 777 images/allinone_upgrade_1M_CRC32.bin
	python ./tools/daplink_create_HiSpark_Trace_allinone.py
	chmod 777 images/HiSpark-Trace_allinone.bin
	python ./tools/daplink_create_HiSpark_Trace_Firmware.py
	chmod 777 images/HiSpark-Trace_Firmware_forDAPLINK.bin
}

function delete_tmp_bin()
{
	rm images/daplink_*.bin
	rm images/*CRC32.bin
	rm images/*.stm32	
} 

function make_images()
{
    DIR=$(cd $(dirname $0) && pwd )

    if [ ! -d "$IMAGES_DIR" ]; then
        mkdir $IMAGES_DIR
    fi
    if [ ! -f "$BUILD_BOOT"/out/daplink_boot.bin ]; then
        echo "daplink boot bin not exist"
        exit $?
    else
        cp $BUILD_BOOT/out/daplink_boot.bin $IMAGES_DIR
    fi
    if [ ! -f "$BUILD_CA7"/out/daplink_A7.bin ]; then
        echo "daplink ca7 bin not exist"
        exit $?     
    else
        cp $BUILD_CA7/out/daplink_A7.bin $IMAGES_DIR
    fi
    if [ ! -f "$BUILD_CM4"/out/daplink_M4.bin ]; then
        echo "daplink cm4 bin not exist"
        exit $?     
    else
        cp $BUILD_CM4/out/daplink_M4.bin $IMAGES_DIR
    fi

    create_images
	delete_tmp_bin	
}

function print_usage() {
    local usage="\
    Usage:
        build.sh            : same of build.sh ca7
        build.sh prepare    : create the directory of build and tar
        build.sh clean      : remove the output and intermediate files      
        build.sh boot       : build boot
        build.sh boot clean : remove output and intermediate file of boot
        build.sh ca7        : build daplink code in Cortex-A
        build.sh ca7 clean  : remove output and intermediate file of Cortex-A
        build.sh cm4        : build daplink code in Cortex-M
        build.sh boot clean : remove output and intermediate file of Cortex-M
        build.sh mkimages   : create images
        build.sh all        : build and create images
        build.sh rebuild    : clean all and rebuild and create images
        build.sh build      : build and create images
        build.sh -h|--help  : print help message
    "
    echo "$usage"
}

function  main()
{
    global_env_init
    glogal_variable_init
    if [[ $# -ne 0 ]]; then     
        case "$1" in 
        clean)
            clean_up
            exit $?         
            ;;
        prepare)
            build_prepare
            exit $?
            ;;
        boot)
            if [[ $# -eq 2 ]]; then
                if [ "$2" == "clean" ]; then
                    build_boot_clean
                fi
            else
                build_prepare
                build_boot
            fi
            exit $?
            ;;
        ca7)
            if [[ $# -eq 2 ]]; then
                if [ "$2" == "clean" ]; then
                    build_ca7_clean
                fi
            else
                build_prepare
                build_ca7
            fi
            exit $?
            ;;
        cm4)
            if [[ $# -eq 2 ]]; then
                if [ "$2" == "clean" ]; then
                    build_cm4_clean
                fi
            else
                build_prepare
                build_cm4
            fi
            exit          $?
            ;;
        mkimages)           
            make_images
            exit $?
            ;;              
        all)
            build_prepare
            build_cm4           
            build_ca7
            build_boot
            make_images
            exit $?
            ;;
        rebuild)
            clean_up
            build_prepare
            build_cm4           
            build_ca7
            build_boot
            make_images
            exit $?
            ;;
        build)
            build_boot_clean
            build_cm4_clean
            build_ca7_clean	    	    
            build_cm4           
            build_ca7
            build_boot
            make_images
            exit $?
            ;;
        -h|--help|*)
            print_usage
            exit $?
            ;;
        esac
    else
        build_prepare
        build_ca7       
    fi
}

main "$@"
