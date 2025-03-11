ARCH          := arm
CROSS_COMPILE := arm-none-eabi-
ifeq ($(OS), Windows_NT)
PLATFORM="Windows"
COMPILE_TIME = $(shell echo %date:~0,4%%date:~5,2%%date:~8,2%%time:~0,2%%time:~3,2%%time:~6,2%)
else
PLATFORM="Unix-Like"
COMPILE_TIME = $(date + "%Y-%M-%d %H:%M:%s")
endif


AR := $(CROSS_COMPILE)ar
CC := $(CROSS_COMPILE)gcc
NM := $(CROSS_COMPILE)nm
CPP := $(CROSS_COMPILE)cpp
LD := $(CROSS_COMPILE)ld
OBJCOPY := $(CROSS_COMPILE)objcopy
OBJDUMP := $(CROSS_COMPILE)objdump

TOP_DIR := .

OUT = $(TOP_DIR)/out
TARGET := daplink_A7
TARGET_ELF := $(OUT)/$(TARGET).elf
TARGET_BIN := $(OUT)/$(TARGET).bin
TARGET_HEX := $(OUT)/$(TARGET).hex
TARGET_ASM := $(OUT)/$(TARGET).asm
TARGET_MAP := $(OUT)/$(TARGET).map

OBJCPFLAGS_ELF_TO_BIN = -Obinary
OBJCPFLAGS_ELF_TO_HEX = -Ibinary -Oihex
OBJCPFLAGS_ELF_TO_LIST = -S

CFLAGS = -Wall -Wstrict-prototypes -Wno-format-security -fno-builtin -ffreestanding -std=gnu11 -fno-strict-aliasing -fno-PIE -Os -fno-stack-protector -fno-delete-null-pointer-checks -fmacro-prefix-map=./= -g -fstack-usage -Wno-format-nonliteral -Werror=date-time -D__ARM__ -Wa,-mimplicit-it=always -mthumb -mthumb-interwork -mabi=aapcs-linux -mword-relocations -fPIE -fPIC -mno-unaligned-access -ffunction-sections -fdata-sections -fno-common -ffixed-r9 -pipe -march=armv7-a -mtune=generic-armv7-a
CFLAGS += -fshort-wchar -fno-strict-aliasing
CFLAGS += -mfloat-abi=soft
CFLAGS += -fshort-enums
CFLAGS += -DNO_RTOS -DUSE_HAL_TICK -DDAPLINK_UART
CFLAGS += -DCORE_CA7 -DSTM32MP153Dxx -D__NO_USB_LIB_C -DDAPLINK_HIC_ID=0x97969949 -DUSE_HAL_DRIVER -DINTERFACE_STM32MP153 -DUSE_LEGACY_CMSIS_RTOS -DOS_CLOCK=25000000
CFLAGS += -DDRAG_N_DROP_SUPPORT -D__OTG_HS_EMBEDDED_PHY
CFLAGS += -DDAPLINK_VERSION=10002000 -DMSC_ENDPOINT
CFLAGS += -DWINUSB_INTERFACE
CFLAGS += -DBULK_ENDPOINT
CFLAGS += -DCDC_ENDPOINT
CFLAGS += -DCONFIG_HAS_VBAR
CFLAGS += -DM4BIN_SIZE_ADDR=0xC004 -DM4_TEXT_BASEADDR=0xBB000
CFLAGS += -DDAPLINK_BASEADDR=0x2FFCA500
CFLAGS += -DSECUREC_HAVE_ERRNO_H=0
CFLAGS += -DDAPLINK_COMPILE_TIME=\"$(COMPILE_TIME)\"
CFLAGS += -DSOFTWARE_VERSION=\"1.0.0.17\"
CFLAGS += -DHISPARK_TRACE -DROLE_MASTER
ASFLAGS = -Wall -march=armv7-a -mtune=cortex-a7 -nostidinc -fno-PIE -g -D__ARM__ -D__ASSEMBLY__ -Wa,-mimplicit-it=always -mthumb -mthumb-interwork -mword-relocations -fno-pic -mno-unaligned-access -ffunction-sections -fdata-sections -fno-common -ffixed-r9 -pipe  -D__LINUX_ARM_ARCH__=7 -mtune=generic-armv7-a

LDSCRIPT = $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/daplink.lds

SRC = $(TOP_DIR)/source/board/stm32mp1xx.c
SRC += $(wildcard $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/cmsis/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver/Src/*.c)
SRC += $(wildcard $(TOP_DIR)/source/usb/*.c)
SRC += $(wildcard $(TOP_DIR)/source/usb/winusb/*.c)
SRC += $(wildcard $(TOP_DIR)/source/usb/bulk/*.c)
SRC += $(wildcard $(TOP_DIR)/source/usb/cdc/*.c)
SRC += $(wildcard $(TOP_DIR)/source/usb/hid/*.c)
SRC += $(wildcard $(TOP_DIR)/source/usb/msc/*.c)
SRC += $(wildcard $(TOP_DIR)/source/daplink/interface/*.c)
SRC += $(wildcard $(TOP_DIR)/source/daplink/settings/*.c)
SRC += $(wildcard $(TOP_DIR)/source/daplink/drag-n-drop/*.c)
SRC += $(wildcard $(TOP_DIR)/source/daplink/cmsis-dap/*.c)
SRC += $(wildcard $(TOP_DIR)/source/daplink/usb2uart/*.c)
SRC += $(wildcard $(TOP_DIR)/source/daplink/*.c)
SRC += $(wildcard $(TOP_DIR)/source/family/vendorHM/306x/*.c)
SRC += $(wildcard $(TOP_DIR)/source/rtos_none/*.c)
SRC += $(wildcard $(TOP_DIR)/source/target/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/fatfs/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/debug/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/drivers/oled/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/drivers/key/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/remoteproc/src/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/msgQueue/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/offline_download/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/display/src/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/dap_ext_features/source/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/crc/*.c)
SRC += $(wildcard $(TOP_DIR)/source/hispark_trace/interrupt/*.c)
SRC += $(wildcard $(TOP_DIR)/source/securec/src/*.c)

ASM_SRC = ${wildcard $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/gcc/*.S}

INC = -I $(TOP_DIR)/source/daplink
INC += -I $(TOP_DIR)/source/daplink/settings
INC += -I $(TOP_DIR)/source/daplink/interface
INC += -I $(TOP_DIR)/source/daplink/cmsis-dap
INC += -I $(TOP_DIR)/source/daplink/drag-n-drop
INC += -I $(TOP_DIR)/source/rtos_none
INC += -I $(TOP_DIR)/source/hic_hal
INC += -I $(TOP_DIR)/source/hic_hal/stm32
INC += -I $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx
INC += -I $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/cmsis
INC += -I $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver
INC += -I $(TOP_DIR)/source/hic_hal/stm32/stm32mp1xx/STM32MP1xx_HAL_Driver/Inc
INC += -I $(TOP_DIR)/source/usb
INC += -I $(TOP_DIR)/source/cmsis-core
INC += -I $(TOP_DIR)/Drivers/CMSIS/Device/ST/STM32MP1xx/Include
INC += -I $(TOP_DIR)/source/target
INC += -I $(TOP_DIR)/source/hispark_trace/fatfs
INC += -I $(TOP_DIR)/source/hispark_trace/debug
INC += -I $(TOP_DIR)/source/hispark_trace/drivers/oled
INC += -I $(TOP_DIR)/source/hispark_trace/drivers/key
INC += -I $(TOP_DIR)/source/hispark_trace/remoteproc/inc
INC += -I $(TOP_DIR)/source/hispark_trace/msgQueue
INC += -I $(TOP_DIR)/source/hispark_trace/offline_download
INC += -I $(TOP_DIR)/source/hispark_trace/display/inc
INC += -I $(TOP_DIR)/source/hispark_trace/dap_ext_features/include
INC += -I $(TOP_DIR)/source/hispark_trace/crc
INC += -I $(TOP_DIR)/source/hispark_trace/interrupt
INC += -I $(TOP_DIR)/source/securec/include

NORMOM_CFLAGS :=$(CFLAGS) $(INC)
NORMOM_ASFLAGS :=$(CFLAGS) -I $(TOP_DIR)/include
LDFLAGS = -nostdlib -T $(LDSCRIPT) -nostartfiles -Wl,--gc-sections -pie -lc -static -Wl,--no-wchar-size-warning
#Set the TOOLCHAIN_DIR in /root/.bashrc or build.sh(the tool chain is gcc-arm-10.3-2021.07-x86_64-arm-none-eabi).
#TOOLCHAIN_DIR=CI_ENV
LIBS = $(TOOLCHAIN_DIR)/lib/gcc/arm-none-eabi/10.3.1/libgcc.a
LIBS += $(TOOLCHAIN_DIR)/arm-none-eabi/lib/libc.a


OBJ := $(SRC:%.c=%.o)
ASMOBJ := $(ASM_SRC:%.S=%.o)

DEF +=

STARTUP_RAW = $(wildcard $(STARTUP))
STARTUP_OBJ = $(STARTUP_RAW:%.c=%.o)

daplink_OBJ = ./out/*.o

all: $(OUT) $(TARGET_BIN)
$(TARGET_BIN): $(TARGET_ELF)
	$(Q)$(OBJCOPY) $(OBJCPFLAGS_ELF_TO_BIN) $(TARGET_ELF)  $(TARGET_BIN)
	$(Q)$(OBJDUMP) -d $(TARGET_ELF) > $(TARGET_ASM)
	$(Q)$(OBJDUMP) -stD $(TARGET_ELF) > $(TARGET_MAP)

$(TARGET_ELF):$(OBJ) $(ASMOBJ)
	$(Q)$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LIBS) -o $@

$(OBJ):%.o:%.c
	@echo Compile  $<
	$(CC) $(NORMOM_CFLAGS) -c $< -o $@

$(ASMOBJ):%.o:%.S
	@echo Compile  $<
	$(CC) $(NORMOM_ASFLAGS) -c $< -o $@

$(OUT):
	mkdir -p $(OUT)

.PHONY: clean
ifeq ($(OS), Windows_NT)
RM  := del /s/q
clean:
	$(RM) ..\CA7\*.o
	$(RM) ..\CA7\*.su
	$(RM) out
else
clean :
	find . -name "*.o" | xargs rm -f
	find . -name "*.su" | xargs rm -f
	-rm -rf $(OUT)/*
endif
