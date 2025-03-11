# 配置编译工具链
TOOLCHAIN    = arm-none-eabi-
CC           = $(TOOLCHAIN)gcc
AS           = $(TOOLCHAIN)as
LD           = $(TOOLCHAIN)ld
AR           = $(TOOLCHAIN)ar
OBJCP        = $(TOOLCHAIN)objcopy
OBJDUMP      = $(TOOLCHAIN)objdump

# 生成的文件名<项目名>
PROJECT                := daplink_M4

TOP_DIR       = .
OUT_DIR       = $(TOP_DIR)/out

# 定义文件格式和文件名
TARGET                 := $(PROJECT)
TARGET_ELF             := $(OUT_DIR)/$(TARGET).elf
TARGET_BIN             := $(OUT_DIR)/$(TARGET).bin
TARGET_HEX             := $(OUT_DIR)/$(TARGET).hex
TARGET_MAP             := $(OUT_DIR)/$(TARGET).map
TARGET_ASM             := $(OUT_DIR)/$(TARGET).asm

OBJCPFLAGS_ELF_TO_BIN   = -Obinary
OBJCPFLAGS_ELF_TO_ASM   = -S
OBJCPFLAGS_ELF_TO_HEX   = -Oihex
OBJCPFLAGS_BIN_TO_HEX   = -Ibinary -Oihex

# 定义路径
SCRIPT_DIR    := $(TOP_DIR)/
STARTUP_DIR   := $(TOP_DIR)/Core/Startup
INC_DIR       := -I $(TOP_DIR)/Drivers/CMSIS/Device/ST/STM32MP1xx/Include
INC_DIR       += -I $(TOP_DIR)/Drivers/CMSIS/Include
INC_DIR       += -I $(TOP_DIR)/Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy
INC_DIR       += -I $(TOP_DIR)/Drivers/STM32MP1xx_HAL_Driver/Inc
INC_DIR       += -I $(TOP_DIR)/Core/Src
INC_DIR       += -I $(TOP_DIR)/Core/Inc
INC_DIR       += -I $(TOP_DIR)/securec/include

SOURCE := $(wildcard $(TOP_DIR)/Core/Src/*.c)
SOURCE += $(wildcard $(TOP_DIR)/Drivers/STM32MP1xx_HAL_Driver/Src/*.c)
SOURCE += $(wildcard $(TOP_DIR)/securec/src/*.c)


# 设置LD链接脚本文件
LDSCRIPT      := $(SCRIPT_DIR)/STM32MP157DAAX_RAM.ld

#需要链接的静态库
#Set the TOOLCHAIN_DIR in ~/.bashrc or in build.sh (the tool chain is gcc-arm-10.3-2021.07-x86_64-arm-none-eabi).
#TOOLCHAIN_DIR=CI_ENV
LDLIBS_DIR       += -L$(TOOLCHAIN_DIR)/lib/gcc/arm-none-eabi/10.3.1/thumb/v7e-m+fp/hard
LDLIBS_DIR       += -L$(TOOLCHAIN_DIR)/arm-none-eabi/lib/thumb/v7e-m+fp/hard

#定义编译标志
CCFLAGS       += -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -ffunction-sections -fdata-sections -Wall -fstack-usage
CCFLAGS       += '-DMETAL_MAX_DEVICE_REGIONS=2' -DSTM32MP157Dxx -DUSE_HAL_DRIVER -DCORE_CM4 -DNO_ATOMIC_64_SUPPORT -DMETAL_INTERNAL -DVIRTIO_SLAVE_ONLY
CCFLAGS       += -DNEED_UART_INIT
CCFLAGS       +=  -static --specs=nano.specs --specs=nosys.specs
CCFLAGS       += $(INC_DIR)
CCFLAGS       += -DSECUREC_HAVE_ERRNO_H=0
ASFLAGS       += -mcpu=cortex-m4 -x assembler-with-cpp -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs
LDFLAGS       += $(LDLIBS_DIR)
LDFLAGS       += -T $(LDSCRIPT) -Map="$(TARGET_MAP)" --gc-sections -static --no-wchar-size-warning --start-group -lc -lm --end-group
# 添加启动文件
SOURCE_ASM    += $(STARTUP_DIR)/startup_stm32mp157daax.s

# 替换文件后缀
C_OBJS        := $(SOURCE:%.c=%.o)
ASM_OBJS      := $(SOURCE_ASM:%.s=%.o)

# 编译命令的定义
COMPILE       = $(CC) $(CCFLAGS) -c $< -o $@
ASSEMBLE      = $(CC) $(ASFLAGS) -c $< -o $@
LINK          = $(LD) $+ $(LDFLAGS) -o $@
ELF_TO_BIN    = $(OBJCP) $(OBJCPFLAGS_ELF_TO_BIN) $< $@
BIN_TO_HEX    = $(OBJCP) $(OBJCPFLAGS_BIN_TO_HEX) $< $@
ELF_TO_ASM    = $(OBJDUMP) $(OBJCPFLAGS_ELF_TO_ASM) $< $@

# 定义伪目标
.PHONY :all clean printf

# 定义规则
all: $(TARGET_HEX)
	@echo "build done"

$(TARGET_HEX): $(TARGET_BIN)
	$(BIN_TO_HEX)

$(TARGET_BIN): $(TARGET_ELF)
	$(ELF_TO_BIN)

$(TARGET_ELF): $(C_OBJS) $(ASM_OBJS)
	$(LINK)

$(C_OBJS):%.o:%.c
	$(shell if [ ! -e $(OUT_DIR) ]; then mkdir -p $(OUT_DIR); fi)
	$(COMPILE)

$(ASM_OBJS):%.o:%.s
	$(ASSEMBLE)

printf:
	@echo $(ASM_OBJS)
	@echo $(ASSEMBLE)

ifeq ($(OS), Windows_NT)
RM  := del /s/q
clean:
#	$(RM) $(C_OBJS) $(ASM_OBJS)
	$(RM) .\*.o
	$(RM) .\*.su
	$(RM) .\*.bin
	$(RM) .\*.hex
	$(RM) .\*.map
	$(RM) .\*.elf
	@echo "clean done"
else
clean:
	find . -name "*.o" | xargs rm -f
	find . -name "*.su" | xargs rm -f
	rm -f $(TARGET_HEX)
	rm -f $(TARGET_BIN)
	rm -f $(TARGET_ELF)
	rm -f $(TARGET_MAP)
	@echo "clean done"
endif
