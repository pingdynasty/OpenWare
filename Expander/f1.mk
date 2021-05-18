# Library path
LIBROOT=$(OPENWARE)/Libraries/Drivers/

# Build path
BUILD=$(BUILDROOT)/Build

# Code Paths
DEVICE=$(LIBROOT)/CMSIS/Device/ST/STM32F1xx
CMSIS=$(LIBROOT)/CMSIS/Include
#PERIPH_FILE=$(LIBROOT)/STM32F1xx_StdPeriph_Driver
DSPLIB=$(LIBROOT)/CMSIS/DSP_Lib/Source
DRIVERS=$(LIBROOT)/STM32F1xx_HAL_Driver

# # Processor specific
# LDSCRIPT ?= $(LIBROOT)/Source/stm32_flash.ld
# STARTUP ?= $(BUILD)/startup_stm32f10x_md_vl.o # medium density value line
# SYSTEM ?= $(BUILD)/system_stm32f10x.o
# PERIPH = $(BUILD)/stm32f10x_gpio.o $(BUILD)/stm32f10x_adc.o $(BUILD)/stm32f10x_rcc.o $(BUILD)/stm32f10x_pwr.o 
# PERIPH += $(BUILD)/stm32f10x_exti.o $(BUILD)/stm32f10x_dac.o $(BUILD)/stm32f10x_tim.o $(BUILD)/stm32f10x_dma.o
# PERIPH += $(BUILD)/stm32f10x_usart.o
# PERIPH += $(BUILD)/stm32f10x_spi.o $(BUILD)/stm32f10x_i2c.o $(BUILD)/stm32f10x_dbgmcu.o 
# PERIPH += $(BUILD)/stm32f10x_flash.o $(BUILD)/stm32f10x_fsmc.o
# PERIPH += $(BUILD)/misc.o # stm32f10x_comp.o 
# SYSCALLS = $(BUILD)/libnosys_gnu.o # Syscalls/syscalls.o

# Compilation Flags
ARCH_FLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
ARCH_FLAGS += -fsingle-precision-constant -ffast-math
# DEF_FLAGS = -DUSE_STDPERIPH_DRIVER  -DSTM32F10X_MD_VL
DEF_FLAGS = -DARM_MATH_CM3
DEF_FLAGS += -DUSE_HAL_DRIVER -DSTM32F100xB
INC_FLAGS = -I$(OPENWARE)/Libraries -I$(CMSIS)
INC_FLAGS += -I$(BUILDROOT)/Source -I$(BUILDROOT)/Inc -I$(BUILDROOT)/MDK-ARM
INC_FLAGS += -I$(DEVICE)/Include
INC_FLAGS += -I$(DRIVERS)/Inc
CFLAGS += $(ARCH_FLAGS) $(INC_FLAGS) $(DEF_FLAGS)
CXXFLAGS += $(ARCH_FLAGS) $(INC_FLAGS) $(DEF_FLAGS)
LDFLAGS += -T$(LDSCRIPT) $(ARCH_FLAGS)

include $(BUILDROOT)/libs.mk
include $(BUILDROOT)/common.mk
