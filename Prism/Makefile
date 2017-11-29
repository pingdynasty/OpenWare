PROJECT = Prism
BUILDROOT = .
OPENWARE ?= $(BUILDROOT)/..

include $(OPENWARE)/STM32F745VG.mk

C_SRC = $(notdir $(wildcard Src/*.c))
CPP_SRC = $(notdir $(wildcard Src/*.cpp))
C_SRC += cs4272.c seps114a.c
CPP_SRC += ScreenBuffer.cpp ScreenBufferColour.cpp Graphics.cpp
S_SRC = startup_stm32f745xx.s

include $(OPENWARE)/sources.mk

OBJS = $(C_SRC:%.c=Build/%.o)
OBJS += $(CPP_SRC:%.cpp=Build/%.o)
OBJS += $(S_SRC:%.s=Build/%.o)

OBJS += $(DRIVERS)/Src/stm32f7xx_hal_qspi.o

include $(OPENWARE)/f7.mk
