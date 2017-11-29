PROJECT = Player
BUILDROOT = .
OPENWARE ?= $(BUILDROOT)/..

include $(OPENWARE)/STM32F746ZG.mk

C_SRC = $(notdir $(wildcard Src/*.c))
CPP_SRC = $(notdir $(wildcard Src/*.cpp))
C_SRC += sdram.c
C_SRC += cs4272.c ssd1309.c
C_SRC += usbh_midi.c
CPP_SRC += ScreenBuffer.cpp ScreenBufferMono.cpp Graphics.cpp
S_SRC = startup_stm32f745xx.s

include $(OPENWARE)/sources.mk

OBJS = $(C_SRC:%.c=Build/%.o)
OBJS += $(CPP_SRC:%.cpp=Build/%.o)
OBJS += $(S_SRC:%.s=Build/%.o)

OBJS += $(DRIVERS)/Src/stm32f7xx_hal_sdram.o
OBJS += $(DRIVERS)/Src/stm32f7xx_ll_fmc.o
OBJS += $(DRIVERS)/Src/stm32f7xx_hal_dac.o
OBJS += $(DRIVERS)/Src/stm32f7xx_hal_dac_ex.o
OBJS += $(DRIVERS)/Src/stm32f7xx_hal_hcd.o
OBJS += $(USB_HOST_FILE)/Core/Src/usbh_core.o
OBJS += $(USB_HOST_FILE)/Core/Src/usbh_pipes.o
OBJS += $(USB_HOST_FILE)/Core/Src/usbh_ioreq.o
OBJS += $(USB_HOST_FILE)/Core/Src/usbh_ctlreq.o
OBJS += $(USB_HOST_FILE)/Class/AUDIO/Src/usbh_audio.o

include $(OPENWARE)/f7.mk
