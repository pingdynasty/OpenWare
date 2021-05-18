PROJECT = Expander
BUILDROOT = .
OPENWARE ?= $(BUILDROOT)/..
OPENOCD ?= openocd -f openocd.cfg

include $(BUILDROOT)/STM32F100CB.mk

C_SRC = $(wildcard Src/*.c)
#C_SRC = $(filter-out Src/test.c, $(wildcard Src/*.c))
C_SRC += $(wildcard Source/*.c)
CPP_SRC = $(wildcard Src/*.cpp)
CPP_SRC += $(wildcard Source/*.cpp)

include $(BUILDROOT)/f1.mk
