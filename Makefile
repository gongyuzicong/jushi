#CC = /home/farsight/toolchain/gcc-arm-none-eabi-5_3-2016q1/bin/arm-none-eabi-gcc
#CFLAGS = -Wall -g -O0

BOOT_SRC = $(wildcard ./boot/*)
#SRC = $($(BOOT_SRC))
BOOT_SRC_NDIR = $(notdir $(BOOT_SRC))
OBJ = $(patsubst %.s,%.o,$(BOOT_SRC))
OBJ_OUTPUT_PATH = ./project/output/

all:
	@echo $(BOOT_SRC)
	@echo $(OBJ)
	@echo $(OBJ_OUTPUT_PATH)


