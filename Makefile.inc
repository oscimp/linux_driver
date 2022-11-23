#define all stuffs needed to compile
OUTPUT_DIR=$(BR_DIR)/output
HOST_DIR=$(OUTPUT_DIR)/host
LINUX_VERS=$(shell grep "^BR2_LINUX_KERNEL_VERSION" $(BR_DIR)/.config 2>/dev/null | cut -d \" -f 2)
LINUX_SRC=$(OUTPUT_DIR)/build/linux-$(LINUX_VERS)
COMPILER_NAME=arm-linux-
CROSS_COMPILE=$(HOST_DIR)/bin/$(COMPILER_NAME)

EXTRA_CFLAGS+=-I$(OSCIMP_DIGITAL_DRIVER)
ifeq ($(BOARD_NAME),)
$(error error is missing BOARD_NAME variable)
endif

DEST_DIR:=$(OSCIMP_DIGITAL_NFS)/$(BOARD_NAME)/modules

ccflags-y := -g -Wall

ifeq ($(KERNELRELEASE),)
# Part executed when called from standard make in this directory:
# (preferably use Makefile in parent directory)

PWD  := $(shell pwd)
PWD      := $(shell if [ "$$PWD" != "" ]; then echo $$PWD; else pwd; fi)
CLEAN_MOD= modules.order *.ko *.mod.c *.mod.o *.symvers $(CLEANMOD)
CLEAN_MOD+= .tmp_versions Module.markers
MODULES := $(wildcard obj-m *.ko)

default:
	$(MAKE) -C $(LINUX_SRC) KBUILD_EXTMOD=$(PWD) modules ARCH=arm \
	CROSS_COMPILE=$(CROSS_COMPILE)

install: default
	@echo $(obj-m)
	mkdir -p $(DEST_DIR)
	cp $(MODULES) $(DEST_DIR)/

clean:
	$(MAKE) -C $(LINUX_SRC) SUBDIRS=$(PWD) clean
	rm -rf $(CLEAN_MOD)

.PHONY: modules clean

endif
