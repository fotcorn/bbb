# To build modules outside of the kernel tree, we run "make"
# in the kernel source tree; the Makefile these then includes this
# Makefile once again.
# This conditional selects whether we are being included from the
# kernel Makefile or not.

MODULE=btn_isr_t1
#KERNEL_VER=3.15.8-bone6
KERNEL_VER=3.15.9
TOOLCHAIN	:= /home/corn/Programme/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/bin
#TOOLCHAIN	:= /opt/crosstools/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/bin
PLATFORM	:= arm-linux-gnueabihf-
CROSS_CMP	:= $(TOOLCHAIN)/$(PLATFORM)
ROOT_FS_PATH	:= /home/corn/projekte/embedded/rootfs
#ROOT_FS_PATH	:= /opt/embedded/bbb/rootfs


ifeq ($(KERNELRELEASE),)

    # Assume the source tree is where the running kernel was built
    # You should set KERNEL_SRC_BUILD in the environment if it's elsewhere

    KERNEL_DIR	?= $(ROOT_FS_PATH)/lib/modules/$(KERNEL_VER)
    KERNEL_SRC = $(KERNEL_DIR)/source
    KERNEL_BUILD = $(KERNEL_DIR)/build
    # The current directory is passed to sub-makes as argument
    PWD := $(shell pwd)

modules:
	@echo 'Invoking build system'
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) ARCH=arm CROSS_COMPILE=$(CROSS_CMP) modules
# module goes to KERNEL_VER/extra

modules_install:
	@echo 'Installing modules'
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) ARCH=arm INSTALL_MOD_PATH=$(ROOT_FS_PATH) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.dtbo *.mod.c .tmp_versions Module.symvers modules.order

.PHONY: modules modules_install clean

else
    # called from kernel build system: just declare what our modules are
    obj-m := $(MODULE).o
endif
