MODULES := hs1101lf.o

ARCH := arm
CROSS_COMPILE := arm-buildroot-linux-gnueabi-
obj-m := $(MODULES)
KDIR = /wks/linux-4.4.16

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

hs1101lf:
	$(MAKEARCH) -C $(KDIR) M=$(shell pwd) modules
clean:
	$(MAKEARCH) -C $(KDIR) M=$(shell pwd) clean	
# TARGET_DIR from buildroot post-build.sh
modules_install: hs1101lf
	$(MAKEARCH) -C $(KDIR) INSTALL_MOD_PATH=$(TARGET_DIR) M=$(shell pwd) modules_install
all: hs1101lf
	
debug_margie: hs1101lf
	$(shell pwd)/CP margie
debug_homer: hs1101lf
	$(shell pwd)/CP homer

    