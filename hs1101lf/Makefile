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
all: hs1101lf
	$(MAKEARCH) -C $(KDIR) INSTALL_MOD_PATH=$(KDIR)/modules M=$(shell pwd) modules_install
	
margie: hs1101lf
	$(shell pwd)/CP margie
homer: hs1101lf
	$(shell pwd)/CP homer

    