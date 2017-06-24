MODULES := hs1101lf.o

ARCH := arm
CROSS_COMPILE := arm-buildroot-linux-gnueabi-
obj-m := $(MODULES)
KDIR = /wks/linux-4.4.16

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

all:
	$(MAKEARCH) -C $(KDIR) M=$(shell pwd) modules
clean:
	$(MAKEARCH) -C $(KDIR) M=$(shell pwd) clean