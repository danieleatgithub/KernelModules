#/wks/linux-4.4.16/arch/arm/boot/dts/at91-sama5d3_acquaa5.dts
MODULES := at91-sama5d3_acquaa5.dts

ARCH := arm
CROSS_COMPILE := arm-buildroot-linux-gnueabi-
obj-m := $(MODULES)
KDIR = /wks/linux-4.4.16

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

dtb:
	$(MAKEARCH) -C $(KDIR) at91-sama5d3_acquaa5.dtb	
zImage:
	$(MAKEARCH) -C $(KDIR) -j8  zImage
modules:
	$(MAKEARCH) -C $(KDIR) -j8  modules	
modules_install:
	$(MAKEARCH) -C $(KDIR) INSTALL_MOD_PATH=$(KDIR)/modules modules_install
clean:
	$(MAKEARCH) -C $(KDIR) clean
all: dtb  zImage modules  modules_install