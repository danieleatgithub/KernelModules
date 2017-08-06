
include /wks/workspace/Homer/homer_deploy/environment.sh	


MODULES := hs1101lf.o
obj-m := $(MODULES)
KDIR = ${BUILDROOT}/output/build/linux-${LINUX_VER}

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

hs1101lf:
	$(MAKEARCH) -C $(KDIR) M=$(HS1101LF) modules
clean:
	$(MAKEARCH) -C $(KDIR) M=$(HS1101LF) clean	
	
# TARGET_DIR from buildroot post-build.sh
modules_install: 
	$(MAKEARCH) -C $(KDIR) INSTALL_MOD_PATH=$(TARGET_DIR) M=$(HS1101LF) modules_install

all: hs1101lf
	
debug_margie: hs1101lf
	$(HS1101LF)/CP margie
debug_homer: hs1101lf
	$(HS1101LF)/CP homer

    