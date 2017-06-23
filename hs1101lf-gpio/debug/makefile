################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
ARCHIVES += \
libUSER_OBJS.a \


# All Target
all: hs1101lf-gpio

# Tool invocations
hs1101lf-gpio: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Buildroot ARM C Linker (/wks/buildroot-2016.11/output)'
	/wks/buildroot-2016.11/output/host/usr/bin/arm-buildroot-linux-gnueabi-gcc  -o "hs1101lf-gpio" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

libUSER_OBJS.a: $(OBJS) $(USER_OBJS)
	@echo 'Invoking: Buildroot ARM Archiver (/wks/buildroot-2016.11/output)'
	/wks/buildroot-2016.11/output/host/usr/bin/arm-buildroot-linux-gnueabi-ar -r  "libUSER_OBJS.a" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ARCHIVES)$(EXECUTABLES) hs1101lf-gpio
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
