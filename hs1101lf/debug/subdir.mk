################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hs1101lf-gpio.c 

OBJS += \
./hs1101lf-gpio.o 

C_DEPS += \
./hs1101lf-gpio.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Buildroot ARM C Compiler (/wks/buildroot-2016.11/output)'
	/wks/buildroot-2016.11/output/host/usr/bin/arm-buildroot-linux-gnueabi-gcc -D__KERNEL__=1 -I/wks/buildroot-2016.11/output/build/linux-headers-4.4.16/arch/arm/include -I/wks/buildroot-2016.11/output/build/linux-headers-4.4.16/arch/arm/include/uapi -I/wks/buildroot-2016.11/output/build/linux-headers-4.4.16/include -I/wks/buildroot-2016.11/output/build/linux-headers-4.4.16/include/uapi -I/wks/buildroot-2016.11/output/build/linux-headers-4.4.16/include/linux -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


