################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lcdIli9341/lcdIli9341.c 

OBJS += \
./Core/Src/lcdIli9341/lcdIli9341.o 

C_DEPS += \
./Core/Src/lcdIli9341/lcdIli9341.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/lcdIli9341/%.o Core/Src/lcdIli9341/%.su: ../Core/Src/lcdIli9341/%.c Core/Src/lcdIli9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-lcdIli9341

clean-Core-2f-Src-2f-lcdIli9341:
	-$(RM) ./Core/Src/lcdIli9341/lcdIli9341.d ./Core/Src/lcdIli9341/lcdIli9341.o ./Core/Src/lcdIli9341/lcdIli9341.su

.PHONY: clean-Core-2f-Src-2f-lcdIli9341

