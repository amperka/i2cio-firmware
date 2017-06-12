################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/openstm32/i2c-io2/i2cio/Src/system_stm32f0xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f0xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f0xx.o: D:/openstm32/i2c-io2/i2cio/Src/system_stm32f0xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


