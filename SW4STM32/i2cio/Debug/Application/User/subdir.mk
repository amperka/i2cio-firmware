################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/openstm32/i2c-io2/i2cio/Src/adc.c \
D:/openstm32/i2c-io2/i2cio/Src/gpio.c \
D:/openstm32/i2c-io2/i2cio/Src/i2c.c \
D:/openstm32/i2c-io2/i2cio/Src/main.c \
D:/openstm32/i2c-io2/i2cio/Src/stm32f0xx_hal_msp.c \
D:/openstm32/i2c-io2/i2cio/Src/stm32f0xx_it.c \
D:/openstm32/i2c-io2/i2cio/Src/tim.c 

OBJS += \
./Application/User/adc.o \
./Application/User/gpio.o \
./Application/User/i2c.o \
./Application/User/main.o \
./Application/User/stm32f0xx_hal_msp.o \
./Application/User/stm32f0xx_it.o \
./Application/User/tim.o 

C_DEPS += \
./Application/User/adc.d \
./Application/User/gpio.d \
./Application/User/i2c.d \
./Application/User/main.d \
./Application/User/stm32f0xx_hal_msp.d \
./Application/User/stm32f0xx_it.d \
./Application/User/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/adc.o: D:/openstm32/i2c-io2/i2cio/Src/adc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/gpio.o: D:/openstm32/i2c-io2/i2cio/Src/gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/i2c.o: D:/openstm32/i2c-io2/i2cio/Src/i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/main.o: D:/openstm32/i2c-io2/i2cio/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f0xx_hal_msp.o: D:/openstm32/i2c-io2/i2cio/Src/stm32f0xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f0xx_it.o: D:/openstm32/i2c-io2/i2cio/Src/stm32f0xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/tim.o: D:/openstm32/i2c-io2/i2cio/Src/tim.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


