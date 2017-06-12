################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c \
D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c 

OBJS += \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_adc.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_adc_ex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_i2c.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_i2c_ex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_tim.o \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_tim_ex.o 

C_DEPS += \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_adc.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_adc_ex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_i2c.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_i2c_ex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_tim.d \
./Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_tim_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_adc.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_adc_ex.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_cortex.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_dma.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_flash_ex.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_gpio.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_i2c.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_i2c_ex.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_i2c_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_pwr_ex.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_rcc_ex.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_tim.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F0xx_HAL_Driver/stm32f0xx_hal_tim_ex.o: D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/openstm32/i2c-io2/i2cio/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/openstm32/i2c-io2/i2cio/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/openstm32/i2c-io2/i2cio/Drivers/CMSIS/Include" -I"D:/openstm32/i2c-io2/i2cio/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


