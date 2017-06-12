################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/openstm32/i2c-io2/i2cio/SW4STM32/startup_stm32f030x6.s 

OBJS += \
./Application/SW4STM32/startup_stm32f030x6.o 


# Each subdirectory must supply rules for building sources it contributes
Application/SW4STM32/startup_stm32f030x6.o: D:/openstm32/i2c-io2/i2cio/SW4STM32/startup_stm32f030x6.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo %cd%
	arm-none-eabi-as -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


