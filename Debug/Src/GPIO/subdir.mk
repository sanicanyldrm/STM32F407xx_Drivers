################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/GPIO/001_led_toggle.c 

OBJS += \
./Src/GPIO/001_led_toggle.o 

C_DEPS += \
./Src/GPIO/001_led_toggle.d 


# Each subdirectory must supply rules for building sources it contributes
Src/GPIO/%.o Src/GPIO/%.su Src/GPIO/%.cyclo: ../Src/GPIO/%.c Src/GPIO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/sanic.DESKTOP-1NSTN5M/Desktop/STM32F407_Driver_Development/STM32F407_CDD/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-GPIO

clean-Src-2f-GPIO:
	-$(RM) ./Src/GPIO/001_led_toggle.cyclo ./Src/GPIO/001_led_toggle.d ./Src/GPIO/001_led_toggle.o ./Src/GPIO/001_led_toggle.su

.PHONY: clean-Src-2f-GPIO

