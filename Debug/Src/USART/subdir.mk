################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/USART/usart_rx_interrupt.c 

OBJS += \
./Src/USART/usart_rx_interrupt.o 

C_DEPS += \
./Src/USART/usart_rx_interrupt.d 


# Each subdirectory must supply rules for building sources it contributes
Src/USART/%.o Src/USART/%.su Src/USART/%.cyclo: ../Src/USART/%.c Src/USART/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/sanic.DESKTOP-1NSTN5M/Desktop/STM32F407_Driver_Development/STM32F407_CDD/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-USART

clean-Src-2f-USART:
	-$(RM) ./Src/USART/usart_rx_interrupt.cyclo ./Src/USART/usart_rx_interrupt.d ./Src/USART/usart_rx_interrupt.o ./Src/USART/usart_rx_interrupt.su

.PHONY: clean-Src-2f-USART

