################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/LIN/lin_hal.c \
../Src/LIN/lin_protocol.c \
../Src/LIN/main.c 

OBJS += \
./Src/LIN/lin_hal.o \
./Src/LIN/lin_protocol.o \
./Src/LIN/main.o 

C_DEPS += \
./Src/LIN/lin_hal.d \
./Src/LIN/lin_protocol.d \
./Src/LIN/main.d 


# Each subdirectory must supply rules for building sources it contributes
Src/LIN/%.o Src/LIN/%.su Src/LIN/%.cyclo: ../Src/LIN/%.c Src/LIN/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/sanic.DESKTOP-1NSTN5M/Desktop/STM32F407_Driver_Development/STM32F407_CDD/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-LIN

clean-Src-2f-LIN:
	-$(RM) ./Src/LIN/lin_hal.cyclo ./Src/LIN/lin_hal.d ./Src/LIN/lin_hal.o ./Src/LIN/lin_hal.su ./Src/LIN/lin_protocol.cyclo ./Src/LIN/lin_protocol.d ./Src/LIN/lin_protocol.o ./Src/LIN/lin_protocol.su ./Src/LIN/main.cyclo ./Src/LIN/main.d ./Src/LIN/main.o ./Src/LIN/main.su

.PHONY: clean-Src-2f-LIN

