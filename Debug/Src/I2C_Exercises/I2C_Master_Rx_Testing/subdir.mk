################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/I2C_Exercises/I2C_Master_Rx_Testing/i2c_master_rx_testing.c 

OBJS += \
./Src/I2C_Exercises/I2C_Master_Rx_Testing/i2c_master_rx_testing.o 

C_DEPS += \
./Src/I2C_Exercises/I2C_Master_Rx_Testing/i2c_master_rx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/I2C_Exercises/I2C_Master_Rx_Testing/%.o Src/I2C_Exercises/I2C_Master_Rx_Testing/%.su Src/I2C_Exercises/I2C_Master_Rx_Testing/%.cyclo: ../Src/I2C_Exercises/I2C_Master_Rx_Testing/%.c Src/I2C_Exercises/I2C_Master_Rx_Testing/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/sanic.DESKTOP-1NSTN5M/Desktop/STM32F407_Driver_Development/STM32F407_CDD/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-I2C_Exercises-2f-I2C_Master_Rx_Testing

clean-Src-2f-I2C_Exercises-2f-I2C_Master_Rx_Testing:
	-$(RM) ./Src/I2C_Exercises/I2C_Master_Rx_Testing/i2c_master_rx_testing.cyclo ./Src/I2C_Exercises/I2C_Master_Rx_Testing/i2c_master_rx_testing.d ./Src/I2C_Exercises/I2C_Master_Rx_Testing/i2c_master_rx_testing.o ./Src/I2C_Exercises/I2C_Master_Rx_Testing/i2c_master_rx_testing.su

.PHONY: clean-Src-2f-I2C_Exercises-2f-I2C_Master_Rx_Testing

