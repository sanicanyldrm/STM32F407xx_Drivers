################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SPI/004_spi_tx_testing.c 

OBJS += \
./Src/SPI/004_spi_tx_testing.o 

C_DEPS += \
./Src/SPI/004_spi_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/SPI/%.o Src/SPI/%.su Src/SPI/%.cyclo: ../Src/SPI/%.c Src/SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/sanic.DESKTOP-1NSTN5M/Desktop/STM32F407_Driver_Development/STM32F407_CDD/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-SPI

clean-Src-2f-SPI:
	-$(RM) ./Src/SPI/004_spi_tx_testing.cyclo ./Src/SPI/004_spi_tx_testing.d ./Src/SPI/004_spi_tx_testing.o ./Src/SPI/004_spi_tx_testing.su

.PHONY: clean-Src-2f-SPI

