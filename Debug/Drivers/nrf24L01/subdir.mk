################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/nrf24L01/MY_NRF24.c 

OBJS += \
./Drivers/nrf24L01/MY_NRF24.o 

C_DEPS += \
./Drivers/nrf24L01/MY_NRF24.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/nrf24L01/%.o Drivers/nrf24L01/%.su Drivers/nrf24L01/%.cyclo: ../Drivers/nrf24L01/%.c Drivers/nrf24L01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DARM_MATH_CM7 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"C:/Users/fabi_/Documents/STM32_Projects/Microphone_wav_SD/Drivers/BSP/Components" -I"C:/Users/fabi_/Documents/STM32_Projects/Microphone_wav_SD/Drivers/BSP/STM32746G-Discovery" -I"C:/Users/fabi_/Documents/STM32_Projects/Microphone_wav_SD/Drivers/wave_audio" -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-nrf24L01

clean-Drivers-2f-nrf24L01:
	-$(RM) ./Drivers/nrf24L01/MY_NRF24.cyclo ./Drivers/nrf24L01/MY_NRF24.d ./Drivers/nrf24L01/MY_NRF24.o ./Drivers/nrf24L01/MY_NRF24.su

.PHONY: clean-Drivers-2f-nrf24L01

