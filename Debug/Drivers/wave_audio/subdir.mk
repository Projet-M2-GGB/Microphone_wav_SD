################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/wave_audio/waverecorder.c 

OBJS += \
./Drivers/wave_audio/waverecorder.o 

C_DEPS += \
./Drivers/wave_audio/waverecorder.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/wave_audio/%.o Drivers/wave_audio/%.su Drivers/wave_audio/%.cyclo: ../Drivers/wave_audio/%.c Drivers/wave_audio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DARM_MATH_CM7 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"C:/Users/fabi_/Documents/STM32_Projects/Microphone_wav_SD/Drivers/BSP/Components" -I"C:/Users/fabi_/Documents/STM32_Projects/Microphone_wav_SD/Drivers/BSP/STM32746G-Discovery" -I"C:/Users/fabi_/Documents/STM32_Projects/Microphone_wav_SD/Drivers/wave_audio" -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-wave_audio

clean-Drivers-2f-wave_audio:
	-$(RM) ./Drivers/wave_audio/waverecorder.cyclo ./Drivers/wave_audio/waverecorder.d ./Drivers/wave_audio/waverecorder.o ./Drivers/wave_audio/waverecorder.su

.PHONY: clean-Drivers-2f-wave_audio

