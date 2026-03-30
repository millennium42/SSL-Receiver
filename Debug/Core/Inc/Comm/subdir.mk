################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Comm/COMM.c \
../Core/Inc/Comm/COMM_PACKETS.c \
../Core/Inc/Comm/NRF24_CORE.c \
../Core/Inc/Comm/NRF24_HAL.c 

OBJS += \
./Core/Inc/Comm/COMM.o \
./Core/Inc/Comm/COMM_PACKETS.o \
./Core/Inc/Comm/NRF24_CORE.o \
./Core/Inc/Comm/NRF24_HAL.o 

C_DEPS += \
./Core/Inc/Comm/COMM.d \
./Core/Inc/Comm/COMM_PACKETS.d \
./Core/Inc/Comm/NRF24_CORE.d \
./Core/Inc/Comm/NRF24_HAL.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Comm/%.o Core/Inc/Comm/%.su Core/Inc/Comm/%.cyclo: ../Core/Inc/Comm/%.c Core/Inc/Comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Comm

clean-Core-2f-Inc-2f-Comm:
	-$(RM) ./Core/Inc/Comm/COMM.cyclo ./Core/Inc/Comm/COMM.d ./Core/Inc/Comm/COMM.o ./Core/Inc/Comm/COMM.su ./Core/Inc/Comm/COMM_PACKETS.cyclo ./Core/Inc/Comm/COMM_PACKETS.d ./Core/Inc/Comm/COMM_PACKETS.o ./Core/Inc/Comm/COMM_PACKETS.su ./Core/Inc/Comm/NRF24_CORE.cyclo ./Core/Inc/Comm/NRF24_CORE.d ./Core/Inc/Comm/NRF24_CORE.o ./Core/Inc/Comm/NRF24_CORE.su ./Core/Inc/Comm/NRF24_HAL.cyclo ./Core/Inc/Comm/NRF24_HAL.d ./Core/Inc/Comm/NRF24_HAL.o ./Core/Inc/Comm/NRF24_HAL.su

.PHONY: clean-Core-2f-Inc-2f-Comm

