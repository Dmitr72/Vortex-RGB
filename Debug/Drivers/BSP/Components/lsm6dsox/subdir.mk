################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lsm6dsox/lsm6dsox.c \
../Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.c 

OBJS += \
./Drivers/BSP/Components/lsm6dsox/lsm6dsox.o \
./Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.o 

C_DEPS += \
./Drivers/BSP/Components/lsm6dsox/lsm6dsox.d \
./Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lsm6dsox/%.o Drivers/BSP/Components/lsm6dsox/%.su Drivers/BSP/Components/lsm6dsox/%.cyclo: ../Drivers/BSP/Components/lsm6dsox/%.c Drivers/BSP/Components/lsm6dsox/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/AD5160 -I../Core/lsm6dsox_app -I../Core/DAC8551 -I../Core/Eeprom -I../Core/RGB -I../Core/System -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/BSP/Components/Common -I../Drivers/BSP/Components/lsm6dsl -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dsox -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsox

clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsox:
	-$(RM) ./Drivers/BSP/Components/lsm6dsox/lsm6dsox.cyclo ./Drivers/BSP/Components/lsm6dsox/lsm6dsox.d ./Drivers/BSP/Components/lsm6dsox/lsm6dsox.o ./Drivers/BSP/Components/lsm6dsox/lsm6dsox.su ./Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.cyclo ./Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.d ./Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.o ./Drivers/BSP/Components/lsm6dsox/lsm6dsox_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lsm6dsox

