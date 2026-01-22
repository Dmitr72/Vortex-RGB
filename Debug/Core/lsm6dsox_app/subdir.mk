################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lsm6dsox_app/lsm6dsox_app.c 

OBJS += \
./Core/lsm6dsox_app/lsm6dsox_app.o 

C_DEPS += \
./Core/lsm6dsox_app/lsm6dsox_app.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lsm6dsox_app/%.o Core/lsm6dsox_app/%.su Core/lsm6dsox_app/%.cyclo: ../Core/lsm6dsox_app/%.c Core/lsm6dsox_app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Core/AD5160 -I../Core/lsm6dsox_app -I../Core/DAC8551 -I../Core/Eeprom -I../Core/RGB -I../Core/System -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/BSP/Components/Common -I../Drivers/BSP/Components/lsm6dsl -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dsox -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lsm6dsox_app

clean-Core-2f-lsm6dsox_app:
	-$(RM) ./Core/lsm6dsox_app/lsm6dsox_app.cyclo ./Core/lsm6dsox_app/lsm6dsox_app.d ./Core/lsm6dsox_app/lsm6dsox_app.o ./Core/lsm6dsox_app/lsm6dsox_app.su

.PHONY: clean-Core-2f-lsm6dsox_app

