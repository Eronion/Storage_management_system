################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/BSP/Components/LCD/LiquidCrystal.c 

OBJS += \
./Drivers/CMSIS/BSP/Components/LCD/LiquidCrystal.o 

C_DEPS += \
./Drivers/CMSIS/BSP/Components/LCD/LiquidCrystal.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/BSP/Components/LCD/LiquidCrystal.o: ../Drivers/CMSIS/BSP/Components/LCD/LiquidCrystal.c Drivers/CMSIS/BSP/Components/LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I"C:/Users/brand/STM32CubeIDE/workspace_1.6.1/Storage management system/Drivers/CMSIS/BSP/Components/LCD" -I"C:/Users/brand/STM32CubeIDE/workspace_1.6.1/Storage management system/Drivers/CMSIS/BSP/B-L475E-IOT01" -I"C:/Users/brand/STM32CubeIDE/workspace_1.6.1/Storage management system/Drivers/CMSIS/BSP/Components" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/BSP/Components/LCD/LiquidCrystal.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

