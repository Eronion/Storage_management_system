################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/hts221/hts221.c 

OBJS += \
./Drivers/BSP/Components/hts221/hts221.o 

C_DEPS += \
./Drivers/BSP/Components/hts221/hts221.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/hts221/hts221.o: ../Drivers/BSP/Components/hts221/hts221.c Drivers/BSP/Components/hts221/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I"C:/Users/brand/STM32CubeIDE/workspace_1.6.1/Storage_management_system/Drivers/BSP/B-L475E-IOT01" -I"C:/Users/brand/STM32CubeIDE/workspace_1.6.1/Storage_management_system/Drivers/BSP/Components/LCD" -I"C:/Users/brand/STM32CubeIDE/workspace_1.6.1/Storage_management_system/Drivers/BSP/Components" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hts221/hts221.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

