################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lvgl/examples/widgets/calendar/lv_example_calendar_1.c 

OBJS += \
./lvgl/examples/widgets/calendar/lv_example_calendar_1.o 

C_DEPS += \
./lvgl/examples/widgets/calendar/lv_example_calendar_1.d 


# Each subdirectory must supply rules for building sources it contributes
lvgl/examples/widgets/calendar/%.o: ../lvgl/examples/widgets/calendar/%.c lvgl/examples/widgets/calendar/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H747xx -c -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/gustavo/STM32CubeIDE/workspace_1.7.0/STM32H7_dual_core/CM7/Core/STIPC/Core/Common/Inc" -I"/home/gustavo/STM32CubeIDE/workspace_1.7.0/STM32H7_dual_core/CM7/Drivers/BSP/STM32H747I-DISCO" -I"/home/gustavo/STM32CubeIDE/workspace_1.7.0/STM32H7_dual_core/CM7/lvgl" -I"/home/gustavo/STM32CubeIDE/workspace_1.7.0/STM32H7_dual_core/CM7/Drivers/BSP/Components/Common" -I"/home/gustavo/STM32CubeIDE/workspace_1.7.0/STM32H7_dual_core/CM7" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

