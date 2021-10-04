################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include lvgl/tests/lv_test_widgets/subdir.mk
-include lvgl/tests/lv_test_fonts/subdir.mk
-include lvgl/tests/lv_test_core/subdir.mk
-include lvgl/tests/subdir.mk
-include lvgl/src/widgets/subdir.mk
-include lvgl/src/misc/subdir.mk
-include lvgl/src/hal/subdir.mk
-include lvgl/src/gpu/subdir.mk
-include lvgl/src/font/subdir.mk
-include lvgl/src/extra/widgets/win/subdir.mk
-include lvgl/src/extra/widgets/tileview/subdir.mk
-include lvgl/src/extra/widgets/tabview/subdir.mk
-include lvgl/src/extra/widgets/spinner/subdir.mk
-include lvgl/src/extra/widgets/spinbox/subdir.mk
-include lvgl/src/extra/widgets/span/subdir.mk
-include lvgl/src/extra/widgets/msgbox/subdir.mk
-include lvgl/src/extra/widgets/meter/subdir.mk
-include lvgl/src/extra/widgets/list/subdir.mk
-include lvgl/src/extra/widgets/led/subdir.mk
-include lvgl/src/extra/widgets/keyboard/subdir.mk
-include lvgl/src/extra/widgets/imgbtn/subdir.mk
-include lvgl/src/extra/widgets/colorwheel/subdir.mk
-include lvgl/src/extra/widgets/chart/subdir.mk
-include lvgl/src/extra/widgets/calendar/subdir.mk
-include lvgl/src/extra/widgets/animimg/subdir.mk
-include lvgl/src/extra/themes/mono/subdir.mk
-include lvgl/src/extra/themes/default/subdir.mk
-include lvgl/src/extra/themes/basic/subdir.mk
-include lvgl/src/extra/layouts/grid/subdir.mk
-include lvgl/src/extra/layouts/flex/subdir.mk
-include lvgl/src/extra/subdir.mk
-include lvgl/src/draw/subdir.mk
-include lvgl/src/core/subdir.mk
-include lvgl/examples/widgets/win/subdir.mk
-include lvgl/examples/widgets/tileview/subdir.mk
-include lvgl/examples/widgets/textarea/subdir.mk
-include lvgl/examples/widgets/tabview/subdir.mk
-include lvgl/examples/widgets/table/subdir.mk
-include lvgl/examples/widgets/switch/subdir.mk
-include lvgl/examples/widgets/spinner/subdir.mk
-include lvgl/examples/widgets/spinbox/subdir.mk
-include lvgl/examples/widgets/span/subdir.mk
-include lvgl/examples/widgets/slider/subdir.mk
-include lvgl/examples/widgets/roller/subdir.mk
-include lvgl/examples/widgets/obj/subdir.mk
-include lvgl/examples/widgets/msgbox/subdir.mk
-include lvgl/examples/widgets/meter/subdir.mk
-include lvgl/examples/widgets/list/subdir.mk
-include lvgl/examples/widgets/line/subdir.mk
-include lvgl/examples/widgets/led/subdir.mk
-include lvgl/examples/widgets/label/subdir.mk
-include lvgl/examples/widgets/keyboard/subdir.mk
-include lvgl/examples/widgets/imgbtn/subdir.mk
-include lvgl/examples/widgets/img/subdir.mk
-include lvgl/examples/widgets/dropdown/subdir.mk
-include lvgl/examples/widgets/colorwheel/subdir.mk
-include lvgl/examples/widgets/checkbox/subdir.mk
-include lvgl/examples/widgets/chart/subdir.mk
-include lvgl/examples/widgets/canvas/subdir.mk
-include lvgl/examples/widgets/calendar/subdir.mk
-include lvgl/examples/widgets/btnmatrix/subdir.mk
-include lvgl/examples/widgets/btn/subdir.mk
-include lvgl/examples/widgets/bar/subdir.mk
-include lvgl/examples/widgets/arc/subdir.mk
-include lvgl/examples/widgets/animimg/subdir.mk
-include lvgl/examples/styles/subdir.mk
-include lvgl/examples/scroll/subdir.mk
-include lvgl/examples/porting/subdir.mk
-include lvgl/examples/layouts/grid/subdir.mk
-include lvgl/examples/layouts/flex/subdir.mk
-include lvgl/examples/get_started/subdir.mk
-include lvgl/examples/event/subdir.mk
-include lvgl/examples/assets/subdir.mk
-include lvgl/examples/anim/subdir.mk
-include hal_stm_lvgl/touchpad/subdir.mk
-include hal_stm_lvgl/tft/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/subdir.mk
-include GUI/assets/subdir.mk
-include GUI/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/subdir.mk
-include Drivers/BSP/STM32H747I-DISCO/subdir.mk
-include Drivers/BSP/Components/wm8994/subdir.mk
-include Drivers/BSP/Components/st7735/subdir.mk
-include Drivers/BSP/Components/s5k5cag/subdir.mk
-include Drivers/BSP/Components/ov9655/subdir.mk
-include Drivers/BSP/Components/otm8009a/subdir.mk
-include Drivers/BSP/Components/mt25tl01g/subdir.mk
-include Drivers/BSP/Components/mfxstm32l152/subdir.mk
-include Drivers/BSP/Components/is42s32800j/subdir.mk
-include Drivers/BSP/Components/ft6x06/subdir.mk
-include Drivers/BSP/Components/adv7533/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/STIPC/Core/Common/Src/subdir.mk
-include Common/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := STM32H7_dual_core_CM7
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32H7_dual_core_CM7.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32H7_dual_core_CM7.list \

OBJCOPY_BIN += \
STM32H7_dual_core_CM7.bin \


# All Target
all: main-build

# Main-build Target
main-build: STM32H7_dual_core_CM7.elf secondary-outputs

# Tool invocations
STM32H7_dual_core_CM7.elf: $(OBJS) $(USER_OBJS) /home/gustavo/STM32CubeIDE/workspace_1.7.0/STM32H7_dual_core/CM7/STM32H747XIHX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM32H7_dual_core_CM7.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"/home/gustavo/STM32CubeIDE/workspace_1.7.0/STM32H7_dual_core/CM7/STM32H747XIHX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32H7_dual_core_CM7.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32H7_dual_core_CM7.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32H7_dual_core_CM7.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32H7_dual_core_CM7.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32H7_dual_core_CM7.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SIZE_OUTPUT)$(OBJDUMP_LIST)$(EXECUTABLES)$(OBJS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS)$(OBJCOPY_BIN) STM32H7_dual_core_CM7.elf
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
