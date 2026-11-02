################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_Device/Target/subdir.mk
-include USB_Device/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := MainBoard_STM32G474RE_V01
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MainBoard_STM32G474RE_V01.elf \

MAP_FILES += \
MainBoard_STM32G474RE_V01.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MainBoard_STM32G474RE_V01.list \

OBJCOPY_HEX += \
MainBoard_STM32G474RE_V01.hex \

OBJCOPY_BIN += \
MainBoard_STM32G474RE_V01.bin \


# All Target
all: main-build

# Main-build Target
main-build: MainBoard_STM32G474RE_V01.elf secondary-outputs

# Tool invocations
MainBoard_STM32G474RE_V01.elf MainBoard_STM32G474RE_V01.map: $(OBJS) $(USER_OBJS) C:\Users\huy\Downloads\TOF_Mag_ICM_TesterFirmware_Indoor\FirmwareSTM32G474RE_V01\STM32G474RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "MainBoard_STM32G474RE_V01.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\huy\Downloads\TOF_Mag_ICM_TesterFirmware_Indoor\FirmwareSTM32G474RE_V01\STM32G474RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MainBoard_STM32G474RE_V01.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MainBoard_STM32G474RE_V01.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MainBoard_STM32G474RE_V01.list"
	@echo 'Finished building: $@'
	@echo ' '

MainBoard_STM32G474RE_V01.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "MainBoard_STM32G474RE_V01.hex"
	@echo 'Finished building: $@'
	@echo ' '

MainBoard_STM32G474RE_V01.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MainBoard_STM32G474RE_V01.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) MainBoard_STM32G474RE_V01.bin MainBoard_STM32G474RE_V01.elf MainBoard_STM32G474RE_V01.hex MainBoard_STM32G474RE_V01.list MainBoard_STM32G474RE_V01.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
