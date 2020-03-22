################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LocalHub1.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LocalHub1.list \

OBJCOPY_BIN += \
LocalHub1.bin \


# All Target
all: LocalHub1.elf secondary-outputs

# Tool invocations
LocalHub1.elf: $(OBJS) $(USER_OBJS) C:\Users\vladyslavN\STM32CubeIDE\workspace_1.1.0\LocalHub1\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "LocalHub1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\vladyslavN\STM32CubeIDE\workspace_1.1.0\LocalHub1\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LocalHub1.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LocalHub1.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LocalHub1.list"
	@echo 'Finished building: $@'
	@echo ' '

LocalHub1.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LocalHub1.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
