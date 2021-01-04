################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32G431_AxxLOAD.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32G431_AxxLOAD.list \

OBJCOPY_BIN += \
STM32G431_AxxLOAD.bin \


# All Target
all: STM32G431_AxxLOAD.elf secondary-outputs

# Tool invocations
STM32G431_AxxLOAD.elf: $(OBJS) $(USER_OBJS) C:\Users\seaxjoh\OneDrive\ -\ ABB\gitRepos\16_AxxLoad\SOFTWARE\STM32G431_AxxLOAD\STM32G431KBTX_FLASH.ld
	arm-none-eabi-gcc -o "STM32G431_AxxLOAD.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\seaxjoh\OneDrive - ABB\gitRepos\16_AxxLoad\SOFTWARE\STM32G431_AxxLOAD\STM32G431KBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32G431_AxxLOAD.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32G431_AxxLOAD.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32G431_AxxLOAD.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32G431_AxxLOAD.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32G431_AxxLOAD.bin"
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
