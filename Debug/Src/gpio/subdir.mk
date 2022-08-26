################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio/0_led_toggle_on_button_press.c 

OBJS += \
./Src/gpio/0_led_toggle_on_button_press.o 

C_DEPS += \
./Src/gpio/0_led_toggle_on_button_press.d 


# Each subdirectory must supply rules for building sources it contributes
Src/gpio/%.o Src/gpio/%.su: ../Src/gpio/%.c Src/gpio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -DSTM32F411E_DISCO -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-gpio

clean-Src-2f-gpio:
	-$(RM) ./Src/gpio/0_led_toggle_on_button_press.d ./Src/gpio/0_led_toggle_on_button_press.o ./Src/gpio/0_led_toggle_on_button_press.su

.PHONY: clean-Src-2f-gpio

