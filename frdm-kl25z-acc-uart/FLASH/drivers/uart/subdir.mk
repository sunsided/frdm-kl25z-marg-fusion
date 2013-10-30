################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../drivers/uart/Retarget.c" \
"../drivers/uart/Serial.c" \
"../drivers/uart/uart.c" \

C_SRCS += \
../drivers/uart/Retarget.c \
../drivers/uart/Serial.c \
../drivers/uart/uart.c \

OBJS += \
./drivers/uart/Retarget.o \
./drivers/uart/Serial.o \
./drivers/uart/uart.o \

C_DEPS += \
./drivers/uart/Retarget.d \
./drivers/uart/Serial.d \
./drivers/uart/uart.d \

OBJS_QUOTED += \
"./drivers/uart/Retarget.o" \
"./drivers/uart/Serial.o" \
"./drivers/uart/uart.o" \

C_DEPS_QUOTED += \
"./drivers/uart/Retarget.d" \
"./drivers/uart/Serial.d" \
"./drivers/uart/uart.d" \

OBJS_OS_FORMAT += \
./drivers/uart/Retarget.o \
./drivers/uart/Serial.o \
./drivers/uart/uart.o \


# Each subdirectory must supply rules for building sources it contributes
drivers/uart/Retarget.o: ../drivers/uart/Retarget.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"drivers/uart/Retarget.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"drivers/uart/Retarget.o"
	@echo 'Finished building: $<'
	@echo ' '

drivers/uart/Serial.o: ../drivers/uart/Serial.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"drivers/uart/Serial.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"drivers/uart/Serial.o"
	@echo 'Finished building: $<'
	@echo ' '

drivers/uart/uart.o: ../drivers/uart/uart.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"drivers/uart/uart.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"drivers/uart/uart.o"
	@echo 'Finished building: $<'
	@echo ' '


