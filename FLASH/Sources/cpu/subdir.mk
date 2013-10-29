################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/cpu/clock.c" \
"../Sources/cpu/systick.c" \

C_SRCS += \
../Sources/cpu/clock.c \
../Sources/cpu/systick.c \

OBJS += \
./Sources/cpu/clock.o \
./Sources/cpu/systick.o \

C_DEPS += \
./Sources/cpu/clock.d \
./Sources/cpu/systick.d \

OBJS_QUOTED += \
"./Sources/cpu/clock.o" \
"./Sources/cpu/systick.o" \

C_DEPS_QUOTED += \
"./Sources/cpu/clock.d" \
"./Sources/cpu/systick.d" \

OBJS_OS_FORMAT += \
./Sources/cpu/clock.o \
./Sources/cpu/systick.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/cpu/clock.o: ../Sources/cpu/clock.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/cpu/clock.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/cpu/clock.o"
	@echo 'Finished building: $<'
	@echo ' '

Sources/cpu/systick.o: ../Sources/cpu/systick.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/cpu/systick.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/cpu/systick.o"
	@echo 'Finished building: $<'
	@echo ' '


