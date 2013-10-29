################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/cpu/clock.c" \

C_SRCS += \
../Sources/cpu/clock.c \

OBJS += \
./Sources/cpu/clock.o \

C_DEPS += \
./Sources/cpu/clock.d \

OBJS_QUOTED += \
"./Sources/cpu/clock.o" \

C_DEPS_QUOTED += \
"./Sources/cpu/clock.d" \

OBJS_OS_FORMAT += \
./Sources/cpu/clock.o \


# Each subdirectory must supply rules for building sources it contributes
Sources/cpu/clock.o: ../Sources/cpu/clock.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"Sources/cpu/clock.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"Sources/cpu/clock.o"
	@echo 'Finished building: $<'
	@echo ' '


