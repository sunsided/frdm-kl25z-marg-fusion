################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../cpu/arm_cm0.c" \

C_SRCS += \
../cpu/arm_cm0.c \

S_SRCS += \
../cpu/cw_crt0.s \

S_SRCS_QUOTED += \
"../cpu/cw_crt0.s" \

S_DEPS_QUOTED += \
"./cpu/cw_crt0.d" \

OBJS += \
./cpu/arm_cm0.o \
./cpu/cw_crt0.o \

C_DEPS += \
./cpu/arm_cm0.d \

S_DEPS += \
./cpu/cw_crt0.d \

OBJS_QUOTED += \
"./cpu/arm_cm0.o" \
"./cpu/cw_crt0.o" \

C_DEPS_QUOTED += \
"./cpu/arm_cm0.d" \

OBJS_OS_FORMAT += \
./cpu/arm_cm0.o \
./cpu/cw_crt0.o \


# Each subdirectory must supply rules for building sources it contributes
cpu/arm_cm0.o: ../cpu/arm_cm0.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/arm_cm0.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"cpu/arm_cm0.o"
	@echo 'Finished building: $<'
	@echo ' '

cpu/cw_crt0.o: ../cpu/cw_crt0.s
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: ARM Ltd Windows GCC Assembler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"cpu/cw_crt0.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"cpu/cw_crt0.o"
	@echo 'Finished building: $<'
	@echo ' '


