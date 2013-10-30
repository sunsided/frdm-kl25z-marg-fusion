################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../common/assert.c" \
"../common/io.c" \
"../common/memtest.c" \
"../common/printf.c" \
"../common/queue.c" \
"../common/stdlib.c" \
"../common/uif.c" \

C_SRCS += \
../common/assert.c \
../common/io.c \
../common/memtest.c \
../common/printf.c \
../common/queue.c \
../common/stdlib.c \
../common/uif.c \

OBJS += \
./common/assert.o \
./common/io.o \
./common/memtest.o \
./common/printf.o \
./common/queue.o \
./common/stdlib.o \
./common/uif.o \

C_DEPS += \
./common/assert.d \
./common/io.d \
./common/memtest.d \
./common/printf.d \
./common/queue.d \
./common/stdlib.d \
./common/uif.d \

OBJS_QUOTED += \
"./common/assert.o" \
"./common/io.o" \
"./common/memtest.o" \
"./common/printf.o" \
"./common/queue.o" \
"./common/stdlib.o" \
"./common/uif.o" \

C_DEPS_QUOTED += \
"./common/assert.d" \
"./common/io.d" \
"./common/memtest.d" \
"./common/printf.d" \
"./common/queue.d" \
"./common/stdlib.d" \
"./common/uif.d" \

OBJS_OS_FORMAT += \
./common/assert.o \
./common/io.o \
./common/memtest.o \
./common/printf.o \
./common/queue.o \
./common/stdlib.o \
./common/uif.o \


# Each subdirectory must supply rules for building sources it contributes
common/assert.o: ../common/assert.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/assert.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/assert.o"
	@echo 'Finished building: $<'
	@echo ' '

common/io.o: ../common/io.c
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/io.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/io.o"
	@echo 'Finished building: $<'
	@echo ' '

common/memtest.o: ../common/memtest.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/memtest.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/memtest.o"
	@echo 'Finished building: $<'
	@echo ' '

common/printf.o: ../common/printf.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/printf.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/printf.o"
	@echo 'Finished building: $<'
	@echo ' '

common/queue.o: ../common/queue.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/queue.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/queue.o"
	@echo 'Finished building: $<'
	@echo ' '

common/stdlib.o: ../common/stdlib.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/stdlib.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/stdlib.o"
	@echo 'Finished building: $<'
	@echo ' '

common/uif.o: ../common/uif.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: ARM Ltd Windows GCC C Compiler'
	"$(ARMSourceryDirEnv)/arm-none-eabi-gcc" "$<" @"common/uif.args" -MMD -MP -MF"$(@:%.o=%.d)" -o"common/uif.o"
	@echo 'Finished building: $<'
	@echo ' '


