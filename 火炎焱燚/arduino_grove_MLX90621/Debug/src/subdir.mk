################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LD_SRCS += \
../src/lscript.ld 

C_SRCS += \
../src/arduino_grove_MLX90621.c \
../src/myi2c.c


OBJS += \
./src/arduino_grove_MLX90621.o \
./src/myi2c.o

C_DEPS += \
./src/arduino_grove_MLX90621.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MicroBlaze gcc compiler'
	mb-gcc -Wall -O1 -g3 -c -fmessage-length=0 -MT"$@" -I../../bsp_iop_arduino/iop_arduino_mb/include -mlittle-endian -mcpu=v11.0 -mxl-soft-mul -Wl,--no-relax -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


