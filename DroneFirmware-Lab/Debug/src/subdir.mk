################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Attitude.c \
../src/Control.c \
../src/DroneFirmware.c \
../src/Mavlink.c \
../src/Motor.c \
../src/Sensor.c 

OBJS += \
./src/Attitude.o \
./src/Control.o \
./src/DroneFirmware.o \
./src/Mavlink.o \
./src/Motor.o \
./src/Sensor.o 

C_DEPS += \
./src/Attitude.d \
./src/Control.d \
./src/DroneFirmware.d \
./src/Mavlink.d \
./src/Motor.d \
./src/Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
src/Attitude.o: ../src/Attitude.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-linux-gnueabi-gcc -I"/home/ens/AM98800/ele674nobug/DroneFirmware-Lab/src" -I"/home/ens/AM98800/ele674nobug/DroneFirmware-Lab/include" -I"/home/ens/AM98800/ele674nobug/DroneFirmware-Lab/include" -O0 -g3 -Wall -c -fmessage-length=0 -pthread -lm -lrt -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/Attitude.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-linux-gnueabi-gcc -I"/home/ens/AM98800/ele674nobug/DroneFirmware-Lab/include" -I"/home/ens/AM98800/ele674nobug/DroneFirmware-Lab/src" -I"/home/ens/AM98800/ele674nobug/DroneFirmware-Lab/include" -O0 -g3 -Wall -c -fmessage-length=0 -pthread -lm -lrt -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


