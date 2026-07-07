################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/dwt.c \
../Core/Src/gps.c \
../Core/Src/icm20602.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/mtf01.c \
../Core/Src/nrf.c \
../Core/Src/pid_controller.c \
../Core/Src/pos_hold.c \
../Core/Src/qmc5883.c \
../Core/Src/serial.c \
../Core/Src/stm32h5xx_hal_msp.c \
../Core/Src/stm32h5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h5xx.c \
../Core/Src/tof.c \
../Core/Src/types.c \
../Core/Src/uart_cmd.c 

OBJS += \
./Core/Src/dwt.o \
./Core/Src/gps.o \
./Core/Src/icm20602.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/mtf01.o \
./Core/Src/nrf.o \
./Core/Src/pid_controller.o \
./Core/Src/pos_hold.o \
./Core/Src/qmc5883.o \
./Core/Src/serial.o \
./Core/Src/stm32h5xx_hal_msp.o \
./Core/Src/stm32h5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h5xx.o \
./Core/Src/tof.o \
./Core/Src/types.o \
./Core/Src/uart_cmd.o 

C_DEPS += \
./Core/Src/dwt.d \
./Core/Src/gps.d \
./Core/Src/icm20602.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/mtf01.d \
./Core/Src/nrf.d \
./Core/Src/pid_controller.d \
./Core/Src/pos_hold.d \
./Core/Src/qmc5883.d \
./Core/Src/serial.d \
./Core/Src/stm32h5xx_hal_msp.d \
./Core/Src/stm32h5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h5xx.d \
./Core/Src/tof.d \
./Core/Src/types.d \
./Core/Src/uart_cmd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H562xx -c -I../Core/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/dwt.cyclo ./Core/Src/dwt.d ./Core/Src/dwt.o ./Core/Src/dwt.su ./Core/Src/gps.cyclo ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/icm20602.cyclo ./Core/Src/icm20602.d ./Core/Src/icm20602.o ./Core/Src/icm20602.su ./Core/Src/kalman.cyclo ./Core/Src/kalman.d ./Core/Src/kalman.o ./Core/Src/kalman.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mtf01.cyclo ./Core/Src/mtf01.d ./Core/Src/mtf01.o ./Core/Src/mtf01.su ./Core/Src/nrf.cyclo ./Core/Src/nrf.d ./Core/Src/nrf.o ./Core/Src/nrf.su ./Core/Src/pid_controller.cyclo ./Core/Src/pid_controller.d ./Core/Src/pid_controller.o ./Core/Src/pid_controller.su ./Core/Src/pos_hold.cyclo ./Core/Src/pos_hold.d ./Core/Src/pos_hold.o ./Core/Src/pos_hold.su ./Core/Src/qmc5883.cyclo ./Core/Src/qmc5883.d ./Core/Src/qmc5883.o ./Core/Src/qmc5883.su ./Core/Src/serial.cyclo ./Core/Src/serial.d ./Core/Src/serial.o ./Core/Src/serial.su ./Core/Src/stm32h5xx_hal_msp.cyclo ./Core/Src/stm32h5xx_hal_msp.d ./Core/Src/stm32h5xx_hal_msp.o ./Core/Src/stm32h5xx_hal_msp.su ./Core/Src/stm32h5xx_it.cyclo ./Core/Src/stm32h5xx_it.d ./Core/Src/stm32h5xx_it.o ./Core/Src/stm32h5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h5xx.cyclo ./Core/Src/system_stm32h5xx.d ./Core/Src/system_stm32h5xx.o ./Core/Src/system_stm32h5xx.su ./Core/Src/tof.cyclo ./Core/Src/tof.d ./Core/Src/tof.o ./Core/Src/tof.su ./Core/Src/types.cyclo ./Core/Src/types.d ./Core/Src/types.o ./Core/Src/types.su ./Core/Src/uart_cmd.cyclo ./Core/Src/uart_cmd.d ./Core/Src/uart_cmd.o ./Core/Src/uart_cmd.su

.PHONY: clean-Core-2f-Src

