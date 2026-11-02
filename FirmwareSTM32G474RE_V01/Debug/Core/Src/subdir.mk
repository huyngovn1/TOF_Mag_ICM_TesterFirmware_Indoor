################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AutoPilot.c \
../Core/Src/AutoPilotInput.c \
../Core/Src/BSP.c \
../Core/Src/Fc.c \
../Core/Src/Global.c \
../Core/Src/Gps.c \
../Core/Src/ICM-42670.c \
../Core/Src/IMU.c \
../Core/Src/Kalman.c \
../Core/Src/MyAHRS.c \
../Core/Src/QMC5883.c \
../Core/Src/Rx.c \
../Core/Src/VL53L0X.c \
../Core/Src/VL53L1X_api.c \
../Core/Src/bmp3.c \
../Core/Src/crsf.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/vl53l1_platform.c 

OBJS += \
./Core/Src/AutoPilot.o \
./Core/Src/AutoPilotInput.o \
./Core/Src/BSP.o \
./Core/Src/Fc.o \
./Core/Src/Global.o \
./Core/Src/Gps.o \
./Core/Src/ICM-42670.o \
./Core/Src/IMU.o \
./Core/Src/Kalman.o \
./Core/Src/MyAHRS.o \
./Core/Src/QMC5883.o \
./Core/Src/Rx.o \
./Core/Src/VL53L0X.o \
./Core/Src/VL53L1X_api.o \
./Core/Src/bmp3.o \
./Core/Src/crsf.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/vl53l1_platform.o 

C_DEPS += \
./Core/Src/AutoPilot.d \
./Core/Src/AutoPilotInput.d \
./Core/Src/BSP.d \
./Core/Src/Fc.d \
./Core/Src/Global.d \
./Core/Src/Gps.d \
./Core/Src/ICM-42670.d \
./Core/Src/IMU.d \
./Core/Src/Kalman.d \
./Core/Src/MyAHRS.d \
./Core/Src/QMC5883.d \
./Core/Src/Rx.d \
./Core/Src/VL53L0X.d \
./Core/Src/VL53L1X_api.d \
./Core/Src/bmp3.d \
./Core/Src/crsf.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../USB_Device/App -I../USB_Device/Target -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AutoPilot.cyclo ./Core/Src/AutoPilot.d ./Core/Src/AutoPilot.o ./Core/Src/AutoPilot.su ./Core/Src/AutoPilotInput.cyclo ./Core/Src/AutoPilotInput.d ./Core/Src/AutoPilotInput.o ./Core/Src/AutoPilotInput.su ./Core/Src/BSP.cyclo ./Core/Src/BSP.d ./Core/Src/BSP.o ./Core/Src/BSP.su ./Core/Src/Fc.cyclo ./Core/Src/Fc.d ./Core/Src/Fc.o ./Core/Src/Fc.su ./Core/Src/Global.cyclo ./Core/Src/Global.d ./Core/Src/Global.o ./Core/Src/Global.su ./Core/Src/Gps.cyclo ./Core/Src/Gps.d ./Core/Src/Gps.o ./Core/Src/Gps.su ./Core/Src/ICM-42670.cyclo ./Core/Src/ICM-42670.d ./Core/Src/ICM-42670.o ./Core/Src/ICM-42670.su ./Core/Src/IMU.cyclo ./Core/Src/IMU.d ./Core/Src/IMU.o ./Core/Src/IMU.su ./Core/Src/Kalman.cyclo ./Core/Src/Kalman.d ./Core/Src/Kalman.o ./Core/Src/Kalman.su ./Core/Src/MyAHRS.cyclo ./Core/Src/MyAHRS.d ./Core/Src/MyAHRS.o ./Core/Src/MyAHRS.su ./Core/Src/QMC5883.cyclo ./Core/Src/QMC5883.d ./Core/Src/QMC5883.o ./Core/Src/QMC5883.su ./Core/Src/Rx.cyclo ./Core/Src/Rx.d ./Core/Src/Rx.o ./Core/Src/Rx.su ./Core/Src/VL53L0X.cyclo ./Core/Src/VL53L0X.d ./Core/Src/VL53L0X.o ./Core/Src/VL53L0X.su ./Core/Src/VL53L1X_api.cyclo ./Core/Src/VL53L1X_api.d ./Core/Src/VL53L1X_api.o ./Core/Src/VL53L1X_api.su ./Core/Src/bmp3.cyclo ./Core/Src/bmp3.d ./Core/Src/bmp3.o ./Core/Src/bmp3.su ./Core/Src/crsf.cyclo ./Core/Src/crsf.d ./Core/Src/crsf.o ./Core/Src/crsf.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/vl53l1_platform.cyclo ./Core/Src/vl53l1_platform.d ./Core/Src/vl53l1_platform.o ./Core/Src/vl53l1_platform.su

.PHONY: clean-Core-2f-Src

