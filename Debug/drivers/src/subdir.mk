################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/st32f407xx_gpio_driver.c \
../drivers/src/stm32f407_i2c_driver.c \
../drivers/src/stm32f407_spi_driver.c 

OBJS += \
./drivers/src/st32f407xx_gpio_driver.o \
./drivers/src/stm32f407_i2c_driver.o \
./drivers/src/stm32f407_spi_driver.o 

C_DEPS += \
./drivers/src/st32f407xx_gpio_driver.d \
./drivers/src/stm32f407_i2c_driver.d \
./drivers/src/stm32f407_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/st32f407xx_gpio_driver.o: ../drivers/src/st32f407xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/Users/neilpatel/STM32CubeIDE/workspace_1.2.0/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/st32f407xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f407_i2c_driver.o: ../drivers/src/stm32f407_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/Users/neilpatel/STM32CubeIDE/workspace_1.2.0/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f407_i2c_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f407_spi_driver.o: ../drivers/src/stm32f407_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"/Users/neilpatel/STM32CubeIDE/workspace_1.2.0/stm32f4xx_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f407_spi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

