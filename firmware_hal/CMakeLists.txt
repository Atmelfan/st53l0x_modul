cmake_minimum_required(VERSION 3.7)
project(sensor_modules C ASM)
set(CMAKE_C_STANDARD 99)

#Paramaters
set(OPT -Os)


#Sources
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32L0xx_HAL_Driver/Src HAL_DRIVER_SOURCES)

set(HAL_SOURCES
        Src/gpio.c
        Src/i2c.c
        Src/spi.c
        Src/stm32l0xx_it.c
        Src/system_stm32l0xx.c
        ${HAL_DRIVER_SOURCES}
        )


#Headers
set(HAL_HEADERS
        Inc/gpio.h
        Inc/i2c.h
        Inc/spi.h
        Inc/stm32_assert.h
        Inc/stm32l0xx_hal_conf.h
        Inc/stm32l0xx_it.h
        Inc/main.h
        )
include_directories(Drivers/STM32L0xx_HAL_Driver/Inc/)
include_directories(Drivers/CMSIS/Device/ST/STM32L0xx/Include/)
include_directories(Drivers/CMSIS/Include/)
include_directories(Inc/)

#ASM sources
set(ASM_SOURCES startup_stm32l011xx.s)
set_source_files_properties(startup_stm32l011xx.s PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp") #Special compile flags

#Compiler
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")

#Processor config
set(CPU "-mcpu=cortex-m0plus")
set(FPU "")
set(FLOAT_ABI "")
set(MCU "${CPU} -mthumb ${FPU} ${FLOAT_ABI}")

#Definitions
add_definitions(-DUSE_FULL_LL_DRIVER)
add_definitions(-DSTM32L011xx)
add_definitions(-DUSE_i2C_2V8)

#flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU} ${OPT} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -MMD -MP -flto")

# link script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L011F4Ux_FLASH.ld)

#linker
set(LIBS "-lc -lm -lnosys")
set(LDFLAGS "${MCU} -flto -specs=nano.specs -T${LINKER_SCRIPT} ${LIBS} -Wl,-Map=symbols.map,--cref -Wl,--gc-sections -Wl,--print-memory-usage ")

#add linker flags
set( CMAKE_CXX_FLAGS  "${LDFLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS  "${LDFLAGS}" )

#VL53L0X
add_subdirectory(vl53l0x_api)
include_directories(${VL53L0x_INCLUDES})

#executable
add_executable(sensormod main.c ${ASM_SOURCES} ${HAL_SOURCES} ${HAL_HEADERS} ${VL53L0X_SOURCES} sensormodule.c)

#set(CMAKE_VERBOSE_MAKEFILE ON)