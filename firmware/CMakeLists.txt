cmake_minimum_required(VERSION 3.7)
project(sensor_modules)

set(CMAKE_C_STANDARD 99)

set(LIBOCM3 libopencm3)

set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/stm32l011d3.ld)
set(DEVICE STM32L0)
#set(CMAKE_VERBOSE_MAKEFILE on)

#I have no fucking idea how any of this works.
#Half of these commands do exactly nothing except break seemingly unrelated commands on removal...


set(GCC_LINKERFLAGS "--static -nostartfiles")
set(GCC_LINKERFLAGS "${GCC_LINKERFLAGS} -T${LINKER_SCRIPT}" )
set(GCC_LINKERFLAGS ${GCC_LINKERFLAGS} -Wl,--print-memory-usage )
set(GCC_LINKERFLAGS "${GCC_LINKERFLAGS} -Wl,--gc-sections -Wl,--strip-all" )
set(GCC_LINKERFLAGS "${GCC_LINKERFLAGS} -Wl,-Map=mem.map" )
#set(GCC_LINKERFLAGS "${GCC_LINKERFLAGS} -Wl,--print-gc-sections" ) #Annoying as hell
set(GCC_LINKERFLAGS "${GCC_LINKERFLAGS}" )


set(GCC_LINKLIBRARIES "-lopencm3_stm32l0")
set(GCC_LINKLIBRARIES "${GCC_LINKLIBRARIES} --specs=nano.specs -Wl,--start-group -lnosys -lgcc -lc -lm -Wl,--end-group -lrdimon" )

#set(GCC_FPFLAGS "-mfloat-abi=soft -mfpu=fpv4-sp-d16" )
set(GCC_ARCHFLAGS "-mthumb -mcpu=cortex-m0plus ${GCC_FPFLAGS}" )

set(GCC_CFLAGS "-g -Os -fno-builtin -Wextra -Wshadow -Wimplicit-function-declaration")
set(GCC_CFLAGS "${GCC_CFLAGS} -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes")
set(GCC_CFLAGS "${GCC_CFLAGS} -fno-common -ffunction-sections -fdata-sections")

set(GCC_CXXFLAGS "-g -O0")
set(GCC_CXXFLAGS "${GCC_CXXFLAGS} -Wextra -Wshadow -Wredundant-decls  -Weffc++")
set(GCC_CXXFLAGS "${GCC_CXXFLAGS} -fno-common -ffunction-sections -fdata-sections")

set(GCC_CPPFLAGS "${GCC_ARCHFLAGS} -MD -Wall -Wundef -I${LIBOCM3}/include -D${DEVICE}" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_CFLAGS} ${GCC_CCPFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_CXXFLAGS} ${GCC_CCPFLAGS}")


set(CMAKE_C_STANDARD 99)

add_subdirectory(vl53l0x_api)
include_directories(vl53l0x_api/core/inc vl53l0x_api/platform/inc)


set(SOURCE_FILES main.c ${VL53L0X_SOURCES})
message(WARNING ${SOURCE_FILES})
include_directories(${LIBOCM3}/include)
link_directories(${LIBOCM3}/lib )
add_definitions( ${GCC_CPPFLAGS})

add_executable(sensor_modules ${SOURCE_FILES})
set_target_properties(sensor_modules PROPERTIES OUTPUT_NAME "sensor_module.elf")
target_link_libraries(sensor_modules ${GCC_LINKLIBRARIES} ${GCC_LINKERFLAGS} ${GCC_ARCHFLAGS})
#set_target_properties(src_carrier PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})