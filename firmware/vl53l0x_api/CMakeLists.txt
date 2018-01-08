
file(GLOB VL53L0X_SOURCES_CORE core/src/*.c)
include_directories(core/inc)
file(GLOB VL53L0X_SOURCES_PLATFORM platform/src/*.c)
include_directories(platform/inc)
set(VL53L0X_SOURCES ${VL53L0X_SOURCES_CORE} ${VL53L0X_SOURCES_PLATFORM} PARENT_SCOPE)
message(WARNING ${VL53L0X_SOURCES})
