//
// Created by atmelfan on 2018-01-18.
//

#include <gpio.h>
#include <i2c.h>
#include <spi.h>
#include <vl53l0x_api.h>

struct {
    VL53L0X_Dev_t dev;
    uint32_t xshut;
} tof_sensors[2];


void init(){
    VL53L0X_DataInit(&tof_sensors[0].dev);
    VL53L0X_StaticInit(&tof_sensors[0].dev);
    VL53L0X_SetDeviceMode(&tof_sensors[0].dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VL53L0X_SetGpioConfig(&tof_sensors[0].dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                          VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_LOW);
}

void update(){


}
