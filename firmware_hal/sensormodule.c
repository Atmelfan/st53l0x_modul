//
// Created by atmelfan on 2018-01-18.
//

#include <gpio.h>
#include <i2c.h>
#include <spi.h>
#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>

#define VL53L0x_BASE_ADDR 0x52

struct {
    VL53L0X_Dev_t dev;
    uint32_t xshut;
} tof_sensors[2] = {
    {
        .xshut = XSH1_Pin,
        .dev = {
            .I2cDevAddr = VL53L0x_BASE_ADDR,
            .comms_speed_khz = 400
        }
    },
    {
        .xshut = XSH2_Pin,
        .dev = {
            .I2cDevAddr = VL53L0x_BASE_ADDR,
            .comms_speed_khz = 400
        }
    }
};

void delay_us(uint16_t us){
    for(int i = 0; i < SystemCoreClock*us; ++i);
}


void init(){
    //LL_I2C_Enable(I2C1);
    LL_GPIO_ResetOutputPin(XSH1_GPIO_Port, tof_sensors[0].xshut);
    LL_GPIO_ResetOutputPin(XSH1_GPIO_Port, tof_sensors[1].xshut);
    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    delay_us(500);
    for(int i = 0; i < 1; ++i){
        LL_GPIO_SetOutputPin(XSH1_GPIO_Port, tof_sensors[i].xshut);
        delay_us(1200);
        /* Init data*/
        VL53L0X_DataInit(&tof_sensors[i].dev);
        /* Init static data*/
        VL53L0X_StaticInit(&tof_sensors[i].dev);
        /* Set mode to continuous*/
        //VL53L0X_SetLimitCheckValue(&tof_sensors[i].dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        //                           (FixPoint1616_t)(0.25*65536));
        //VL53L0X_SetLimitCheckValue(&tof_sensors[i].dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        //                           (FixPoint1616_t)(32*65536));

        VL53L0X_SetDeviceMode(&tof_sensors[i].dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        /* Set interrupt pin to active low on meas complete*/
        VL53L0X_SetGpioConfig(&tof_sensors[i].dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                              VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_LOW);
        VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&tof_sensors[i].dev, 20000);

        /*Change address*/
        VL53L0X_SetDeviceAddress(&tof_sensors[i].dev, (uint8_t)(VL53L0x_BASE_ADDR + i + 2));
        tof_sensors[i].dev.I2cDevAddr = (uint8_t)(VL53L0x_BASE_ADDR + i + 2);
        VL53L0X_StartMeasurement(&tof_sensors[i].dev);
    }
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
    delay_us(500);

}

uint8_t ready;
VL53L0X_RangingMeasurementData_t range;
void update(){

    for(int i = 0; i < 1; ++i){
        VL53L0X_Error status = VL53L0X_GetMeasurementDataReady(&tof_sensors[i].dev, &ready);
        if(ready && status == VL53L0X_ERROR_NONE){
            VL53L0X_GetRangingMeasurementData(&tof_sensors[i].dev, &range);
            VL53L0X_ClearInterruptMask(&tof_sensors[i].dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);


        }
        delay_us(1000);
        LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}


void aspi_rx_handler(){

}

void aspi_start_handler(){

}

void aspi_end_handler(){

}
