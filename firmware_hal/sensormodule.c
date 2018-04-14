//
// Created by atmelfan on 2018-01-18.
//

#include <gpio.h>
#include <i2c.h>
#include <spi.h>
#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>
#include <vl53l0x_def.h>
#include <stdbool.h>
#include <stm32l011xx.h>
#include "aspi.h"
#include "registers.h"

#define VL53L0x_BASE_ADDR 0x52
#define GPA53L0X_ADDR 0x20

struct {
    VL53L0X_Dev_t dev;
    uint32_t xshut;
    GPIO_TypeDef* xport;
} tof_sensors[2] = {
        {
                .xshut = XSH1_Pin, .xport = XSH1_GPIO_Port,
                .dev = {
                        .I2cDevAddr = VL53L0x_BASE_ADDR,
                        .comms_speed_khz = 400
                }
        },
        {
                .xshut = XSH2_Pin, .xport = XSH2_GPIO_Port,
                .dev = {
                        .I2cDevAddr = VL53L0x_BASE_ADDR,
                        .comms_speed_khz = 400
                }
        }
};

void delay_us(uint32_t us){
    for(int i = 0; i < SystemCoreClock*us; ++i);
}


void init(){
    LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

    //LL_I2C_Enable(I2C1);
    for (int j = 0; j < 2; ++j) {
        LL_GPIO_ResetOutputPin(tof_sensors[j].xport, tof_sensors[j].xshut);
    }
    delay_us(5000);

    LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    for(int i = 0; i < 2; ++i){
        LL_GPIO_SetOutputPin(tof_sensors[i].xport, tof_sensors[i].xshut);
        delay_us(1200);
        /* Init data*/
        VL53L0X_DataInit(&tof_sensors[i].dev);
        /* Init static data*/
        VL53L0X_StaticInit(&tof_sensors[i].dev);


        uint8_t VhvSettings = 0;
        uint8_t PhaseCal = 0;
        VL53L0X_PerformRefCalibration(&tof_sensors[i].dev, &VhvSettings, &PhaseCal);


        uint32_t refSpadCount = 0;
        uint8_t isApertureSpads = 0;
        VL53L0X_PerformRefSpadManagement(&tof_sensors[i].dev, &refSpadCount, &isApertureSpads);

        /* Set mode to continuous High speed*/
        VL53L0X_SetLimitCheckValue(&tof_sensors[i].dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                   (FixPoint1616_t)(0.25*65536));
        VL53L0X_SetLimitCheckValue(&tof_sensors[i].dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                   (FixPoint1616_t)(32*65536));
        VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&tof_sensors[i].dev, 20000);
        VL53L0X_SetDeviceMode(&tof_sensors[i].dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

        /* Set interrupt pin to active low on meas complete*/
        VL53L0X_SetGpioConfig(&tof_sensors[i].dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                              VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_HIGH);

        /*Start*/
        VL53L0X_StartMeasurement(&tof_sensors[i].dev);

        /*Change address for future access*/
        VL53L0X_SetDeviceAddress(&tof_sensors[i].dev, (uint8_t)((i + 1)*2));
        tof_sensors[i].dev.I2cDevAddr = (uint8_t)((i + 1)*2);

    }
    delay_us(500);

    /*  */
    MODIFY_REG(SPI1->CR1, 0x00, SPI_CR1_SSI);//Set SSI to
    MODIFY_REG(SPI1->CR1, 0x00, SPI_CR1_RXONLY);
    MODIFY_REG(SPI1->CR1, SPI_CR1_CPHA, 0x00);
    MODIFY_REG(SPI1->CR1, SPI_CR1_CPOL, 0x00);

    /* Enable SPI1  */
    LL_SPI_Enable(SPI1);
    LL_SPI_EnableIT_RXNE(SPI1);

    delay_us(500);

}
#define AVERAGE_WINDOW_SIZE 8

/*Averaging*/
uint8_t average_index = 0;
uint16_t range_mm[2];
uint16_t range_history[AVERAGE_WINDOW_SIZE][2];
uint16_t range_mmlock[2];

/*Measurement*/
VL53L0X_RangingMeasurementData_t range;
uint8_t ready;

/*SPI control vars*/
volatile bool write_mode = false;

uint8_t config_reg = 0x01;

#define NUM_AVERAGES (config_reg & 0x07)
#define FORCE_LED (config_reg & 0x08)

volatile uint8_t addr = 0x00, count = 0x00;
void update(){


    if(LL_GPIO_IsInputPinSet(DGPIO_GPIO_Port, DGPIO_Pin)){
        /*Read sensor*/
        for(int i = 0; i < 2; ++i){
            /*Read sensor if ready*/
            VL53L0X_GetRangingMeasurementData(&tof_sensors[i].dev, &range);
            if(!range.RangeStatus){//VALID
                //range_mm[i] = range.RangeMilliMeter;
                range_history[average_index][i] = range.RangeMilliMeter;
            }else if(range.RangeStatus == 2) {//NO SIGNAL
                //range_mm[i] = range.RangeMilliMeter;
                range_history[average_index][i] = 8192;
            }else if(range.RangeStatus == 4){
                range_history[average_index][i] = range.RangeMilliMeter;
            }else{
                //range_mm[i] |= 0xFFFF;
                range_history[average_index][i] = 0xFFFF;
            }
            VL53L0X_ClearInterruptMask(&tof_sensors[i].dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            delay_us(100);
        }

        if(NUM_AVERAGES > 1){
            uint16_t totweight = 0;
            uint16_t weight = AVERAGE_WINDOW_SIZE;
            uint32_t range_average[] = {0,0};
            /* Average */
            for (int j = 0, indx = average_index; j < NUM_AVERAGES; ++j) {

                /* Sum range history with weight */
                for (int i = 0; i < 2; ++i) {
                    range_average[i] += weight*range_history[j][i];
                }

                /* Add weight to total weight, reduce weight by 2 */
                totweight += weight;
                weight /= 2;

                /* Index rollover */
                if(indx)
                    indx--;
                else
                    indx = AVERAGE_WINDOW_SIZE - 1;

            }
            /* Save to registers */
            __disable_irq();
            for (int i = 0; i < 2; ++i) {
                range_mm[i] = (uint16_t)(range_average[i] / totweight);
            }
            __enable_irq();
        }else{
            /* Do not average */
            __disable_irq();
            for (int i = 0; i < 2; ++i) {

                range_mm[i] = range_history[average_index][i];
            }
            __enable_irq();
        }


        /* Increment history pointer */
        average_index++;
        average_index %= AVERAGE_WINDOW_SIZE;



        //LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    /*Status LED*/
    if(range_mm[0] < 100 || range_mm[1] < 100 || count || FORCE_LED){
        LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
    }else{
        LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
    }

}



/* Enable, disable SPI by setting SSI bit (assuming SSM) */
#define ENABLE_SPI() MODIFY_REG(SPI1->CR1, SPI_CR1_SSI, 0x00)
#define DISABLE_SPI() MODIFY_REG(SPI1->CR1, 0x00, SPI_CR1_SSI)

/* Enable/disable transmit functionality (setting MISO to HiZ) */
#define ENABLE_SPI_TX() MODIFY_REG(SPI1->CR1, SPI_CR1_RXONLY, 0x00);
#define DISABLE_SPI_TX() MODIFY_REG(SPI1->CR1, 0x00, SPI_CR1_RXONLY)

void sensor_exti4_select() {
    /* Reset SPI parameters and disable output*/
    addr = 0x00;
    count = 0x00;
    DISABLE_SPI_TX();

    if (!LL_GPIO_IsInputPinSet(CS_GPIO_Port, CS_Pin)) {

        /* Selected, enable SPI & lock register values*/
        ENABLE_SPI();
        //LL_SPI_TransmitData8(SPI1, GPA53L0X_ADDR);
        range_mmlock[0] = range_mm[0];
        range_mmlock[1] = range_mm[1];
    } else {

        /* Deselected, disable SPI and toggle LED if addressed*/
        if(!READ_BIT(SPI1->CR1, SPI_CR1_SSI)){
           // LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }
        DISABLE_SPI();
    }
}



uint8_t sensor_spi_rxc(uint8_t t){

    /*First byte == address*/
    if(count == 0){
        /* Enable transmit if address matches otherwise disable SPI to prevent further interrupts */
        if((t & 0xFE) == GPA53L0X_ADDR){
            ENABLE_SPI_TX();
            /*Write mode if LSB is set*/
            write_mode = ((t & 0x01) != 0);
            count++;
            //LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        }else{
            DISABLE_SPI();
        }
        return GPA53L0X_CONFIRM;
    }

    /*Second byte == index*/
    if(count == 1){
        addr = t;
        count++;
    }

    /*Etc = data*/
    switch(addr++){
        case GPA53L0X_CONFIG:

            return config_reg;
        case GPA53L0X_SENSOR1L:
            //return 0x55;
            //MANDUS HACK!
            if(write_mode){
                config_reg = t;
            }
            return (uint8_t)((range_mmlock[0] >> 0) & 0x00FF);
        case GPA53L0X_SENSOR1H:
            //return 0xAA;
            return (uint8_t)((range_mmlock[0] >> 8) & 0x00FF);
        case GPA53L0X_SENSOR2L:
            //return 0xF3;
            return (uint8_t)((range_mmlock[1] >> 0) & 0x00FF);
        case GPA53L0X_SENSOR2H:
            //return 0xF4;
            return (uint8_t)((range_mmlock[1] >> 8) & 0x00FF);
        case GPA53L0X_FUNCTIONL:
            //return 0xF3;
            return (uint8_t)(( (range_mmlock[0] + range_mmlock[1]) >> 1) & 0x00FF);
        case GPA53L0X_FUNCTIONH:
            //return 0xF4;
            return (uint8_t)(( (range_mmlock[0] + range_mmlock[1]) >> 9) & 0x00FF);
        default:
            break;
    }
    return 0xFF;
}
