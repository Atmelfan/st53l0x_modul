#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
//#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <vl53l0x_def.h>

static void i2c_setup(void)
{
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);
    //rcc_set_i2c_clock_hsi(I2C1);

    i2c_reset(I2C1);
    /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO7);
    i2c_peripheral_disable(I2C1);
    //configure ANFOFF DNF[3:0] in CR1
    i2c_enable_analog_filter(I2C1);
    //i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
    /* HSI is at 8Mhz */
    i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);
    //configure No-Stretch CR1 (only relevant in slave mode)
    i2c_enable_stretching(I2C1);
    //addressing mode
    i2c_set_7bit_addr_mode(I2C1);
    i2c_peripheral_enable(I2C1);
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
                    GPIO14 | GPIO15);
}

static void usart_setup(void)
{
    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2| GPIO3);

    /* Setup UART parameters. */
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART2);
}


static void clock_setup(void)
{
    //rcc_clock_setup_in_hse_12mhz_out_72mhz();

    /* Enable GPIOA, GPIOB, GPIOC clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_I2C1);

    /* Enable SPI1 Periph and gpio clocks */
    rcc_periph_clock_enable(RCC_SPI1);
}

static void spi_setup(void) {

}

static void usart_puts(const char* s){
    while(*s){
        usart_send_blocking(USART2, *s);
        s++;
    }
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
int main(){

    clock_setup();
    i2c_setup();
    usart_setup();
    gpio_setup();


    //printf("Hello, World!\n");
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    int status_int;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;

    pMyDevice->I2cDevAddr      = 0x52;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;

//    Status = VL53L0X_i2c_init(SerialCommStr, 460800);
//    if (Status != VL53L0X_ERROR_NONE) {
//        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
//    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        //printf ("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(&MyDevice); // Data initialization
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        if(Status == VL53L0X_ERROR_NONE)
        {
            //printf("VL53L0X_GetDeviceInfo:\n");
            //printf("Device Name : %s\n", DeviceInfo.Name);
            //printf("Device Type : %s\n", DeviceInfo.Type);
            //printf("Device ID : %s\n", DeviceInfo.ProductId);
            //printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
            //printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

            if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
                //printf("Error expected cut 1.1 but found cut %d.%d\n", DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        //printf ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        //printf ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal); // Device Initialization
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        //printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); // Device Initialization
        //printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        //printf ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
    }

    // Enable/Disable Sigma and Signal check
//    if (Status == VL53L0X_ERROR_NONE) {
//        Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
//    }
//    if (Status == VL53L0X_ERROR_NONE) {
//        Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
//    }
//
//    if (Status == VL53L0X_ERROR_NONE) {
//        Status = VL53L0X_SetLimitCheckEnable(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
//    }
//
//    if (Status == VL53L0X_ERROR_NONE) {
//        Status = VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));
//    }
    for(;;){
        if(Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice, &RangingMeasurementData);
            usart_send(USART1, RangingMeasurementData.RangeMilliMeter);

            VL53L0X_GetLimitCheckCurrent(pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);
        }

    }



    return 0;
}
#pragma clang diagnostic pop