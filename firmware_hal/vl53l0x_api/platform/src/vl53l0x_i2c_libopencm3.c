/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/*!
 * \file   VL53L0X_platform.c
 * \brief  Code function defintions for Doppler Testchip Platform Layer
 *
 */

#include <stdio.h>    // sprintf(), vsnprintf(), printf()


#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

#include <stm32l0xx_ll_i2c.h>

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01


int32_t VL53L0X_comms_close(void)
{
    int status = STATUS_FAIL;

    return status;
}

//void delay_us(uint16_t us){
//    for(int i = 0; i < SystemCoreClock*us; ++i);
//}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t reg, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    /* TODO write_multi
     *
     */
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, (uint32_t)count+1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    int i = 0; while(!LL_I2C_IsActiveFlag_STOP(I2C1)){
        /* Check TXIS flag value in ISR register */
        if(LL_I2C_IsActiveFlag_TXIS(I2C1)){
            /* Write data in Transmit Data register.
            TXIS flag is cleared by writing data in TXDR register */
            LL_I2C_TransmitData8(I2C1, i == 0 ? reg : (*pdata++));
            i++;
        }
    }
    LL_I2C_ClearFlag_STOP(I2C1);

//    while(!LL_I2C_IsActiveFlag_STOP(I2C1)){
//      if(LL_I2C_IsActiveFlag_TXE(I2C1)){
//        LL_I2C_TransmitData8(I2C1, 0xFF);
//      }
//    }
//    LL_I2C_ClearFlag_STOP(I2C1);

    //i2c_transfer7(I2C1, address, pdata, (size_t)count, NULL, 0);
    return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    /* TODO read_multi
     *
     */
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    int i = 0; while(!LL_I2C_IsActiveFlag_STOP(I2C1)){
        /* Check TXIS flag value in ISR register */
        if(LL_I2C_IsActiveFlag_TXIS(I2C1)){
            /* Write data in Transmit Data register.
            TXIS flag is cleared by writing data in TXDR register */
            LL_I2C_TransmitData8(I2C1, index);
            i++;
        }
    }
    LL_I2C_ClearFlag_STOP(I2C1);
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRSLAVE_7BIT, (uint32_t)count, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    i = 0; while(!LL_I2C_IsActiveFlag_STOP(I2C1)){
        /* Check TXIS flag value in ISR register */
        if(LL_I2C_IsActiveFlag_RXNE(I2C1)){
            /* Write data in Transmit Data register.
            TXIS flag is cleared by writing data in TXDR register */
            *pdata = LL_I2C_ReceiveData8(I2C1);
            pdata++;
            i++;
        }
    }
    LL_I2C_ClearFlag_STOP(I2C1);
    //i2c_transfer7(I2C1, address, NULL, 0, pdata, (size_t)count);

    return status;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
    const int32_t cbyte_count = 1;

#ifdef VL53L0X_LOG_ENABLE
    trace_print(TRACE_LEVEL_INFO,"Write reg : 0x%02X, Val : 0x%02X\n", index, data);
#endif

    status = VL53L0X_write_multi(address, index, &data, cbyte_count);

    return status;

}


int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;

    uint8_t  buffer[BYTES_PER_WORD];

    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    if(index%2 == 1)
    {
        status = VL53L0X_write_multi(address, index, &buffer[0], 1);
        status = VL53L0X_write_multi(address, index + 1, &buffer[1], 1);
        // serial comms cannot handle word writes to non 2-byte aligned registers.
    }
    else
    {
        status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);
    }

    return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t  buffer[BYTES_PER_DWORD];

    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;

}


int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

#ifdef VL53L0X_LOG_ENABLE
    trace_print(TRACE_LEVEL_INFO,"Read reg : 0x%02X, Val : 0x%02X\n", index, *pdata);
#endif

    return status;

}


int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
	*pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;

}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
	uint8_t  buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;

}





int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    int32_t status = STATUS_OK;
    float wait_ms = (float)wait_us/1000.0f;

//    /*
//     * Use windows event handling to perform non-blocking wait.
//     */
//    HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
//    WaitForSingleObject(hEvent, (int)(wait_ms + 0.5f));

    return status;

}


int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    int32_t status = STATUS_OK;

//    /*
//     * Use windows event handling to perform non-blocking wait.
//     */
//    HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
//    WaitForSingleObject(hEvent, wait_ms);
    return status;

}


int32_t VL53L0X_set_gpio(uint8_t level)
{
    int32_t status = STATUS_OK;
    //status = VL53L0X_set_gpio_sv(level);
    return status;

}


int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    int32_t status = STATUS_OK;
    return status;
}


int32_t VL53L0X_release_gpio(void)
{
    int32_t status = STATUS_OK;
    return status;

}

int32_t VL53L0X_cycle_power(void)
{
    int32_t status = STATUS_OK;
	return status;
}


int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{

       *ptimer_freq_hz = 0;
       return STATUS_FAIL;
}


int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
       *ptimer_count = 0;
       return STATUS_FAIL;
}
