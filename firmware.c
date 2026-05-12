/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include <unistd.h>

static const char *TAG = "";

#define I2C_MASTER_SCL_IO           GPIO_NUM_1
#define I2C_MASTER_SDA_IO           GPIO_NUM_0
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define MPU_ADDR                    0x68



esp_err_t mpuWriteReg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t data){
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    return i2c_master_transmit(handle, buf, 2, 1000);
}

esp_err_t mpuRWFromReg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t * data, size_t len){


    return i2c_master_transmit_receive(handle, &reg, 1, data, len, 2000);
}


static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) //initialize bus config
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_main(void)
{

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);//initialize i2c communication
    ESP_LOGI(TAG, "I2C initialized successfully");


    uint8_t reg = 0x75;
    uint8_t data[10];
    uint8_t gdata[10];
    ESP_ERROR_CHECK(mpuRWFromReg(dev_handle, reg, data, 1));
    ESP_LOGI(TAG, "value: %X", data[0]);



    ESP_ERROR_CHECK(mpuWriteReg(dev_handle, 0x6B, 0));//whoami register
    ESP_ERROR_CHECK(mpuWriteReg(dev_handle, 0x19, 7));//sample rate register
    ESP_ERROR_CHECK(mpuWriteReg(dev_handle, 0x1C, 1<<3));//configure accelerometer to +- 4g


    // int64_t last_time = esp_timer_get_time();
    while(1){

        // int64_t now = esp_timer_get_time();
        // int64_t dt = now-last_time;
        // last_time = now;

        mpuRWFromReg(dev_handle, 0x3B, data, 6);

        int16_t RAWX = data[0] << 8 | data[1];//accelerometer data
        int16_t RAWY = data[2] << 8 | data[3];
        int16_t RAWZ = data[4] << 8 | data[5];

        float accel_divider = 8192;
        float xg = (float)RAWX/accel_divider;
        float yg = (float)RAWY/accel_divider;
        float zg = (float)RAWZ/accel_divider;

        


        mpuRWFromReg(dev_handle, 0x43, gdata, 6);
        int16_t GYX = gdata[0] << 8 | gdata[1];
        int16_t GYY = gdata[2] << 8 | gdata[3];
        int16_t GYZ = gdata[4] << 8 | gdata[5];
        float gyro_divider = 131;

        float x_dps = (float)GYX/gyro_divider;
        float y_dps = (float)GYY/gyro_divider;
        float z_dps = (float)GYZ/gyro_divider;


        ESP_LOGI(TAG, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", xg, yg, zg, x_dps, y_dps, z_dps);//ax ay az gx gy gz
 

        vTaskDelay(pdMS_TO_TICKS(10));
    }






}
