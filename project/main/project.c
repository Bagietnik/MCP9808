#include <stdio.h>
#include "MCP9808.h"

static const char *TAG = "i2c-mcp9808";

void app_main(void)
{
    float temperature;
    uint8_t resolution;
    uint8_t  bit_value;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(mcp9808_register_write_byte(MPU9250_RES_REG, MPU9250_RES_0_0625));
    ESP_ERROR_CHECK(mcp9808_read_resolution(&resolution));
    ESP_LOGI(TAG, "The resolution has been saved correctly: %d", resolution);

    mcp9808_register_bits_mapping(0x02);

    ESP_ERROR_CHECK(mcp9808_register_write_byte(MPU9250_TEMP_UPPER_LIMIT_REG, 0x73));
    ESP_LOGI(TAG, "Upper Temp saved succesfully");

    while(1)
    {
        ESP_ERROR_CHECK(mcp9808_read_temperature(&temperature));
        ESP_LOGI(TAG, "Temperature: %f", temperature);


        mcp9808_register_bits_mapping(0x02);
        ESP_LOGI(TAG, "Successful completion of mapping");
        /*
        ESP_ERROR_CHECK(mcp9808_single_bit_read(&bit_value, 0, MPU9250_CONF_REG));
        ESP_LOGI(TAG, "Bit value: %d", bit_value);
        */

        vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS);

    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}