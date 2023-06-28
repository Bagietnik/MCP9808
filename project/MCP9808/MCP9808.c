#include <stdio.h>
#include "MCP9808.h"

void funct(void)
{
    printf("Test");
}

uint16_t RegBitVal[2][16];

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mcp9808_register_bits_mapping(uint8_t reg_addr)
{
    uint8_t data[2];
    uint16_t register_value;
    esp_err_t ret;

    ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    register_value = data[0] | data[1];
   
    return ESP_OK;
}


/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mcp9808_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Read a 16-bit temperature value from the MCP9808 sensor
 */
esp_err_t mcp9808_read_temperature(float *temperature)
{
    uint8_t data[2] = {0,0};
    esp_err_t ret;

    // Set the register address to read the temperature
    uint8_t reg_addr = MPU9250_TEMP_REG;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    data[0] = data[0] & 0x1F; //Masking 3 bits from UpperByte

    if((data[0] & 0x10) == 0x10)
    {
        data[0] = data[0] & 0x0F; //Clear SIGN
        *temperature = 256.0 - (((data[0] * 16.0) + (data[1] / 16.0)));
    }else{
        *temperature = ((data[0] * 16.0) + (data[1] / 16.0));
    }

    return ESP_OK;
}

/**
 * @brief Read a 8 bit resolution of temperature measurment value from the MCP9808 sensor
 */
esp_err_t mcp9808_read_resolution(uint8_t *resolution)
{
    uint8_t data[1];
    esp_err_t ret;

    uint8_t reg_addr = MPU9250_RES_REG;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }

    *resolution = data[0];
    
    return ESP_OK;
}

/**
 * @brief Read a single bit value from the given byte
 */
esp_err_t mcp9808_single_bit_read(uint8_t *bit_value, uint8_t bit_id, uint8_t reg_addr)
{
    uint8_t _reg_addr = reg_addr;
    uint8_t _bit_id = bit_id;
    uint8_t data[2];
    uint16_t reg_value = 0;

    esp_err_t ret;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &_reg_addr, 1, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    reg_value = data[0];
    
    *bit_value = (reg_value & (1 << _bit_id)) >> _bit_id;
    
    return ESP_OK;
}


/*******************************************************************************/

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t mcp9808_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

