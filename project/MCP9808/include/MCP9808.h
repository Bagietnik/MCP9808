#ifndef MCP9808_H
#define MCP9808_H

#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define MCP9808_SET_RES             CONFIG_MCP9808_RESOLUTION
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define SAMPLE_TIME                 10000                      /*Temp value sampling every 10 seconds*/


#define MPU9250_SENSOR_ADDR               0x18       /*!< Slave address of the MPU9250 sensor */
#define MPU9250_ID_REG                    0x06          
                
#define MPU9250_CONF_REG_ADDR               0x01       /*Sensor configuration register*/
#define MPU9250_TEMP_UPPER_LIMIT_REG        0x02
#define MPU9250_TEMP_LOWER_LIMIT_REG        0x03
#define MPU9250_TEMP_CRITICAL_LIMIT_REG     0x04
#define MPU9250_TEMP_REG                    0x05       /*Ambient temperature value*/
#define MPU9250_RES_REG                     0x08       /*Resolution register*/


#define MPU9250_RES_0_5                 0x00       /*+0.5 C*/
#define MPU9250_RES_0_25                0x01       /*+0.25 C*/
#define MPU9250_RES_0_125               0x02       /*+0.125 C*/
#define MPU9250_RES_0_0625              0x03       /*+0.0625 C*/

void funct(void);

esp_err_t mcp9808_register_bits_mapping(uint8_t reg_addr);
esp_err_t mcp9808_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_master_init(void);
esp_err_t mcp9808_read_temperature(float *temperature);
esp_err_t mcp9808_read_resolution(uint8_t *resolution);
esp_err_t mcp9808_single_bit_read(uint8_t *bit_value, uint8_t bit_id, uint8_t reg_addr);
esp_err_t mcp9808_register_write_byte(uint8_t reg_addr, uint8_t data);

#endif