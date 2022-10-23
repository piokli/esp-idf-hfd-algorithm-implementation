/*
 * i2c_helper.h
 *
 *  Created on: 2 gru 2019
 *      Author: Piokli
 */

#ifndef COMPONENTS_I2C_HELPER_I2C_HELPER_H_
#define COMPONENTS_I2C_HELPER_I2C_HELPER_H_

#include <esp_err.h>
#include <driver/i2c.h>

/* I2C Master's Settings */
#define I2C_MASTER_NUM 1 				// as it was default in I2C's example sdkconfig.h; it is master's port number - maybe it's because there can be more
#define I2C_MASTER_SDA_IO 23			// gpio number for i2c slave data
#define I2C_MASTER_SCL_IO 22			// gpio number for i2c slave clock
#define I2C_MASTER_FREQ_HZ 100000		// default as in sdkconfig.h of I2C's example 100kH is max for barometer in normal mode
#define I2C_MASTER_RX_BUF_DISABLE 0		// I2C master doesn't need buffer
#define I2C_MASTER_TX_BUF_DISABLE 0 	// I2C master doesn't need buffer

/* Useful Macros */
#define WRITE_BIT I2C_MASTER_WRITE      // I2C master write
#define READ_BIT I2C_MASTER_READ        // I2C master read
#define ACK_CHECK_EN 0x1				// I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0				// I2C master will not check ack from slave
#define ACK_VAL 0x0                     // I2C ack value
#define NACK_VAL 0x1                    // I2C nack value


/*
 * @brief Initiates I2C with defined settings
 *        @note
 *        Call once before using I2C functionality
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_helper_master_init(void);

/*
 * @brief Write buffer data to chosen register
 *
 * @param slave_id slave's I2C address
 * @param reg_addr chosen register
 * @param data_wr pointer to data buffer
 * @param size how many bytes has data_wr
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_helper_write_reg(uint8_t slave_id, uint8_t reg_addr, uint8_t *data_wr, size_t size);

/*
 * @brief Read data from chosen register
 *
 * @param slave_id slave's I2C address
 * @param reg_addr chosen register
 * @param data_rd pointer to where store the data
 * @param size how many bytes to read
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_helper_read_reg(uint8_t slave_id, uint8_t reg_addr, uint8_t *data_rd, size_t size);

#endif /* COMPONENTS_I2C_HELPER_I2C_HELPER_H_ */
