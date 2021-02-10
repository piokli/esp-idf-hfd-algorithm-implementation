/*
 * lps25h.c
 *
 *  Created on: 2 gru 2019
 *      Author: Piokli
 */

#include "lps25h.h"
#include <driver/i2c.h>
#include "../i2c_helper/i2c_helper.h"
#include "esp_log.h"

static const char* TAG = "lps25h";

esp_err_t lps25h_test_connection(void)
{
	uint8_t get_id;

	esp_err_t ret = i2c_helper_read_reg(LPS25H_I2C_ADDR, LPS25H_WHO_AM_I_ADDR, &get_id, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    if (get_id != LPS25H_WHO_ID)
    {
    	ESP_LOGW(TAG, "Failed to connect to LPS25H!");
    	return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Connected to LPS25H");

    return ESP_OK;
}

esp_err_t lps25h_default_setup(void)
{
	uint8_t ctrl_reg1_setup = LPS25H_POWER_UP |
			                  LPS25H_DATA_OUTPUT_RATE_25_HZ;

	uint8_t res_conf_setup = LPS25H_PRESS_AVG_512 |
			                 LPS25H_TEMP_AVG_16;

	// Forgot to implement FIFO Mean Mode on the first time...

	uint8_t fifo_ctrl_setup = LPS25H_FIFO_MEAN_MODE |
			              	  LPS25H_FIFO_MEAN_SAMPLES_16;

	esp_err_t ret = i2c_helper_write_reg(LPS25H_I2C_ADDR, LPS25H_CTRL_REG1_ADDR, &ctrl_reg1_setup, 1); // size equals 1, maybe should create a variable?
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_helper_write_reg(LPS25H_I2C_ADDR, LPS25H_FIFO_CTRL_ADDR, &fifo_ctrl_setup, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_helper_write_reg(LPS25H_I2C_ADDR, LPS25H_RES_CONF_ADDR, &res_conf_setup, 1);

	return ret;
}

esp_err_t lps25h_read_press_raw(uint32_t *pressure)
{

	uint8_t buff_size = 3;
	//All declarations below work:
	//uint8_t* buff = (uint8_t *)malloc(buff_size);
	//uint8_t buff[buff_size]; // <- reference to it with &buff
	uint8_t *buff = malloc(sizeof(uint8_t) * buff_size);

	// MSB enables address auto increment! (p. 15 LPS25H datasheet)
	esp_err_t ret = i2c_helper_read_reg(LPS25H_I2C_ADDR, LPS25H_PRESS_OUT_XL_ADDR | (1 << 7), buff, buff_size);
    if (ret != ESP_OK) {
        return ret;
    }
    //ESP_LOGI(TAG, "p_h: %d p_l: %d p_xl: %d", buff[2], buff[1], buff[0]);
    //ESP_LOGI(TAG, "%f", (((buff[2] << 16) | (buff[1] << 8) | buff[0])) / 4096.0);
    *pressure = (uint32_t)((buff[2] << 16) | (buff[1] << 8) | buff[0]);

    free(buff);

    return ret;
}
