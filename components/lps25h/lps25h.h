/*
 * lps25h.h
 *
 *  Created on: 2 gru 2019
 *      Author: Piokli
 */

#ifndef COMPONENTS_LPS25H_LPS25H_H_
#define COMPONENTS_LPS25H_LPS25H_H_

#include <esp_err.h>

/* LPS25H Register Macros */
#define LPS25H_REF_P_XL_ADDR          0x08
#define	LPS25H_REF_P_L_ADDR           0x09
#define	LPS25H_REF_P_H_ADDR           0x0A

#define	LPS25H_WHO_AM_I_ADDR          0x0F
#define	LPS25H_RES_CONF_ADDR          0x10

#define	LPS25H_CTRL_REG1_ADDR         0x20
#define	LPS25H_CTRL_REG2_ADDR         0x21
#define	LPS25H_CTRL_REG3_ADDR         0x22
#define	LPS25H_CTRL_REG4_ADDR         0x23
#define	LPS25H_INT_CFG_ADDR           0x24
#define	LPS25H_INT_SOURCE_ADDR        0x25

#define	LPS25H_STATUS_REG_ADDR        0x27
#define	LPS25H_PRESS_OUT_XL_ADDR      0x28
#define	LPS25H_PRESS_OUT_L_ADDR       0x29
#define	LPS25H_PRESS_OUT_H_ADDR       0x2A
#define	LPS25H_TEMP_OUT_L_ADDR        0x2B
#define	LPS25H_TEMP_OUT_H_ADDR        0x2C

#define	LPS25H_FIFO_CTRL_ADDR         0x2E
#define	LPS25H_FIFO_STATUS_ADDR       0x2F
#define	LPS25H_THIS_P_L_ADDR          0x30
#define	LPS25H_THIS_P_H_ADDR          0x31

#define	LPS25H_RPDS_L_ADDR            0x39
#define	LPS25H_RPDS_H_ADDR            0x3A

/* LPS25H I2C Address */
#define LPS25H_I2C_ADDR 0x5D

/* LPS25H WHO_AM_I identifier */
#define LPS25H_WHO_ID 0xBD     // add primary and secondary

//-------------------------------------------------------------

/* LPS25H CTRL_REG1 Settings */    //@TODO define all settings from datasheet
#define LPS25H_POWER_UP     0x80
#define LPS25H_BDU_SET      0x04

#define LPS25H_DATA_OUTPUT_RATE_ONE_SHOT     0x00
#define LPS25H_DATA_OUTPUT_RATE_1_HZ         0x10
#define LPS25H_DATA_OUTPUT_RATE_7_HZ         0x20
#define LPS25H_DATA_OUTPUT_RATE_12_5_HZ      0x30
#define LPS25H_DATA_OUTPUT_RATE_25_HZ        0x40

/* LPS25H STATUS_REG Options */
#define LPS25H_PRESS_DATA_AVAIABLE    0x02
#define LPS25H_TEMP_DATA_AVAIABLE     0x01
#define LPS25H_PRESS_DATA_OVERRUN     0x10
#define LPS25H_TEMP_DATA_OVERRUN      0x08

/* LPS25H Data Resolution Configuration (Internal Average) */
#define LPS25H_PRESS_AVG_8      0x00
#define LPS25H_PRESS_AVG_32     0x01
#define LPS25H_PRESS_AVG_128    0x02
#define LPS25H_PRESS_AVG_512    0x03
#define LPS25H_TEMP_AVG_8       0x00
#define LPS25H_TEMP_AVG_16      0x04
#define LPS25H_TEMP_AVG_32      0x08
#define LPS25H_TEMP_AVG_64      0x0C

/* LPS25H FIFO_CTRL Options */
#define LPS25H_BYPASS_MODE      		0x00
#define LPS25H_FIFO_MODE      			0x20
#define LPS25H_STREAM_MODE      		0x40
#define LPS25H_STREAM_TO_FIFO_MODE      0x60
#define LPS25H_BYPASS_TO_STREAM_MODE    0x80
//#define LPS25H_UNAVAIABLE     		0xA0
#define LPS25H_FIFO_MEAN_MODE      		0xC0
#define LPS25H_BYPASS_TO_FIFO_MODE      0xE0
//number of samples for moving average (watermark levels WTM_POINT[4:0])
#define LPS25H_FIFO_MEAN_SAMPLES_2		0x01
#define LPS25H_FIFO_MEAN_SAMPLES_4      0x03
#define LPS25H_FIFO_MEAN_SAMPLES_8      0x07
#define LPS25H_FIFO_MEAN_SAMPLES_16   	0x0F
#define LPS25H_FIFO_MEAN_SAMPLES_32   	0x1F

/* LPS25H Functions */

esp_err_t lps25h_test_connection(void);

esp_err_t lps25h_default_setup(void);

esp_err_t lps25h_read_press_raw(uint32_t *pressure);

#endif /* COMPONENTS_LPS25H_LPS25H_H_ */
