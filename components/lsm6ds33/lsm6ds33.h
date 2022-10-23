/*
 * lsm6ds33.h
 *
 *  Created on: 30 lis 2019
 *      Author: Piokli
 */

#ifndef COMPONENTS_LSM6DS33_LSM6DS33_H_
#define COMPONENTS_LSM6DS33_LSM6DS33_H_

#include <driver/i2c.h>
#include <esp_err.h>
#include "../i2c_helper/i2c_helper.h"

/* LSM6DS33 Register Macros */
#define LSM6DS33_FUNC_CFG_ACCESS_ADDR    0x01

#define LSM6DS33_FIFO_CTRL1_ADDR         0x06
#define LSM6DS33_FIFO_CTRL2_ADDR         0x07
#define LSM6DS33_FIFO_CTRL3_ADDR         0x08
#define LSM6DS33_FIFO_CTRL4_ADDR         0x09
#define LSM6DS33_FIFO_CTRL5_ADDR         0x0A
#define LSM6DS33_ORIENT_CFG_G_ADDR       0x0B

#define LSM6DS33_INT1_CTRL_ADDR          0x0D
#define LSM6DS33_INT2_CTRL_ADDR          0x0E
#define LSM6DS33_WHO_AM_I_ADDR           0x0F
#define LSM6DS33_CTRL1_XL_ADDR           0x10
#define LSM6DS33_CTRL2_G_ADDR            0x11
#define LSM6DS33_CTRL3_C_ADDR            0x12
#define LSM6DS33_CTRL4_C_ADDR            0x13
#define LSM6DS33_CTRL5_C_ADDR            0x14
#define LSM6DS33_CTRL6_C_ADDR            0x15
#define LSM6DS33_CTRL7_G_ADDR            0x16
#define LSM6DS33_CTRL8_XL_ADDR           0x17
#define LSM6DS33_CTRL9_XL_ADDR           0x18
#define LSM6DS33_CTRL10_C_ADDR           0x19

#define LSM6DS33_WAKE_UP_SRC_ADDR        0x1B
#define LSM6DS33_TAP_SRC_ADDR            0x1C
#define LSM6DS33_D6D_SRC_ADDR            0x1D
#define LSM6DS33_STATUS_REG_ADDR         0x1E

#define LSM6DS33_OUT_TEMP_L_ADDR         0x20
#define LSM6DS33_OUT_TEMP_H_ADDR         0x21
#define LSM6DS33_OUTX_L_G_ADDR           0x22
#define LSM6DS33_OUTX_H_G_ADDR           0x23
#define LSM6DS33_OUTY_L_G_ADDR           0x24
#define LSM6DS33_OUTY_H_G_ADDR           0x25
#define LSM6DS33_OUTZ_L_G_ADDR           0x26
#define LSM6DS33_OUTZ_H_G_ADDR           0x27
#define LSM6DS33_OUTX_L_XL_ADDR          0x28
#define LSM6DS33_OUTX_H_XL_ADDR          0x29
#define LSM6DS33_OUTY_L_XL_ADDR          0x2A
#define LSM6DS33_OUTY_H_XL_ADDR          0x2B
#define LSM6DS33_OUTZ_L_XL_ADDR          0x2C
#define LSM6DS33_OUTZ_H_XL_ADDR          0x2D

#define LSM6DS33_FIFO_STATUS1_ADDR       0x3A
#define LSM6DS33_FIFO_STATUS2_ADDR       0x3B
#define LSM6DS33_FIFO_STATUS3_ADDR       0x3C
#define LSM6DS33_FIFO_STATUS4_ADDR       0x3D
#define LSM6DS33_FIFO_DATA_OUT_L_ADDR    0x3E
#define LSM6DS33_FIFO_DATA_OUT_H_ADDR    0x3F
#define LSM6DS33_TIMESTAMP0_REG_ADDR     0x40
#define LSM6DS33_TIMESTAMP1_REG_ADDR     0x41
#define LSM6DS33_TIMESTAMP2_REG_ADDR     0x42

#define LSM6DS33_STEP_TIMESTAMP_L_ADDR   0x49
#define LSM6DS33_STEP_TIMESTAMP_H_ADDR   0x4A
#define LSM6DS33_STEP_COUNTER_L_ADDR     0x4B
#define LSM6DS33_STEP_COUNTER_H_ADDR     0x4C

#define LSM6DS33_FUNC_SRC_ADDR           0x53

#define LSM6DS33_TAP_CFG_ADDR            0x58
#define LSM6DS33_TAP_THS_6D_ADDR         0x59
#define LSM6DS33_INT_DUR2_ADDR           0x5A
#define LSM6DS33_WAKE_UP_THS_ADDR        0x5B
#define LSM6DS33_WAKE_UP_DUR_ADDR        0x5C
#define LSM6DS33_FREE_FALL_ADDR          0x5D
#define LSM6DS33_MD1_CFG_ADDR            0x5E
#define LSM6DS33_MD2_CFG_ADDR            0x5F

/* LSM6DS33 I2C Address */
#define LSM6DS33_I2C_ADDR 0x6B

/* LSM6DS33 WHO_AM_I identifier */
#define LSM6DS33_WHO_ID 0x69			// add primary and secondary addr

//----------------------------------------------------------------------

/* LSM6DS33 CTRL1_XL Option Masks (Accelerometer) */
#define LSM6DS33_ACC_POWER_DOWN            0x00
#define LSM6DS33_ACC_DATA_RATE_12_5_HZ     0x10
#define LSM6DS33_ACC_DATA_RATE_26_HZ       0x20
#define LSM6DS33_ACC_DATA_RATE_52_HZ       0x30
#define LSM6DS33_ACC_DATA_RATE_104_HZ      0x40
#define LSM6DS33_ACC_DATA_RATE_208_HZ      0x50
#define LSM6DS33_ACC_DATA_RATE_416_HZ      0x60
#define LSM6DS33_ACC_DATA_RATE_833_HZ      0x70
#define LSM6DS33_ACC_DATA_RATE_1_66_KHZ    0x80
#define LSM6DS33_ACC_DATA_RATE_3_33_KHZ    0x90
#define LSM6DS33_ACC_DATA_RATE_6_66_KHZ    0xA0

#define LSM6DS33_ACC_FULL_SCALE_2_G        0x00
#define LSM6DS33_ACC_FULL_SCALE_16_G       0x04
#define LSM6DS33_ACC_FULL_SCALE_4_G        0x08
#define LSM6DS33_ACC_FULL_SCALE_8_G        0x0C

#define LSM6DS33_ACC_AA_BANDWIDTH_400_HZ   0x00
#define LSM6DS33_ACC_AA_BANDWIDTH_200_HZ   0x01
#define LSM6DS33_ACC_AA_BANDWIDTH_100_HZ   0x02
#define LSM6DS33_ACC_AA_BANDWIDTH_50_HZ    0x03

/* LSM6DS33 CTRL1_XL Option Masks (Gyroscope) */
#define LSM6DS33_GYRO_POWER_DOWN             0x00
#define LSM6DS33_GYRO_DATA_RATE_12_5_HZ      0x10
#define LSM6DS33_GYRO_DATA_RATE_26_HZ        0x20
#define LSM6DS33_GYRO_DATA_RATE_52_HZ        0x30
#define LSM6DS33_GYRO_DATA_RATE_104_HZ       0x40
#define LSM6DS33_GYRO_DATA_RATE_208_HZ       0x50
#define LSM6DS33_GYRO_DATA_RATE_416_HZ       0x60
#define LSM6DS33_GYRO_DATA_RATE_833_HZ       0x70
#define LSM6DS33_GYRO_DATA_RATE_1_66_KHZ     0x80

#define LSM6DS33_GYRO_FULL_SCALE_250_DPS     0x00
#define LSM6DS33_GYRO_FULL_SCALE_500_DPS     0x04
#define LSM6DS33_GYRO_FULL_SCALE_1000_DPS    0x08
#define LSM6DS33_GYRO_FULL_SCALE_2000_DPS    0x0C
#define LSM6DS33_GYRO_FULL_SCALE_125_DPS     0x02


/* Structure for better keeping the x, y and z values of accelerometer and gyroscope */
struct vector {
	float x;
	float y;
	float z;
};

/* LSM6DS33 functions */

esp_err_t lsm6ds33_test_connection(void);

esp_err_t lsm6ds33_default_setup(void);

esp_err_t lsm6ds33_read_acc_raw(struct vector *a);

esp_err_t lsm6ds33_read_gyro_raw(struct vector *g);

void lsm6ds33_vector_calculate_acc_raw(struct vector *a);

void lsm6ds33_vector_calculate_gyro_raw(struct vector *g);

void lsm6ds33_vector_normalise(struct vector *a);

float lsm6ds33_vector_magnitude_of(struct vector v);


#endif /* COMPONENTS_LSM6DS33_LSM6DS33_H_ */
