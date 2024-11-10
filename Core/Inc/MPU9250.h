/*
 *  Created on: Feb 28, 2019
 *      Author: Desert
 */

#ifndef MPU9250_H
#define MPU9250_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#define _MPU9250_I2C		hi2c1

extern I2C_HandleTypeDef _MPU9250_I2C;



typedef float float32_t;

typedef struct {
    int16_t accel[3];  // 加速度计数据
    int16_t gyro[3];   // 陀螺仪数据
    int16_t mag[3];    // 磁力计数据
    float temp;        // 温度数据
} IMUData;


typedef enum GyroRange_ {
    GYRO_RANGE_250DPS = 0,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS,
} GyroRange;

typedef enum AccelRange_ {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G,
} AccelRange;

typedef enum DLPFBandwidth_ {
    DLPF_BANDWIDTH_184HZ = 0,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ,
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
    LP_ACCEL_ODR_0_24HZ = 0,
    LP_ACCEL_ODR_0_49HZ,
    LP_ACCEL_ODR_0_98HZ,
    LP_ACCEL_ODR_1_95HZ,
    LP_ACCEL_ODR_3_91HZ,
    LP_ACCEL_ODR_7_81HZ,
    LP_ACCEL_ODR_15_63HZ,
    LP_ACCEL_ODR_31_25HZ,
    LP_ACCEL_ODR_62_50HZ,
    LP_ACCEL_ODR_125HZ,
    LP_ACCEL_ODR_250HZ,
    LP_ACCEL_ODR_500HZ,
} SampleRateDivider;

uint8_t MPU9250_Init(void);
/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetData(int16_t* AccData, int16_t* MagData, int16_t* GyroData, float32_t *TempData);

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd);
/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth);
/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range);
/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range);

#endif /* MPU9250_H_ */


