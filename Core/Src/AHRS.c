#include "AHRS.h"
#include "MPU9250.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float w1 = 0.0f, w2 = 0.0f, w3 = 0.0f;
static uint32_t lastUpdate = 0, now = 0;

float invSqrt(float x) { return 1.0f / sqrtf(x); }

void AHRS_Init() {
    MPU9250_Init();
    MPU9250_SetAccelRange(ACCEL_RANGE_4G);    // 设置加速度计量程为 4G
    MPU9250_SetGyroRange(GYRO_RANGE_500DPS);  // 设置陀螺仪量程为 500DPS
    MPU9250_SetDLPFBandwidth(DLPF_BANDWIDTH_92HZ); // 设置低通滤波带宽为 92Hz
    MPU9250_SetSampleRateDivider(LP_ACCEL_ODR_125HZ); // 设置采样率为 125Hz
}

void AHRS_GetQuaternion(float* quat) {
    quat[0] = q0;
    quat[1] = q1;
    quat[2] = q2;
    quat[3] = q3;
}

void AHRS_Update() {
    IMUData imuData;
    MPU9250_GetData(imuData.accel, imuData.mag, imuData.gyro, NULL);

    // 获取加速度、陀螺仪和磁力计原始数据
    float ax = imuData.accel[0] * 4.0f * 9.81f / 32768.0f; // 4G 量程
    float ay = imuData.accel[1] * 4.0f * 9.81f / 32768.0f;
    float az = imuData.accel[2] * 4.0f * 9.81f / 32768.0f;
    float gx = imuData.gyro[0] * 500.0f / 32768.0f * M_PI / 180.0f; // 500DPS 量程
    float gy = imuData.gyro[1] * 500.0f / 32768.0f * M_PI / 180.0f;
    float gz = imuData.gyro[2] * 500.0f / 32768.0f * M_PI / 180.0f;
    float mx = imuData.mag[0] * 0.146f; // 将原始数据转换为 µT
    float my = imuData.mag[1] * 0.146f;
    float mz = imuData.mag[2] * 0.146f;

    // 时间更新
    now = HAL_GetTick();
    float deltaTime = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    // 数据预处理：加速度和磁力计的中值滤波
    ax = MedianFilter(ax);
    ay = MedianFilter(ay);
    az = MedianFilter(az);
    mx = MedianFilter(mx);
    my = MedianFilter(my);
    mz = MedianFilter(mz);

    // 归一化加速度计数据
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 归一化磁力计数据
    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // 陀螺仪数据减去偏差
    gx -= w1;
    gy -= w2;
    gz -= w3;

    // 四元数预测：一阶龙格库塔法
    float halfT = deltaTime / 2.0f;
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 归一化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    // 互补滤波：使用加速度计修正陀螺仪
    const float alpha = 0.98f;  // 滤波系数
    float pitch = asinf(2 * (q0 * q2 - q3 * q1));
    float roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));

    float pitchAccel = atan2f(ay, az);
    float rollAccel = atan2f(ax, sqrt(ay * ay + az * az));

    pitch = alpha * pitch + (1.0f - alpha) * pitchAccel;
    roll = alpha * roll + (1.0f - alpha) * rollAccel;

    // 重新计算四元数（根据修正的 pitch 和 roll）
    float sinHalfRoll = sinf(roll / 2.0f);
    float cosHalfRoll = cosf(roll / 2.0f);
    float sinHalfPitch = sinf(pitch / 2.0f);
    float cosHalfPitch = cosf(pitch / 2.0f);

    q0 = cosHalfRoll * cosHalfPitch;
    q1 = sinHalfRoll * cosHalfPitch;
    q2 = cosHalfRoll * sinHalfPitch;
    q3 = sinHalfRoll * sinHalfPitch;
}

int compare_float(const void* a, const void* b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);  // 如果fa > fb返回正数，如果fa < fb返回负数，若相等则返回0
}

float MedianFilter(float input) {
    static float buffer[3];  // 缓存3个数据
    static int index = 0;

    // 将输入的数据存入缓冲区
    buffer[index] = input;
    index = (index + 1) % 3;  // 循环更新index

    // 排序并返回中间值
    float sorted[3];
    memcpy(sorted, buffer, sizeof(buffer));  // 拷贝数据
    qsort(sorted, 3, sizeof(float), compare_float);  // 排序
    return sorted[1];  // 返回中间值
}