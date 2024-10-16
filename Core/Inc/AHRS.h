#ifndef AHRS_H
#define AHRS_H

#include "MPU9250.h"
#include <stdint.h>

void AHRS_Init(void);
void AHRS_Update(void);
void AHRS_GetQuaternion(float* quat);
float MedianFilter(float input);
int compare_int16(const void* a, const void* b);

#endif
