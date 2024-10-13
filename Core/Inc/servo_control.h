// servo_control.h
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#define LEG_UPPER_LENGTH 80.9 // 大腿长度，单位：毫米
#define LEG_LOWER_LENGTH 68.75 // 小腿长度，单位：毫米
#define NUM_SERVOS 8
#define MAX_ROTATION_TIME_PER_DEGREE 1000.0f / 545.0f

// 舵机标号枚举
typedef enum {
    H1 = 0,
    H2,
    H3,
    H4,
    H5,
    H6,
    H7,
    H8,
    H9
} ServoID;

typedef struct {
    ServoID servo;
    float target_angle;
    float speed;
    float remaining_time;
    bool is_active;
} ServoControl;

extern ServoControl servos[8];


// 初始化舵机
void Servo_Init(void);
void Set_Servo_Angle(ServoID servo, float angle) ;
void Init_Servos();


#endif // SERVO_CONTROL_H