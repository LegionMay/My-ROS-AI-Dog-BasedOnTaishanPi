#include "robocontrol.h"
#include "servo_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>


#define M_PI 3.14159265358979323846

// 全局状态变量
volatile RobotAction current_action = ACTION_STOP;

// 控制机器人动作的函数
void Move_Forward() { current_action = ACTION_FORWARD; }
void Move_Backward() { current_action = ACTION_BACKWARD; }
void Turn_Left() { current_action = ACTION_TURN_LEFT; }
void Turn_Right() { current_action = ACTION_TURN_RIGHT; }
void Stop() { current_action = ACTION_STOP; }

/*** 逆运动学：通过给定的足尖位置，计算关节角度 ***/
void InverseKinematics(float x, float y, float* theta1, float* theta2) {
    float L1 = LEG_UPPER_LENGTH;
    float L2 = LEG_LOWER_LENGTH;

    float D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    float phi2 = acosf(D);  // 小腿角度
    float phi1 = atan2f(y, x) - atan2f(L2 * sinf(phi2), L1 + L2 * cosf(phi2));  // 大腿角度

    *theta1 = phi1;  // 大腿角度
    *theta2 = phi2;  // 小腿角度
}

/*** 计算步态：根据时间计算每一步的足尖位置 ***/
void CalculateGait(float time, float* x, float* y) {
    float stepLength = 50.0;  // 步长，单位：毫米
    float stepHeight = 30.0;  // 抬脚高度，单位：毫米

    if (time < 0.5) {
        // 抬脚并向前移动
        *x = stepLength * (2 * time);
        *y = stepHeight * sinf(M_PI * time);
    } else {
        // 放下脚并向后滑动
        *x = stepLength * (2 * (1.0 - time));
        *y = 0;
    }
}

/*** 舵机标号获取函数 ***/
ServoID GetServoIDForLeg(int leg, bool isUpperLeg) {
    if (isUpperLeg) {
        return (ServoID)((leg * 2) + 1);  // 大腿舵机（双数）
    } else {
        return (ServoID)((leg * 2));  // 小腿舵机（单数）
    }
}

/*** 前进步态控制 ***/
void Gait_Forward(void) {
    for (int leg = 0; leg < NUM_SERVOS / 2; leg++) {  // 每个舵机一条腿（NUM_SERVOS / 2 表示四条腿）
        float time = (leg == 0 || leg == 3) ? 0.0 : 0.5;  // 对角腿同步：Leg 0 和 Leg 3 相位为0，Leg 1 和 Leg 2 相位为0.5
        float x, y;
        CalculateGait(time, &x, &y);  // 计算足尖位置

        float theta1, theta2;
        InverseKinematics(x, y, &theta1, &theta2);  // 计算关节角度

        // 局部变量定义，控制大腿和小腿舵机
        ServoID upperLegServo = GetServoIDForLeg(leg, true);  // 获取大腿舵机
        ServoID lowerLegServo = GetServoIDForLeg(leg, false);  // 获取小腿舵机

        Set_Servo_Angle(upperLegServo, theta1);  // 控制大腿舵机
        Set_Servo_Angle(lowerLegServo, theta2);  // 控制小腿舵机
    }
}

/*** 后退步态控制 ***/
void Gait_Backward(void) {
    for (int leg = 0; leg < NUM_SERVOS / 2; leg++) {
        float time = (leg == 0 || leg == 3) ? 0.0 : 0.5;  // 对角腿同步
        float x, y;
        CalculateGait(1.0 - time, &x, &y);  // 反向运动

        float theta1, theta2;
        InverseKinematics(x, y, &theta1, &theta2);  // 计算关节角度

        // 局部变量定义，控制大腿和小腿舵机
        ServoID upperLegServo = GetServoIDForLeg(leg, true);
        ServoID lowerLegServo = GetServoIDForLeg(leg, false);

        Set_Servo_Angle(upperLegServo, theta1);  // 控制大腿舵机
        Set_Servo_Angle(lowerLegServo, theta2);  // 控制小腿舵机
    }
}

/*** 左转步态控制 ***/
void Gait_Turn_Left(void) {
    for (int leg = 0; leg < NUM_SERVOS / 2; leg++) {
        float time = (leg == 0 || leg == 3) ? 0.0 : 0.5;
        float x, y;

        // 左侧腿向后，右侧腿向前
        if (leg == 0 || leg == 3) {
            CalculateGait(1.0 - time, &x, &y);  // 左侧腿后移
        } else {
            CalculateGait(time, &x, &y);  // 右侧腿前移
        }

        float theta1, theta2;
        InverseKinematics(x, y, &theta1, &theta2);

        // 局部变量定义，控制大腿和小腿舵机
        ServoID upperLegServo = GetServoIDForLeg(leg, true);
        ServoID lowerLegServo = GetServoIDForLeg(leg, false);

        Set_Servo_Angle(upperLegServo, theta1);
        Set_Servo_Angle(lowerLegServo, theta2);
    }
}

/*** 右转步态控制 ***/
void Gait_Turn_Right(void) {
    for (int leg = 0; leg < NUM_SERVOS / 2; leg++) {
        float time = (leg == 0 || leg == 3) ? 0.0 : 0.5;
        float x, y;

        // 左侧腿向前，右侧腿向后
        if (leg == 0 || leg == 3) {
            CalculateGait(time, &x, &y);  // 左侧腿前移
        } else {
            CalculateGait(1.0 - time, &x, &y);  // 右侧腿后移
        }

        float theta1, theta2;
        InverseKinematics(x, y, &theta1, &theta2);

        // 局部变量定义，控制大腿和小腿舵机
        ServoID upperLegServo = GetServoIDForLeg(leg, true);
        ServoID lowerLegServo = GetServoIDForLeg(leg, false);

        Set_Servo_Angle(upperLegServo, theta1);
        Set_Servo_Angle(lowerLegServo, theta2);
    }
}

/*** 原地踏步步态控制 ***/
void Gait_March_In_Place(void) {
    for (int leg = 0; leg < NUM_SERVOS / 2; leg++) {
        float time = (leg == 0 || leg == 3) ? 0.0 : 0.5;
        float x, y;

        CalculateGait(time, &x, &y);  // 在原地移动

        float theta1, theta2;
        InverseKinematics(x, y, &theta1, &theta2);

        // 局部变量定义，控制大腿和小腿舵机
        ServoID upperLegServo = GetServoIDForLeg(leg, true);
        ServoID lowerLegServo = GetServoIDForLeg(leg, false);

        Set_Servo_Angle(upperLegServo, theta1);  // 控制大腿舵机
        Set_Servo_Angle(lowerLegServo, theta2);  // 控制小腿舵机
    }
}
