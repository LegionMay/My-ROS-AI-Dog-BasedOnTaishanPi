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

/*** 计算步态：根据时间计算每一步的足尖位置 ***/
void CalculateGait(float time, float* x, float* y) {
    float stepLength = 20.0;  // 步长，单位：毫米
    float stepHeight = 15.0;  // 抬脚高度，单位：毫米

    if (time < 0.5) {
        // 大腿向前摆动时，小腿逐渐伸展
        *x = stepLength * (2 * time);  // 向前移动
        *y = stepHeight * sinf(M_PI * time);  // 抬脚
    } else {
        // 大腿向后摆动时，小腿应收回
        *x = stepLength * (2 * (1.0 - time));  // 向后滑动
        *y = stepHeight * sinf(M_PI * time);  // 小腿应抬起防止碰到地面
    }
}



/*** 逆运动学：通过给定的足尖位置，计算关节角度 ***/
void InverseKinematics(float x, float y, float* theta1, float* theta2) {
    float L1 = LEG_UPPER_LENGTH;  // 大腿长度
    float L2 = LEG_LOWER_LENGTH;  // 小腿长度
    const float MIN_KNEE_ANGLE = 110.0f;  // 小腿最小收缩角度
    const float MAX_KNEE_ANGLE = 70.0f;  // 小腿最大伸展角度

    // 计算足尖到肩部的距离
    float dist = sqrtf(x * x + y * y);

    // 保证目标位置在机械腿的可达范围内
    if (dist > (L1 + L2)) dist = L1 + L2;
    if (dist < fabsf(L1 - L2)) dist = fabsf(L1 - L2);

    // 小腿角度计算
    float cos_theta2 = (dist * dist - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    cos_theta2 = fminf(fmaxf(cos_theta2, -1.0f), 1.0f);  // 限制在[-1, 1]之间
    float phi2 = acosf(cos_theta2);  // 小腿角度 (弧度)

    // 大腿角度计算
    float phi1 = atan2f(y, x) - atan2f(L2 * sinf(phi2), L1 + L2 * cosf(phi2));

    // 弧度转换为角度，并限制在[0, 180]度范围
    *theta1 = fminf(fmaxf(phi1 * 180.0f / M_PI, 0), 180);  // 大腿角度

    // 将小腿角度限制
    *theta2 = fminf(fmaxf(phi2 * 180.0f / M_PI, MIN_KNEE_ANGLE), MAX_KNEE_ANGLE);  // 小腿角度
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
    float phase_leg1_3 = 0.0;  // Leg 1 和 Leg 3 的初始相位
    float phase_leg2_4 = 0.5;  // Leg 2 和 Leg 4 的初始相位 (相对滞后 0.5)

    while (true) {
        // 控制 Leg 1 和 Leg 3 (左前右后)
        for (int leg = 0; leg < 4; leg += 2) {  // leg == 0 -> Leg 1, leg == 2 -> Leg 3
            float x, y;
            CalculateGait(phase_leg1_3, &x, &y);  // 计算足尖位置
            float theta1, theta2;
            InverseKinematics(x, y, &theta1, &theta2);  // 计算关节角度

            // 控制大腿和小腿舵机
            if (leg == 0) {  // Leg 1 (左前腿)
                Set_Servo_Angle(GetServoIDForLeg(leg, true), 90 - theta1);  // 设置大腿角度
                if(theta2 - 90 <= 20)
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 - (theta2 - 90));  // 小腿角度调整为相对范围
                else
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 - 20);

            } else {  // Leg 3 (右后腿)
                Set_Servo_Angle(GetServoIDForLeg(leg, true), 90 + theta1);  // 设置大腿角度
                if(theta2 - 90 <= 20)
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 + (theta2 - 90));  // 小腿角度调整为相对范围
                else
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 + 20);
            }
        }

        // 控制 Leg 2 和 Leg 4 (右前左后)
        for (int leg = 1; leg < 4; leg += 2) {  // leg == 1 -> Leg 2, leg == 3 -> Leg 4
            float x, y;
            CalculateGait(phase_leg2_4, &x, &y);  // 计算足尖位置
            float theta1, theta2;
            InverseKinematics(x, y, &theta1, &theta2);  // 计算关节角度

            // 控制大腿和小腿舵机
            if (leg == 1) {  // Leg 2 (右前腿)
                Set_Servo_Angle(GetServoIDForLeg(leg, true), 90 - theta1);  // 设置大腿角度
                if(theta2 - 90 <= 20)
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 - (theta2 - 90));  // 小腿角度调整为相对范围
                else
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 - 20);
            } else {  // Leg 4 (左后腿)
                Set_Servo_Angle(GetServoIDForLeg(leg, true), 90 + theta1);  // 设置大腿角度
                if(theta2 - 90 <= 20)
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 + (theta2 - 90));  // 小腿角度调整为相对范围
                else
                    Set_Servo_Angle(GetServoIDForLeg(leg, false), 90 + 20);
            }
        }

        // 更新相位，确保同步步态
        phase_leg1_3 += 0.1;
        phase_leg2_4 += 0.1;
        if (phase_leg1_3 >= 1.0) phase_leg1_3 -= 1.0;
        if (phase_leg2_4 >= 1.0) phase_leg2_4 -= 1.0;

        vTaskDelay(pdMS_TO_TICKS(50));  // 延时 50 毫秒，控制步态节奏
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