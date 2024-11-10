#include "robocontrol.h"
#include "servo_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>

#define PI 3.14159
#define STEP_INCREMENT 1.0f  // 每次调整的角度步进
#define MAX_ADJUSTMENT 15.0f // 最大调整范围，防止过度调整

extern QueueHandle_t quatQueue;  // 四元数数据队列

// 全局状态变量
volatile RobotAction current_action = ACTION_STOP;

// 控制动作函数
void Move_Forward() { current_action = ACTION_FORWARD; }
void Move_Backward() { current_action = ACTION_BACKWARD; }
void Turn_Left() { current_action = ACTION_TURN_LEFT; }
void Turn_Right() { current_action = ACTION_TURN_RIGHT; }
void Stop() { current_action = ACTION_STOP; }

ServoID GetServoIDForLeg(int leg, bool isUpperLeg) {
    if (isUpperLeg) {
        return (ServoID)((leg * 2) + 1);  // 大腿舵机（双数）
    } else {
        return (ServoID)((leg * 2));  // 小腿舵机（单数）
    }
}

//计算步态
void CalculateGait(float time, float* theta1, float* theta2 ,float* LeftTheta, float* RigthTheta, float* FrontTheta, float* RearTheta) {
    switch (current_action) {
        case ACTION_FORWARD:
           if (time < 0.5) {
                // 大腿向前摆动时，小腿逐渐伸展
                *theta1 = 60;  // 向前移动70/45
                *theta2 = -30;  // 伸脚-30/5
            } else {
                // 大腿向后摆动时，小腿应收回
                *theta1 = 40;  // 向后滑动15/25
                *theta2 = -10;  // 小腿应抬起防止碰到地面20/-10
            }
            break;
        case ACTION_BACKWARD:
            if (time < 0.5) {
                // 大腿向后摆动时，小腿逐渐伸展
                *theta1 = 60;
                *theta2 = -30;
            } else {
                // 大腿向前摆动时，小腿应收回
                *theta1 = 40;
                *theta2 = -10;
            }
            break;
        case ACTION_TURN_LEFT:
            if (time < 0.5) {
                // 左侧腿向后，右侧腿向前

                *LeftTheta = 10;

            } else {
                // 左侧腿向前，右侧腿向后


                *LeftTheta = 10;
            }
            break;
        case ACTION_TURN_RIGHT:
            if (time < 0.5) {
                // 左侧腿向前，右侧腿向后

                *RigthTheta = 10;

            } else {
                // 左侧腿向后，右侧腿向前


                *RigthTheta = 10;
            }
            break;
        case ACTION_MARCH_IN_PLACE:
            if (time < 0.5) {
                // 左腿抬起，右腿支撑
                *theta1 = 40;
                *theta2 = -10;
            } else {
                // 右腿抬起，左腿支撑
                *theta1 = 40;
                *theta2 = 10;
            }
            break;
        case ACTION_STOP:
            // 保持站立姿态
            *theta1 = 40;  // 大腿角度中立
            *theta2 = -10; // 小腿角度中立
            break;
    }
}


// 实时计算俯仰和横滚偏差并应用小步进调整
void DynamicStabilizationAdjustment() {
    float current_adjust_pitch_front = 0.0f,
          current_adjust_pitch_rear = 0.0f,
          current_adjust_roll_left = 0.0f,
          current_adjust_roll_right = 0.0f;
    float quat[4];

    while (true) {
        if (xQueueReceive(quatQueue, &quat, pdMS_TO_TICKS(10)) == pdPASS) {

            // 四元数转为俯仰和横滚角
            float pitch = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                                 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2])) * 180.0f / M_PI;
            float roll = asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1])) * 180.0f / M_PI;

            // 根据偏差判断方向并调整前后俯仰调整量
            if (pitch > 0) {
                current_adjust_pitch_front = fminf(current_adjust_pitch_front + STEP_INCREMENT, MAX_ADJUSTMENT); // 前腿抬高
                current_adjust_pitch_rear = fmaxf(current_adjust_pitch_rear - STEP_INCREMENT, -MAX_ADJUSTMENT); // 后腿降低
            } else if (pitch < 0) {
                current_adjust_pitch_front = fmaxf(current_adjust_pitch_front - STEP_INCREMENT,-MAX_ADJUSTMENT); // 前腿降低
                current_adjust_pitch_rear = fminf(current_adjust_pitch_rear + STEP_INCREMENT, MAX_ADJUSTMENT);  // 后腿抬高
            }

            // 根据偏差判断方向并调整左右横滚调整量
            if (roll > 0) {
                current_adjust_roll_left = fminf(current_adjust_roll_left + STEP_INCREMENT, MAX_ADJUSTMENT); // 左腿抬高
                current_adjust_roll_right = fmaxf(current_adjust_roll_right - STEP_INCREMENT, -MAX_ADJUSTMENT); // 右腿降低
            } else if (roll < 0) {
                current_adjust_roll_left = fmaxf(current_adjust_roll_left - STEP_INCREMENT, -MAX_ADJUSTMENT); // 左腿降低
                current_adjust_roll_right = fminf(current_adjust_roll_right + STEP_INCREMENT, MAX_ADJUSTMENT);  // 右腿抬高
            }

            // 应用增量调整
            for(int i = 0; i < 4; i++) {
                if (i == 0 || i == 3) {  // 前后腿
                    servos[GetServoIDForLeg(i, true)].target_angle  += (i == 0 ? current_adjust_pitch_front : current_adjust_pitch_rear);
                    servos[GetServoIDForLeg(i, false)].target_angle += (i == 0 ? current_adjust_pitch_front : current_adjust_pitch_rear);
                } else {  // 左右腿
                    servos[GetServoIDForLeg(i, true)].target_angle += (i == 1 ? current_adjust_roll_left : current_adjust_roll_right);
                    servos[GetServoIDForLeg(i, false)].target_angle += (i == 1 ? current_adjust_roll_left : current_adjust_roll_right);
                }
            }
        }
    }
}



void GaitControl() {
    float phase_leg1_4 = 0.0;  // Leg 1 和 Leg 4 的相位
    float phase_leg2_3 = 0.5;  // Leg 2 和 Leg 3 的相位，相对滞后 0.5

    while (true) {

            // 控制 Leg 1 和 Leg 4 (对角步态)
            for (int leg = 0; leg <= 3; leg += 3) {  // leg == 0 -> Leg 1, leg == 3 -> Leg 4
                float theta1, theta2, LeftTheta, RigthTheta, FrontTheta, RearTheta;
                CalculateGait(phase_leg1_4, &theta1, &theta2, &LeftTheta, &RigthTheta, &FrontTheta, &RearTheta);

                if (leg == 0) {  // Leg 1 (左前腿)
                    servos[GetServoIDForLeg(leg, true)].target_angle = 90 - theta1 - LeftTheta;
                    servos[GetServoIDForLeg(leg, false)].target_angle = 90 + theta2;
                } else {  // Leg 4 (右后腿)
                    servos[GetServoIDForLeg(leg, true)].target_angle = 90 + theta1 + RigthTheta;
                    servos[GetServoIDForLeg(leg, false)].target_angle = 90 - theta2;
                }
            }

            // 控制 Leg 2 和 Leg 3 (对角步态)
            for (int leg = 1; leg <= 2; leg += 1) {  // leg == 1 -> Leg 2, leg == 2 -> Leg 3
                float theta1, theta2, LeftTheta, RigthTheta, FrontTheta, RearTheta;
                CalculateGait(phase_leg2_3, &theta1, &theta2, &LeftTheta, & RigthTheta, &FrontTheta, &RearTheta);

                if (leg == 1) {  // Leg 2 (右前腿)
                    servos[GetServoIDForLeg(leg, true)].target_angle = 90 + theta1 + LeftTheta;
                    servos[GetServoIDForLeg(leg, false)].target_angle = 90 - theta2;
                } else {  // Leg 3 (左后腿)
                    servos[GetServoIDForLeg(leg, true)].target_angle = 90 - theta1 - RigthTheta;
                    servos[GetServoIDForLeg(leg, false)].target_angle = 90 + theta2;
                }
            }

            // 更新相位，确保同步步态
            phase_leg1_4 += 0.1;
            phase_leg2_3 += 0.1;
            if (phase_leg1_4 >= 1.0) phase_leg1_4 -= 1.0;
            if (phase_leg2_3 >= 1.0) phase_leg2_3 -= 1.0;

            vTaskDelay(pdMS_TO_TICKS(50));  // 延时 50 毫秒，控制步态节奏
        }

}



//舵机初始位置
void SetInitServosPosition() {
    // 遍历所有舵机，设置每个舵机的角度
    for (int i = 0; i < 8; i++) {
        Set_Servo_Angle(i , 90);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 定义舵机的初始标准站立角度
void SetStandbyPosition() {
    // 舵机角度数组，每个元素表示对应舵机的角度偏移量
    int angles[] = {
            90 - 20,  // H1 (左前小腿)
            90 - 30,  // H2 (左前大腿)
            90 + 20,  // H3 (右前小腿)
            90 + 30,  // H4 (右前大腿)
            90 - 20,  // H5 (左后小腿)
            90 - 30,  // H6 (左后大腿)
            90 + 20,  // H7 (右后小腿)
            90 + 30   // H8 (右后大腿)
    };

    // 遍历所有舵机，设置每个舵机的角度
    for (int i = 0; i < 8; i++) {
        Set_Servo_Angle(i , angles[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
