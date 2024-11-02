#include "robocontrol.h"
#include "servo_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>

#define PI 3.14159265358979323846
#define Tf 0.5  // 步态周期

// PID参数
float Kp_pitch = 1.5f, Ki_pitch = 0.05f, Kd_pitch = 0.3f;
float Kp_roll = 1.5f, Ki_roll = 0.05f, Kd_roll = 0.3f;

// PID状态变量
float prev_error_pitch = 0.0f, integral_pitch = 0.0f;
float prev_error_roll = 0.0f, integral_roll = 0.0f;

extern QueueHandle_t quatQueue;  // 四元数数据队列


typedef struct {
    float x, y;  // 足尖位置
} FootPosition;

// 全局状态变量
volatile RobotAction current_action = ACTION_STOP;

// 控制机器人动作的函数
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


//PID控制
void CalculateBalanceAdjustment(float quat[4], float* adjust_pitch, float* adjust_roll) {
    // 四元数转欧拉角，计算俯仰（pitch）和横滚（roll）
    float pitch = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
                         1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2])) * 180.0f / M_PI;
    float roll = asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1])) * 180.0f / M_PI;

    // 设定目标角度
    float target_pitch = 0.0f;
    float target_roll = 0.0f;

    // 计算误差
    float error_pitch = target_pitch - pitch;
    float error_roll = target_roll - roll;

    // 计算积分项
    integral_pitch += error_pitch;
    integral_roll += error_roll;

    // 计算微分项
    float derivative_pitch = error_pitch - prev_error_pitch;
    float derivative_roll = error_roll - prev_error_roll;

    // PID 输出
    *adjust_pitch = Kp_pitch * error_pitch + Ki_pitch * integral_pitch + Kd_pitch * derivative_pitch;
    *adjust_roll = Kp_roll * error_roll + Ki_roll * integral_roll + Kd_roll * derivative_roll;

    // 更新前一误差
    prev_error_pitch = error_pitch;
    prev_error_roll = error_roll;
}

//步态计算
void CalculateFootTrajectory(float t, float x_target, float z_target, FootPosition *pos, RobotAction gait) {
    switch (gait) {
        case ACTION_FORWARD:
            pos->x = t < Tf ? x_target * (t / Tf) : x_target * (2.0f - t / Tf);
            pos->y = t < Tf ? z_target * sinf(PI * t / Tf) : 0;
            break;
        case ACTION_BACKWARD:
            pos->x = t < Tf ? -x_target * (t / Tf) : -x_target * (2.0f - t / Tf);
            pos->y = t < Tf ? z_target * sinf(PI * t / Tf) : 0;
            break;
        case ACTION_TURN_LEFT:
            pos->x = (t < Tf ? x_target * (t / Tf) : x_target * (2.0f - t / Tf)) * (gait == ACTION_TURN_LEFT ? -1 : 1);
            pos->y = t < Tf ? z_target * sinf(PI * t / Tf) : 0;
            break;
        case ACTION_TURN_RIGHT:
            pos->x = (t < Tf ? x_target * (t / Tf) : x_target * (2.0f - t / Tf)) * (gait == ACTION_TURN_RIGHT ? 1 : -1);
            pos->y = t < Tf ? z_target * sinf(PI * t / Tf) : 0;
            break;
        case ACTION_MARCH_IN_PLACE:
            pos->x = 0;
            pos->y = z_target * sinf(PI * t / Tf);
            break;
    }
}

//逆运动学
void InverseKinematics(float x, float y, float *theta1, float *theta2) {
    float L1 = LEG_UPPER_LENGTH;
    float L2 = LEG_LOWER_LENGTH;

    float dist = sqrtf(x * x + y * y);
    dist = fminf(fmaxf(dist, fabsf(L1 - L2)), (L1 + L2));

    float cos_theta2 = (dist * dist - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    float phi2 = acosf(fmaxf(fminf(cos_theta2, 1.0f), -1.0f));
    float phi1 = atan2f(y, x) - atan2f(L2 * sinf(phi2), L1 + L2 * cosf(phi2));

    *theta1 = phi1 * 180.0f / PI;
    *theta2 = phi2 * 180.0f / PI;
}

//步态控制
void GaitControl() {
    float t = 0;
    FootPosition pos1, pos2;
    float x_target = 50.0f, z_target = 30.0f;
    float theta1[4], theta2[4];
    float quat[4];
    float adjust_pitch = 0.0f, adjust_roll = 0.0f;

    while (1) {
        // 从IMU数据队列接收四元数数据
        if (xQueueReceive(quatQueue, &quat, pdMS_TO_TICKS(10)) == pdPASS) {
            CalculateBalanceAdjustment(quat, &adjust_pitch, &adjust_roll);

            // 控制四条腿的步态和平衡
            for (int i = 0; i < 4; i++) {
                float t_shifted = t + (i % 2 == 0 ? 0 : Tf);

                // 计算每条腿的足尖轨迹
                CalculateFootTrajectory(fmodf(t_shifted, 2 * Tf), x_target, z_target,
                                        (i == 0 || i == 2) ? &pos1 : &pos2, current_action);

                // 计算逆运动学角度
                InverseKinematics((i == 0 || i == 2) ? pos1.x : pos2.x,
                                  (i == 0 || i == 2) ? pos1.y : pos2.y,
                                  &theta1[i], &theta2[i]);

                int leg = i;
                float finalTheta1 = theta1[leg];
                float finalTheta2 = theta2[leg];

                // 将俯仰和横滚调整分别应用到不同方向的腿
                if (leg == 0 || leg == 2) {  // 前后对角腿
                    finalTheta1 += adjust_pitch;
                    finalTheta2 += adjust_pitch;
                } else {  // 左右对角腿
                    finalTheta1 += adjust_roll;
                    finalTheta2 += adjust_roll;
                }

                // 设置舵机角度，根据腿的位置调整方向
                if (leg % 2 == 0) {  // 单数腿
                    servos[GetServoIDForLeg(leg, true)].target_angle = 90 + ((leg == 0) ? finalTheta1 : -finalTheta1);
                    servos[GetServoIDForLeg(leg, false)].target_angle = 90 + ((leg == 0) ? -finalTheta2 : finalTheta2);
                } else {  // 双数腿
                    servos[GetServoIDForLeg(leg, true)].target_angle = 90 + ((leg == 1) ? -finalTheta1 : finalTheta1);
                    servos[GetServoIDForLeg(leg, false)].target_angle = 90 + ((leg == 1) ? finalTheta2 : -finalTheta2);
                }
            }

            // 更新步态相位
            t += 0.05;
            if (t >= 2 * Tf) t -= 2 * Tf;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

///*** 计算步态：根据时间计算每一步的足尖位置 ***/
//void CalculateGait(float time, float* theta1, float* theta2) {
//
//    if (time < 0.5) {
//        // 大腿向前摆动时，小腿逐渐伸展
//        * theta1 = 45;  // 向前移动70
//        * theta2 = -10;  // 抬脚-30
//    } else {
//        // 大腿向后摆动时，小腿应收回
//        * theta1 = 25;  // 向后滑动15
//        * theta2 = 5;  // 小腿应抬起防止碰到地面20
//    }
//}
//
//
//
//
//
//
///*** 舵机标号获取函数 ***/
//ServoID GetServoIDForLeg(int leg, bool isUpperLeg) {
//    if (isUpperLeg) {
//        return (ServoID)((leg * 2) + 1);  // 大腿舵机（双数）
//    } else {
//        return (ServoID)((leg * 2));  // 小腿舵机（单数）
//    }
//}
//
/////*** 前进步态控制 ***/
//
//void Gait_Forward(void) {
//    float phase_leg1_4 = 0.0;  // Leg 1 和 Leg 4 的相位
//    float phase_leg2_3 = 0.5;  // Leg 2 和 Leg 3 的相位，相对滞后 0.5
//
//    while (true) {
//        // 控制 Leg 1 和 Leg 4 (对角步态)
//        for (int leg = 0; leg <= 3; leg += 3) {  // leg == 0 -> Leg 1, leg == 3 -> Leg 4
//            float theta1, theta2;
//            CalculateGait(phase_leg1_4, &theta1, &theta2);
//
//            if (leg == 0) {  // Leg 1 (左前腿)
//                servos[GetServoIDForLeg(leg, true)].target_angle = 90 - theta1;  // 大腿角度
//                servos[GetServoIDForLeg(leg, false)].target_angle = 90 + theta2;  // 小腿角度
//            } else {  // Leg 4 (右后腿)
//                servos[GetServoIDForLeg(leg, true)].target_angle = 90 + theta1;  // 大腿角度
//                servos[GetServoIDForLeg(leg, false)].target_angle = 90 - theta2;  // 小腿角度
//            }
//        }
//
//        // 控制 Leg 2 和 Leg 3 (对角步态)
//        for (int leg = 1; leg <= 2; leg += 1) {  // leg == 1 -> Leg 2, leg == 2 -> Leg 3
//            float theta1, theta2;
//            CalculateGait(phase_leg2_3, &theta1, &theta2);
//
//            if (leg == 1) {  // Leg 2 (右前腿)
//                servos[GetServoIDForLeg(leg, true)].target_angle = 90 + theta1;  // 大腿角度
//                servos[GetServoIDForLeg(leg, false)].target_angle = 90 - theta2;  // 小腿角度
//            } else {  // Leg 3 (左后腿)
//                servos[GetServoIDForLeg(leg, true)].target_angle = 90 - theta1;  // 大腿角度
//                servos[GetServoIDForLeg(leg, false)].target_angle = 90 + theta2;  // 小腿角度
//            }
//        }
//
//        // 更新相位，确保同步步态
//        phase_leg1_4 += 0.1;
//        phase_leg2_3 += 0.1;
//        if (phase_leg1_4 >= 1.0) phase_leg1_4 -= 1.0;
//        if (phase_leg2_3 >= 1.0) phase_leg2_3 -= 1.0;
//
//        vTaskDelay(pdMS_TO_TICKS(50));  // 延时 50 毫秒，控制步态节奏
//    }
//}
