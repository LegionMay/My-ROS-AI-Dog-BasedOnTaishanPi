#ifndef ROBOCONTROL_H
#define ROBOCONTROL_H

//#include "freertos.h"
#include "servo_control.h"
#include "stdbool.h"

#define LEG_UPPER_LENGTH 80.9 // 大腿长度，单位：毫米
#define LEG_LOWER_LENGTH 68.75 // 小腿长度，单位：毫米



// 机器人动作状态
typedef enum {
    ACTION_STOP,
    ACTION_FORWARD,
    ACTION_BACKWARD,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_MARCH_IN_PLACE
} RobotAction;

extern volatile RobotAction current_action;

// 动作控制函数声明
void Move_Forward();
void Move_Backward();
void Turn_Left();
void Turn_Right();
void Stop();
void March_In_Place();

void Gait_Forward(void);
void Gait_Backward(void);
void Gait_Turn_Left(void);
void Gait_Turn_Right(void);
void Gait_March_In_Place(void);


ServoID GetServoIDForLeg(int leg, bool isUpperLeg);

void InverseKinematics(float x, float y, float* theta1, float* theta2);
void CalculateGait(float time, float* theta1, float* theta2);

void CalculateBalanceAdjustment(float quat[4], float* adjust_pitch, float* adjust_roll);
void GaitControl();

#endif // ROBOCONTROL_H
