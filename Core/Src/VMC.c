#include "VMC.h"
#include "math.h"
#include "servo_control.h"
#include "robocontrol.h"

#define K_POSITION 1.0f
#define GRAVITY_COMPENSATION 9.81f

void VMC_GenerateForces(int leg, float* quat, float* targetX, float* targetY, float* targetZ) {
    float roll = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
    float pitch = asinf(2.0f * (quat[0] * quat[2] - quat[3] * quat[1]));

    float position_error_x = -sin(pitch);
    float position_error_y = sin(roll);

    float virtualForceX = -K_POSITION * position_error_x;
    float virtualForceY = -K_POSITION * position_error_y;
    float virtualForceZ = GRAVITY_COMPENSATION;

    *targetX = virtualForceX * 10.0f;
    *targetY = virtualForceY * 10.0f;
    *targetZ = virtualForceZ * 10.0f;
}

void VMC_LegControl(int leg, float x, float y, float z) {
    float theta1, theta2;
    InverseKinematics(x, y, &theta1, &theta2);
    
    ServoID upperLegServo = GetServoIDForLeg(leg, true);
    ServoID lowerLegServo = GetServoIDForLeg(leg, false);

    Set_Servo_Angle(upperLegServo, theta1);
    Set_Servo_Angle(lowerLegServo, theta2);
}
