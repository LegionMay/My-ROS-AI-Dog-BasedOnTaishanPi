// servo_control.c

#include "servo_control.h"

// 定义180度舵机的PWM信号范围
#define SERVO_MIN_PULSE_WIDTH 500   // 最小脉宽 (0.5 ms)
#define SERVO_MAX_PULSE_WIDTH 2500  // 最大脉宽 (2.5 ms)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

ServoControl servos[8];

static TIM_HandleTypeDef* servo_timers[] = {
    &htim3,
    &htim4,
    &htim5,
    &htim8,
    &htim12,
    &htim13,
    &htim14,
    &htim15,
    &htim16
};

static uint32_t servo_channels[] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_1,
    TIM_CHANNEL_1
};

void Servo_Init(void) {
    int num_servos = sizeof(servo_timers) / sizeof(servo_timers[0]);
    // 初始化所有定时器的PWM频率为50Hz (周期为20ms)
    for (int i = 0; i < num_servos; i++) {
        __HAL_TIM_SET_PRESCALER(servo_timers[i], 239); // 240MHz / (239 + 1) = 1MHz
        __HAL_TIM_SET_AUTORELOAD(servo_timers[i], 19999); // 1MHz / 20000 = 50Hz
        HAL_TIM_PWM_Start(servo_timers[i], servo_channels[i]);
    }
}

void Set_Servo_Angle(ServoID servo, float angle) {
    if (servo < 0 || servo >= sizeof(servo_timers) / sizeof(servo_timers[0])) {
        return; // 无效的舵机索引
    }
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 将角度转换为对应的脉宽值 (0.5ms到2.5ms)
    uint32_t pulse_width = SERVO_MIN_PULSE_WIDTH + (uint32_t)((angle / 180.0f) * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH));
    __HAL_TIM_SET_COMPARE(servo_timers[servo], servo_channels[servo], pulse_width);
}

// 初始化舵机控制数组
void Init_Servos() {
    for (int i = 0; i < sizeof(servos) / sizeof(servos[0]); i++) {
        servos[i].servo = i;
        servos[i].target_angle = 0.0f;
    }
}