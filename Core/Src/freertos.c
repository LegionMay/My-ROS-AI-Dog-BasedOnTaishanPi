/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "servo_control.h"
#include "MPU9250.h"
#include "robocontrol.h"
#include <stdio.h>
#include <string.h>
#include "VMC.h"
#include "AHRS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1;
IMUData imuData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 每个步态阶段的持续时间（毫秒） */
#define GAIT_PHASE_DURATION 1  // 根据需要调整

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 定义队列用于姿态四元数数据传输
QueueHandle_t quatQueue;

/* 步态阶段的定义 */
typedef enum {
    GAIT_PHASE_1 = 0,
    GAIT_PHASE_2,
    GAIT_PHASE_3,
    GAIT_PHASE_4
} GaitPhase;



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// 定义数据存储变量
int16_t AccData[3] = {0};
int16_t MagData[3] = {0};
int16_t GyroData[3] = {0};
float TempData = 0.0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AttitudeTask */
osThreadId_t AttitudeTaskHandle;
const osThreadAttr_t AttitudeTask_attributes = {
  .name = "AttitudeTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ServoControlTas */
osThreadId_t ServoControlTasHandle;
const osThreadAttr_t ServoControlTas_attributes = {
  .name = "ServoControlTas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for GaitControlTask */
osThreadId_t GaitControlTaskHandle;
const osThreadAttr_t GaitControlTask_attributes = {
  .name = "GaitControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for SerialCommTask */
osThreadId_t SerialCommTaskHandle;
const osThreadAttr_t SerialCommTask_attributes = {
  .name = "SerialCommTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void print_accel_data(int16_t AccData[3]);
void print_mag_data(int16_t MagData[3]);
void print_gyro_data(int16_t GyroData[3]);
void print_temp_data(float32_t TempData);

void UART_ProcessCommand(uint8_t* buffer);
void SendAttitudeToHost(int16_t q0, int16_t q1, int16_t q2, int16_t q3);
void SendAttitudeToPC(int16_t q0, int16_t q1, int16_t q2, int16_t q3);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAttitudeTask(void *argument);
void StartServoControlTask(void *argument);
void StartGaitControlTask(void *argument);
void StartSerialCommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    Servo_Init();
    //MPU9250_Init();
    AHRS_Init();  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
   quatQueue = xQueueCreate(10, sizeof(int16_t[4]));  // 创建队列，存储四元数数据
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of AttitudeTask */
  AttitudeTaskHandle = osThreadNew(StartAttitudeTask, NULL, &AttitudeTask_attributes);

  /* creation of ServoControlTas */
  ServoControlTasHandle = osThreadNew(StartServoControlTask, NULL, &ServoControlTas_attributes);

  /* creation of GaitControlTask */
  GaitControlTaskHandle = osThreadNew(StartGaitControlTask, NULL, &GaitControlTask_attributes);

  /* creation of SerialCommTask */
  SerialCommTaskHandle = osThreadNew(StartSerialCommTask, NULL, &SerialCommTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    MPU9250_GetData(imuData.accel, imuData.mag, imuData.gyro, NULL);  // 读取IMU数据
        // 获取加速度、陀螺仪和磁力计原始数据
        int16_t ax = imuData.accel[0], ay = imuData.accel[1], az = imuData.accel[2];
        int16_t gx = imuData.gyro[0] , gy = imuData.gyro[1] , gz = imuData.gyro[2] ;
        int16_t mx = imuData.mag[0], my = imuData.mag[1], mz = imuData.mag[2];
         // 打印四元数数据
        char printBuffer[256];
        int len = snprintf(printBuffer, sizeof(printBuffer), "A:%d,%d,%d,%d,G:%d,%d,%d\n", ax, ay, az, gx,gy,gz);
        HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAttitudeTask */
/**
* @brief Function implementing the AttitudeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAttitudeTask */
void StartAttitudeTask(void *argument)
{
  /* USER CODE BEGIN StartAttitudeTask */
  //AHRS_Init();  AHRS_Init();  
  IMUData imuData;
  /* Infinite loop */
  for(;;)
  {
        MPU9250_GetData(imuData.accel, imuData.mag, imuData.gyro, NULL);  // 读取IMU数据
        // 获取加速度、陀螺仪和磁力计原始数据
        int16_t ax = imuData.accel[0], ay = imuData.accel[1], az = imuData.accel[2];
        int16_t gx = imuData.gyro[0] , gy = imuData.gyro[1] , gz = imuData.gyro[2] ;
        int16_t mx = imuData.mag[0], my = imuData.mag[1], mz = imuData.mag[2];
         // 打印四元数数据
        char printBuffer[256];
        int len = snprintf(printBuffer, sizeof(printBuffer), "A:%d,%d,%d,%d,G:%d,%d,%d\n", ax, ay, az, gx,gy,gz);
        HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);
        

        AHRS_Update();  // 更新姿态四元数
        // 获取并处理姿态四元数
        int16_t quat[4] = {0};
        SendAttitudeToHost(quat[0], quat[1], quat[2], quat[3]);  // 发送姿态四元数给上位机
        AHRS_GetQuaternion(quat);
        
        
        // 将姿态四元数发送给GaitControlTask
        xQueueSend(quatQueue, &quat, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10)); 
  }
  /* USER CODE END StartAttitudeTask */
}

/* USER CODE BEGIN Header_StartServoControlTask */
/**
* @brief Function implementing the ServoControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoControlTask */
void StartServoControlTask(void *argument)
{
  /* USER CODE BEGIN StartServoControlTask */
  /* Infinite loop */
  for(;;)
  {
    MPU9250_GetData(imuData.accel, imuData.mag, imuData.gyro, NULL);  // 读取IMU数据
        // 获取加速度、陀螺仪和磁力计原始数据
        int16_t ax = imuData.accel[0], ay = imuData.accel[1], az = imuData.accel[2];
        int16_t gx = imuData.gyro[0] , gy = imuData.gyro[1] , gz = imuData.gyro[2] ;
        int16_t mx = imuData.mag[0], my = imuData.mag[1], mz = imuData.mag[2];
         // 打印四元数数据
        char printBuffer[256];
        int len = snprintf(printBuffer, sizeof(printBuffer), "A:%d,%d,%d,%d,G:%d,%d,%d\n", ax, ay, az, gx,gy,gz);
        HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);

    osDelay(1);
  }
  /* USER CODE END StartServoControlTask */
}

/* USER CODE BEGIN Header_StartGaitControlTask */
/**
* @brief Function implementing the GaitControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGaitControlTask */
void StartGaitControlTask(void *argument)
{
  /* USER CODE BEGIN StartGaitControlTask */
  /* Infinite loop */
  for(;;)
  {

    MPU9250_GetData(imuData.accel, imuData.mag, imuData.gyro, NULL);  // 读取IMU数据
        // 获取加速度、陀螺仪和磁力计原始数据
        int16_t ax = imuData.accel[0], ay = imuData.accel[1], az = imuData.accel[2];
        int16_t gx = imuData.gyro[0] , gy = imuData.gyro[1] , gz = imuData.gyro[2] ;
        int16_t mx = imuData.mag[0], my = imuData.mag[1], mz = imuData.mag[2];
         // 打印四元数数据
        char printBuffer[256];
        int len = snprintf(printBuffer, sizeof(printBuffer), "A:%d,%d,%d,%d,G:%d,%d,%d\n", ax, ay, az, gx,gy,gz);
        HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);


    switch (current_action) {
            case ACTION_FORWARD:
                Gait_Forward();
                break;
            case ACTION_BACKWARD:
                Gait_Backward();
                break;
            case ACTION_TURN_LEFT:
                Gait_Turn_Left();
                break;
            case ACTION_TURN_RIGHT:
                Gait_Turn_Right();
                break;
            case ACTION_MARCH_IN_PLACE:
                Gait_March_In_Place();
                break;
            case ACTION_STOP:
            default:
                
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 每50ms更新一次步态
    osDelay(1);
  }
  /* USER CODE END StartGaitControlTask */
}

/* USER CODE BEGIN Header_StartSerialCommTask */
/**
* @brief Function implementing the SerialCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialCommTask */
void StartSerialCommTask(void *argument)
{
  /* USER CODE BEGIN StartSerialCommTask */
  uint8_t rxBuffer[10];  // 接收缓存
  /* Infinite loop */
  for(;;)
  {

    MPU9250_GetData(imuData.accel, imuData.mag, imuData.gyro, NULL);  // 读取IMU数据
        // 获取加速度、陀螺仪和磁力计原始数据
        int16_t ax = imuData.accel[0], ay = imuData.accel[1], az = imuData.accel[2];
        int16_t gx = imuData.gyro[0] , gy = imuData.gyro[1] , gz = imuData.gyro[2] ;
        int16_t mx = imuData.mag[0], my = imuData.mag[1], mz = imuData.mag[2];
         // 打印四元数数据
        char printBuffer[256];
        int len = snprintf(printBuffer, sizeof(printBuffer), "A:%d,%d,%d,%d,G:%d,%d,%d\n", ax, ay, az, gx,gy,gz);
        HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);
        osDelay(1);

    // 监听串口命令
    if (HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), HAL_MAX_DELAY) == HAL_OK) {
      UART_ProcessCommand(rxBuffer);  // 处理命令
    }
  }
  /* USER CODE END StartSerialCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void print_accel_data(int16_t AccData[3]) {
  char buffer[50];
  int len = snprintf(buffer, sizeof(buffer), "AccData: X=%d, Y=%d, Z=%d\n", AccData[0], AccData[1], AccData[2]);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

void print_mag_data(int16_t MagData[3]) {
  char buffer[50];
  int len = snprintf(buffer, sizeof(buffer), "MagData: X=%d, Y=%d, Z=%d\n", MagData[0], MagData[1], MagData[2]);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

void print_gyro_data(int16_t GyroData[3]) {
  char buffer[50];
  int len = snprintf(buffer, sizeof(buffer), "GyroData: X=%d, Y=%d, Z=%d\n", GyroData[0], GyroData[1], GyroData[2]);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}



void UART_ProcessCommand(uint8_t* buffer) {
    if (buffer[0] == 0xAA && buffer[2] == 0x55) {  // 判断头和尾标志
        uint8_t command = buffer[1];  // 提取指令类型
        int16_t quat[4];
        switch (command) {
            case 0x01:  // 前进
                Move_Forward();
                break;
            case 0x02:  // 后退
                Move_Backward();
                break;
            case 0x03:  // 左转
                Turn_Left();
                break;
            case 0x04:  // 右转
                Turn_Right();
                break;
            case 0x05:  // 停止
                Stop();
                break;
            case 0x06:  // 请求姿态数据
                
                AHRS_GetQuaternion(quat);  // 获取当前四元数姿态数据
                SendAttitudeToHost(quat[0], quat[1], quat[2], quat[3]);  // 发送姿态数据给上位机
                break;
            default:
                break;
        }
    }
}

/* SendAttitudeToHost - Sends the current attitude (quaternion) to the host via UART */
void SendAttitudeToHost(int16_t q0, int16_t q1, int16_t q2, int16_t q3) {
    uint8_t txBuffer[18];  // 传输数据缓存，18字节
    txBuffer[0] = 0xAA;  // 开头标志
    txBuffer[1] = 0x06;  // 数据类型（姿态数据）
    
    // 将四元数数据转换为字节
    memcpy(&txBuffer[2], &q0, sizeof(int16_t));
    memcpy(&txBuffer[6], &q1, sizeof(int16_t));
    memcpy(&txBuffer[10], &q2, sizeof(int16_t));
    memcpy(&txBuffer[14], &q3, sizeof(int16_t));
    
    txBuffer[17] = 0x55;  // 结束标志
    
    // 通过UART发送姿态数据
    HAL_UART_Transmit(&huart1, txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);

    // 打印四元数数据
    char printBuffer[128];
    int len = snprintf(printBuffer, sizeof(printBuffer), "%d,%d,%d,%d\n", q0, q1, q2, q3);
    HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);
}



void SendAttitudeToPC(int16_t q0, int16_t q1, int16_t q2, int16_t q3) {
  char txBuffer[50];  // 传输数据缓存
  int len = snprintf(txBuffer, sizeof(txBuffer), "%d,%d,%d,%d\n", q0, q1, q2, q3);

  // 通过UART发送姿态数据
  HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, len, HAL_MAX_DELAY);
}
/* USER CODE END Application */

