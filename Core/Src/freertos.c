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
#include "semphr.h"
#include "servo_control.h"
#include "MPU9250.h"
#include "robocontrol.h"
#include <stdio.h>
#include <string.h>
#include "VMC.h"
#include "AHRS.h"
#include "stdarg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1;
IMUData imuData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 每个步�?�阶段的持续时间（毫秒） */

#define RX_BUFFER_SIZE 10 // 接收缓冲区大�?

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// 定义队列用于姿�?�四元数数据传输
QueueHandle_t quatQueue; // 四元数队�?
SemaphoreHandle_t uartMutex;  // 串口互斥信号�?
SemaphoreHandle_t uartTxCompleteSemaphore; // 串口发�?�完成信号量
SemaphoreHandle_t uartRxCompleteSemaphore; // 串口接收完成信号�?

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// 定义数据存储变量
int16_t AccData[3] = {0};
int16_t MagData[3] = {0};
int16_t GyroData[3] = {0};
float TempData = 0.0;
uint8_t rxBuffer[RX_BUFFER_SIZE]; // 接收缓冲�?
// 用于标记DMA是否空闲
volatile uint8_t dma_tx_ready = 1;

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
  .priority = (osPriority_t) osPriorityRealtime2,
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
  .priority = (osPriority_t) osPriorityRealtime,
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
uint32_t inHandlerMode(void);
void print_usart1(char *format, ...);
void UART_ProcessCommand(uint8_t* buffer);
void SendAttitudeToHost(float q[4]);
void SendAttitudeToPC(float q[4]);
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
    //HAL_UART_Transmit(&huart1, (uint8_t*)"FreertosInit", 100, HAL_MAX_DELAY);
    //HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"InitDMA", 100);



    HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);  // 启动 UART 接收 DMA


    // Init_Servos();
    // Set_Servo_Angle(H1, 90);  // 设置舵机角度
    // Set_Servo_Angle(H2, 90);
    // Set_Servo_Angle(H3, 90);
    // Set_Servo_Angle(H4, 90);
    // Set_Servo_Angle(H5, 90);
    // Set_Servo_Angle(H6, 90);
    // Set_Servo_Angle(H7, 90);
    // Set_Servo_Angle(H8, 90);
    // HAL_Delay(1000);
    // Set_Servo_Angle(H1, 90 - 20);  // 设置舵机角度
    // Set_Servo_Angle(H2, 90 - 30);
    // Set_Servo_Angle(H3, 90 + 20);
    // Set_Servo_Angle(H4, 90 + 30);
    // Set_Servo_Angle(H5, 90 - 20);
    // Set_Servo_Angle(H6, 90 - 30);
    // Set_Servo_Angle(H7, 90 + 20);
    // Set_Servo_Angle(H8, 90 + 30);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    uartMutex = xSemaphoreCreateMutex();  // 创建串口互斥信号�?
    uartTxCompleteSemaphore = xSemaphoreCreateBinary();  // 创建串口发�?�完成信号量
    uartRxCompleteSemaphore = xSemaphoreCreateBinary();  // 创建串口接收完成信号�?
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
    //AHRS_Init();
    IMUData imuData;
    /* Infinite loop */
    for(;;)
    {
        //HAL_UART_Transmit(&huart1, (uint8_t*)"StartAttitudeTask", 100, HAL_MAX_DELAY);

        MPU9250_GetData(imuData.accel, imuData.mag, imuData.gyro, NULL);  // 读取IMU数据
        // 获取加�?�度、陀螺仪和磁力计原始数据
        float ax = imuData.accel[0] * 4.0f * 9.81f / 32768.0f; // 4G 量程
        float ay = imuData.accel[1] * 4.0f * 9.81f / 32768.0f;
        float az = imuData.accel[2] * 4.0f * 9.81f / 32768.0f;
        float gx = imuData.gyro[0] * 500.0f / 32768.0f * M_PI / 180.0f; // 500DPS 量程
        float gy = imuData.gyro[1] * 500.0f / 32768.0f * M_PI / 180.0f;
        float gz = imuData.gyro[2] * 500.0f / 32768.0f * M_PI / 180.0f;
        float mx = imuData.mag[0] * 0.146f; // 将原始数据转换为 µT
        float my = imuData.mag[1] * 0.146f;
        float mz = imuData.mag[2] * 0.146f;


        AHRS_Update();  // 更新姿�?�四元数
        // 获取并处理姿态四元数
        float quat[4] = {0};
        AHRS_GetQuaternion(quat);

        // 打印传感器数�?
        char printBuffer[1024];
        int len = snprintf(printBuffer, sizeof(printBuffer),
                           "%.2f,%.2f,%.2f,%.2f\n",
                           quat[0], quat[1], quat[2], quat[3]);
        //HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);
        //HAL_UART_Transmit_DMA(&huart1, (uint8_t*)printBuffer, len);
        //print_usart1("Quat: %.2f, %.2f, %.2f, %.2f\n", quat[0], quat[1], quat[2], quat[3]);

        // 将姿态四元数发�?�给GaitControlTask
        xQueueSend(quatQueue, &quat, portMAX_DELAY);

        // 准备发�?�四元数数据
        uint8_t txBuffer[22];
        memset(txBuffer, 0, sizeof(txBuffer));  // 清除发�?�缓冲区
        txBuffer[0] = 0xAA;  // 帧头
        txBuffer[1] = 0x06;  // 数据类型 (姿�?�数�?)
        memcpy(&txBuffer[2], &quat[0], sizeof(float));  // 复制四元数数�?
        memcpy(&txBuffer[6], &quat[1], sizeof(float));
        memcpy(&txBuffer[10], &quat[2], sizeof(float));
        memcpy(&txBuffer[14], &quat[3], sizeof(float));
        txBuffer[18] = 0x55;  // 帧尾

        //HAL_UART_Transmit(&huart1, txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);

        if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY && dma_tx_ready) {
            dma_tx_ready = 0;

            // 使用DMA发�?�数�?
            HAL_UART_Transmit_DMA(&huart1, txBuffer, sizeof(txBuffer));

            // 等待发�?�完�?
            xSemaphoreTake(uartTxCompleteSemaphore, portMAX_DELAY);
        }

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

    Init_Servos();
    // Set_Servo_Angle(H1, 90);  // 设置舵机角度
    // Set_Servo_Angle(H2, 90);
    // Set_Servo_Angle(H3, 90);
    // Set_Servo_Angle(H4, 90);
    // Set_Servo_Angle(H5, 90);
    // Set_Servo_Angle(H6, 90);
    // Set_Servo_Angle(H7, 90);
    // Set_Servo_Angle(H8, 90);
    // vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1�?
    // Set_Servo_Angle(H1, 90 - 20);  // 设置舵机角度
    // Set_Servo_Angle(H2, 90 - 30);
    // Set_Servo_Angle(H3, 90 + 20);
    // Set_Servo_Angle(H4, 90 + 30);
    // Set_Servo_Angle(H5, 90 - 20);
    // Set_Servo_Angle(H6, 90 - 30);
    // Set_Servo_Angle(H7, 90 + 20);
    // Set_Servo_Angle(H8, 90 + 30);

    vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1�?


    //Gait_Forward();

    /* Infinite loop */
    for(;;)
    {

        // HAL_UART_Transmit(&huart1, (uint8_t*)"StartServoControlTask", 100, HAL_MAX_DELAY);

        //Gait_Forward();

        vTaskDelay(pdMS_TO_TICKS(1));
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

        //HAL_UART_Transmit(&huart1, (uint8_t*)"StartGaitControlTask", 100, HAL_MAX_DELAY);

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
        vTaskDelay(pdMS_TO_TICKS(50)); // �?50ms更新�?次步�?

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

    /* Infinite loop */
    for(;;)
    {
        // 等待 DMA 接收到数�?
        if (xSemaphoreTake(uartRxCompleteSemaphore, portMAX_DELAY) == pdTRUE) {
            // 线程安全：�?�过互斥量保�? UART 资源
            if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
                // 处理接收到的数据
                UART_ProcessCommand(rxBuffer);
                // 释放互斥量，允许其他任务访问 UART
                xSemaphoreGive(uartMutex);
            }
            // 重新启动 DMA 接收
            HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // 延时 1 毫秒
    }
  /* USER CODE END StartSerialCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void UART_ProcessCommand(uint8_t* buffer) {
    if (buffer[0] == 0xAA && buffer[2] == 0x55) {  // 判断头和尾标�?
        uint8_t command = buffer[1];  // 提取指令类型
        float quat[4];
        switch (command) {
            case 0x01:  // 前进
                Move_Forward();
                break;
            case 0x02:  // 后�??
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
            case 0x06:  // 请求姿�?�数�?

                AHRS_GetQuaternion(quat);  // 获取当前四元数姿态数�?
                SendAttitudeToHost(quat);  // 发�?�姿态数据给上位�?
                break;
            default:
                break;
        }
    }
}

/* SendAttitudeToHost - Sends the current attitude (quaternion) to the host via UART */
void SendAttitudeToHost(float q[4]) {
    uint8_t txBuffer[22];  // 传输数据缓存�?22字节
    txBuffer[0] = 0xAA;  // �?头标�?
    txBuffer[1] = 0x06;  // 数据类型（姿态数据）

    // 将四元数数据转换为字�?
    memcpy(&txBuffer[2], &q[0], sizeof(float));
    memcpy(&txBuffer[6], &q[1], sizeof(float));
    memcpy(&txBuffer[10], &q[2], sizeof(float));
    memcpy(&txBuffer[14], &q[3], sizeof(float));

    txBuffer[18] = 0x55;  // 结束标志

    // 通过UART发�?�姿态数�?
    HAL_UART_Transmit(&huart1, txBuffer, sizeof(txBuffer), HAL_MAX_DELAY);

    // 打印四元数数�?
    char printBuffer[128];
    int len = snprintf(printBuffer, sizeof(printBuffer), "%.6f,%.6f,%.6f,%.6f\n", q[0], q[1], q[2], q[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)printBuffer, len, HAL_MAX_DELAY);
}

void SendAttitudeToPC(float q[4]) {
    char txBuffer[50];  // 传输数据缓存
    int len = snprintf(txBuffer, sizeof(txBuffer), "%.6f,%.6f,%.6f,%.6f\n", q[0], q[1], q[2], q[3]);

    // 通过UART发�?�姿态数�?
    HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, len, HAL_MAX_DELAY);
}

// DMA 发�?�完成回�?
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        //HAL_UART_Transmit(&huart1, (uint8_t*)"TxCompleteCplt", 100, HAL_MAX_DELAY);
        dma_tx_ready = 1;  // 标记DMA发�?�空�?
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 发�?�完成信号量
        xSemaphoreGiveFromISR(uartTxCompleteSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DMA 接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 通知接收完成
        xSemaphoreGiveFromISR(uartRxCompleteSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// �?测是否处于中断上下文
uint32_t inHandlerMode(void)
{
    return __get_IPSR() != 0U;
}

// 串口非阻塞发送函数，支持中断模式下安全调�?
void print_usart1(char *format, ...)
{
    char buf[64];  // 发�?�缓�?

    // �?查是否在中断模式
    if (inHandlerMode() != 0)
    {
        // 禁用全局中断，确保中断中不会有重入问�?
        taskDISABLE_INTERRUPTS();
    }
    else
    {
        // �?查串口是否忙，如果忙则让出CPU
        while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX)
        {
            taskYIELD();
        }
    }

    // 格式化要发�?�的字符�?
    va_list ap;
    va_start(ap, format);
    vsprintf(buf, format, ap);
    va_end(ap);

    // 使用中断方式发�?�数�?
    HAL_UART_Transmit_IT(&huart1, (uint8_t *)buf, strlen(buf));

    // 如果在中断模式下，恢复中�?
    if (inHandlerMode() != 0)
    {
        taskENABLE_INTERRUPTS();
    }
}
/* USER CODE END Application */

