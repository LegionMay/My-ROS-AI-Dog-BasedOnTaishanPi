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
#include "AHRS.h"
#include "stdarg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1;
IMUData imuData;

typedef enum {
    STATE_WAIT_HEADER,    // 等待帧头0xAA
    STATE_WAIT_COMMAND,   // 等待命令字节
    STATE_WAIT_FOOTER     // 等待帧尾0x55
} UART_ParseState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_Queue_SIZE 64     // UART接收队列大小
#define RX_BUFFER_SIZE 3     // UART接收数据大小
#define RING_BUFFER_SIZE 128 // 环形缓冲区大小
//#define UART_DMA_IDLE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

QueueHandle_t rxQueue;                     // 串口接收队列
QueueHandle_t quatQueue;                   // 四元数队列
SemaphoreHandle_t uartMutex;               // 串口互斥信号
SemaphoreHandle_t uartTxCompleteSemaphore; // 串口发送完成信号量
SemaphoreHandle_t uartRxCompleteSemaphore; // 串口接收完成信号量

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

char printBuffer[100];
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t txBuffer[22];
uint8_t data = 0;

uint8_t ringBuffer[RING_BUFFER_SIZE]; // 定义环形缓冲区
volatile uint16_t writeIndex = 0;
volatile uint16_t readIndex = 0;

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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for ServoControlTas */
osThreadId_t ServoControlTasHandle;
const osThreadAttr_t ServoControlTas_attributes = {
  .name = "ServoControlTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for GaitControlTask */
osThreadId_t GaitControlTaskHandle;
const osThreadAttr_t GaitControlTask_attributes = {
  .name = "GaitControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for SerialCommTask */
osThreadId_t SerialCommTaskHandle;
const osThreadAttr_t SerialCommTask_attributes = {
  .name = "SerialCommTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint32_t inHandlerMode(void);
void print_usart1(char *format, ...);
void SendAttitudeToHost(float q[4]);
void vApplicationIdleHook(void);
void UART_SendData_IT(uint8_t *data, uint16_t size);
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

    //memset(rxBuffer, 0, sizeof(rxBuffer));
    huart1.gState = HAL_UART_STATE_READY;
    HAL_UART_Receive_IT(&huart1, &data, 1);

#ifdef UART_DMA_IDLE
//    if (huart1.RxState != HAL_UART_STATE_READY) {
//        HAL_UART_DeInit(&huart1);
//        HAL_UART_Init(&huart1);
//    }
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)rxBuffer, RX_BUFFER_SIZE);

//    启动 UART DMA 接收，支持 IDLE 中断
//    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE) != HAL_OK) {
//        //HAL_UART_Transmit_IT(&huart1, (uint8_t*)"DMA Error", strlen("DMA Error"));
//        HAL_Delay(1000);
//    } else {
//        //HAL_UART_Transmit_IT(&huart1, (uint8_t*)"DMA Started", strlen("DMA Started"));
//        HAL_Delay(1000);
//    }
#endif

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    uartMutex = xSemaphoreCreateMutex();                    // 串口互斥信号量
    uartTxCompleteSemaphore = xSemaphoreCreateBinary();     // 串口发送完成信号量
    uartRxCompleteSemaphore = xSemaphoreCreateBinary();     // 串口接收完成信号量
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    quatQueue = xQueueCreate(10, sizeof(int16_t[4]));        //四元数数据队列
    rxQueue = xQueueCreate(RX_Queue_SIZE, sizeof(uint8_t)); //串口接收数据队列
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
    vTaskStartScheduler();  // 确保启动调度�??????
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
    AHRS_Init();                    // 初始化姿态解算
    float quat[4] = {0};

    /* Infinite loop */
    for(;;)
    {
        AHRS_Update();              // 更新四元数
        AHRS_GetQuaternion(quat);   // 获取并处理姿态四元数

        int len = snprintf(printBuffer, sizeof(printBuffer),
                           "%.2f,%.2f,%.2f,%.2f\n",
                           quat[0], quat[1], quat[2], quat[3]);

        //UART_SendData_IT((uint8_t *)printBuffer, len);

         xQueueSend(quatQueue, &quat, portMAX_DELAY); // 将姿态四元数发送到队列

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
//    Set_Servo_Angle(H1, 90);  // 设置舵机角度
//    Set_Servo_Angle(H2, 90);
//    Set_Servo_Angle(H3, 90);
//    Set_Servo_Angle(H4, 90);
//    Set_Servo_Angle(H5, 90);
//    Set_Servo_Angle(H6, 90);
//    Set_Servo_Angle(H7, 90);
//    Set_Servo_Angle(H8, 90);
    //   vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1ms
//     Set_Servo_Angle(H1, 90 - 20);  // 设置舵机角度
//     Set_Servo_Angle(H2, 90 - 30);
//     Set_Servo_Angle(H3, 90 + 20);
//     Set_Servo_Angle(H4, 90 + 30);
//     Set_Servo_Angle(H5, 90 - 20);
//     Set_Servo_Angle(H6, 90 - 30);
//     Set_Servo_Angle(H7, 90 + 20);
//     Set_Servo_Angle(H8, 90 + 30);

    // vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1s


    //Gait_Forward();

    /* Infinite loop */
    for(;;)
    {
        //判断并更新舵机状�??????
        for(uint8_t i=0; i<8; i++){
            if(servos[i].current_angle != servos[i].target_angle){
               //Set_Servo_Angle(i, servos[i].target_angle);
                servos[i].current_angle = servos[i].target_angle;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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
    current_action = ACTION_FORWARD;

    Init_Servos();
//    Set_Servo_Angle(H1, 90);  // 设置舵机角度
//    Set_Servo_Angle(H2, 90);
//    Set_Servo_Angle(H3, 90);
//    Set_Servo_Angle(H4, 90);
//    Set_Servo_Angle(H5, 90);
//    Set_Servo_Angle(H6, 90);
//    Set_Servo_Angle(H7, 90);
//    Set_Servo_Angle(H8, 90);
//    vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1ms
//    Set_Servo_Angle(H1, 90 - 20);  // 设置舵机角度
//    Set_Servo_Angle(H2, 90 - 30);
//    Set_Servo_Angle(H3, 90 + 20);
//    Set_Servo_Angle(H4, 90 + 30);
//    Set_Servo_Angle(H5, 90 - 20);
//    Set_Servo_Angle(H6, 90 - 30);
//    Set_Servo_Angle(H7, 90 + 20);
//    Set_Servo_Angle(H8, 90 + 30);
//
//    vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1s

    /* Infinite loop */
    for(;;)
    {
        GaitControl();
        vTaskDelay(pdMS_TO_TICKS(50));

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
    uint8_t receivedByte;
    UART_ParseState state = STATE_WAIT_HEADER;
    uint8_t command = 0;

    /* Infinite loop */
    for(;;)
    {


        // 从队列读取数据
        if (xQueueReceive(rxQueue, &receivedByte, portMAX_DELAY) == pdTRUE) {
            switch (state) {
                case STATE_WAIT_HEADER:
                    if (receivedByte == 0xAA) {
                        state = STATE_WAIT_COMMAND;     // 检测到帧头，进入命令解析状态
                    }
                    break;

                case STATE_WAIT_COMMAND:
                    command = receivedByte;             // 记录命令字节
                    state = STATE_WAIT_FOOTER;
                    break;

                case STATE_WAIT_FOOTER:
                    if (receivedByte == 0x55) {          // 检测到帧尾，解析完整命令
                        switch (command) {
                            case 0x01:
                                Move_Forward();
                                UART_SendData_IT((uint8_t *) "Forward\n", 8);
                                break;
                            case 0x02:
                                Move_Backward();
                                UART_SendData_IT((uint8_t *) "Backward\n", 9);
                                break;
                            case 0x03:
                                Turn_Left();
                                UART_SendData_IT((uint8_t *) "Left\n", 5);
                                break;
                            case 0x04:
                                Turn_Right();
                                UART_SendData_IT((uint8_t *) "Right\n", 6);
                                break;
                            case 0x05:
                                Stop();
                                UART_SendData_IT((uint8_t *) "Stop\n", 5);
                                break;
                            case 0x06: {
                                float quat[4];
                                if (xQueueReceive(quatQueue, &quat, pdMS_TO_TICKS(10)) == pdTRUE) {
                                    SendAttitudeToHost(quat);  // 发送姿态数据
                                } else {
                                    UART_SendData_IT((uint8_t *) "No Data\n", 8);
                                }
                                break;
                            }
                            default:
                                HAL_UART_Transmit(&huart1, (uint8_t *) "Unknown Command\n", 16, HAL_MAX_DELAY);
                                break;
                        }
                    }
                    // 重置状态以准备接收下一帧
                    state = STATE_WAIT_HEADER;
                    break;
            }

        }



        //vTaskDelay(pdMS_TO_TICKS(1));  // 延时 1 毫秒
    }
  /* USER CODE END StartSerialCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */




void SendAttitudeToHost(float q[4]) {

    memset(txBuffer, 0, sizeof(txBuffer));          // 清除发送缓冲区
    txBuffer[0] = 0xAA;                             // 帧头
    txBuffer[1] = 0x06;                             // 数据类型
    memcpy(&txBuffer[2], &q[0], sizeof(float));     // 复制四元数
    memcpy(&txBuffer[6], &q[1], sizeof(float));
    memcpy(&txBuffer[10], &q[2], sizeof(float));
    memcpy(&txBuffer[14], &q[3], sizeof(float));
    txBuffer[18] = 0x55;  // 帧尾

    UART_SendData_IT(txBuffer, sizeof(txBuffer));

}

// 线程安全的串口发送函数
void UART_SendData_IT(uint8_t *data, uint16_t size) {
    // 获取互斥量
    if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
            huart1.gState = HAL_UART_STATE_READY;
            HAL_UART_Transmit_IT(&huart1, data, size);  // 使用中断发送数据
            if (xSemaphoreTake(uartTxCompleteSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
                xSemaphoreGive(uartMutex);                          // 超时释放互斥量
            }
        xSemaphoreGive(uartMutex);                                  // 释放互斥量
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        //串口变为就绪态
        huart1.gState = HAL_UART_STATE_READY;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 发送完成信号量
        xSemaphoreGiveFromISR(uartTxCompleteSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

#ifndef UART_DMA_IDLE
//UART接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
//        ringBuffer[writeIndex] = data;  // 存入环形缓冲区
//        writeIndex = (writeIndex + 1) % RING_BUFFER_SIZE;
        HAL_UART_Receive_IT(&huart1, &data, 1);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 如果队列满，将旧数据丢弃
//        if (uxQueueSpacesAvailable(rxQueue) == 0) {
//            xQueueReceive(rxQueue, NULL, 0);  // 丢弃旧数据
//        }
        xQueueSendFromISR(rxQueue, &data, &xHigherPriorityTaskWoken);  // 将接收数据放入队列
        //xSemaphoreGiveFromISR(uartRxCompleteSemaphore, &xHigherPriorityTaskWoken);
        huart1.gState = HAL_UART_STATE_READY;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                   // 若有高优先级任务则切换
    }
}
#elif defined UART_DMA_IDLE
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if(huart->Instance == USART1){
       // HAL_UART_Transmit_IT(&huart1, (uint8_t*)"StartDMARX", 100);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);

    }
}
#endif

/* 自定义空闲任务钩子函�?????? */
void vApplicationIdleHook(void) {
    vTaskDelay(1);
}

/* USER CODE END Application */

