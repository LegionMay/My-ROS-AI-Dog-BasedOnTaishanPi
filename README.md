# My-ROS-AI-Dog-BasedOnTaishanPi  
## 1. 迈出第一步  
### 1.1 泰山派SDK编译与环境配置  
泰山派交叉编译器地址可参考  ```/home/osboxes/RK3566APP/tspi_linux_sdk_20230916/Release/prebuilts/gcc/linux-x86/aarch64/gcc-buildroot-9.3.0-2020.03-x86_64_aarch64-rockchip-linux-gnu```  
OpenCV库SDK下路径  ```/home/osboxes/RK3566APP/tspi_linux_sdk_20230916/Release/external/rknpu2/examples/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV```    
可以参考[这篇博客](https://blog.csdn.net/Qiqili990707/article/details/135100663) 在STM32H743上基于FreeRTOS搭建micro-ROS环境。    
然而SDK自带的OpenCV库是不完整的，甚至缺少一些常用的函数。  
### 1.2 STM32部署FreeRTOS  
在这一步，我们基于cubemx为STM32H743部署FreeRTOS  
<img width="956" alt="72ad0387150ed69459199221d9d80da" src="https://github.com/user-attachments/assets/85a8e867-ca81-4ffb-8bef-148250aed3a0">

### 1.3 ROS环境搭建  
参考 (https://github.com/fishros/install) 快速搭建ROS环境  
## 2. 基于FreeRTOS开发下位机    
### 2.1 舵机控制  
在任务中，我利用基于事件的状态机来控制八个舵机，使用一个独立的舵机控制任务更新舵机状态：  
```
typedef struct {
    ServoID servo;
    float target_angle;
    float current_angle;
} ServoControl;
```
```
//判断并更新舵机状态
        for(uint8_t i=0; i<8; i++){
            if(servos[i].current_angle != servos[i].target_angle){
               Set_Servo_Angle(i, servos[i].target_angle);
                servos[i].current_angle = servos[i].target_angle;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
```
### 2.2 MPU9250 IIC读写   
### 2.3 使用互补滤波进行AHRS九轴姿态融合    
### 2.4 步态控制    
### 2.5 在任务中实现串口通信    
首先确保编译器支持浮点数格式化  
另外，根据[STM32H743内存地址的分配](https://www.armbbs.cn/forum.php?mod=viewthread&tid=123953)：  
DTCM： 0x20000000 ~ 0x20020000(size:128K)  
AXI SRAM（RAM_D1) : 0x24000000 ~ 0x24080000(size:512K)  
AHB SRAM（RAM_D2）：0x30000000 ~ 0x30048000(size:288K)  
SRAM4（RAM_D3）：0x38000000 ~ 0x38010000(size:64K)  
普通方式定义的全局变量都被分配到DTCM内存上  
其中DTCM和ITCM不支持DMA1、DMA2，后面的几块内存都是支持DMA1和DMA2的。  
在缓冲数组前加上__attribute__((section(".RAM_D2")))指定分配空间即可  
除此之外，要想在FreeRTOS任务中实现连续的串口中断发送，需要在发送完成回调函数中手动恢复串口状态为就绪态  

在这里，我使用了串口的中断接收模式，在接收完成回调函数中通过队列作为消息缓冲区向串口数据解析任务传递接收到的指令。
在串口数据解析任务中，我利用有限状态机判断帧头帧尾和指令内容：  
```
typedef enum {
    STATE_WAIT_HEADER,    // 等待帧头0xAA
    STATE_WAIT_COMMAND,   // 等待命令字节
    STATE_WAIT_FOOTER     // 等待帧尾0x55
} UART_ParseState;
```
同时，我还利用互斥量和串口发送完成信号量实现了一个线程安全的串口发送函数：
```
// 线程安全的串口发送函数
void UART_SendData_IT(uint8_t *data, uint16_t size) {
    // 获取互斥量
    if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
            HAL_UART_Transmit_IT(&huart1, data, size);  // 使用中断发送数据
            if (xSemaphoreTake(uartTxCompleteSemaphore, pdMS_TO_TICKS(500)) != pdTRUE) {
                xSemaphoreGive(uartMutex);                          // 超时释放互斥量
            }
        xSemaphoreGive(uartMutex);                                  // 释放互斥量
    }
}
```
<img width="970" alt="dd3eb012ef10ecc18c1abe1ce6fcf68" src="https://github.com/user-attachments/assets/a3747ac6-c8db-4137-84a9-c27457fb415f">

### 2.6 实现多种基本步态  
## 3. 泰山派ROS开发  
### 3.1 配置激光雷达ROS环境  
我采用的YDLIDAR X2激光雷达官方提供了ROS1 与 ROS2 支持。
#### ROS1  
根据官方SDK和使用手册来驱动激光雷达，首先编译并安装SDK  
```$ git clone https://github.com/YDLIDAR/YDLidar-SDK.git
$ cd YDLidar-SDK/build  
$ cmake ..  
$ make  
$ sudo make install
```
创建/ydlidar_ws/src/，
```git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git```克隆、构建ydlidar_ros_driver并设置环境  
``` $ git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git     
$ cd ydlidar_ws
$ catkin_make
$ source ./devel/setup.sh
```
可以添加永久工作区环境变量  
```$ echo "source ~/ydlidar_ws/devel/setup.bash" >> ~/.bashrc  
$ source ~/.bashrc  
$ echo $ROS_PACKAGE_PATH
```
使用启动文件运行 ydlidar_ros_driver并利用rviz查看扫描结果  
```$ roslaunch ydlidar_ros_driver X2.launch
$roslaunch ydlidar_ros_driver lidar_view.launch
```
#### ROS2  
参考https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/master ，大体如上所述，记得根据自己的激光雷达型号和串口修改相应的.py和.yaml文件。  
这样就成功连接的激光雷达：  
<img width="1093" alt="ee1b0e7ba54520ba57ae3b90c3dc009" src="https://github.com/user-attachments/assets/8ad233ed-eb75-440d-8154-39f3afa0b4db">  

![920ba163fa977b7833bc164a6cb42b1](https://github.com/user-attachments/assets/75b7e348-bf4d-483b-a431-a6709fff12a0)

### 3.2 修改设备树  

![image](https://github.com/user-attachments/assets/0089677e-19bf-4453-9ba5-8088fed43785)  

    
泰山派在官方系统镜像下能够直接使用的串口只有UART1和UART3，其中UART1为调试串口，UART3分配给了激光雷达，所以我们必须通过修改设备树来启用其他串口从而与下位机进行通信。  
在SDK的/kernel/arch/arm64/boot/dts/rockchip/路径下可以找到tspi-rk3566-user-v10-linux.dts，这就是我们要修改的文件。  
  
<img width="1208" alt="b42d0f90420b9cbe1638e56e7ce2821" src="https://github.com/user-attachments/assets/0f73c341-269c-4e7a-aa0c-6a71ce33e55b">  
  
将PWM8和PWM9改为disabled，并加上UART4  
  
<img width="449" alt="414d1c57350c0442180626664094609" src="https://github.com/user-attachments/assets/6cda6f48-b03c-4498-a44f-fe5748457889">  
  
<img width="1210" alt="16c197f8d00ecb7d1ab4698e051b23b" src="https://github.com/user-attachments/assets/7abc254c-683a-4780-b958-fe08ef1910f3">  
  
修改保存后我们就可以用```make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- dtbs```命令编译出新的dtb文件，注意在Makefile里加上这一行  
  
<img width="1206" alt="66281bb1f7bc1b0e8ba42739ac0e450" src="https://github.com/user-attachments/assets/c922b4e7-1d9a-4ec5-a853-fed541374a83">  
  
<img width="776" alt="cea1fcc9b6e2e64b2991f11663bd4e0" src="https://github.com/user-attachments/assets/456e480a-8c1f-409e-8c06-4d4e7e65ee3b">  
  
不过这里我采取的是在SDK目录下通过```./build.sh kernel```命令编译内核生成boot.image并烧录的方法  
  
<img width="1280" alt="9010fb83e56a0877379fc316085981a" src="https://github.com/user-attachments/assets/362f5817-0553-4373-9bb5-184308c508d5">   
  
这样就编译成功了，注意要提前安装交叉编译器并配置环境变量  
  
<img width="734" alt="e76bbb10a585972d23055b951ee1564" src="https://github.com/user-attachments/assets/e49b2b27-6651-415a-9b29-1f2871008456">
  
这样烧录进开发板后重启系统，我们可以发现出现了新的串口设备ttyS4，这说明我们成功了  
  
<img width="457" alt="d5a20b23786c10a5a9fb9c67a8df01b" src="https://github.com/user-attachments/assets/cf0b4a0d-561d-4d81-a82f-e395eb160caf">   
  







### 3.3 实现串口通信   



### 3.4 实现避障算法  
