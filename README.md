# My-ROS-AI-Dog-BasedOnTaishanPi  
## 1. 迈出第一步  
### 1.1 泰山派SDK编译与环境配置  
泰山派交叉编译器地址可参考  ```/home/osboxes/RK3566APP/tspi_linux_sdk_20230916/Release/prebuilts/gcc/linux-x86/aarch64/gcc-buildroot-9.3.0-2020.03-x86_64_aarch64-rockchip-linux-gnu```  
OpenCV库SDK下路径  ```/home/osboxes/RK3566APP/tspi_linux_sdk_20230916/Release/external/rknpu2/examples/3rdparty/opencv/opencv-linux-aarch64/share/OpenCV```    
参考[这篇博客](https://blog.csdn.net/Qiqili990707/article/details/135100663)在STM32H743上基于FreeRTOS搭建micro-ROS环境。    
然而SDK自带的OpenCV库是不完整的，甚至缺少一些常用的函数。  
### 1.2 STM32部署FreeRTOS  
在这一步，我们基于cubemx为STM32H743部署FreeRTOS  
### 1.3 ROS环境搭建  
## 2. 基于FreeRTOS部署运动控制算法  
### 2.1 编写舵机控制相关函数  
### 2.2 部署运动学逆解算法  
### 2.3 编写MPU9250读写相关代码   
### 2.4 使用扩展卡尔曼滤波(EKF)进行AHRS九轴姿态融合  
### 2.5 部署VMC算法  
### 2.6 实现多种基本步态  
## 3. 泰山派ROS开发  
### 3.1 配置ROS环境  
### 3.2 实现导航与避障算法  
### 3.3 修改设备树并实现串口通信  
