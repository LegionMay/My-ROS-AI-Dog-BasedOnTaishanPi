#ifndef VMC_H
#define VMC_H

#include <stdint.h>

/**
 * @brief 生成腿部虚拟力并计算目标位置
 * 
 * @param leg        当前计算的腿部编号
 * @param quat       当前姿态的四元数
 * @param targetX    输出的目标X位置
 * @param targetY    输出的目标Y位置
 * @param targetZ    输出的目标Z位置
 */
void VMC_GenerateForces(int leg, float* quat, float* targetX, float* targetY, float* targetZ);

/**
 * @brief 控制单条腿，根据虚拟力和逆运动学求解关节角度并控制舵机
 * 
 * @param leg    腿部编号
 * @param x      目标X位置
 * @param y      目标Y位置
 * @param z      目标Z位置
 */
void VMC_LegControl(int leg, float x, float y, float z);

/**
 * @brief 辅助函数，计算腿的目标X位置
 * 
 * @param leg      腿部编号
 * @param forceX   X方向的虚拟力
 * @return float   足尖的目标X位置
 */
float CalculateLegTargetX(int leg, float forceX);

/**
 * @brief 辅助函数，计算腿的目标Y位置
 * 
 * @param leg      腿部编号
 * @param forceY   Y方向的虚拟力
 * @return float   足尖的目标Y位置
 */
float CalculateLegTargetY(int leg, float forceY);

/**
 * @brief 辅助函数，计算腿的目标Z位置
 * 
 * @param leg      腿部编号
 * @param forceZ   Z方向的虚拟力
 * @return float   足尖的目标Z位置
 */
float CalculateLegTargetZ(int leg, float forceZ);

#endif
