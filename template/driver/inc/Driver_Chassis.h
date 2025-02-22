/**
 * @file Driver_Chassis.h
 * @brief 底盘运动驱动
 */

#ifndef __DRIVER_CHASSIS_H
#define __DRIVER_CHASSIS_H

#include "stm32f4xx.h"
#include "Driver_PID.h"

typedef struct {
    // 转子转速
    float rotorSpeed[4];
    // 转矩
    float rotorTorgue[4];

    float   vx;
    float   vy;
    float   vw;
    float   realvx;
    float   realvy;
    float   realvw;
    float   Fx;
    float   Fy;
    float   T;
    // 功率限制
    PID_Type PID_Power;
    float    power;
    float    maxPower;
    float    targetPower;
    float    referencePower;
    float    lastReferencePower;
    float    interval;
    uint32_t fittingCounter;
    float    powerScale;
    float    powerBuffer;
    float    maxPowerBuffer;
} ChassisData_Type;

/**
 * @brief 初始化底盘
 */
void Chassis_Init(ChassisData_Type *ChassisData);

/**
 * @brief 更新麦轮转速
 * @param XSpeed 左右 m/s
 * @param YSpeed 前后 m/s
 * @param WSpeed 旋转 rad/s 逆时针为正
 * @note 电机位置：左上角0,逆时针依次增加
 * @note 转子的角速度(rad/s) = 电机减速比 * 轮子角速度
 */
void Chassis_Update(ChassisData_Type *ChassisData, float XSpeed, float YSpeed, float WSpeed);

/**
 * @brief 更新扭矩前馈
 */
void Chassis_Updata_FT(ChassisData_Type * cd, float Fx, float Fy, float T);

/**
 * @brief 修正旋转后底盘的前进方向
 * @param angle 期望的前进方向
 */
void Chassis_Fix(ChassisData_Type *ChassisData, float angle);

/**
 * @brief 全向运动学逆解算
 */
void Chassis_Calculate_Rotor_Speed(ChassisData_Type *ChassisData);
/**
 * @brief 全向运动学正解算
 */
void Chassis_Calculate_Real_Speed(ChassisData_Type *cd, int16_t * motor_Speed);
/**
 * @brief 全向动力学逆解算
 */
void Chassis_Calculate_Rotor_Torgue(ChassisData_Type *cd);
/**
 * @brief 设置转子速度上限 (rad/s)
 * @param wheelSpeed
 * @param rotorSpeed
 */
void Chassis_Limit_Rotor_Speed(ChassisData_Type *ChassisData, float maxRotorSpeed);

/**
 * @brief 按比例更新速度
 * @param scale 速度缩放比例
 */
void Chassis_Scale_Rotor_Speed(ChassisData_Type *ChassisData, float scale);

/**
 * @brief 设置功率更新上限
 * @param maxPower       功率上限
 * @param targetPower    目标功率
 * @param referencePower 参考功率 (来源裁判系统或电流计)
 * @param interval       任务周期
 */
void Chassis_Limit_Power(ChassisData_Type *cd, float targetPower, float referencePower, float referencePowerBuffer, float interval);

/**
 * @brief 整合速度环pid的output和扭矩前馈
 * 
 * @param motorCurrentOutput  电流输出，速度环在调用此函数前需加入到该变量
 * @param cd    扭矩前馈
 */
void Chassis_Current_Output_Integrate(float *motorCurrentOutput, ChassisData_Type* cd);

/**
 * @brief 电流衰减法限制功率
 * 
 * @param motorCurrentOutput    整合完的电流输出
 * @param MCO_With_PowerLimit   经过功率限制后的电流输出
 * @param realMotorSpeed        运动学正解算出的电机转子角速度
 * @param targetPower           裁判系统的功率限制值
 */
void Chassis_Calculate_Power_Limit(float* motorCurrentOutput, int16_t* MCO_With_PowerLimit, float *realMotorSpeed, float targetPower);
#endif
