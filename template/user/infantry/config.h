/**
 * @brief 机器人参数
 * @note  默认参数及参数列表见 default_config.h
 */

#include "default_config.h"

// 步兵编号
#define ROBOT_MIAO (Robot_Id == 1)
#define ROBOT_WANG (Robot_Id == 2)
#define ROBOT_SHARK (Robot_Id == 3)

// 调试
#define DEBUG_ENABLED 0          // 调试开关
#define SERIAL_DEBUG_PORT USART6 // 串口调试端口

// 运动参数
#define GIMBAL_PITCH_MIN -38
#define GIMBAL_PITCH_MAX 13
#define CHASSIS_ROTOR_SPEED 550

// 底盘配置
#define CHASSIS_MOTOR_REDUCTION_RATE 19.2f  // 底盘电机减速比
#define CHASSIS_SIZE_K 0.385f               // 测量值, 机器人中心点到XY边缘的距离之和
#define CHASSIS_INVERSE_WHEEL_RADIUS 13.16f // 测量值, 麦克纳姆轮半径的倒数
#define CHASSIS_RADIUS 0.250f               // 测量值，全向轮电机到底盘俯视图几何中心的距离
// k0,k1,k2,k3,k4,k5
#define SECOND_MACLAURIN_COEFFICIENT {0.6641993412640775,     \
                                      0.006444284468539646,   \
                                      0.0001423857226262331,  \
                                      0.017644430204543864,   \
                                      0.1650143850678086,     \
                                      3.096721772539512e-05   \
                                    }      

//拨弹配置
#define STIR_MOTOR_REDUCTION_RATE 36.0f // 拨弹电机减速比

//射击配置
#define FIRE_MOTOR_REDUCTION_RATE 19.2f // 射击电机减速比

//云台配置
#define HAS_SLIP_RING 1                  // 该步兵拥有滑环
#define GIMBAL_MOTOR_REDUCTION_RATE 1.0f // 云台电机减速比

// 陀螺仪设置
#define BOARD_FRONT_IS_UP 1                     // 板子正面朝上
#define BOARD_SHORT_SIDE_IS_PARALLEL_TO_PITCH 0 // 板子短边朝下

#define GYROSCOPE_YAW_FILTER_THRESHOLD 0.005f // 零飘修正阈值   (2024/6/8)感觉不太好用，是我不会用吗？
#define GYROSCOPE_YAW_MODIFICATION -0.002f    // 零飘修正值
#define GYROSCOPE_LSB 16.384f                 // 陀螺仪敏感度 2^16/4000
#define ACCELERATE_LSB 4096.0f                // 加速度计敏感度 2^16/16

// DMA
#define DMA_BUFFER_LENGTH 128 // DMA发送接收长度

//电机相关参数
#define C2T_3508 0.3f  //3508电机转矩对电流系数
#define CurrentMap_C620 0.0012f  //c620电调电流对输入值的映射系数
#define CurrentMap_C620_Inverse 833.333f //
