#ifndef __ARM_H__
#define __ARM_H__

#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "crc_ccitt.h"
#include "DrEmpower_can.h"

// ==================== 硬件资源统一定义 ====================
/**
 * @brief CAN2控制器句柄（机械臂所有电机均挂载在CAN2总线）
 * @note  等价于&hfdcan2，统一宏定义便于后续修改
 */
#define CAN_HANDLE_2 (&hfdcan2)

// ==================== 电机ID宏定义 ====================
/** 灵足电机ID（Robstride04协议，CAN2总线） */
#define MOTOR_LINGZU_1_ID 0x01 // 灵足1号电机ID
#define MOTOR_LINGZU_2_ID 0x02 // 灵足2号电机ID
#define MOTOR_LINGZU_3_ID 0x03 // 灵足3号电机ID

/** 大然电机ID（DrEmpower协议，CAN2总线） */
#define MOTOR_DARAN_1_ID 0x0b // 大然11号电机ID（0x0b=十进制11）
#define MOTOR_DARAN_2_ID 0x0c // 大然12号电机ID（0x0c=十进制12）
#define MOTOR_DARAN_3_ID 0x0d // 大然13号电机ID（0x0d=十进制13）

/** 达妙电机ID（DM4310协议，CAN2总线） */
#define MOTOR_DAMIAO_4_ID 0x04 // 达妙4号电机ID
#define MOTOR_DAMIAO_5_ID 0x05 // 达妙5号电机ID
#define MOTOR_DAMIAO_6_ID 0x06 // 达妙6号电机ID

/** 余数电机ID（预留，未在代码中实际使用） */
#define MOTOR_YUSHU_1_ID 2 // 余数1号电机ID
#define MOTOR_YUSHU_2_ID 3 // 余数2号电机ID

// ==================== 枚举类型定义 ====================
/**
 * @brief 达妙电机状态枚举
 * @note  标识电机使能/禁用状态
 */
typedef enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0, // 电机禁用状态
    Motor_DM_Status_ENABLE,      // 电机使能状态
} Motor_DM_Status;

/**
 * @brief 机械臂肩部电机类型枚举
 * @note  用于切换不同品牌电机的控制逻辑
 */
typedef enum
{
    SHOULDER_TYPE_LINGZU = 0, // 肩部电机类型：灵足电机
    SHOULDER_TYPE_DARAN = 1   // 肩部电机类型：大然电机
} ShoulderType_t;

// ==================== 数据结构体定义 ====================
/**
 * @brief 机械臂电机数据结构体
 * @note  存储电机的当前/目标角度、速度，单位均为弧度(rad)、弧度/秒(rad/s)
 */
typedef struct
{
    float current_angle;    // 当前角度（单位：rad）
    float current_velocity; // 当前速度（单位：rad/s）
    float target_angle;     // 目标角度（单位：rad）
    float target_velocity;  // 目标速度（单位：rad/s）
} ArmMotorData_t;

// ==================== 全局变量声明 ====================
/** 灵足电机对象（3路，Robstride04协议） */
extern RobStride_Motor_t motor1; // 灵足1号电机对象
extern RobStride_Motor_t motor2; // 灵足2号电机对象
extern RobStride_Motor_t motor3; // 灵足3号电机对象

/** 大然电机状态/电气参数结构体数组（3路） */
extern struct servo_state servo_state_daran[3];   // 大然电机状态（角度、速度、扭矩等）
extern struct servo_volcur servo_volcur_daran[3]; // 大然电机电压/电流参数

/** 达妙电机状态数组（6路） */
extern Motor_DM_Status DM_Status[6]; // 存储每个达妙电机的使能/禁用状态

/** 全局肩部电机类型选择变量 */
extern ShoulderType_t g_ShoulderType;

/** 电机控制数据结构体数组（按品牌分类） */
extern ArmMotorData_t Linzu_motor_data[3];  // 灵足电机控制数据（3路）
extern ArmMotorData_t Daran_motor_data[3];  // 大然电机控制数据（3路）
extern ArmMotorData_t Damiao_motor_data[3]; // 达妙电机控制数据（3路）

// ==================== 函数声明 ====================
/**
 * @brief 机械臂初始化函数
 * @retval 无
 * @note   初始化灵足/大然/达妙电机，配置CAN2通信，设置默认肩部电机类型
 */
void Arm_Init(void);

/** 灵足电机控制函数（按编号区分） */
void Arm_Linzu_motor1(void); // 控制1号灵足电机
void Arm_Linzu_motor2(void); // 控制2号灵足电机
void Arm_Linzu_motor3(void); // 控制3号灵足电机

/** 大然电机控制函数（按编号区分） */
void Arm_Daran_motor1(void); // 控制1号大然电机
void Arm_Daran_motor2(void); // 控制2号大然电机
void Arm_Daran_motor3(void); // 控制3号大然电机

/** 达妙电机控制函数（按编号区分） */
void Arm_Damiao_motor4(void); // 控制4号达妙电机
void Arm_Damiao_motor5(void); // 控制5号达妙电机
void Arm_Damiao_motor6(void); // 控制6号达妙电机

/** 电机状态数据更新函数 */
void Arm_Linzu_Data_update(void); // 更新灵足电机当前角度/速度
void Arm_Daran_Data_update(void); // 更新大然电机当前角度/速度

/** 全局函数声明（跨文件调用） */
extern void Arm_All_Data_update(void); // 批量更新所有电机状态数据
extern void Arm_all_tx(void);          // 发送所有电机控制指令

#endif