#ifndef __HEAD_H__
#define __HEAD_H__
#include <stdint.h>
#include "can_receive_send.h"
#include "ktech_motor.h"

// ==================== 硬件资源统一定义 ====================
/**
 * @brief CAN1控制器句柄（头部连杆电机挂载在CAN1总线）
 * @note  等价于&hfdcan1，统一宏定义便于后续修改
 */
#define CAN_HANDLE_1 (&hfdcan1)

/** 头部连杆电机ID宏定义（CAN1总线） */
#define MOTOR_LINKONG_1_ID 0x01 // 头部连杆1号电机ID
#define MOTOR_LINKONG_2_ID 0x02 // 头部连杆2号电机ID

/** 电机旋转方向宏定义 */
#define DIR_CW  0x00    // 顺时针方向
#define DIR_CCW 0x01    // 逆时针方向

// ==================== 数据结构体定义 ====================
/**
 * @brief 头部电机（连杆电机）控制数据结构体
 * @note  存储电机的当前状态、目标参数和旋转方向，适配科泰（KTech）电机协议
 */
typedef struct
{
    float current_velocity; // 当前速度（单位：rad/s，弧度/秒）
    float current_angle;    // 当前角度（单位：rad，弧度）
    uint8_t direction;      // 旋转方向（0=顺时针(DIR_CW)，1=逆时针(DIR_CCW)）
    uint32_t target_angle;  // 目标角度（分辨率0.01°/LSB，如18000对应180°）
    uint16_t max_speed;     // 最大转速（单位：dps，度/秒，360 dps即360°/秒）
} Head_MotorData_t;

// ==================== 全局变量声明 ====================
/**
 * @brief 头部连杆电机控制数据数组（2路）
 * @note  head_motor_data[0]对应1号连杆电机，head_motor_data[1]对应2号连杆电机
 */
extern Head_MotorData_t head_motor_data[2];

/**
 * @brief 科泰电机对象数组（2路）
 * @note  存储科泰电机的协议解析数据（反馈角度、速度、电流等），对应2个头部连杆电机
 */
extern KTech_Motor_t motor_linkong[2];

// ==================== 函数声明 ====================
/**
 * @brief 头部电机初始化函数
 * @retval 无
 * @note   初始化头部2路连杆电机，配置CAN1通信、科泰电机协议参数
 */
extern void Head_Init(void);

/**
 * @brief 控制头部1号连杆电机
 * @retval 无
 * @note   根据head_motor_data[0]的目标参数，通过CAN1发送控制指令
 */
extern void Head_Lk_motor1(void);

/**
 * @brief 头部所有电机控制指令发送函数
 * @retval 无
 * @note   批量发送头部2路连杆电机的控制指令，建议在主循环周期性调用
 */
extern void Head_all_tx(void);

/**
 * @brief 请求头部两个电机反馈编码器数据
 * @retval 无
 */
extern void Head_RequestFeedback(void);
extern void Head_RequestFeedbackByIndex(uint8_t motor_index);
extern uint8_t Head_FeedbackReady(void);
extern uint8_t Head_MotorFeedbackReady(uint8_t motor_index);
extern uint32_t Head_GetFeedbackCount(uint8_t motor_index);
extern void Head_NotifyFeedback(uint8_t motor_index);
extern void Head_TxMotorByIndex(uint8_t motor_index);

/**
 * @brief 使能头部两个电机
 * @retval 无
 */
extern void Head_Motor_Enable(void);
extern void Head_Motor_SendEnableCommand(void);

/**
 * @brief 失能头部两个电机
 * @retval 无
 */
extern void Head_Motor_Disable(void);

/**
 * @brief 保存当前位置为头部电机零点
 * @retval 无
 */
extern void Head_save_position(void);

/**
 * @brief 更新头部连杆电机当前状态数据
 * @retval 无
 * @note   从motor_linkong数组读取科泰电机反馈数据，更新head_motor_data的current_angle/current_velocity
 */
extern void Head_Lk_Data_update(void);

#endif
