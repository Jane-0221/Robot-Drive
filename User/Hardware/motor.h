#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stdint.h"

#include "User_math.h"
#include "CAN_receive_send.h"
// 自定义区
#define USE_DJI_MOTOR
#define QUANTITY_OF_CAN 3       // 你所用的can的数量，（比所用的多就行了，喵板上设3别改了）
#define QUANTITY_OF_DJIMOTOR 11 // 挂载电机最多的一路can上所用的电机

// 常数区
#define PI 3.1415926f
#define ECD_TO_ANGEL_DJI 0.043945f //(360/8192),将编码器值转化为角度制
#define DJIMOTOR_MAX_CURRENT 16384 // M2006+M3508最大电流 20A / DJI_MAX_CURRENT
#define DJIMOTOR_T_A 0.3 //3508转矩常数
#define USE_LZMotor     (1)  
#ifdef USE_DJI_MOTOR
typedef enum
{
    /* data */
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
} Motor_Type_e;

/* DJImotorCAN send and receive ID */
typedef enum
{
    CAN_20063508_1_4_send_ID = 0x200,

    CAN_20063508_5_8_send_ID = 0x1FF,

    CAN_6020_1_4_send_ID = 0x1FE,

    CAN_6020_5_7_send_ID = 0x2FE,

    CAN_ID1 = 0x201,
} DJIcan_send_id_e;

// 6020从 CAN_1_5(对应ID1) 到 CAN_1_6020_7（对应ID7）
// 3508/2006从 CAN_1_1(对应ID1) 到 CAN_1_8（对应ID8）
// 显然，6020与3508/2006存在重叠ID,设置时请注意
typedef enum
{
    CAN_1_1 = 0,  // 0
    CAN_1_2,      // 1
    CAN_1_3,      // 2
    CAN_1_4,      // 3
    CAN_1_5,      // 4
    CAN_1_6,      // 5
    CAN_1_7,      // 6
    CAN_1_8,      // 7
    CAN_1_6020_5, // 8
    CAN_1_6020_6, // 9
    CAN_1_6020_7, // 10

    CAN_2_1, // 11
    CAN_2_2, // 12
    CAN_2_3,
    CAN_2_4,
    CAN_2_5, // 15
    CAN_2_6,
    CAN_2_7, // 17
    CAN_2_8,
    CAN_2_6020_5,
    CAN_2_6020_6,
    CAN_2_6020_7, // 21

    CAN_3_1, // 22
    CAN_3_2, // 23
    CAN_3_3,
    CAN_3_4,
    CAN_3_5,
    CAN_3_6,
    CAN_3_7,
    CAN_3_8,
    CAN_3_6020_5,
    CAN_3_6020_6,
    CAN_3_6020_7, // 32

} DJIcan_id;

typedef struct
{
    // 控制数据
    int16_t set; // 设定的电流 / 电压
    Motor_Type_e Motor_type;

    // 原始数据
    uint16_t ecd;          // 编码器数值
    int16_t speed_rpm;     // 转速RPM
    int16_t given_current; // 实际转矩电流
    uint8_t temperate;     // 温度（获取不到）
    uint16_t last_ecd;     // 上一次编码器的数值

    // 计算数据
    long long ecd_cnt;  // 编码器计数器
    double angle_cnt;   // 转过的总角度 degree
    double angle_zero;  // 编码器0点角度 degree
    double angle;       // -180~180 degree
    double round_speed; // 出轴转速 rpm
} DJI_motor_data_s;
// extern DJIMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR];
void DJIMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id);
void DJIMotor_setzero(double zero_angle, DJIcan_id motor_id);
void DJIMotor_set(int16_t val, DJIcan_id motor_id);
void DJIMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
void DJIMotor_send_current(void);

#endif //! USE_DJI_MOTOR

#if (USE_LZMotor == 1)
#include "LZ_motor_driver.h"

// ???CAN???????????
#define QUANTITY_OF_LZMOTOR 6

// ???ID???????3·CAN???·6???????
typedef enum {
    LZ_CAN_1_1 = 0,
    LZ_CAN_1_2,
    LZ_CAN_1_3,
    LZ_CAN_1_4,
    LZ_CAN_1_5,
    LZ_CAN_1_6,
    
    LZ_CAN_2_1,
    LZ_CAN_2_2,
    LZ_CAN_2_3,
    LZ_CAN_2_4,
    LZ_CAN_2_5,
    LZ_CAN_2_6,
    
    LZ_CAN_3_1,
    LZ_CAN_3_2,
    LZ_CAN_3_3,
    LZ_CAN_3_4,
    LZ_CAN_3_5,
    LZ_CAN_3_6,
    
    LZ_MOTOR_NUM
} LZ_Motor_ID_t;

void LZMotor_init(LZ_Motor_ID_t motor_id);
void LZMotor_velocity_init(LZ_Motor_ID_t motor_id) ;
void LZMotor_position_init(LZ_Motor_ID_t motor_id) ;

void LZMotor_set_params(LZ_Motor_ID_t motor_id, float pos, float vel, float tor, float kp, float kd);
void LZMotor_set_pos_param(LZ_Motor_ID_t motor_id, float pos, float vel) ;
void LZMotor_set_vel_param(LZ_Motor_ID_t motor_id, float vel,float current_limit) ;

void LZMotor_velocity_enable(LZ_Motor_ID_t motor_id) ;
void LZMotor_position_enable(LZ_Motor_ID_t motor_id) ;

int  LZMotor_send_command(LZ_Motor_ID_t motor_id);



void LZMotor_disable(LZ_Motor_ID_t motor_id) ;
void LZMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

// ?????????????
extern LZ_Motor_t LZ_Motors[QUANTITY_OF_CAN][QUANTITY_OF_LZMOTOR];

#endif // USE_LINGZU_MOTOR
#ifdef USE_UNTREE_MOTOR

typedef struct
{
    // 定义 发送格式化数据

    // 待发送的各项数据
    unsigned short motor_LR; // 电机左右，0左1右
    unsigned short motor_id; // 电机ID，0xBB代表全部电机
    unsigned short mode;     // 0:空闲, 5:开环转动, 10:闭环FOC控制
    // 实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
    float T;   // 期望关节的输出力矩（减速器实际的力矩）（Nm）
    float W;   // 期望关节速度（电机本身的速度）(rad/s)
    float Pos; // 期望关节位置（rad）
    float K_P; // 关节刚度系数
    float K_W; // 关节速度系数

} MOTOR_send;

#endif // USE_UNTREE_MOTOR

#endif // !__MOTOR_H__