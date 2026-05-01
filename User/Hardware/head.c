#include "head.h"
#include "ktech_motor.h"

KTech_Motor_t motor_linkong[2];         // 凌空电机结构体定义
Head_MotorData_t head_motor_data[2];    // 头部电机数据结构体定义

void Head_Init()
{
    ktech_motor_init(MOTOR_LINKONG_1_ID);
    // 将电机1从关闭状态切换到运行状态
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);

    // 电机1数据初始化
    head_motor_data[0].direction = DIR_CW;   // 0:顺时针, 1:逆时针
    head_motor_data[0].target_angle = 18000;
    head_motor_data[0].max_speed = 360;


    ktech_motor_init(MOTOR_LINKONG_2_ID);
    // 将电机2从关闭状态切换到运行状态
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
    
    // 电机2数据初始化
    head_motor_data[1].direction = DIR_CW;   // 0:顺时针, 1:逆时针
    head_motor_data[1].target_angle =18000;
    head_motor_data[1].max_speed = 360;
};

// 循环执行的电机1控制函数
void Head_Lk_motor1(void)
{
    // 静态变量，记录上一次执行时的目标角度值，用于和当前值对比
    // 注意：结构体中target_angle是uint32_t类型，此处同步设为uint32_t
    static uint32_t last_target_angle = 0;
    
    // 获取最新的目标角度，简化代码阅读
    uint32_t current_target = head_motor_data[0].target_angle;

    /************************* 旋转方向判断逻辑 *************************/
    // 当目标值 > 上一次目标值时，顺时针旋转，方向设为DIR_CW
    if (current_target > last_target_angle)
    {
        head_motor_data[0].direction = DIR_CW;
    }
    // 当目标值 < 上一次目标值时，逆时针旋转，方向设为DIR_CCW
    else if (current_target < last_target_angle)
    {
        head_motor_data[0].direction = DIR_CCW;
    }
    // 目标值相等时，方向保持不变，不做修改，防止不必要的参数刷新

    /************************* 执行电机指令 *************************/
    ktech_pos_single2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_1_ID, 
                      head_motor_data[0].direction, 
                      current_target, 
                      head_motor_data[0].max_speed);

    /************************* 记录当前值，用于下一次循环对比 *************************/
    last_target_angle = current_target;
}

// 循环执行的电机2控制函数
void Head_Lk_motor2()
{
    // 静态变量，记录上一次执行时的目标角度值，用于和当前值对比
    // 注意：结构体中target_angle是uint32_t类型，此处同步设为uint32_t
    static uint32_t last_target_angle = 0;
    
    // 获取最新的目标角度，简化代码阅读
    uint32_t current_target = head_motor_data[1].target_angle;

    /************************* 旋转方向判断逻辑 *************************/
    // 当目标值 > 上一次目标值时，顺时针旋转，方向设为DIR_CW
    if (current_target > last_target_angle)
    {
        head_motor_data[1].direction = DIR_CW;
    }
    // 当目标值 < 上一次目标值时，逆时针旋转，方向设为DIR_CCW
    else if (current_target < last_target_angle)
    {
        head_motor_data[1].direction = DIR_CCW;
    }
    // 目标值相等时，方向保持不变，不做修改，防止不必要的参数刷新

    /************************* 执行电机指令 *************************/
    ktech_pos_single2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_2_ID, 
                      head_motor_data[1].direction, 
                      current_target, 
                      head_motor_data[1].max_speed);

    /************************* 记录当前值，用于下一次循环对比 *************************/
    last_target_angle = current_target;
}

// 头部电机整体发送函数
void Head_all_tx()
{
    Head_Lk_motor1();
    HAL_Delay(1);
    Head_Lk_motor2();
}

// 头部电机状态数据更新函数
void Head_Lk_Data_update()
{
    // 将电机1编码器值转换为角度值 (假设一圈编码器分辨率为65536)
    head_motor_data[0].current_angle = motor_linkong[0].fb.encoder / 65536.0f * 360.0f;     
    head_motor_data[0].current_velocity = motor_linkong[0].fb.speed;                 

    // 将电机2编码器值转换为角度值 
    head_motor_data[1].current_angle = motor_linkong[1].fb.encoder / 65536.0f * 360.0f;     
    head_motor_data[1].current_velocity = motor_linkong[1].fb.speed;                 
}