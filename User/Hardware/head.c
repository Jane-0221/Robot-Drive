#include "head.h"
#include "ktech_motor.h"

#define HEAD_SINGLE_TURN_UNITS 36000U
#define HEAD_HALF_TURN_UNITS 18000
#define HEAD_SPEED_MIN 1U
#define HEAD_SPEED_MAX 150U
#define HEAD_SPEED_FULL_ERROR_UNITS 6000U

KTech_Motor_t motor_linkong[2];         // 凌空电机结构体定义
Head_MotorData_t head_motor_data[2];    // 头部电机数据结构体定义

static volatile uint8_t head_motor_enabled = 1U;

static uint32_t Head_NormalizeAngle(uint32_t angle)
{
    return angle % HEAD_SINGLE_TURN_UNITS;
}

static uint32_t Head_CurrentAngleToUnits(float angle_deg)
{
    if (angle_deg <= 0.0f)
    {
        return 0U;
    }

    return ((uint32_t)(angle_deg * 100.0f + 0.5f)) % HEAD_SINGLE_TURN_UNITS;
}

static int32_t Head_GetShortestDelta(uint32_t current_angle, uint32_t target_angle)
{
    int32_t delta = (int32_t)target_angle - (int32_t)current_angle;

    if (delta > HEAD_HALF_TURN_UNITS)
    {
        delta -= (int32_t)HEAD_SINGLE_TURN_UNITS;
    }
    else if (delta < -HEAD_HALF_TURN_UNITS)
    {
        delta += (int32_t)HEAD_SINGLE_TURN_UNITS;
    }

    return delta;
}

static uint16_t Head_CalcDynamicSpeed(uint8_t motor_index, uint32_t target_angle)
{
    uint32_t current_angle = Head_CurrentAngleToUnits(head_motor_data[motor_index].current_angle);
    int32_t delta = Head_GetShortestDelta(current_angle, target_angle);
    uint32_t error_units = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;

    if (error_units >= HEAD_SPEED_FULL_ERROR_UNITS)
    {
        return (uint16_t)HEAD_SPEED_MAX;
    }

    return (uint16_t)(HEAD_SPEED_MIN +
                     ((error_units * (HEAD_SPEED_MAX - HEAD_SPEED_MIN)) /
                      HEAD_SPEED_FULL_ERROR_UNITS));
}

static void Head_UpdateShortestDirection(uint8_t motor_index, uint32_t target_angle)
{
    uint32_t current_angle = Head_CurrentAngleToUnits(head_motor_data[motor_index].current_angle);
    int32_t delta = Head_GetShortestDelta(current_angle, target_angle);

    if (delta > 0)
    {
        head_motor_data[motor_index].direction = DIR_CW;
    }
    else if (delta < 0)
    {
        head_motor_data[motor_index].direction = DIR_CCW;
    }
}

void Head_Init()
{
    ktech_motor_init(MOTOR_LINKONG_1_ID);
    // 将电机1从关闭状态切换到运行状态
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);

    // 电机1数据初始化
    head_motor_data[0].direction = DIR_CW;   // 0:顺时针, 1:逆时针
    head_motor_data[0].target_angle = 0;
    head_motor_data[0].max_speed = 10;


    ktech_motor_init(MOTOR_LINKONG_2_ID);
    // 将电机2从关闭状态切换到运行状态
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
    
    // 电机2数据初始化
    head_motor_data[1].direction = DIR_CW;   // 0:顺时针, 1:逆时针
    head_motor_data[1].target_angle = 0;
    head_motor_data[1].max_speed = 10;
};

// 循环执行的电机1控制函数
void Head_Lk_motor1(void)
{
    uint32_t current_target = Head_NormalizeAngle(head_motor_data[0].target_angle);
    uint16_t speed_limit;

    Head_UpdateShortestDirection(0U, current_target);
    speed_limit = Head_CalcDynamicSpeed(0U, current_target);

    ktech_pos_single2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_1_ID, 
                      head_motor_data[0].direction, 
                      current_target, 
                      speed_limit);
}

// 循环执行的电机2控制函数
void Head_Lk_motor2()
{
    uint32_t current_target = Head_NormalizeAngle(head_motor_data[1].target_angle);
    uint16_t speed_limit;

    Head_UpdateShortestDirection(1U, current_target);
    speed_limit = Head_CalcDynamicSpeed(1U, current_target);

    ktech_pos_single2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_2_ID, 
                      head_motor_data[1].direction, 
                      current_target, 
                      speed_limit);
}

// 头部电机整体发送函数
void Head_all_tx()
{
    if (head_motor_enabled == 0U)
    {
        return;
    }

    Head_Lk_motor1();
    HAL_Delay(1);
    Head_Lk_motor2();
}

void Head_RequestFeedback(void)
{
    ktech_read_status2(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);
    HAL_Delay(1);
    ktech_read_status2(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
}

void Head_Motor_Enable(void)
{
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);
    HAL_Delay(1);
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
    head_motor_enabled = 1U;
}

void Head_Motor_Disable(void)
{
    head_motor_enabled = 0U;
    ktech_motor_off(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);
    HAL_Delay(1);
    ktech_motor_off(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
}

// 保存当前位置为零点，并保持新零点位置
static void Head_SaveZeroAndHold(uint8_t motor_index, uint16_t motor_id)
{
    uint16_t speed_limit;

    ktech_motor_stop(CAN_HANDLE_1, motor_id);
    HAL_Delay(1);

    ktech_set_zero(CAN_HANDLE_1, motor_id);
    head_motor_data[motor_index].target_angle = 0U;
    HAL_Delay(1);

    ktech_motor_on(CAN_HANDLE_1, motor_id);
    HAL_Delay(1);

    speed_limit = Head_CalcDynamicSpeed(motor_index, 0U);
    ktech_pos_single2(CAN_HANDLE_1,
                      motor_id,
                      head_motor_data[motor_index].direction,
                      0U,
                      speed_limit);
}

void Head_save_position(void)
{
    Head_SaveZeroAndHold(0U, MOTOR_LINKONG_1_ID);
    HAL_Delay(1);
    Head_SaveZeroAndHold(1U, MOTOR_LINKONG_2_ID);
    head_motor_enabled = 1U;
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
