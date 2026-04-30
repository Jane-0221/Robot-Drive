#include "arm.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "pid.h"
#include "remote_control.h"
#include "gpio.h"
#include "gom_protocol.h"
#include "usart.h"
#include "LZ_motor_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "DrEmpower_can.h"
#include "stdio.h"

// ===================== 全局变量定义 =====================
/**
 * @brief 机械臂肩部电机类型（枚举值：SHOULDER_TYPE_LINGZU/DAREN/DAMIAO）
 * @note  用于切换不同品牌电机的控制逻辑（灵足/大然/达妙）
 */
ShoulderType_t g_ShoulderType;

/**
 * @brief 大然舵机状态结构体数组（3路）
 * @note  存储大然舵机的状态信息（角度、速度、扭矩等）
 */
struct servo_state servo_state_daran[3];

/**
 * @brief 大然舵机电压/电流结构体数组（3路）
 * @note  存储大然舵机的电气参数（输入电压、相电流等）
 */
struct servo_volcur servo_volcur_daran[3];

/**
 * @brief 达妙电机状态结构体数组（6路）
 * @note  存储DM4310电机的位置、速度、电流等状态
 */
extern Motor_DM_Status DM_Status[6];

/**
 * @brief 灵足电机对象（3路）
 * @note  Robstride04协议驱动，对应灵足1/2/3号电机
 */
RobStride_Motor_t motor1; // 灵足1号电机
RobStride_Motor_t motor2; // 灵足2号电机
RobStride_Motor_t motor3; // 灵足3号电机

/**
 * @brief 灵足电机数据结构体数组（3路）
 * @note  存储灵足电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Linzu_motor_data[3];

/**
 * @brief 大然电机数据结构体数组（3路）
 * @note  存储大然电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Daran_motor_data[3];

/**
 * @brief 达妙电机数据结构体数组（3路）
 * @note  存储达妙电机的目标、当前角度、速度等控制参数
 */
ArmMotorData_t Damiao_motor_data[3];

/**
 * @brief 数据回复使能标志
 * @note  浮点型：0.0f=禁用回复，非0.0f=启用回复（未在本代码中实际使用）
 */
float reply_enable = 0.0f;

#define ARM_REENABLE_CHECK_PERIOD_MS 100U
#define ARM_DAMIAO_ENABLE_STATE 2U
#define ARM_TARGET_ANGLE_LIMIT_RAD 0.785398163f

static float Arm_ClampTargetAngle(float angle)
{
    if (angle > ARM_TARGET_ANGLE_LIMIT_RAD)
    {
        return ARM_TARGET_ANGLE_LIMIT_RAD;
    }

    if (angle < -ARM_TARGET_ANGLE_LIMIT_RAD)
    {
        return -ARM_TARGET_ANGLE_LIMIT_RAD;
    }

    return angle;
}

// ===================== 函数定义 =====================
/**
 * @brief 机械臂初始化函数
 * @retval 无
 * @note   1. 核心功能：选择肩部电机类型，初始化灵足/大然/达妙电机，配置CAN2通信；
 *         2. 灵足电机：初始化RobStride协议，设置CSP位置模式，使能电机，开启主动上报；
 *         3. 大然电机：清除错误，设置模式2（位置模式），配置参数22001（比例系数）；
 *         4. 达妙电机：初始化位置模式，暂未配置具体参数；
 *         5. 所有电机均挂载在CAN2总线上。
 */
void Arm_Init()
{
    // 选择默认肩部电机类型（注释为灵足，实际启用大然）
    g_ShoulderType = SHOULDER_TYPE_LINGZU;
    // g_ShoulderType = SHOULDER_TYPE_DARAN;

    /* 灵足电机初始化（使用CAN2总线） */
    // 1号灵足电机初始化
    RobStride_Motor_Init(&motor1, MOTOR_LINGZU_1_ID, false);                             // 初始化电机对象（ID为灵足1号）
    Get_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005);                        // 读取电机参数，0X7005为参数地址
    HAL_Delay(10);                                                                       // 延时确保通信稳定
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j'); // 设置CSP位置控制模式
    Enable_Motor(&motor1, (hcan_t *)CAN_HANDLE_2);                                       // 使能电机
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7017, 1.0f, 'p');             // 设置参数0X7017，比例1.0
    HAL_Delay(10);
    // 开启主动上报（0x00=关闭，0x01=开启）
    RobStride_Motor_ProactiveEscalationSet(&motor1, CAN_HANDLE_2, 0x01);

    // 2号灵足电机初始化（逻辑2号）
    RobStride_Motor_Init(&motor2, MOTOR_LINGZU_2_ID, false);
    Get_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor2, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor2, CAN_HANDLE_2, 0x01);

    // 3号灵足电机初始化（逻辑3号）
    RobStride_Motor_Init(&motor3, MOTOR_LINGZU_3_ID, false);
    Get_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor3, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor3, CAN_HANDLE_2, 0x01);

    /* 大然电机初始化（使用CAN2总线） */
    clear_error(CAN_HANDLE_2, MOTOR_DARAN_1_ID);                    // 清除1号大然电机错误
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 2);                    // 设置模式2（位置模式）
    write_property(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 22001, 3, 1.0f); // 写入参数22001（比例系数1.0）

    clear_error(CAN_HANDLE_2, MOTOR_DARAN_2_ID);                    // 清除2号大然电机错误
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 2);                    // 设置位置模式
    write_property(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 22001, 3, 1.0f); // 配置比例系数

    clear_error(CAN_HANDLE_2, MOTOR_DARAN_3_ID);                    // 清除3号大然电机错误
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 2);                    // 设置位置模式
    write_property(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 22001, 3, 1.0f); // 配置比例系数

    /* 达妙电机初始化（使用CAN2总线） */
    arm_motor_init(&arm_motor[Motor4], MOTOR_DAMIAO_4_ID, POS_MODE); // 4号达妙电机初始化（位置模式）
    arm_motor_init(&arm_motor[Motor5], MOTOR_DAMIAO_5_ID, POS_MODE); // 5号达妙电机初始化（位置模式）
    arm_motor_init(&arm_motor[Motor6], MOTOR_DAMIAO_6_ID, POS_MODE); // 6号达妙电机初始化（位置模式）

    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE); // 使能4号达妙电机位置模式
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE); // 使能5号达妙电机位置模式
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE); // 使能6号达妙电机位置模式
    set_DM_mode(Motor4, POS_MODE);
    set_DM_mode(Motor5, POS_MODE);
    set_DM_mode(Motor6, POS_MODE);

    Damiao_motor_data[0].target_angle = 0.0f;
    Damiao_motor_data[1].target_angle = 0.0f;
    Damiao_motor_data[2].target_angle = 0.0f;
    Damiao_motor_data[0].target_velocity = 1.0f;
    Damiao_motor_data[1].target_velocity = 1.0f;
    Damiao_motor_data[2].target_velocity = 1.0f;
    // 初始化灵足电机目标参数（角度10°，速度1.0r/min）
    // Set_ZeroPos(&motor1, CAN_HANDLE_2);
    // Set_ZeroPos(&motor2, CAN_HANDLE_2);
    // Set_ZeroPos(&motor3, CAN_HANDLE_2);

    Linzu_motor_data[0].target_angle = 0.0f;
    Linzu_motor_data[1].target_angle = 0.0f;
    Linzu_motor_data[2].target_angle = 0.0f;
    Linzu_motor_data[0].target_velocity = 1.0f;
    Linzu_motor_data[1].target_velocity = 1.0f;
    Linzu_motor_data[2].target_velocity = 1.0f;

    // 初始化大然电机目标参数（角度10°，速度分别为90/20/20r/min）
    Daran_motor_data[0].target_angle = 0.0f;
    Daran_motor_data[1].target_angle = 0.0f;
    Daran_motor_data[2].target_angle = 0.0f;
    Daran_motor_data[0].target_velocity = 20.0f;
    Daran_motor_data[1].target_velocity = 20.0f;
    Daran_motor_data[2].target_velocity = 20.0f;
    // 初始化灵足电机位置为0

    // 初始化电机失能
    // Disenable_Motor(&motor1, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // Disenable_Motor(&motor2, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // Disenable_Motor(&motor3, CAN_HANDLE_2, 0U);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
    // osDelay(1);
    // disable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);
}

/**
 * @brief 控制1号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[0]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor1()
{
    RobStride_Motor_CSP_control(&motor1, CAN_HANDLE_2, Arm_ClampTargetAngle(Linzu_motor_data[0].target_angle), Linzu_motor_data[0].target_velocity);
}

/**
 * @brief 控制2号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[1]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor2()
{
    RobStride_Motor_CSP_control(&motor2, CAN_HANDLE_2, Arm_ClampTargetAngle(Linzu_motor_data[1].target_angle), Linzu_motor_data[1].target_velocity);
}

/**
 * @brief 控制3号灵足电机（CSP位置模式）
 * @retval 无
 * @note   根据Linzu_motor_data[2]的目标角度、速度，通过CAN2发送控制指令
 */
void Arm_Linzu_motor3()
{
    RobStride_Motor_CSP_control(&motor3, CAN_HANDLE_2, Arm_ClampTargetAngle(Linzu_motor_data[2].target_angle), Linzu_motor_data[2].target_velocity);
}

/**
 * @brief 控制1号大然电机（位置模式）
 * @retval 无
 * @note   1. 延时1ms确保通信稳定；
 *         2. 设置目标角度、速度、加速度（10.0f）、立即生效（1）；
 *         3. 指令通过CAN2发送至1号大然电机。
 */
void Arm_Daran_motor1()
{
    osDelay(1);
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_1_ID, Daran_motor_data[0].target_angle, Daran_motor_data[0].target_velocity, 10.0f, 1);
}

/**
 * @brief 控制2号大然电机（位置模式）
 * @retval 无
 * @note   逻辑2号大然电机，使用Daran_motor_data[1]的目标参数
 */
void Arm_Daran_motor2()
{
    osDelay(1);
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_2_ID, Daran_motor_data[1].target_angle, Daran_motor_data[1].target_velocity, 10.0f, 1);
}

/**
 * @brief 控制3号大然电机（位置模式）
 * @retval 无
 * @note   逻辑3号大然电机，使用Daran_motor_data[2]的目标参数
 */
void Arm_Daran_motor3()
{
    osDelay(1);
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_3_ID, Daran_motor_data[2].target_angle, Daran_motor_data[2].target_velocity, 10.0f, 1);
}

/**
 * @brief 控制4号达妙电机（位置模式）
 * @retval 无
 * @note   1. 设置位置模式；
 *         2. 配置目标位置/速度；
 *         3. 设置位置速度控制参数（5=位置参数，10=速度参数）；
 *         4. 固定目标位置为20（可根据需求修改）。
 */
void Arm_Damiao_motor4()
{
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, Arm_ClampTargetAngle(Damiao_motor_data[0].target_angle), Damiao_motor_data[0].target_velocity);
}

/**
 * @brief 控制5号达妙电机（位置模式）
 * @retval 无
 * @note   逻辑5号达妙电机，位置参数10，速度参数1
 */
void Arm_Damiao_motor5()
{
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, Arm_ClampTargetAngle(Damiao_motor_data[1].target_angle), Damiao_motor_data[1].target_velocity);
}

/**
 * @brief 控制6号达妙电机（位置模式）
 * @retval 无
 * @note   1. 设置位置模式；
 *         2. 配置目标位置/速度；
 *         3. 固定目标位置为10，使用电机6的位置设定值作为控制参数；
 *         4. 速度参数固定为1。
 */
void Arm_Damiao_motor6()
{
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, Arm_ClampTargetAngle(Damiao_motor_data[2].target_angle), Damiao_motor_data[2].target_velocity);
}

/**
 * @brief 更新灵足电机当前状态数据
 * @retval 无
 * @note   从motor1/2/3的RobStride协议缓存中，读取当前角度、速度，更新到Linzu_motor_data
 */
void Arm_Linzu_Data_update()
{
    Linzu_motor_data[0].current_angle = motor1.Pos_Info.Angle;
    Linzu_motor_data[1].current_angle = motor2.Pos_Info.Angle;
    Linzu_motor_data[2].current_angle = motor3.Pos_Info.Angle;
    Linzu_motor_data[0].current_velocity = motor1.Pos_Info.Speed;
    Linzu_motor_data[1].current_velocity = motor2.Pos_Info.Speed;
    Linzu_motor_data[2].current_velocity = motor3.Pos_Info.Speed;
}

/**
 * @brief 更新大然电机当前状态数据
 * @retval 无
 * @note   1. 从daran_motor_state缓存中读取角度、速度（扭矩注释未使用）；
 *         2. 更新到Daran_motor_data的current_angle/current_velocity成员；
 *         3. 调试打印代码已注释，如需查看可取消注释。
 */
void Arm_Daran_Data_update()
{

    Daran_motor_data[0].current_angle = daran_motor_state[0].angle;
    Daran_motor_data[1].current_angle = daran_motor_state[1].angle;
    Daran_motor_data[2].current_angle = daran_motor_state[2].angle;
    Daran_motor_data[0].current_velocity = daran_motor_state[0].speed;
    Daran_motor_data[1].current_velocity = daran_motor_state[1].speed;
    Daran_motor_data[2].current_velocity = daran_motor_state[2].speed;
}
void Arm_Damiao_Data_update()
{
    Damiao_motor_data[0].current_angle = arm_motor[3].para.pos;
    Damiao_motor_data[1].current_angle = arm_motor[4].para.pos;
    Damiao_motor_data[2].current_angle = arm_motor[5].para.pos;
    Damiao_motor_data[0].current_velocity = arm_motor[3].para.vel;
    Damiao_motor_data[1].current_velocity = arm_motor[4].para.vel;
    Damiao_motor_data[2].current_velocity = arm_motor[5].para.vel;
}

/**
 * @brief 批量更新所有电机状态数据
 * @retval 无
 * @note   1. 先更新灵足电机数据；
 *         2. 延时1ms后更新大然电机数据；
 *         3. 达妙电机数据更新逻辑暂未实现。
 */
void Arm_All_Data_update()
{
    Arm_Linzu_Data_update();
    HAL_Delay(1);    HAL_Delay(1);    HAL_Delay(1);
    Arm_Daran_Data_update();
    HAL_Delay(1);
    Arm_Damiao_Data_update();
}

/**
 * @brief 机械臂电机控制指令发送函数
 * @retval 无
 * @note   1. 根据g_ShoulderType选择发送灵足/大然电机控制指令；
 *         2. 灵足电机：依次发送1/2/3号指令，间隔1ms FreeRTOS延时；
 *         3. 大然电机：依次发送1/2/3号指令，间隔1ms FreeRTOS延时；
 *         4. 达妙电机控制指令已注释，如需启用可取消注释。
 */
void Arm_all_tx()
{
    if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
    {
        Arm_Linzu_motor1();
        osDelay(1);
        Arm_Linzu_motor2();
        osDelay(1);
        Arm_Linzu_motor3();
        osDelay(1);
    }
    else if (g_ShoulderType == SHOULDER_TYPE_DARAN)
    {
        Arm_Daran_motor1();
        osDelay(1);
        Arm_Daran_motor2();
        osDelay(1);
        Arm_Daran_motor3();
        osDelay(1);
    }

    Arm_Damiao_motor4();
    osDelay(1);
    Arm_Damiao_motor5();
    osDelay(1);
    Arm_Damiao_motor6();
    osDelay(1);
}

static void Arm_ReenableLinzuMotor(RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    if (motor->Pos_Info.pattern == 2U)
    {
        return;
    }

    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(motor, CAN_HANDLE_2);
    RobStride_Motor_CSP_control(motor, CAN_HANDLE_2, Arm_ClampTargetAngle(motor_data->target_angle), motor_data->target_velocity);
}

static void Arm_ReenableDamiaoMotor(uint16_t motor_id, uint16_t motor_index, ArmMotorData_t *motor_data)
{
    if (arm_motor[motor_index].para.state == ARM_DAMIAO_ENABLE_STATE)
    {
        return;
    }

    enable_motor_mode(CAN_HANDLE_2, motor_id, POS_MODE);
    set_DM_mode(motor_index, POS_MODE);
    pos_speed_ctrl(CAN_HANDLE_2, motor_id, Arm_ClampTargetAngle(motor_data->target_angle), motor_data->target_velocity);
}

void Arm_CheckAndReenableDisabledMotors(void)
{
    static uint32_t last_check_tick = 0U;
    uint32_t now_tick = HAL_GetTick();

    if ((now_tick - last_check_tick) < ARM_REENABLE_CHECK_PERIOD_MS)
    {
        return;
    }

    last_check_tick = now_tick;

    Arm_ReenableLinzuMotor(&motor1, &Linzu_motor_data[0]);
    Arm_ReenableLinzuMotor(&motor2, &Linzu_motor_data[1]);
    Arm_ReenableLinzuMotor(&motor3, &Linzu_motor_data[2]);

    Arm_ReenableDamiaoMotor(MOTOR_DAMIAO_4_ID, Motor4, &Damiao_motor_data[0]);
    Arm_ReenableDamiaoMotor(MOTOR_DAMIAO_5_ID, Motor5, &Damiao_motor_data[1]);
    Arm_ReenableDamiaoMotor(MOTOR_DAMIAO_6_ID, Motor6, &Damiao_motor_data[2]);
}

#define ARM_LINZU_ZERO_STOP_DELAY_MS 20U
#define ARM_LINZU_ZERO_APPLY_DELAY_MS 50U
#define ARM_LINZU_ZERO_REENABLE_DELAY_MS 20U

static void Arm_Linzu_SendStop(RobStride_Motor_t *motor)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = ((uint32_t)Communication_Type_MotorStop << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    canx_send_ext_data(CAN_HANDLE_2, ext_id, txdata, 8);
}

static void Arm_Linzu_SendSetZero(RobStride_Motor_t *motor)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = ((uint32_t)Communication_Type_SetPosZero << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    txdata[0] = 1U;
    canx_send_ext_data(CAN_HANDLE_2, ext_id, txdata, 8);
}

static void Arm_Linzu_SaveZeroAndHold(RobStride_Motor_t *motor, ArmMotorData_t *motor_data)
{
    motor_data->target_angle = 0.0f;
    motor_data->target_velocity = 1.0f;

    Arm_Linzu_SendStop(motor);
    osDelay(ARM_LINZU_ZERO_STOP_DELAY_MS);
    Arm_Linzu_SendSetZero(motor);
    osDelay(ARM_LINZU_ZERO_APPLY_DELAY_MS);

    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    Enable_Motor(motor, CAN_HANDLE_2);
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    Set_RobStride_Motor_parameter(motor, CAN_HANDLE_2, 0X7017, motor_data->target_velocity, 'p');
    osDelay(ARM_LINZU_ZERO_REENABLE_DELAY_MS);
    RobStride_Motor_CSP_control(motor, CAN_HANDLE_2, Arm_ClampTargetAngle(motor_data->target_angle), motor_data->target_velocity);
}

void Arm_save_position(void)
{
    Damiao_motor_data[0].target_angle = 0.0f;
    Damiao_motor_data[1].target_angle = 0.0f;
    Damiao_motor_data[2].target_angle = 0.0f;
    Damiao_motor_data[0].target_velocity = 1.0f;
    Damiao_motor_data[1].target_velocity = 1.0f;
    Damiao_motor_data[2].target_velocity = 1.0f;

    Arm_Linzu_SaveZeroAndHold(&motor1, &Linzu_motor_data[0]);
    osDelay(1);
    Arm_Linzu_SaveZeroAndHold(&motor2, &Linzu_motor_data[1]);
    osDelay(1);
    Arm_Linzu_SaveZeroAndHold(&motor3, &Linzu_motor_data[2]);
    osDelay(1);

    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID);
    osDelay(1);
    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID);
    osDelay(1);
    CAN_Send_Save_Zero(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID);
    osDelay(1);

    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID);
    osDelay(1);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, Arm_ClampTargetAngle(Damiao_motor_data[0].target_angle), Damiao_motor_data[0].target_velocity);
    osDelay(1);
    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID);
    osDelay(1);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, Arm_ClampTargetAngle(Damiao_motor_data[1].target_angle), Damiao_motor_data[1].target_velocity);
    osDelay(1);
    CAN_Send_Enter(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID);
    osDelay(1);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, Arm_ClampTargetAngle(Damiao_motor_data[2].target_angle), Damiao_motor_data[2].target_velocity);
}
