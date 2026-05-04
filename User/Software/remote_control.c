#include "remote_control.h"
#include "ramp_generator.h"
#include "IMU_updata.h"
#include "Stm32_time.h"
#include <tim.h>
#include "pid.h"
#include "UART_data_txrx.h"
#include "Sbus.h"
#include "head.h"
#include "arm.h"
#include "lift_control.h"
#include "pump_control.h"
#include "arm_sv.h"
#include "uart_protocol.h" // 添加PC通信协议头文件
#include "arm_sv.h"
#include "omni_wheel.h"
#include "cmsis_os.h"
#include "Robstride04.h"
// 遥控器值
#define LOW_VALUE 353
#define MID_VALUE 1024
#define HIGH_VALUE 1694
#define RANGE 50
#define ARM_MOTOR_STEP 0.001f
#define ARM_SELECTED_MOTOR_INVALID 0xFFU
#define ARM_MOTOR_CH3_ENABLE_THRESHOLD 1500U
#define ARM_MOTOR_CH3_DISABLE_THRESHOLD 500U
#define ARM_MOTOR_CH3_LATCH_NONE 0U
#define ARM_MOTOR_CH3_LATCH_ENABLE 1U
#define ARM_MOTOR_CH3_LATCH_DISABLE 2U
#define HEAD_RAD_TO_ANGLE_UNIT (18000.0f / 3.14159265358979323846f)
#define HEAD_SINGLE_TURN_UNITS 36000L
#define PC_ARM_MOTOR_DEBUG_STATE_NONE 0xFFU
// extern DnData_t pc_dn_data;

volatile uint8_t pc_arm_motor_enable_state_debug[ARM_LOGICAL_MOTOR_COUNT] = {
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
    PC_ARM_MOTOR_DEBUG_STATE_NONE,
};
volatile uint8_t pc_arm_motor_last_index_debug = PC_ARM_MOTOR_DEBUG_STATE_NONE;
volatile uint8_t pc_arm_motor_last_enable_state_debug = PC_ARM_MOTOR_DEBUG_STATE_NONE;
volatile uint32_t pc_arm_motor_command_count_debug = 0U;

static volatile uint8_t arm_motor_disable_active = 0U;
static uint8_t arm_motor_disable_latched = 0U;
static volatile uint8_t head_motor_disable_active = 0U;
static uint8_t head_motor_enabled_latched = 1U;
static uint8_t arm_save_position = 0U;
static uint8_t arm_motor_ch3_latch[ARM_LOGICAL_MOTOR_COUNT] = {0U};
static uint8_t arm_motor_save_zero_latched = 0U;
static uint8_t sbus_match(uint16_t value, uint16_t target)
{
    return (value >= (target - RANGE)) && (value <= (target + RANGE));
}

static float sbus_axis_to_float(uint16_t value, float scale)
{
    int32_t delta = (int32_t)value - MID_VALUE;

    // if ((delta > -RANGE) && (delta < RANGE))
    // {
    //     return 0.0f;
    // }

    return (float)delta / scale;
}

static uint32_t head_radian_to_target_angle(float radian)
{
    float angle_unit_f = radian * HEAD_RAD_TO_ANGLE_UNIT;
    int32_t angle_unit = (angle_unit_f >= 0.0f) ? (int32_t)(angle_unit_f + 0.5f) : (int32_t)(angle_unit_f - 0.5f);

    while (angle_unit < 0)
    {
        angle_unit += HEAD_SINGLE_TURN_UNITS;
    }

    while (angle_unit >= HEAD_SINGLE_TURN_UNITS)
    {
        angle_unit -= HEAD_SINGLE_TURN_UNITS;
    }

    return (uint32_t)angle_unit;
}

void remote_control_init()
{
}

uint8_t Arm_Motor_Disable_Updata(void)
{
    uint8_t disable_request = (SBUS_CH.CH5 == LOW_VALUE) &&
                              (SBUS_CH.CH6 == HIGH_VALUE) &&
                              (SBUS_CH.CH7 == HIGH_VALUE) &&
                              (SBUS_CH.CH8 == HIGH_VALUE);

    arm_motor_disable_active = disable_request;

    if (disable_request == 0U)
    {
        arm_motor_disable_latched = 0U;
        return 0U;
    }

    if (arm_motor_disable_latched != 0U)
    {
        return 1U;
    }
    for (uint8_t logical_motor = 0U; logical_motor < ARM_LOGICAL_MOTOR_COUNT; logical_motor++)
    {
        (void)Arm_DisableMotorByIndex(logical_motor);
        osDelay(1);
    }

    arm_motor_disable_latched = 1U;
    return 1U;
}

uint8_t Arm_Motor_Disable_IsActive(void)
{
    return arm_motor_disable_active;
}

uint8_t Head_Motor_Enable_Disable_Updata(void)
{
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        head_motor_disable_active = 1U;

        if (head_motor_enabled_latched != 0U)
        {
            Head_Motor_Disable();
            head_motor_enabled_latched = 0U;
        }

        return 1U;
    }

    if (SBUS_CH.CH8 == LOW_VALUE)
    {
        head_motor_disable_active = 0U;

        if (head_motor_enabled_latched == 0U)
        {
            Head_Motor_Enable();
            head_motor_enabled_latched = 1U;
        }

        return 0U;
    }

    return head_motor_disable_active;
}

uint8_t Head_Motor_Disable_IsActive(void)
{
    return head_motor_disable_active;
}

uint8_t Arm_Save_Position_IsActive(void)
{
    return arm_save_position;
}

void Pump_Control_Updata(void)
{
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        pump_state = PUMP_ON;
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        pump_state = PUMP_OFF;
    }
}

static uint8_t Arm_GetRemoteSelectedMotor(void)
{
    uint8_t motor_offset;

    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        motor_offset = 0U;
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        motor_offset = 3U;
    }
    else
    {
        return ARM_SELECTED_MOTOR_INVALID;
    }

    switch (SBUS_CH.CH7)
    {
    case HIGH_VALUE:
        return motor_offset;

    case MID_VALUE:
        return motor_offset + 1U;

    case LOW_VALUE:
        return motor_offset + 2U;

    default:
        return ARM_SELECTED_MOTOR_INVALID;
    }
}

static void Arm_ResetMotorCh3Latches(void)
{
    uint8_t i;

    for (i = 0U; i < ARM_LOGICAL_MOTOR_COUNT; i++)
    {
        arm_motor_ch3_latch[i] = ARM_MOTOR_CH3_LATCH_NONE;
    }
}

void Arm_Motor_Control_Updata(void)
{
    uint8_t selected_motor = Arm_GetRemoteSelectedMotor();

    if (selected_motor >= ARM_LOGICAL_MOTOR_COUNT)
    {
        if ((SBUS_CH.CH3 <= ARM_MOTOR_CH3_ENABLE_THRESHOLD) &&
            (SBUS_CH.CH3 >= ARM_MOTOR_CH3_DISABLE_THRESHOLD))
        {
            Arm_ResetMotorCh3Latches();
        }

        if (SBUS_CH.CH5 != HIGH_VALUE)
        {
            arm_motor_save_zero_latched = 0U;
        }

        return;
    }

    if (SBUS_CH.CH1 != 0U)
    {
        if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
        {
            (void)Arm_AdjustMotorTargetByIndex(selected_motor, ARM_MOTOR_STEP);
        }
        else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
        {
            (void)Arm_AdjustMotorTargetByIndex(selected_motor, -ARM_MOTOR_STEP);
        }
    }

    if (SBUS_CH.CH3 > ARM_MOTOR_CH3_ENABLE_THRESHOLD)
    {
        if (arm_motor_ch3_latch[selected_motor] != ARM_MOTOR_CH3_LATCH_ENABLE)
        {
            (void)Arm_EnableMotorByIndex(selected_motor);
            arm_motor_ch3_latch[selected_motor] = ARM_MOTOR_CH3_LATCH_ENABLE;
        }
    }
    else if (SBUS_CH.CH3 < ARM_MOTOR_CH3_DISABLE_THRESHOLD)
    {
        if (arm_motor_ch3_latch[selected_motor] != ARM_MOTOR_CH3_LATCH_DISABLE)
        {
            (void)Arm_DisableMotorByIndex(selected_motor);
            arm_motor_ch3_latch[selected_motor] = ARM_MOTOR_CH3_LATCH_DISABLE;
        }
    }
    else
    {
        Arm_ResetMotorCh3Latches();
    }

    if (SBUS_CH.CH5 == HIGH_VALUE)
    {
        if (arm_motor_save_zero_latched == 0U)
        {
            (void)Arm_SaveMotorZeroByIndex(selected_motor);
            arm_motor_save_zero_latched = 1U;
        }
    }
    else
    {
        arm_motor_save_zero_latched = 0U;
    }
}
void arm_save_home_position(void)
{
    if (SBUS_CH.CH5 == HIGH_VALUE && SBUS_CH.CH6 == HIGH_VALUE)
    {
        arm_save_position = 1U;
        Linzu_motor_data[0].target_angle = 0.0f;
        Linzu_motor_data[1].target_angle = 0.0f;
        Linzu_motor_data[2].target_angle = 0.0f;
        Damiao_motor_data[0].target_angle = 0.0f;
        Damiao_motor_data[1].target_angle = 0.0f;
        Damiao_motor_data[2].target_angle = 0.0f;

        return;
    }
    arm_save_position = 0U;
}
void head_save_home_position(void)
{
    if (SBUS_CH.CH5 == HIGH_VALUE && SBUS_CH.CH6 == HIGH_VALUE)
    {

        head_motor_data[0].target_angle = 0.0f;
        head_motor_data[1].target_angle = 0.0f;
        Head_save_position();
        return;
    }
}

void Head_Motor_Control_Updata(void)
{
    // 替换原有if-else if结构为switch语句
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        switch (SBUS_CH.CH6)
        {
        case HIGH_VALUE:
            // head_motor_data[0].target_angle = 0;
            // head_motor_data[1].target_angle = 0; //

            // duties_tx.duty0 = 0.12;
            //  duties_tx.duty1 = 0.12;
            //  duties_tx.duty2 = 0.12;
            //  duties_tx.duty3 = 0.12;
            //  duties_tx.duty4 = 0.12;
            //  duties_tx.duty5 = 0.12;
            // motor_radians[0] = PI / 4;
            // motor_radians[1] = PI / 3;
            // motor_radians[2] = PI / 2;
            // motor_radians[3] = PI / 2;
            // motor_radians[4] = PI / 2;
            // motor_radians[5] = PI / 2;
            if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
            {
                motor_radians[0] += 0.003f;
            }
            else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
            {
                motor_radians[0] -= 0.003f;
            }

            if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
            {
                motor_radians[1] += 0.003f;
            }
            else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
            {
                motor_radians[1] -= 0.003f;
            }

            break;

        case MID_VALUE:
            // head_motor_data[0].target_angle = 9000;
            // head_motor_data[1].target_angle = 9000; // 头部电机

            // duties_tx.duty0 = 0.075;
            // duties_tx.duty1 = 0.075;
            // duties_tx.duty2 = 0.075;
            // duties_tx.duty3 = 0.075;
            // duties_tx.duty4 = 0.075;
            // duties_tx.duty5 = 0.075;
            // motor_radians[0] = 0.0f;
            // motor_radians[1] = 0.0f;
            // motor_radians[2] = 0.0f;
            // motor_radians[3] = 0.0f;
            // motor_radians[4] = 0.0f;
            // motor_radians[5] = 0.0f;
            if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
            {
                motor_radians[2] += 0.003f;
            }
            else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
            {
                motor_radians[2] -= 0.003f;
            }

            if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
            {
                motor_radians[3] += 0.003f;
            }
            else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
            {
                motor_radians[3] -= 0.003f;
            }

            break;

        case LOW_VALUE:
            // head_motor_data[0].target_angle = 18000;
            // head_motor_data[1].target_angle = 18000;

            // duties_tx.duty0 = 0.03;
            // duties_tx.duty1 = 0.03;
            // duties_tx.duty2 = 0.03;
            // duties_tx.duty3 = 0.03;
            // duties_tx.duty4 = 0.03;
            // duties_tx.duty5 = 0.03;
            // motor_radians[0] = -PI / 4;
            // motor_radians[1] = -PI / 3;
            // motor_radians[2] = -PI / 2;
            // motor_radians[3] = -PI / 2;
            // motor_radians[4] = -PI / 4;
            // motor_radians[5] = -PI / 4;
            if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
            {
                motor_radians[4] += 0.003f;
            }
            else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
            {
                motor_radians[4] -= 0.003f;
            }

            if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
            {
                motor_radians[5] += 0.003f;
            }
            else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
            {
                motor_radians[5] -= 0.003f;
            }

            break;

        default:
            // 可选：处理SBUS_CH.CH6不是上述三个值的情况
            // 如果不需要特殊处理，这里可以留空
            break;
        }
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        switch (SBUS_CH.CH6)
        {
        case HIGH_VALUE: // 挥手到左边（第三张图的弧度值）
            motor_radians[0] = -0.101999983;
            motor_radians[1] = -0.29700008;
            motor_radians[2] = 1.80600858;
            motor_radians[3] = 2.40001273;
            motor_radians[4] = 0.0119999889;
            motor_radians[5] = -0.0150000118;
            break;

        case MID_VALUE: // 举手（中间姿态，第二张图的弧度值）
            motor_radians[0] = -0.101999983;
            motor_radians[1] = -0.29700008;
            motor_radians[2] = 1.79100847;
            motor_radians[3] = 1.88400912;
            motor_radians[4] = 0.0119999889;
            motor_radians[5] = -0.0150000118;
            break;

        case LOW_VALUE: // 挥手到右边（第一张图的弧度值）
            motor_radians[0] = -0.101999983;
            motor_radians[1] = -0.29700008;
            motor_radians[2] = 1.88700914;
            motor_radians[3] = 1.30800509;
            motor_radians[4] = 0.0119999889;
            motor_radians[5] = -0.0150000118;
            break;

        // 默认情况：保持当前关节角度（避免无操作时数组值异常）
        default:
            // 可选：如果需要默认姿态，可赋值为举手姿态
            // motor_radians[0] = -0.0450000018f;
            // ... 其他关节赋值
            break;
        }
    }
}

void Up_Down_Motor_Control_Updata(void)
{
    switch (SBUS_CH.CH7)
    {
    case HIGH_VALUE:
        aim_tx_height = 100;
        break;
    case LOW_VALUE:
        aim_tx_height = 700;
        break;
    case MID_VALUE:
        aim_tx_height = 400;
        break;
    default:
        break;
    }
}

void Chassis_Control_Updata(void)
{
    float chassis_vx = 0.0f;
    float chassis_vy = 0.0f;
    float chassis_yaw = 0.0f;

    if (sbus_match(SBUS_CH.CH8, LOW_VALUE) &&
        sbus_match(SBUS_CH.CH6, LOW_VALUE) &&
        sbus_match(SBUS_CH.CH7, LOW_VALUE) &&
        sbus_match(SBUS_CH.CH5, LOW_VALUE))
    {
        chassis_vx = -sbus_axis_to_float(SBUS_CH.CH2, 300.0f);
        chassis_vy = sbus_axis_to_float(SBUS_CH.CH1, 300.0f);
        chassis_yaw = sbus_axis_to_float(SBUS_CH.CH3, 200.0f);
    }

    up_tx_data.chassis_vx = chassis_vx;
    up_tx_data.chassis_vy = chassis_vy;
    up_tx_data.chassis_yaw = chassis_yaw;
    x = chassis_vx;
    y = chassis_vy;
    w = chassis_yaw;
}

// PC控制函数实现

/**
 * @brief PC控制气泵更新函数
 * @note 使用PC传入的气泵状态信息进行控制
 */
void PC_Pump_Control_Updata(void)
{
    // 使用PC传入的气泵状态信息
    if (pc_dn_data.pc_pump_state == 1)
    {
        pump_state = PUMP_ON;
    }
    else if (pc_dn_data.pc_pump_state == 0)
    {
        pump_state = PUMP_OFF;
    }
}

/**
 * @brief PC控制头部电机更新函数
 * @note 使用PC传入的电机角度信息（弧度单位）进行控制
 */
void PC_Head_Motor_Control_Updata(void)
{
    // 使用PC传入的头部电机目标角度信息（弧度）
    head_motor_data[0].target_angle = head_radian_to_target_angle(pc_dn_data.pc_target_head_motor_angles[0]);
    head_motor_data[1].target_angle = head_radian_to_target_angle(pc_dn_data.pc_target_head_motor_angles[1]);
}

/**
 * @brief PC控制升降电机更新函数
 * @note 使用PC传入的升降目标高度信息进行控制
 */
void PC_Up_Down_Motor_Control_Updata(void)
{
    // 使用PC传入的升降目标高度信息（0.1mm单位）
    aim_tx_height = pc_dn_data.pc_target_lift_height;
}

/**
 * @brief PC控制机械臂舵机和电机更新函数
 * @note 使用PC传入的舵机和电机角度值（弧度单位）分别赋值给motor_radians数组
 *       pc_target_servo_angles[0~5]对应舵机0~5
 *       pc_target_motor_angles[0~5]对应电机0~5
 */
void PC_Arm_Motor_Control_Updata(void)
{
    DnData_t pc_snapshot;
    float target_motor_angles[ARM_LOGICAL_MOTOR_COUNT];

    if (UART_Protocol_CopyLatestDnData(&pc_snapshot) == 0U)
    {
        return;
    }

    // 将PC传入的6路舵机角度值赋值给motor_radians数组
    motor_radians[0] = pc_snapshot.pc_target_servo_angles[0];
    motor_radians[1] = pc_snapshot.pc_target_servo_angles[1];
    motor_radians[2] = pc_snapshot.pc_target_servo_angles[2];
    motor_radians[3] = pc_snapshot.pc_target_servo_angles[3];
    motor_radians[4] = pc_snapshot.pc_target_servo_angles[4];
    motor_radians[5] = pc_snapshot.pc_target_servo_angles[5];
    // 将PC传入的6路电机角度值赋值给电机臂目标角度
    target_motor_angles[0] = pc_snapshot.pc_target_motor_angles[0];
    target_motor_angles[1] = pc_snapshot.pc_target_motor_angles[1];
    target_motor_angles[2] = pc_snapshot.pc_target_motor_angles[2];
    target_motor_angles[3] = pc_snapshot.pc_target_motor_angles[3];
    target_motor_angles[4] = pc_snapshot.pc_target_motor_angles[4];
    target_motor_angles[5] = pc_snapshot.pc_target_motor_angles[5];
    Arm_SetPcTargetAngles(target_motor_angles, HAL_GetTick());
}

void PC_Arm_Motor_Enable_Disable_Updata(void)
{
    PcArmMotorCtrl_t command;

    if (UART_Protocol_GetArmMotorCtrlCommand(&command) == 0U)
    {
        return;
    }

    if ((command.motor_index >= ARM_LOGICAL_MOTOR_COUNT) ||
        (command.enable_state > 1U))
    {
        return;
    }

    pc_arm_motor_enable_state_debug[command.motor_index] = command.enable_state;
    pc_arm_motor_last_index_debug = command.motor_index;
    pc_arm_motor_last_enable_state_debug = command.enable_state;
    pc_arm_motor_command_count_debug++;

    if (Arm_Motor_Disable_Updata() != 0U)
    {
        return;
    }

    if (command.enable_state != 0U)
    {
        (void)Arm_EnableMotorByIndex(command.motor_index);
    }
    else
    {
        (void)Arm_DisableMotorByIndex(command.motor_index);
    }
}

void pc_up_tx_data(void)
{
    up_tx_data.chassis_vx = x;
    up_tx_data.chassis_vy = y;
    up_tx_data.chassis_yaw = w;
}

void pc_arm_tx_data(void)
{
    if (g_ShoulderType == SHOULDER_TYPE_DARAN)
    {
        up_tx_data.arm_motor_angle_1 = Daran_motor_data[0].current_angle;
        up_tx_data.arm_motor_angle_2 = Daran_motor_data[1].current_angle;
        up_tx_data.arm_motor_angle_3 = Daran_motor_data[2].current_angle;
    }
    else
    {
        up_tx_data.arm_motor_angle_1 = Arm_WrapAngleToPi(Linzu_motor_data[0].current_angle);
        up_tx_data.arm_motor_angle_2 = Arm_WrapAngleToPi(Linzu_motor_data[1].current_angle);
        up_tx_data.arm_motor_angle_3 = Arm_WrapAngleToPi(Linzu_motor_data[2].current_angle);
    }

    up_tx_data.arm_motor_angle_4 = Damiao_motor_data[0].current_angle;
    up_tx_data.arm_motor_angle_5 = Damiao_motor_data[1].current_angle;
    up_tx_data.arm_motor_angle_6 = Damiao_motor_data[2].current_angle;
    up_tx_data.lift_height = lift_height_final;
    up_tx_data.head_motor_angle_1 = head_motor_data[0].current_angle;
    up_tx_data.head_motor_angle_2 = head_motor_data[1].current_angle;
}
